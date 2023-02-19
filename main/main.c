#include <stdio.h>
#include <stdlib.h>
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/Timers.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_attr.h"
#include "soc/rtc.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "driver/adc.h"
#include "esp_log.h"
#include "rotary_encoder.h"
#include "driver/timer.h"
#include <math.h>

#define CLASS 3

#define TIMER_DIVIDER         (16)  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define ClassSetIntervalSec 0.5

const char* tagSteps="Steps"; 
const char* tagClass="Class"; 
const char* tagSpeed="Speed"; 
enum{
    origin, speed, position, class, mode_main       //实际未启用
}motor_mode;
/**
 * @brief 实现转动到指定角度控制的结构体* 
 */

struct position_handle{
    int16_t angle;//设定转动角度，angle=0可以实现复位
    char motor_position_status;//0:初始，1:正篇，2:负偏，4:无偏
    int16_t pos_error;//与目标位置的距离
    int16_t pos_target;
    float i_target;//仅用于在control环节做额定电流设置
};
struct position_handle position_struct1={0,0,0,0,0};
struct position_handle *position_struct;

/**
 * @brief 进行档位设置的结构体 * 
 */
struct class_handle{
    uint8_t num;
    char motor_position_status;//0:初始，1:正篇，2:负偏，4:无偏       //实际未启用
    char motor_class_status;//0:初始，1:正篇，2:负偏，4:无偏       //实际未启用
    int16_t class_range;//记录一个档位大小
    int16_t half_class_range;
    int16_t class_cnt;//记录当前档位
    int16_t degree_cnt;//当前档位分度值
    int16_t enc_error;//与位置0的距离，用于划分当前的分区
    float i_target;//仅用于在control环节做额定电流设置
}; 
struct class_handle class_struct1={0,0,0,0,0,0,0,0,0.0};
struct class_handle * class_struct; 
bool classflag=0;
#define Pos_PE 22
#define V_Current_PE 10

struct SpeedHandle{
    float v_current;
    float v_target;
    int16_t e_sum;
    float v_error;
    float i_target;//仅用于在control环节做额定电流设置
    int32_t enc_cnt_p;
    float v_current_p[7];
};
struct SpeedHandle * speed_struct;
struct SpeedHandle speed_struct1={0,0,0,0,0,0};

//动态脱手检测：
inline bool stable_judgment(struct SpeedHandle *speed_struct){
    int16_t v_sum;
    for(uint8_t i=0;i<7;i++){
        v_sum += speed_struct->v_current_p[i];
    }
    return ((v_sum/7)-speed_struct->v_target)>=//TODO
}

inline bool increasing_judgment(struct SpeedHandle *speed_struct){
    if(speed_struct->v_target>=0){
        for(uint8_t i=1;i<7;){
            if(speed_struct->v_current_p[i]>=speed_struct->v_current_p[i-1])
                i++;
            else
                return false;
        }
        return true;
    }
    else{
        for(uint8_t i=1;i<7;){
        if(speed_struct->v_current_p[i]<=speed_struct->v_current_p[i-1])
            i++;
        else
            return false;
        }
        return true;
    }
};

inline bool decreasing_judgment(struct SpeedHandle *speed_struct){
    if(speed_struct->v_target>=0){
        for(uint8_t i=1;i<7;){
            if(speed_struct->v_current_p[i]>=speed_struct->v_current_p[i-1])
                i++;
            else
                return false;
        }
        return true;
    }
    else{
        for(uint8_t i=1;i<7;){
        if(speed_struct->v_current_p[i]<=speed_struct->v_current_p[i-1])
            i++;
        else
            return false;
        }
        return true;
    }
};

bool speedflag=0;     //1,自动运行；0,不自动运行
#define i_limit 0.45f

struct StartHandle{
    char CW;    //clockwise
    int16_t v_start;
    int16_t v_max;
    int16_t v_end;
    int32_t enc_cnt_p;
    int16_t v_current;
};
struct StartHandle start_handle0={0,0,0,0,0,0};
struct StartHandle * start_handle=&start_handle0;
#define V_End_TH 20
#define V_Start_TH 100
#define Blocked_Rotation_Voltage 3.0f

TimerHandle_t rotary_settings_timer;
bool loopflag=0;

    /* basic encoder */
    rotary_encoder_t *encoder = NULL;
    int16_t enc_cnt;//目前的脉冲数

    char motor_speed_status=0;//0:静止，1:正转，2:反转，3:停止，4:最大正转，5:最大反转
    #define PID_P_V 0.0049797f
    #define PID_I_V 0.00044505f  //0.0445052390679643*1e-2
    #define MAX_V 500
    #define MIN_V 70//只在速度足够大才控制电流r_handle

    int32_t g_adc_offset;
    
    float e_sum;
    uint8_t windup = 0;
    float i_current;
    float i_error;
    float i_offset;
    float u=0;

    #define ONE_ROUND 2156 //输出轴转过一周的脉冲数
    #define ONE_CLASS 2156 //一个档位的脉冲数量
    #define HALF_CLASS 100
    #define MAX_I 0.13f
    #define DELAY_MS 10
    #define PARA_ENCODER 14.28f //(DELAY_HZ)100*2pi/44 11线编码器


static void PwmIsr(void *arg);
static void ClassSetIsr(struct class_handle * class_struct);
void PysbMotorInitA();//初始化ESP32电机中断，GPIO，电流偏差等
void PysbMotorInit();

void PysbMotorPositionControl(struct position_handle * position_struct);
inline void PysbMotorClassPositionSet(struct class_handle * class_struct);
void PysbMotorClassPositionControl(struct class_handle * class_struct);
inline void PysbMotorSpeedSampling(struct SpeedHandle * speed_struct);
void PysbMotorSpeedControl(struct SpeedHandle * speed_struct);

void RotartSettingsCallback( TimerHandle_t rotary_settings_timer);
void PysbMotorLoop(struct StartHandle * start_handle);
void PysbMotorLoopReset(struct StartHandle * start_handle);

void printf_speed(){
    printf("speed.v_current:%f\n",speed_struct1.v_current);
    printf("speed.v_target:%f\n",speed_struct1.v_target);
    /* printf("speed.i_target:%f\n",speed_struct1.i_target);
    printf("speed.e_sum:%d\n",speed_struct1.e_sum); */
}

void printf_class(){
    printf("class.enc_error:%d\n",class_struct1.enc_error);
    printf("class.class_cnt:%d\n",class_struct1.class_cnt);
    printf("class.degree_cnt:%d\n",class_struct1.degree_cnt);
}

void printf_startloop(){
    printf("startloop:CW:%c\n",start_handle0.CW);
    printf("startloop:v_start:%d\n",start_handle0.v_start);
    printf("startloop:v_max:%d\n",start_handle0.v_max);
    printf("startloop:v_end:%d\n",start_handle0.v_end);
    printf("u:%f\n",u);
}

QueueHandle_t power_queue;
QueueHandle_t position_queue;
QueueHandle_t class_queue;
QueueHandle_t speed_queue;
int32_t g_adc_offset; 

inline void PysbMotorInit(){
    PysbMotorInitA();
}

inline void PysbMotorInitA(){
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11);
    for (uint8_t i = 0; i < 100; i++)
    {
        g_adc_offset += adc1_get_raw(ADC1_CHANNEL_7);
    }
    g_adc_offset /= 100;
    i_offset= g_adc_offset * 0.000634921f;
    {
    uint32_t pcnt_unit = 0;
    rotary_encoder_config_t config = ROTARY_ENCODER_DEFAULT_CONFIG((rotary_encoder_dev_t)pcnt_unit, 39, 36);
    rotary_encoder_new_ec11(&config, &encoder);
    encoder->set_glitch_filter(encoder, 1);
    encoder->start(encoder);
    }
   
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 22);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, 23);

    MCPWM0.operators[0].gen_stmp_cfg.gen_a_upmethod = 1;
    MCPWM0.operators[0].gen_stmp_cfg.gen_b_upmethod = 1;
    MCPWM0.operators[0].gen_stmp_cfg.gen_a_shdw_full = 1;
    MCPWM0.operators[0].gen_stmp_cfg.gen_b_shdw_full = 1;

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 4000;
    pwm_config.cmpr_a = 50.0f;
    pwm_config.cmpr_b = 50.0f;
    pwm_config.counter_mode =  MCPWM_UP_DOWN_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_1;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

    power_queue = xQueueCreate(1, sizeof(int32_t));
    position_queue = xQueueCreate(1, sizeof(float));
    class_queue = xQueueCreate(1, sizeof(float));
    speed_queue = xQueueCreate(1, sizeof(float));

    MCPWM0.int_ena.val = MCPWM_TIMER0_TEZ_INT_ENA;
    mcpwm_isr_register(MCPWM_UNIT_0, PwmIsr, NULL, ESP_INTR_FLAG_IRAM, NULL);
    mcpwm_isr_register(MCPWM_UNIT_1, ClassSetIsr, class_struct, ESP_INTR_FLAG_IRAM, NULL);

    speed_struct=&speed_struct1;
    //动态脱手检测：
    for(uint8_t i=0;i<7;i++){
        speed_struct->v_current_p[i]=0;
    }
    class_struct=&class_struct1;

    class_struct1.num=CLASS;
    class_struct1.class_range=ONE_ROUND/class_struct1.num;
    class_struct1.half_class_range=class_struct1.class_range/2;

    rotary_settings_timer = xTimerCreate( "rotary_settings_timer", 
                                            300 / portTICK_PERIOD_MS,   // The timer period in ticks.
                                            pdFALSE,        // The timers will auto-reload themselves when they expire.
                                            ( void * ) 0,  // Assign each timer a unique id equal to its array index.
                                            RotartSettingsCallback // Each timer calls the same callback when it expires.
                                        );

    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE, 
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = true,
    }; // default clock source is APB
    timer_init(TIMER_GROUP_0, TIMER_0, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, ClassSetIntervalSec * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);

    timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, ClassSetIsr, class_struct, 0);

    timer_start(TIMER_GROUP_0, TIMER_0);
}

inline int16_t SpeedSample(){
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    int16_t enc_cnt_ss = encoder->get_counter_value(encoder);
    int16_t  enc_cnt_pss = enc_cnt_ss;
    vTaskDelayUntil(&xLastWakeTime, 10/ portTICK_PERIOD_MS);
    enc_cnt_ss = encoder->get_counter_value(encoder);
    int16_t v_sample=(enc_cnt_ss - enc_cnt_pss) * PARA_ENCODER;
    return v_sample;
}

void RotartSettingsCallback( TimerHandle_t rotary_settings_timer){
    loopflag=0;
    printf("callback running\n");
    //判定最终速度较大，进入自动运行模式
}

void PysbMotorLoop(struct StartHandle * start_handle){
    //检测v_current循环
    if(start_handle->v_start<=V_Start_TH&&start_handle->v_start>=-V_Start_TH){
        printf("v_startSampling\n");
    }
    while(start_handle->v_start<=V_Start_TH&&start_handle->v_start>=-V_Start_TH){
        start_handle->v_start = SpeedSample();
    }

    loopflag=1;     //计时结束后复位为0
    xTimerStart(rotary_settings_timer,0);
    
    while(loopflag!=0){
        int16_t v_max_positive=0;
        int16_t v_max_negative=0;
        printf("setting\n"); 
        start_handle->v_current =SpeedSample();
        if(start_handle->v_current>=0){
            if(start_handle->v_current>v_max_positive){
                v_max_positive=start_handle->v_current;
                if(start_handle->v_max<0)
                {
                    if(v_max_positive>-start_handle->v_max){
                        start_handle->v_max=v_max_positive;
                    }
                }
                else{
                    if(v_max_positive>start_handle->v_max){
                        start_handle->v_max=v_max_positive;
                    }
                }
            }//refresh v_max
            start_handle->CW='L';
            u=-Blocked_Rotation_Voltage;
        }
        else if(start_handle->v_current<0){
            if(start_handle->v_current<v_max_negative){
                v_max_negative=start_handle->v_current;
                if(start_handle->v_max>0)
                {
                    if(v_max_negative<-start_handle->v_max){
                        start_handle->v_max=v_max_negative;
                    }
                }
                else{
                    if(v_max_negative<start_handle->v_max){
                        start_handle->v_max=v_max_negative;
                    }
                }
            }//refresh v_max
            start_handle->CW='R';
            u=Blocked_Rotation_Voltage;
            ESP_LOGI(tagClass,"Class:%d",class_struct1.class_cnt);
        }

        start_handle->v_end=start_handle->v_current;//refresh v_end

        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, (1 - (u + 5.0f) / 10.0f) * 100);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, (u + 5.0f) / 10.0f * 100);

        start_handle->enc_cnt_p = enc_cnt;
    }

    if(start_handle0.v_end>=V_End_TH||start_handle0.v_end<=-V_End_TH){
        float raw_val;
        float i_error_speed;
        //动态脱手检测：
        bool stableflag;
        bool judgmentflag;
        speedflag=1;
        speed_struct1.v_target=start_handle0.v_max*0.5;
        uint8_t j=0;    //动态脱手检测：用于记录前7个数据
        if(speed_struct1.v_target>=300){
            speed_struct1.v_target=300;
        }
        else if(speed_struct1.v_target<=-300){
            speed_struct1.v_target=-300;
        }
        printf("auto running\n"); 
        //TODO1
        if(start_handle0.v_end>=V_End_TH){
            /* wheelDriverBleTx(ControlBlock, u8 *tx_buffer, u16 tx_buffer_len) */
        }
        else{
            /* wheelDriverBleTx(ControlBlock, u8 *tx_buffer, u16 tx_buffer_len) */
        }
        
        while(speedflag!=0){
            PysbMotorSpeedSampling(speed_struct);   //当检测到速度被捏停退出循环，TODO2，当检测到速度重新设定时进入改变v_target流程
            PysbMotorSpeedControl(speed_struct);    //控制速度
            /* printf_speed(); */
            //动态脱手检测：
            if(j!=7){
                speed_struct->v_current_p[j]=speed_struct->v_current;
                j++;
            }
            else{
                for(uint8_t i=1;i<6;i++){
                    speed_struct->v_current_p[i-1]=speed_struct->v_current_p[i];
                }
                speed_struct->v_current_p[6]=speed_struct->v_current;
            }
            judgmentflag=(increasing_judgment(speed_struct)||decreasing_judgment(speed_struct));
            if(judgmentflag){
                ESP_LOGI(tagSpeed,"go into speedsetting");
            }
            printf("%f\t%f\n",speed_struct1.v_current,speed_struct1.v_target);
            /* ESP_LOGI(tagSpeed,"%f\t%f",speed_struct1.v_current,speed_struct1.v_target); */
            /* ESP_LOGI(tagClass,"Class:%d",class_struct1.class_cnt); */
            while(xQueueReceive(power_queue, &raw_val, portMAX_DELAY) != pdTRUE){
            //等待中断中读入电流值
                printf("speed power waiting\n");
            }
            i_current = (float)(raw_val) * 0.000634921f;	//raw_val to current(A);
            i_error_speed = (* speed_struct).i_target - i_current;
            u =i_error_speed * 8;//TODO
            if (u > 4.99f)
            {
                u = 4.99f;
                windup = 1;
            }
            else if (u < -4.99f)
            {
                u = -4.99f;
                windup = 2;     //TODO：对于积分环节的启用
            }
            /* printf("u:%f",u); */
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, (1 - (u + 5.0f) / 10.0f) * 100);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, (u + 5.0f) / 10.0f * 100);
        }
        
        speedflag=0;//自动运行结束
        ESP_LOGI(tagSteps,"exist auto running\n");
        {
            speed_struct->v_target=0;
            speed_struct->v_current=0;
            speed_struct->e_sum=0;
            //动态脱手检测：
            judgmentflag=0;
            for(uint8_t i=0;i<7;i++){
                speed_struct->v_current_p[i]=0;
            }
        }

        //TODO1: 发送信息：用户停止自动运行
        /* wheelDriverBleTx(Target target=ControlBlock, u8 *tx_buffer, u16 tx_buffer_len); */

        classflag=1;
        ESP_LOGI(tagSteps,"class lock running\n");//TODO可以设置多次单次运行以后默认单词运行的方式
        
        enc_cnt=encoder->get_counter_value(encoder);
        float class_v_current;
        int16_t class_cnt_p=class_struct->class_cnt;
        while(classflag!=0){
            float raw_val;
            enc_cnt=encoder->get_counter_value(encoder);
            PysbMotorClassPositionSet(class_struct);    //计算当前位置误差
            //TODO1
            if(class_struct->class_cnt-class_cnt_p>=1){   
                /* wheelDriverBleTx(Target target=ControlBlock, u8 *tx_buffer, u16 tx_buffer_len); */
            }   //正向旋转一格
            else if(class_struct->class_cnt-class_cnt_p<=1){
                /* wheelDriverBleTx(Target target=ControlBlock, u8 *tx_buffer, u16 tx_buffer_len); */
            }   //负向旋转一格
            PysbMotorClassPositionControl(class_struct);//力矩反馈TODO
            class_v_current = SpeedSample();
            printf("class_v_current:%f\n",class_v_current);
            printf_class();
            while(xQueueReceive(power_queue, &raw_val, portMAX_DELAY) != pdTRUE){
            //等待中断中读入电流值
                ESP_LOGI(tagSteps,"class power waiting\n");    //DEL:这段时间很有可能是不存在的
            }
            i_current = (float)(raw_val ) * 0.000634921f;	//raw_val to current(A);
            i_error = (* class_struct).i_target - i_current;
            u =8*i_error;
            classflag=((* class_struct).degree_cnt<=Pos_PE&&(* class_struct).degree_cnt>=-Pos_PE)&&class_v_current<=V_Current_PE&&class_v_current>=-V_Current_PE;//当检测到位置误差小于22并且当前速度趋于0退出循环
            classflag=!classflag;
            if(classflag==false){
                ESP_LOGI(tagSteps,"exist class lock\n");
                classflag=0;    //结束自动运行
            }
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, (1 - (u + 5.0f) / 10.0f) * 100);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, (u + 5.0f) / 10.0f * 100);
        }
    }
    else{                                           //(start_handle0.v_end<=V_End_TH&&start_handle0.v_end<=-V_End_TH)
        classflag=1;
        ESP_LOGI(tagSteps,"class lock running\n");//TODO0:可以设置多次单次运行以后默认单词运行的方式
        enc_cnt=encoder->get_counter_value(encoder);
        float class_v_current;
        while(classflag!=0){
            float raw_val;
            enc_cnt=encoder->get_counter_value(encoder);
            PysbMotorClassPositionSet(class_struct);    //计算当前位置误差
            PysbMotorClassPositionControl(class_struct);//力矩反馈
            class_v_current = SpeedSample();
            printf("class_v_current:%f\n",class_v_current);
            printf_class();
            while(xQueueReceive(power_queue, &raw_val, portMAX_DELAY) != pdTRUE){
            //等待中断中读入电流值
                ESP_LOGI(tagSteps,"class power waiting\n");    //DEL:这段时间很有可能是不存在的
            }
            i_current = (float)(raw_val ) * 0.000634921f;	//raw_val to current(A);
            i_error = (* class_struct).i_target - i_current;
            u =8*i_error;
            classflag=((* class_struct).degree_cnt<=Pos_PE&&(* class_struct).degree_cnt>=-Pos_PE)&&class_v_current<=V_Current_PE&&class_v_current>=-V_Current_PE;//当检测到位置误差小于22并且当前速度趋于0退出循环
            classflag=!classflag;
            if(classflag==false){
                ESP_LOGI(tagSteps,"exist class lock\n");
                classflag=0;    //结束自动运行
            }
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, (1 - (u + 5.0f) / 10.0f) * 100);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, (u + 5.0f) / 10.0f * 100);
        }      
    }
}
    
void PysbMotorLoopReset(struct StartHandle * start_handle){
    start_handle->v_end=0;
    start_handle->v_max=0;
    start_handle->CW=0;
}

void app_main(void){
    PysbMotorInit();
    while(1){
        PysbMotorLoop(start_handle);
        printf_startloop();

        start_handle->v_end=0;
        start_handle->v_start=0;
        start_handle->v_max=0;
        start_handle->CW=0;
        u=0;
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, (1 - (u + 5.0f) / 10.0f) * 100);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, (u + 5.0f) / 10.0f * 100);
        /* printf_debug();
        printf("i_error:%f\n",i_error);
        printf("i_current:%f\n",i_current);
        printf("u:%f\n",u);
        vTaskDelay(1000/portTICK_RATE_MS); */
    }
}

void ClassSetIsr(struct class_handle * class_struct){
    {
        enc_cnt = encoder->get_counter_value(encoder);
        (*class_struct).enc_error=enc_cnt;
        
        if((*class_struct).enc_error>0){
            (*class_struct).motor_position_status=1;
        }//pos正偏
        else if((*class_struct).enc_error<0){
            (*class_struct).motor_position_status=2;
        }//pos负偏

        int16_t _enc_error=(*class_struct).enc_error+(*class_struct).half_class_range;

        if((*class_struct).motor_position_status==1){
            (* class_struct).motor_class_status=1;
            (* class_struct).class_cnt=_enc_error/(*class_struct).class_range;   
        }//正转
        else if((*class_struct).motor_position_status==2){
            (* class_struct).motor_class_status=2;
            (* class_struct).class_cnt=_enc_error/(*class_struct).class_range-1;
        }//enc_cnt<0，反转        
    }
}

static void IRAM_ATTR PwmIsr(void *arg)
{
    uint32_t mcpwm_intr_status;
    int32_t raw_val;
    mcpwm_intr_status = MCPWM0.int_st.val;
    BaseType_t high_task_awoken = pdFALSE;
    if (mcpwm_intr_status && MCPWM_TIMER0_TEZ_INT_ENA)
    {
        raw_val = adc1_get_raw(ADC1_CHANNEL_7);
        xQueueSendFromISR(power_queue, &raw_val, &high_task_awoken);
    }
    MCPWM0.int_clr.val = mcpwm_intr_status;
    portYIELD_FROM_ISR(high_task_awoken);
    
}

/*--------------------------------------------------------------------------------------------------------*/

//误差计算节点的切换，计算误差，设置档位和分度值
inline void PysbMotorClassPositionSet(struct class_handle * class_struct){
    (*class_struct).enc_error=enc_cnt;
    if((* class_struct).enc_error==0){
        (*class_struct).motor_position_status=4;
    }
    else if((*class_struct).enc_error>0){
        (*class_struct).motor_position_status=1;
    }//pos正偏
    else{
        (*class_struct).motor_position_status=2;
    }//pos负偏

    int16_t _enc_error=(*class_struct).enc_error+(*class_struct).half_class_range;

    if((*class_struct).motor_position_status==4){
        (* class_struct).motor_class_status=4;
    }
    else if((*class_struct).motor_position_status==1){
        (* class_struct).motor_class_status=1;
        (* class_struct).class_cnt=_enc_error/(*class_struct).class_range;   
    }//正转
    else if((*class_struct).motor_position_status==2){
        (* class_struct).motor_class_status=2;
        (* class_struct).class_cnt=_enc_error/(*class_struct).class_range-1;
    }//enc_cnt<0，反转

    (* class_struct).degree_cnt=enc_cnt-(* class_struct).class_cnt*(*class_struct).class_range;
}

#define k1 0.0241f
#define k2 0.000000241f
void PysbMotorPositionControl(struct position_handle *position_struct){
    /* enc_cnt=encoder->get_counter_value(encoder); */
    (*position_struct).pos_error=enc_cnt-(*position_struct).pos_target;
    if((*position_struct).pos_error>15&&(*position_struct).pos_error<=22){
        (*position_struct).i_target=0;
    }
    else if((*position_struct).pos_error>0&&(*position_struct).pos_error<=15){
        (*position_struct).i_target=0;;
    }
        else if((*position_struct).pos_error>22&&(*position_struct).pos_error<500){
        (*position_struct).i_target=k2*(-(*position_struct).pos_error-22)+0.53;
    }
    else if((*position_struct).pos_error<15&&(*position_struct).pos_error>=-22){
        (*position_struct).i_target=0;;
    }
    else if((*position_struct).pos_error<0&&(*position_struct).pos_error>=-15){
        (*position_struct).i_target=0;
    }
    else if((*position_struct).pos_error<-22&&(*position_struct).pos_error>-500){
        (*position_struct).i_target=k2*(-(*position_struct).pos_error+22)-0.53;
    }
    else if((*position_struct).pos_error==0){
        (*position_struct).i_target=0;
    }
    else{
        (*position_struct).i_target=(-((float)(*position_struct).pos_error/(float)HALF_CLASS))*1.5f;
    }
}

void PysbMotorClassPositionControl(struct class_handle * class_struct){
    if((* class_struct).degree_cnt>15&&(* class_struct).degree_cnt<=22){
        (* class_struct).i_target=-0.45;
    }
    else if((* class_struct).degree_cnt>0&&(* class_struct).degree_cnt<=15){
        (* class_struct).i_target=0;;
    }
        else if((* class_struct).degree_cnt>22&&(* class_struct).degree_cnt<500){
        (* class_struct).i_target=k2*(-(* class_struct).degree_cnt-22)+0.53;
    }
    else if((* class_struct).degree_cnt<-15&&(* class_struct).degree_cnt>=-22){
        (* class_struct).i_target=0.45;
    }
    else if((* class_struct).degree_cnt<0&&(* class_struct).degree_cnt>=-15){
        (* class_struct).i_target=0;
    }
    else if((* class_struct).degree_cnt<-22&&(* class_struct).degree_cnt>-500){
        (* class_struct).i_target=k2*(-(* class_struct).degree_cnt+22)-0.53;
    }
    else if((* class_struct).degree_cnt==0){
        (* class_struct).i_target=0;
    }
    else{
        (* class_struct).i_target=(-((float)(* class_struct).degree_cnt/(float)HALF_CLASS))*1.5f;
    }
}

/* ------------------------------------------------------------------------------------------------*/

inline void PysbMotorSpeedSampling(struct SpeedHandle * speed_struct){
    speed_struct->v_current = SpeedSample();
    if(speed_struct->v_target>0){
        if(speed_struct->v_current<=10){
            printf("exist auto running\n");
            speedflag=0;    //结束自动运行
        }
    }
    if(speed_struct->v_target<0){
        if(speed_struct->v_current>=-10){
            printf("exist auto running\n");
            speedflag=0;    //结束自动运行
        }
    }
}

void PysbMotorSpeedControl(struct SpeedHandle * speed_struct){ 
    speed_struct->v_error = speed_struct->v_current - speed_struct->v_target;//TODO:easy to make wrong direction

    {
        speed_struct->e_sum +=speed_struct-> v_error;
    }
    if (speed_struct->v_target > MIN_V || speed_struct->v_target < -MIN_V)//we only control current if velocity is high enough
        speed_struct->i_target = speed_struct->v_error* 0.0249797f +speed_struct-> e_sum * 0.00044505f ;//TODO: +speed_struct-> e_sum * 0.00044505f
    else{
        speed_struct->i_target = 0;
        speed_struct->e_sum = 0;
    }

    if (speed_struct->v_target > MAX_V){
        speed_struct->v_target = MAX_V;
    }
    else if (speed_struct->v_target < -MAX_V){
       speed_struct-> v_target = -MAX_V;
    }
//设定最大电流，同时也检测v_error是否反向了
     
    if(speed_struct->v_target<240)      //低速控制
    {
        if (speed_struct->i_target >= i_limit){
            speed_struct->i_target = i_limit;
        } //rotating B direction
        if (speed_struct->i_target <= -i_limit) {
            speed_struct->i_target = -i_limit;
        }
    }

}
//TODO1：发送信息API

/* void wheelDriverBleTx(Target target=ControlBlock, u8 *tx_buffer, u16 tx_buffer_len){
    //调用蓝牙或者路由器的API
} */