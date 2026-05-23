#ifndef __Current_Loop_H__
#define __Current_Loop_H__

#include <stdint.h>
#include "Hal_Math.h"
#include "Motor_parameter.h"

#define MOTOR_ADC_OFFSET_SAMPLE_CNT                 (100U)
#define WEAK_VOLTAGE_COMPENSATION                   (2.0f/3.0f)
#define PWM_OPEN                                    (1U)
#define PWM_CLOSE                                   (0U)


enum Motor_State{
    MOTOR_IDLE = 0,
    MOTOR_READY,
    MOTOR_OFFSET_CHECK,
    MOTOR_RUN,
    MOTOR_default,
};

typedef struct Current_Loop_Input
{
    float Ia_fb_raw, Ib_fb_raw, Ic_fb_raw;
    float Udc_ADISR;
} Current_Loop_Input_t;

typedef struct Current_Loop_Output
{
    float PWM_duty_a, PWM_duty_b, PWM_duty_c, PWM_duty_d;
} Current_Loop_Output_t;

typedef struct Current_Loop
{
    // Define any necessary variables and structures for the current task
    uint32_t FREQ_HZ;                                       //电流环频率
    float Loop_time_s;                                      //电流环循环时间
    MOTOR_t *pMotor;                            //电机参数指针  
    Hal_PI_t Id_PI;                             //d轴电流PI控制器参数
    Hal_PI_t Iq_PI;                                     //q轴电流PI控制器参数
    float Ud_Target, Uq_Target;                             
    float Id_Ref, Iq_Ref;
    float Id_fb, Iq_fb;
    float Is_fb;
    float Ualpha_Ref, Ubeta_Ref, sinVal, cosVal;
    float theta;
    float ialpha_fb, ibeta_fb;
    float Ia_fb, Ib_fb, Ic_fb;
    float Ia_fb_offset, Ib_fb_offset, Ic_fb_offset;
    uint32_t offset_check_cnt;
    float PWM_duty_a, PWM_duty_b, PWM_duty_c;
    float A_Max, A_Min, B_Max, B_Min, C_Max, C_Min;
    uint32_t Phase_check_cnt;
    uint8_t sector;
    enum Motor_State Motor_State;
    uint8_t PWM_OPEN_Flag;
    float Speed_fb_1ms;
    uint8_t avg_count;
    uint32_t Loop_count;
    Lookup_Table_t ID_PI_Kp_Lookup;  
    Lookup_Table_t IQ_PI_Kp_Lookup; 
    Lookup_Table_t ID_PI_Ki_Lookup; 
    Lookup_Table_t IQ_PI_Ki_Lookup; 
}Current_Loop_t;


void Current_Loop_Init(void);
void Current_Loop_run(void);

#endif // SVPWM_H