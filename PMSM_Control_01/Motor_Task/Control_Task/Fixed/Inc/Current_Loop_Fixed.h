#ifndef __Current_Loop_FIXED_H__
#define __Current_Loop_FIXED_H__

#include <stdint.h>
#include "Hal_Math_Fixed.h"
#include "Motor_Config_Fixed.h"

#define MOTOR_ADC_OFFSET_SAMPLE_CNT                 (100U)
#define WEAK_VOLTAGE_COMPENSATION                   (2.0f/3.0f)
#define PWM_OPEN                                    (1U)
#define PWM_CLOSE                                   (0U)


enum Motor_State{
    MOTOR_IDLE = 0,
    MOTOR_READY,
    MOTOR_OFFSET_CHECK,
    MOTOR_RS_IDENTIFY,
    MOTOR_RUN,
    MOTOR_FAULT,
    MOTOR_WAIT,
};

typedef struct Current_Loop_Input_Fixed
{
    q15_t Ia_fb_raw, Ib_fb_raw, Ic_fb_raw;
    q15_t Udc_ADISR;
} Current_Loop_Input_Fixed_t;

typedef struct Current_Loop_Output_Fixed
{
    float PWM_HZ_Coeff;
    float PWM_duty_a, PWM_duty_b, PWM_duty_c, PWM_duty_d;
} Current_Loop_Output_Fixed_t;

typedef struct Current_Loop_Fixed
{
    // Define any necessary variables and structures for the current task
    float FREQ_HZ;                                       //电流环频率
    float Loop_time_s;                                      //电流环循环时间
    enum Motor_State Motor_State;
    /*电压电流，PI控制*/
    Motor_Config_Fixed_t *pMotor;                             //电机参数指针  
    Hal_PI_Q31_t Id_PI;                                     //d轴电流PI控制器参数
    Hal_PI_Q31_t Iq_PI;                                     //q轴电流PI控制器参数
    q31_t Ud_Target, Uq_Target;                             
    q31_t Id_Ref, Iq_Ref;
    q31_t Id_fb, Iq_fb;
    q31_t Is_fb;
    q31_t Ualpha_Ref, Ubeta_Ref, sinVal, cosVal;
    q31_t theta;
    q31_t ialpha_fb, ibeta_fb;
    q31_t Ia_fb, Ib_fb, Ic_fb;
    q31_t Ia_fb_offset, Ib_fb_offset, Ic_fb_offset;
    q31_t PWM_duty_a, PWM_duty_b, PWM_duty_c;
    uint8_t sector;
    q31_t PWM_FREQ_Coeff;                               //变载频系数，PWM_FREQ_Coeff = CUR_HZ / FREQ_HZ

    /*缺相诊断*/
    uint32_t offset_check_cnt;
    q31_t A_Max, B_Max, C_Max;
    uint32_t Phase_check_cnt;
    uint32_t Phase_check_cnt_THD;

    /*其他参数*/
    uint32_t Motor_Wait_Cnt;
    uint8_t PWM_OPEN_Flag;
    uint8_t Dead_Zone_Enable_Flag;
    q31_t Bus_Current, Bus_Current_LPF;
    q31_t Speed_fb_1ms;
    uint8_t avg_count;
    uint32_t Loop_count;

}Current_Loop_Fixed_t;


// void Current_Loop_Init(void);
// void Current_Para_Updata(float speed, float Ts);

extern Current_Loop_Input_Fixed_t Current_Loop_Input_Fixed;
extern Current_Loop_Output_Fixed_t Current_Loop_Output_Fixed;

#endif // __Current_Loop_Fixed_H__