#ifndef __Current_Loop_FIXED_H__
#define __Current_Loop_FIXED_H__

#include "arm_math.h"
#include "Hal_Math_Fixed.h"

typedef struct Current_Loop_Fixed
{
    // Define any necessary variables and structures for the current task
    q31_t FREQ_HZ;                                       //电流环频率
    q31_t Loop_time_s;                                      //电流环循环时间
    /*电压电流，PI控制*/
    Hal_PI_q31_t Id_PI;                                     //d轴电流PI控制器参数
    Hal_PI_q31_t Iq_PI;                                     //q轴电流PI控制器参数
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
}Current_Loop_Fixed_t;

#endif // __Current_Loop_Fixed_H__