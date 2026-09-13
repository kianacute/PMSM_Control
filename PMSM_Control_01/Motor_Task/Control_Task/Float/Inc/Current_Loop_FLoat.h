#ifndef __Current_Loop_H__
#define __Current_Loop_H__

#include <stdint.h>
#include "Hal_Math.h"
#include "Motor_Control.h"

typedef struct Current_Loop_Float
{
    // Define any necessary variables and structures for the current task
    float FREQ_HZ;                                       //电流环频率
    float Loop_time_s;                                      //电流环循环时间
    /*电压电流，PI控制*/
    Hal_PI_f32_t Id_PI;                                     //d轴电流PI控制器参数
    Hal_PI_f32_t Iq_PI;                                     //q轴电流PI控制器参数
    float Ud_Target, Uq_Target;                             
    float Id_Ref, Iq_Ref;
    float Id_fb, Iq_fb;
    float Is_fb;
    float Ualpha_Ref, Ubeta_Ref, sinVal, cosVal;
    float theta;
    float ialpha_fb, ibeta_fb;
    float Ia_fb, Ib_fb, Ic_fb;
    float Ia_fb_offset, Ib_fb_offset, Ic_fb_offset;
    float PWM_duty_a, PWM_duty_b, PWM_duty_c;
    uint8_t sector;
    float PWM_FREQ_Coeff;                               //变载频系数，PWM_FREQ_Coeff = CUR_HZ / FREQ_HZ
    float Bus_Current, Bus_Current_LPF;
    
    /*缺相诊断*/
    uint32_t offset_check_cnt;
    float A_Max, B_Max, C_Max;
    uint32_t Phase_check_cnt;
    uint32_t Phase_check_cnt_THD;

    /*其他参数*/
    uint32_t Motor_Wait_Cnt;
    uint8_t Dead_Zone_Enable_Flag;

    float Speed_fb_1ms;
    uint8_t avg_count;
}Current_Loop_Float_t;

void Current_Para_Updata_Float(Motor_Control_t *pControl, float speed, float Ts);

#endif // __Current_Loop_H__