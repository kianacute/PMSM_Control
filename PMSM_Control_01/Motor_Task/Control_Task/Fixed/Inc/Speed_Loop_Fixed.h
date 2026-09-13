#ifndef __Speed_LOOP_FIXED_H__
#define __Speed_LOOP_FIXED  _H__

#include "Hal_Math.h"
#include "Motor_Control.h"

typedef struct Speed_Loop_Fixed
{
    q31_t FREQ_Hz;                      // 循环周期
    uint64_t IF_Start_Cnt;               // 速度控制非空闲状态计时器
    q31_t target_iq, target_id, target_is; // 目标电流
    q31_t Speed_Low_Id;
    q31_t Speed_Command;                   // 速度命令
    q31_t Speed_Ref, Speed_Fb;             // 速度参考值和反馈值
    q31_t Speed_Fb_1s;
    q31_t Speed_Sub_Step, Speed_Add_Step;  // 速度增减步长
    q31_t Speed_Align_Id_A;
    uint32_t Speed_Align_Time_Count;
    Hal_PI_q31_t Speed_PI;                     // 速度PI控制器参数
    uint32_t Speed_Switch_Cnt;             // IF模式切换计数器
    uint8_t Speed_Switch_Flag;             // 速度闭环标志
    q31_t Vs;
    q31_t Voltage_err;
    Hysteresis_Comp_TypeDef_q31_t Weak_Control_Hcomp;      // 弱磁滞回比较器
    Hal_PI_q31_t Weak_Pi;                                // 弱磁PI控制器参数
    uint8_t Align_Finish_Flag;                       // 对准完成标志
    Hysteresis_Comp_TypeDef_q31_t Speed_Middle_High_Hcomp; // 中高档速度滞回比较器
    Hal_PI_q31_t Derating_Pi;                            // 限功率PI控制器参数
    q31_t Derating_Factor;
    q31_t MTPA_Id;
    q31_t Flux_Weak_Id;
    q31_t PWM_SWITCH_FREQ;
    q31_t PWM_CUR_FREQ;
} Speed_Loop_Fixed_t;


#endif // __Speed_LOOP_FIXED_H__
