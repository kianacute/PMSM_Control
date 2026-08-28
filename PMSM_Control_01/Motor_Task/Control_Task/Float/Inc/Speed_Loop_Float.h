#ifndef __Speed_LOOP_H__
#define __Speed_LOOP_H__

#include "Hal_Math_Float.h"
#include "Motor_Config_Float.h"
#include "Motor_Control.h"

#define SPEED_ADD_STEP (1000 / 1000.0f)
#define SPEED_SUB_STEP (1000 / 1000.0f)
#define SPEED_ID_ADD_STEP (1.0 / 1000.0f)
#define SPEED_ID_SUB_STEP (1.0 / 1000.0f)
#define SPEED_SWITCH_ID_SUB_STEP (0.001f)


#define PWM_SWITH_FREQ_MAX  (20000.0f)
#define PWM_SWITH_FREQ_MIN  (1000.0f)
#define PWM_SWITH_FREQ_STEP    (2000.0f/1000.0f)

typedef struct Speed_Loop_FLoat
{
    float FREQ_Hz;                      // 循环周期
    uint64_t IF_Start_Cnt;               // 速度控制非空闲状态计时器
    float target_iq, target_id, target_is; // 目标电流
    float Speed_Command;                   // 速度命令
    float Speed_Ref, Speed_Fb;             // 速度参考值和反馈值
    float Speed_Fb_1s;
    float Speed_Sub_Step, Speed_Add_Step;  // 速度增减步长
    Hal_PI_f32_t Speed_PI;                     // 速度PI控制器参数
    uint32_t Speed_Switch_Cnt;             // IF模式切换计数器
    uint8_t Speed_Switch_Flag;             // 速度闭环标志
    float Vs;
    float Voltage_err;
    Hysteresis_Comp_TypeDef_f32_t Weak_Control_Hcomp;      // 弱磁滞回比较器
    Hal_PI_f32_t Weak_Pi;                                // 弱磁PI控制器参数
    uint8_t Align_Finish_Flag;                       // 对准完成标志
    Hysteresis_Comp_TypeDef_f32_t Speed_Middle_High_Hcomp; // 中高档速度滞回比较器
    Hal_PI_f32_t Derating_Pi;                            // 限功率PI控制器参数
    float Derating_Factor;
    float MTPA_Id;
    float Flux_Weak_Id;
    float PWM_SWITCH_FREQ;
    float PWM_CUR_FREQ;
} Speed_Loop_Float_t;

void SPEED_Init_Float(Motor_Control_t *pControl);
void SPEED_Idle_Task_Float(Motor_Control_t *pControl);
void SPEED_Align_Task_Float(Motor_Control_t *pControl);
void SPEED_Open_Task_Float(Motor_Control_t *pControl);
void SPEED_Switch_Task_Float(Motor_Control_t *pControl);
void SPEED_Low_Task_Float(Motor_Control_t *pControl);
void SPEED_Middle_Task_Float(Motor_Control_t *pControl);
void SPEED_High_Task_Float(Motor_Control_t *pControl);

#endif // __Speed_Loop_H__
