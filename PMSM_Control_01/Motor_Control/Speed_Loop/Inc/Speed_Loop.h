#ifndef __Speed_LOOP_H__
#define __Speed_LOOP_H__

#include "Hal_Math.h"
#include "Motor_Config.h"

#define SPEED_ADD_STEP (2000 / 1000.0f)
#define SPEED_SUB_STEP (1000 / 1000.0f)
#define SPEED_ID_ADD_STEP (1.0 / 1000.0f)
#define SPEED_ID_SUB_STEP (1.0 / 1000.0f)
#define SPEED_SWITCH_ID_SUB_STEP (0.001f)

enum Speed_LoopState_t
{
    Speed_Loop_Idle = 0,
    Speed_Loop_Align,
    Speed_Loop_Open,
    Speed_Loop_Switch,
    Speed_Loop_Low,
    Speed_Loop_Middle,
    Speed_Loop_High,
};

/* 一阶LADRC控制器
 * 参考: https://zhuanlan.zhihu.com/p/664345718
 * 一阶系统模型: dy/dt = f + b0*u
 * LESO (线性扩张状态观测器): 估计系统输出z1和总扰动z2
 * LSEF (线性误差反馈): u = (wc*(ref - z1) - z2) / b0
 * 参数整定: wo = (3~10)*wc, b0越大抗扰越弱
 */
typedef struct LADRC_FirstOrder
{
    float h;       // 采样周期 (s)
    float b0;      // 控制增益, 一阶系统: b0 = Kt/J (转矩常数/转动惯量)
    float wo;      // 观测器带宽 (rad/s), beta1=2*wo, beta2=wo^2
    float wc;      // 控制器带宽 (rad/s), kp = wc
    float z1;      // 输出估计值 (速度观测)
    float z2;      // 总扰动估计值 (负载+模型不确定性)
    float u;       // 当前控制量
    float out_max; // 输出上限
    float out_min; // 输出下限
} LADRC_FirstOrder_t;

typedef struct Speed_Loop
{
    uint32_t FREQ_Hz;                      // 循环周期
    enum Speed_LoopState_t spd_ctrl_state; // 速度控制状态
    uint64_t spd_ctrl_timer;               // 速度控制非空闲状态计时器
    float target_iq, target_id, target_is; // 目标电流
    float Speed_Command;                   // 速度命令
    float Speed_Ref, Speed_Fb;             // 速度参考值和反馈值
    float Speed_Sub_Step, Speed_Add_Step;  // 速度增减步长
    Motor_Config_t *pMotor;                // 电机参数指针
    Hal_PI_t Speed_PI;                     // 速度PI控制器参数
    uint32_t Speed_Switch_Cnt;             // IF模式切换计数器
    uint8_t Speed_Switch_Flag;             // 速度闭环标志
    float Vs;
    float Voltage_err;
    Hysteresis_Comp_TypeDef Weak_Control_Hcomp;      // 弱磁滞回比较器
    Hal_PI_t Weak_Pi;                                // 弱磁PI控制器参数
    LADRC_FirstOrder_t Speed_LADRC;                  // 一阶LADRC控制器
    uint8_t Align_Finish_Flag;                       // 对准完成标志
    Hysteresis_Comp_TypeDef Speed_Middle_High_Hcomp; // 中高档速度滞回比较器
    Hal_PI_t Derating_Pi;                            // 限功率PI控制器参数
    float Derating_Factor;
} Speed_Loop_t;

void Speed_Loop_Init(void);
void Speed_Loop_Task(void);

#endif // __Speed_Loop_H__
