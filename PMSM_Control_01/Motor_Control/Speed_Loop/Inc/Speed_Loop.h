#ifndef __Speed_LOOP_H__
#define __Speed_LOOP_H__

#include "Hal_Math.h"
#include "Motor_Config.h"

#define SPEED_ADD_STEP (1000 / 1000.0f)
#define SPEED_SUB_STEP (1000 / 1000.0f)
#define SPEED_ID_ADD_STEP (1.0 / 1000.0f)
#define SPEED_ID_SUB_STEP (1.0 / 1000.0f)
#define SPEED_SWITCH_ID_SUB_STEP (0.0005f)

enum Speed_LoopState_t
{
    Speed_Loop_IDLE = 0,
    Speed_Loop_ALIGN,
    Speed_Loop_OPEN,
    Speed_Loop_SWITCH,
    Speed_Loop_RUN,
};

typedef struct Speed_Loop
{
    uint32_t FREQ_Hz;                               // 循环周期
    enum Speed_LoopState_t spd_ctrl_state;          // 速度控制状态
    uint32_t spd_ctrl_timer;                        // 速度控制状态计时器
    uint32_t tick_count_idle;                       // 进入空闲状态的时间戳
    float target_iq, target_id, target_is;          // 目标电流
    float Speed_Command;                            // 速度命令
    float Speed_Ref, Speed_Fb;                      // 速度参考值和反馈值
    float Speed_Sub_Step, Speed_Add_Step;           // 速度增减步长
    Motor_Config_t *pMotor;                         // 电机参数指针 
    Hal_PI_t Speed_PI;                              // 速度PI控制器参数
    uint32_t Speed_Switch_Cnt;                      // IF模式切换计数器
    uint8_t Speed_Switch_Flag;                      // 速度闭环标志  
    float Vs;  
    float Voltage_err;
    Hysteresis_Comp_TypeDef Weak_Control_Hcomp;     // 弱磁滞回比较器
    Hal_PI_t Weak_Pi;                               // 弱磁PI控制器参数
} Speed_Loop_t;

void Speed_Loop_Init(void);
void Speed_Loop_Task(void);

#endif // __Speed_Loop_H__
