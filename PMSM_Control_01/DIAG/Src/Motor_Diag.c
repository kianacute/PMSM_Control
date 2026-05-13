#include "Motor_Diag.h"
#include "speed_ctrl.h"
#include "Current_Task.h"
#include "Hal_Math.h"


extern Speed_Ctrl_t Speed_Ctrl;
extern Current_Task_t Current_Task;

Hysteresis_Comp_TypeDef Motor_Over_Speed_Diag;
Hysteresis_Comp_TypeDef Motor_Phase_Lock;
Hysteresis_Comp_TypeDef Motor_Block_Detect_Diag;

void MOTOR_PHASE_LOCK(void)
{
    
}

void MOTOR_BLOCK_DETECT(void)
{

} 


void Motor_Diag_Init(void)
{
    Hysteresis_Comp_Init(&Motor_Over_Speed_Diag, MOTOR_OVER_SPEED_THRESHOLD, 0, MOTOR_OVER_SPEED_THRESHOLD_DELAY);
    Hysteresis_Comp_Init(&Motor_Phase_Lock, 0, 0, 0);
    Hysteresis_Comp_Init(&Motor_Block_Detect_Diag, 0, 0, 0);
}

void Motor_Diag_Task(void)
{
    Hysteresis_Comp_Process_Add(&Motor_Over_Speed_Diag, Speed_Ctrl.Speed_Fb);
}