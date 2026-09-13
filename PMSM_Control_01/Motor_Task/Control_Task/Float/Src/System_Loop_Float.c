#include "System_Loop_Float.h"
#include "arm_math.h"
#include "Current_Loop_Float.h"
#include "Hal_Math.h"
#include "Observer_Float.h"
#include "Speed_Loop_Float.h"
#include "System_Diag.h"
#include "Motor_Diag.h"

extern uint8_t MOTOR_Run_flag;
extern float Speed_Command;

uint8_t System_Fault_Flag = 0;

System_Loop_Float_t System_Loop_FLoat;

void SYSTEM_Init_Float(Motor_Control_t *pControl)
{
    pControl->System_Loop.pSystem_Loop = (void*)&System_Loop_FLoat;
    Motor_Diag_Init();
    System_Diag_Init();
    Speed_Command = 1000.0f;
    System_Loop_FLoat.Run_flag = 0;
    Hysteresis_Comp_Init_f32(&System_Loop_FLoat.System_Hv_Comp, SYSTEM_HV_STANDY_THD_V, 5.0f, SYSTEM_HV_STANDY_TIME_S); // 系统高压滞回比较器
}

void SYSTEM_LV_Standy_Float(Motor_Control_t *pControl)
{
    System_Loop_Float_t *pSystem_Loop = (System_Loop_Float_t *)pControl->System_Loop.pSystem_Loop;
    // SYSTEM_Init_Float(pControl);
    pControl->System_Loop.Status = SYSTEM_HV_STANDY;
    pSystem_Loop->Run_flag = 0;
}

void SYSTEM_HV_Standy_Float(Motor_Control_t *pControl)
{
    System_Loop_Float_t *pSystem_Loop = (System_Loop_Float_t *)pControl->System_Loop.pSystem_Loop;
    Motor_Control_Input_t *pMotor_Control_Input = (Motor_Control_Input_t *)&pControl->Input;
    Hysteresis_Comp_Process_Add_f32(&pSystem_Loop->System_Hv_Comp, pMotor_Control_Input->Udc_ADISR); // Update the high voltage hysteresis comparator
    if(pSystem_Loop->System_Hv_Comp.comp_out) // Check if the DC bus voltage is above a certain threshold
    {
        pControl->System_Loop.Status = SYSTEM_RUN;
        pSystem_Loop->Run_flag = 0;
    }
}


extern uint8_t System_Diag_Fault_Flag;

void SYSTEM_Run_Float(Motor_Control_t *pControl)
{
    System_Loop_Float_t *pSystem_Loop = (System_Loop_Float_t *)pControl->System_Loop.pSystem_Loop;
    Speed_Loop_Float_t *pSpeed_Loop = (Speed_Loop_Float_t *)pControl->Speed_Loop.pSpeed_Loop;
    System_Fault_Flag = System_Diag_Fault_Flag; // Combine system and motor diagnostic fault flags
    if(System_Fault_Flag != 0)
    {
        pControl->System_Loop.Status = SYSTEM_FAULT;
        pSpeed_Loop->Speed_Command = 0;
        return;
    }
    if(MOTOR_Run_flag == 1 && Speed_Command > 50.0f)
    {
        pSpeed_Loop->Speed_Command = Speed_Command / MOTOR_RPM_BASE; // Convert speed command to base units
        pSystem_Loop->Run_flag = 1;
    }
    else 
    {
        pSpeed_Loop->Speed_Command = 0;
        pSystem_Loop->Run_flag = 0;
    }
    return;
}

void SYSTEM_Fault_Float(Motor_Control_t *pControl)
{
    System_Loop_Float_t *pSystem_Loop = (System_Loop_Float_t *)pControl->System_Loop.pSystem_Loop;
    pSystem_Loop->Fault_cnt++;
    pSystem_Loop->Run_flag = 0;
    pControl->System_Loop.Status = SYSTEM_WAIT;
}

void SYSTEM_Wait_Float(Motor_Control_t *pControl)
{
    System_Loop_Float_t *pSystem_Loop = (System_Loop_Float_t *)pControl->System_Loop.pSystem_Loop;
    pControl->System_Loop.Status = SYSTEM_HV_STANDY;
    System_Diag_Fault_Flag = 0;
    pSystem_Loop->Run_flag = 0;
}



