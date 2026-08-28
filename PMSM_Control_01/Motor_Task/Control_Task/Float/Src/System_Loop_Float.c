#include "System_Loop_Float.h"
#include "arm_math.h"
#include "Current_Loop_Float.h"
#include "Motor_Config_Float.h"
#include "Hal_Math_Float.h"
#include "Observer_Float.h"
#include "Speed_Loop_Float.h"
#include "System_Diag.h"
#include "Motor_Diag.h"

extern uint8_t MOTOR_Run_flag;
extern float Speed_Command;

uint8_t System_Fault_Flag = 0;

System_Loop_FLoat_t System_Loop_FLoat;

void SYSTEM_Init_Float(Motor_Control_t *pControl)
{
    pControl->System_Loop.pSystem_Loop = (void*)&System_Loop_FLoat;
    Motor_Diag_Init();
    System_Diag_Init();
    Speed_Command = 1000.0f;
    System_Loop_FLoat.Run_flag = 0;
}

void SYSTEM_LV_Standy_Float(Motor_Control_t *pControl)
{
    System_Loop_FLoat_t *pSystem_Loop = (System_Loop_FLoat_t *)pControl->System_Loop.pSystem_Loop;
    vTaskDelay(SYSTEM_LV_INIT_TIME);
    pSystem_Loop->system_state = SYSTEM_HV_STANDY;
    pSystem_Loop->Run_flag = 0;
}

void SYSTEM_HV_Standy_Float(Motor_Control_t *pControl)
{
    System_Loop_FLoat_t *pSystem_Loop = (System_Loop_FLoat_t *)pControl->System_Loop.pSystem_Loop;
    Motor_Control_Input_t *pMotor_Control_Input = (Motor_Control_Input_t *)&pControl->Input;
    if(pMotor_Control_Input->Udc_ADISR > 20.0f) // Check if the DC bus voltage is above a certain threshold
    {
        vTaskDelay(SYSTEM_HV_STANDY_TIME);
        pSystem_Loop->system_state = SYSTEM_RUN;
        pSystem_Loop->Run_flag = 0;
    }
}


extern uint8_t System_Diag_Fault_Flag;

void SYSTEM_Run_Float(Motor_Control_t *pControl)
{
    System_Loop_FLoat_t *pSystem_Loop = (System_Loop_FLoat_t *)pControl->System_Loop.pSystem_Loop;
    Speed_Loop_Float_t *pSpeed_Loop = (Speed_Loop_Float_t *)pControl->Speed_Loop.pSpeed_Loop;
    System_Fault_Flag = System_Diag_Fault_Flag; // Combine system and motor diagnostic fault flags
    if(System_Fault_Flag != 0)
    {
        pSystem_Loop->system_state = SYSTEM_FAULT;
        pSpeed_Loop->Speed_Command = 0;
        return;
    }
    if(MOTOR_Run_flag == 1 && Speed_Command > 50.0f)
    {
        pSpeed_Loop->Speed_Command = Speed_Command; 
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
    System_Loop_FLoat_t *pSystem_Loop = (System_Loop_FLoat_t *)pControl->System_Loop.pSystem_Loop;
    pSystem_Loop->Fault_cnt++;
    pSystem_Loop->Run_flag = 0;
    vTaskDelay(SYSTEM_WAIT_TIME);
    pSystem_Loop->system_state = SYSTEM_WAIT;
}

void SYSTEM_Wait_Float(Motor_Control_t *pControl)
{
    vTaskDelay(SYSTEM_WAIT_TIME);
    System_Loop_FLoat_t *pSystem_Loop = (System_Loop_FLoat_t *)pControl->System_Loop.pSystem_Loop;
    pSystem_Loop->system_state = SYSTEM_HV_STANDY;
    System_Diag_Fault_Flag = 0;
    pSystem_Loop->Run_flag = 0;
}



