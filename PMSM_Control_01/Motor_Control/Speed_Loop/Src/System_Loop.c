#include "System_Loop.h"
#include "arm_math.h"
#include "Current_Loop.h"
#include "Motor_parameter.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Hal_Math.h"
#include "FreeRTOS.h"
#include "Observer.h"
#include "Speed_Loop.h"
#include "System_Diag.h"
#include "Motor_Diag.h"

extern uint8_t MOTOR_Run_flag;
extern float Speed_Command;
extern MOTOR_t PMSM_42JS;
extern struct NonFluxObserver_Parameter NonFlux_OB;
extern Current_Loop_t Current_Loop;
extern Speed_Loop_t Speed_Loop;
extern Current_Loop_Input_t Current_Loop_Input;
extern Current_Loop_Output_t Current_Loop_Output;

uint8_t System_Fault_Flag = 0;

SYSTEM_t System = {
    .FREQ_Hz = SYSTEM_HZ,
};

void SYSTEM_Init(void)
{
    // Initialization code for the system
    // e.g., setting up peripherals, initializing variables, etc.
    Current_Loop_Init();
    Speed_Loop_Init();
    Motor_Diag_Init();
    System_Diag_Init();
    Speed_Command = 1000.0f;
}

void SYSTEM_LV_Standy()
{
    vTaskDelay(SYSTEM_LV_INIT_TIME);
    System.system_state = SYSTEM_HV_STANDY;
}

void SYSTEM_HV_Standy()
{
    if(Current_Loop_Input.Udc_ADISR > 20.0f) // Check if the DC bus voltage is above a certain threshold
    {
        vTaskDelay(SYSTEM_HV_STANDY_TIME);
        System.system_state = SYSTEM_CMD_STANDY;
    }
}

void SYSTEM_CMD_Standy()
{
    if (MOTOR_Run_flag == 1 && Speed_Command > 50.0f) // Check if the motor run command is received and speed command is valid
    {
        System.system_state = SYSTEM_RUN;
    }
}

extern uint8_t System_Diag_Fault_Flag;
extern uint8_t Motor_Diag_Fault_Flag;

void SYSTEM_Run()
{
    System_Fault_Flag = System_Diag_Fault_Flag | Motor_Diag_Fault_Flag; // Combine system and motor diagnostic fault flags
    if(System_Fault_Flag != 0)
    {
        System.system_state = SYSTEM_FAULT;
        Speed_Loop.Speed_Command = 0;
        return;
    }
    if(MOTOR_Run_flag == 1)
    {
        Speed_Loop.Speed_Command = Speed_Command; 
    }
    else 
    {
        Speed_Loop.Speed_Command = 0;
        if(Speed_Loop.Speed_Ref < 50.0f)
        {
            System.system_state = SYSTEM_WAIT;
        }
    }
    return;
}

void SYSTEM_Fault()
{
    System.Fault_cnt++;
    vTaskDelay(SYSTEM_WAIT_TIME);
    System.system_state = SYSTEM_WAIT;
}

void SYSTEM_Wait()
{
    vTaskDelay(SYSTEM_WAIT_TIME);
    System.system_state = SYSTEM_CMD_STANDY;
    System_Diag_Fault_Flag = 0;
    Motor_Diag_Fault_Flag = 0;
}

void SYSTEM_Task(void)
{
    switch (System.system_state)
    {
    case SYSTEM_LV_STANDY:
        SYSTEM_LV_Standy();
        break;
    case SYSTEM_HV_STANDY:
        SYSTEM_HV_Standy();
        break;
    case SYSTEM_CMD_STANDY:
        SYSTEM_CMD_Standy();
        break;
    case SYSTEM_RUN:
        SYSTEM_Run();
        break;
    case SYSTEM_FAULT:
        SYSTEM_Fault();
        break;
    case SYSTEM_WAIT:
        SYSTEM_Wait();
        break;
    default:
        break;
    }
}



