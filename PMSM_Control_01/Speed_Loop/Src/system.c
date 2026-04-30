#include "system.h"
#include "arm_math.h"
#include "Current_Task.h"
#include "Motor_parameter.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Hal_Math.h"
#include "FreeRTOS.h"
#include "Observer.h"
#include "speed_ctrl.h"

extern uint8_t MOTOR_Run_flag;
extern float Speed_Command;
extern MOTOR_t PMSM_42JS;
extern struct NonFluxObserver_Parameter NonFlux_OB;
extern Current_Task_t Current_Task;
extern Speed_Ctrl_t Speed_Ctrl;
uint8_t System_Fault_Flag = 0;

SYSTEM_t System = {
    .FREQ_Hz = SYSTEM_HZ,
};

void SYSTEM_Init(void)
{
    // Initialization code for the system
    // e.g., setting up peripherals, initializing variables, etc.
    Current_Task_Init();
    Speed_Ctrl_Init();
}

void SYSTEM_LV_Standy()
{
    vTaskDelay(SYSTEM_LV_INIT_TIME);
    System.system_state = SYSTEM_HV_STANDY;
}

void SYSTEM_HV_Standy()
{
    if(Current_Task.Udc_ADISR > 20.0f) // Check if the DC bus voltage is above a certain threshold
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

void SYSTEM_Run()
{
    if(MOTOR_Run_flag == 1)
    {
        Speed_Ctrl.Speed_Command = Speed_Command; 
    }
    else 
    {
        Speed_Ctrl.Speed_Command = 0;
        if(Speed_Ctrl.Speed_Ref < 50.0f)
        {
            System.system_state = SYSTEM_WAIT;
        }
    }
    return;
}

void SYSTEM_Fault()
{
    Speed_Command = 0.0f; // Stop the motor
    vTaskDelay(SYSTEM_WAIT_TIME);
    System.system_state = SYSTEM_CMD_STANDY;
}

void SYSTEM_Wait()
{
    vTaskDelay(SYSTEM_WAIT_TIME);
    System.system_state = SYSTEM_CMD_STANDY;
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
