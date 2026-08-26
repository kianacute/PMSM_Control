#include "Motor_Control.h"

void Current_Loop_Switch(Motor_Control_t *pControl)
{
    // Code to switch current task states
    if (pControl->System.Status == SYSTEM_RUN)
    {
        switch (pControl->Current_Loop.Status)
        {
        case MOTOR_IDLE:
            // Handle idle state
            MOTOR_IDLE_TASK();
            break;
        case MOTOR_READY:
            // Handle ready state
            MOTOR_READY_TASK();
            break;
        case MOTOR_OFFSET_CHECK:
            // Handle offset check state
            MOTOR_OFFSET_CHECK_TASK();
            break;
        case MOTOR_RUN:
            // Handle run state
            MOTOR_RUN_TASK();
            break;
        case MOTOR_FAULT:
            // Handle fault state
            // Add fault handling code here
            MOTOR_FAULT_TASK();
            break;
        case MOTOR_WAIT:
            // Handle wait state
            MOTOR_WAIT_TASK();
            break;
        default:
            break;
        }
    }
    else
    {
        MOTOR_IDLE_TASK();
        pControl->Current_Loop.Status = MOTOR_IDLE;
    }
    pControl->Current_Loop.Loop_count++;
}

void Speed_Loop_Task(Motor_Control_t *pControl)
{
    if (pControl->System.Status == SYSTEM_RUN && pControl->Current_Loop.Status == MOTOR_RUN)
    {
        pControl->Speed_Loop.Loop_count++;
        switch (pControl->Speed_Loop.Status)
        {
        case SPEED_IDLE:
            // Handle idle state
            SPEED_Idle_Task();
            break;
        case SPEED_ALIGN:
            // Handle align state
            SPEED_Align_Task();
            break;
        case SPEED_OPEN:
            // Handle open state
            SPEED_Open_Task();
            break;
        case SPEED_SWITCH:
            // Handle switch state
            SPEED_Switch_Task();
            break;
        case SPEED_LOW:
            // Handle low state
            SPEED_Low_Task();
            break;
        case SPEED_MIDDLE:
            // Handle middle state
            SPEED_Middle_Task();
            break;
        case SPEED_HIGH:
            // Handle high state
            SPEED_High_Task();
            break;
        default:
            pControl->Speed_Loop.Status = SPEED_IDLE;
            break;
        }
    }
    else
    {
        pControl->Speed_Loop.Status = SPEED_IDLE;
    }
    
}

void SYSTEM_Task(Motor_Control_t *pControl)
{
    pControl->System.Loop_count++;
    switch (pControl->System.Status)
    {
    case SYSTEM_LV_STANDY:
        SYSTEM_LV_Standy();
        break;
    case SYSTEM_HV_STANDY:
        SYSTEM_HV_Standy();
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
