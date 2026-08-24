#include "Motor_Control.h"

void Current_Loop_Switch(void)
{
    // Code to switch current task states
    if (System.system_state == SYSTEM_RUN)
    {
        switch (Current_Loop.Motor_State)
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
        Current_Loop.Motor_State = MOTOR_IDLE;
    }
    Current_Loop.Loop_count++;
}


void Speed_Run(void)
{
    Speed_Loop.spd_ctrl_timer++;
    switch (Speed_Loop.spd_ctrl_state)
    {
    case Speed_Loop_Idle:
        // Handle idle state
        SPEED_Idle_Task();
        break;
    case Speed_Loop_Align:
        // Handle align state
        SPEED_Align_Task();
        break;
    case Speed_Loop_Open:
        // Handle open state
        SPEED_Open_Task();
        break;
    case Speed_Loop_Switch:
        // Handle switch state
        SPEED_Switch_Task();
        break;
    case Speed_Loop_Low:
        // Handle low state
        SPEED_Low_Task();
        break;
    case Speed_Loop_Middle:
        // Handle middle state
        SPEED_Middle_Task();
        break;
    case Speed_Loop_High:
        // Handle high state
        SPEED_High_Task();
        break;
    default:
        Speed_Loop.spd_ctrl_state = Speed_Loop_Idle;
        break;
    }
}

void SYSTEM_Task(void)
{
    System.System_cnt++;
    switch (System.system_state)
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
