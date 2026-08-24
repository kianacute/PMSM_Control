#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__ 

#include "Motor_FOC_Config.h"


enum Motor_State{
    MOTOR_IDLE = 0,
    MOTOR_READY,
    MOTOR_OFFSET_CHECK,
    MOTOR_RS_IDENTIFY,      
    MOTOR_RUN,
    MOTOR_FAULT,
    MOTOR_WAIT,
};

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


enum SYSTEM_State_t
{
    SYSTEM_LV_STANDY = 0,
    SYSTEM_HV_STANDY,
    SYSTEM_RUN,
    SYSTEM_FAULT,
    SYSTEM_WAIT,
};



    #ifdef MOTOR_CONTROL_FLOAT

        #include "Current_Loop_FLoat.h"
        #include "Speed_Loop_Float.h"
        #include "System_Loop_Float.h"


        typedef struct Current_Loop_Input
        {
            float Ia_fb_raw, Ib_fb_raw, Ic_fb_raw;
            float Udc_ADISR;
        } Current_Loop_Input_t;

        typedef struct Current_Loop_Output
        {
            float PWM_HZ_Coeff;
            float PWM_duty_a, PWM_duty_b, PWM_duty_c, PWM_duty_d;
        } Current_Loop_Output_t;

        /*电机状态机*/
        #define         MOTOR_IDLE_TASK               MOTOR_IDLE_TASK_FLoat                  
        #define         MOTOR_READY_TASK              MOTOR_READY_TASK_FLoat                  
        #define         MOTOR_OFFSET_CHECK_TASK       MOTOR_OFFSET_CHECK_TASK_FLoat                  
        #define         MOTOR_RUN_TASK                MOTOR_RUN_TASK_FLoat                  
        #define         MOTOR_FAULT_TASK              MOTOR_FAULT_TASK_FLoat                  
        #define         MOTOR_WAIT_TASK               MOTOR_WAIT_TASK_FLoat                  

        /*速度状态机*/

        #define         SPEED_Idle_Task          SPEED_Idle_Task_FLoat
        #define         SPEED_Align_Task         SPEED_Align_Task_FLoat
        #define         SPEED_Open_Task          SPEED_Open_Task_FLoat
        #define         SPEED_Switch_Task        SPEED_Switch_Task_FLoat
        #define         SPEED_Low_Task           SPEED_Low_Task_FLoat
        #define         SPEED_Middle_Task        SPEED_Middle_Task_FLoat
        #define         SPEED_High_Task          SPEED_High_Task_FLoat
        #define         SPEED_Run_Task           SPEED_Run_Task_FLoat

        /*系统状态机*/

        #define         SYSTEM_Init              SYSTEM_Init_FLoat
        #define         SYSTEM_LV_Standy         SYSTEM_LV_Standy_FLoat
        #define         SYSTEM_HV_Standy         SYSTEM_HV_Standy_FLoat
        #define         SYSTEM_Run               SYSTEM_Run_FLoat
        #define         SYSTEM_Fault             SYSTEM_Fault_FLoat 
        #define         SYSTEM_Wait              SYSTEM_Wait_FLoat

    #elif defined MOTOR_CONTROL_FIXED

        #include "Current_Loop_Fixed.h"
        #include "Speed_Loop_Fixed.h"
        #include "System_Loop_Fixed.h"

        /*电机状态机*/
        #define         MOTOR_IDLE_TASK               MOTOR_IDLE_TASK_Fixed                  
        #define         MOTOR_READY_TASK              MOTOR_READY_TASK_Fixed                  
        #define         MOTOR_OFFSET_CHECK_TASK       MOTOR_OFFSET_CHECK_TASK_Fixed                  
        #define         MOTOR_RUN_TASK                MOTOR_RUN_TASK_Fixed                  
        #define         MOTOR_FAULT_TASK              MOTOR_FAULT_TASK_Fixed                  
        #define         MOTOR_WAIT_TASK               MOTOR_WAIT_TASK_Fixed                  

        /*速度状态机*/

        #define         SPEED_Idle_Task          SPEED_Idle_Task_Fixed
        #define         SPEED_Align_Task         SPEED_Align_Task_Fixed
        #define         SPEED_Open_Task          SPEED_Open_Task_Fixed
        #define         SPEED_Switch_Task        SPEED_Switch_Task_Fixed
        #define         SPEED_Low_Task           SPEED_Low_Task_Fixed
        #define         SPEED_Middle_Task        SPEED_Middle_Task_Fixed
        #define         SPEED_High_Task          SPEED_High_Task_Fixed
        #define         SPEED_Run_Task           SPEED_Run_Task_Fixed

        /*系统状态机*/

        #define         SYSTEM_Init              SYSTEM_Init_Fixed
        #define         SYSTEM_LV_Standy         SYSTEM_LV_Standy_Fixed
        #define         SYSTEM_HV_Standy         SYSTEM_HV_Standy_Fixed
        #define         SYSTEM_Run               SYSTEM_Run_Fixed
        #define         SYSTEM_Fault             SYSTEM_Fault_Fixed 
        #define         SYSTEM_Wait              SYSTEM_Wait_Fixed

    #endif

#endif