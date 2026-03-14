#include "speed_ctrl.h"
#include "arm_math.h"
#include "Current_Task.h"
#include "Motor_parameter.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Hal_Math.h"
#include "FreeRTOS.h"

enum SpeedCtrlState_t spd_ctrl_state;
uint32_t spd_ctrl_timer = 0;

extern uint8_t MOTOR_Run_flag;

extern MOTOR_t PMSM_42JS;
uint32_t tick_count_idle;

float target_iq, target_id, target_is;

float Speed_Ref, Speed_Fb;

extern float Speed_Command;

float Speed_Sub_Step, Speed_Add_Step;

static MOTOR_t *pMotor;

Hal_PI_t Speed_PI;

void Speed_Ctrl_Init(void)
{
    spd_ctrl_state = SPEED_CTRL_IDLE;
    spd_ctrl_timer = 0;
    pMotor = &PMSM_42JS;
    Speed_PI.kp = 5.1755e-04;
    Speed_PI.ki = 9.1609e-05;
    Speed_PI.out_max = 5.0f;
    Speed_PI.out_min = -5.0f;
    Speed_Command = 2500.0f;
}

void SPEED_CTRL_IDLE_Task(void);
void SPEED_CTRL_ALIGN_Task();
void SPEED_CTRL_OPEN_Task(void);
void SPEED_CTRL_SWITCH_Task(void);
void SPEED_CTRL_RUN_Task(void);
void SPEED_CTRL_WAIT_Task(void);

uint32_t speed_cnt = 0;

void Speed_Ctrl_Task(void)
{
    speed_cnt++;
    switch (spd_ctrl_state)
    {
    case SPEED_CTRL_ALIGN:
        // Handle align state
        SPEED_CTRL_ALIGN_Task();
        spd_ctrl_timer++;
        break;
    case SPEED_CTRL_IDLE:
        // Handle idle state
        SPEED_CTRL_IDLE_Task();
        break;
    case SPEED_CTRL_OPEN:
        // Handle open state
        SPEED_CTRL_OPEN_Task();
        spd_ctrl_timer++;
        break;
    case SPEED_CTRL_SWITCH:
        // Handle switch state
        SPEED_CTRL_SWITCH_Task();
        spd_ctrl_timer++;
        break;
    case SPEED_CTRL_RUN:
        // Handle run state
        SPEED_CTRL_RUN_Task();
        spd_ctrl_timer++;
        break;
    case SPEED_CTRL_WAIT:
        // Handle brake state
        SPEED_CTRL_WAIT_Task();
        spd_ctrl_timer++;
        break;
    default:
        spd_ctrl_state = SPEED_CTRL_IDLE;
        break;
    }
}

uint8_t align_done = 0;

void SPEED_CTRL_IDLE_Task(void)
{
    if (MOTOR_Run_flag == 1)
    {
        spd_ctrl_state = SPEED_CTRL_ALIGN;
        tick_count_idle = xTaskGetTickCount();
        Speed_Ref = 0;
        align_done = 0;
    }
    else
    {
        spd_ctrl_state = SPEED_CTRL_IDLE;
    }
}

extern float theta;

void SPEED_CTRL_ALIGN_Task()
{
    // Alignment logic can be implemented here if needed
    target_id = 1.0f;
    target_iq = 0;
    Speed_Ref = 0;
    // theta = 0; // Align to d-axis
    vTaskDelay(100);
    align_done = 1;
    vTaskDelay(100);
    target_id = 0.0f;
    vTaskDelay(100);
    // vTaskDelay(1000);
    // theta = 0.17*6;
    // vTaskDelay(5000);
    // theta = 0.17*12;
    // vTaskDelay(5000);
    // theta = 0.17*18;
    // vTaskDelay(5000);
    spd_ctrl_state = SPEED_CTRL_OPEN;
}

void SPEED_CTRL_OPEN_Task(void)
{
    float tick_count = (xTaskGetTickCount() - tick_count_idle) / 1000.0f;
    Speed_Ref = Lookup_Table_Linear(tick_count, pMotor->IF_Start_Ramp_Sec,
                                    pMotor->IF_Start_Speed_RPM, pMotor->IF_Start_Profile_Length);
    target_iq = Lookup_Table_Linear(tick_count, pMotor->IF_Start_Ramp_Sec, 
                                    pMotor->IF_Start_Iq_A, pMotor->IF_Start_Profile_Length);
    target_id = 0;
    if(Speed_Ref >= 600)
    {
        spd_ctrl_state = SPEED_CTRL_SWITCH;
    }
}

extern uint8_t Speed_Switch_Flag;

void SPEED_CTRL_SWITCH_Task(void)
{
    if (Speed_Switch_Flag == 0)
    {
        target_iq -= (SPEED_SWITCH_ID_SUB_STEP);
        if (target_iq < 0)
        {
            target_iq = 0;
        }
    }
    else
    {
        Speed_PI.integral = target_iq;
        spd_ctrl_state = SPEED_CTRL_RUN;
    }
}


void SPEED_CTRL_RUN_Task(void)
{
    if (MOTOR_Run_flag == 1)
    {
        Speed_Ref = Oblique_Wave(Speed_Command, Speed_Ref, SPEED_ADD_STEP, SPEED_SUB_STEP);
        target_iq = Hal_PI_f32(&Speed_PI, Speed_Ref - Speed_Fb);
        if(Speed_Ref <= 600 && Speed_Ref >= -600)
        {
            target_id = Oblique_Wave(0.3f, target_id, SPEED_ID_ADD_STEP, SPEED_ID_SUB_STEP);
        }
        else
        {
            target_id = Oblique_Wave(0.0f, target_id, SPEED_ID_ADD_STEP, SPEED_ID_SUB_STEP);
        }
    }
    else if (MOTOR_Run_flag == 0)
    {
        Speed_Ref = Oblique_Wave(0.0f, Speed_Ref, SPEED_ADD_STEP, SPEED_SUB_STEP);
        target_iq = Hal_PI_f32(&Speed_PI, Speed_Ref - Speed_Fb);
        target_id = Oblique_Wave(0.0f, target_id, SPEED_ID_ADD_STEP, SPEED_ID_SUB_STEP);
        if (Speed_Ref < 600)
        {
            spd_ctrl_state = SPEED_CTRL_WAIT;
        }
    }
}

void SPEED_CTRL_WAIT_Task(void)
{
    vTaskDelay(2000);
    spd_ctrl_state = SPEED_CTRL_IDLE;
}
