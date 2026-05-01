#include "speed_ctrl.h"
#include "arm_math.h"
#include "Current_Task.h"
#include "Motor_parameter.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Hal_Math.h"
#include "FreeRTOS.h"
#include "Observer.h"
#include "system.h"

extern uint8_t MOTOR_Run_flag;
extern float Speed_Command;
extern MOTOR_t PMSM_42JS;
extern struct NonFluxObserver_Parameter NonFlux_OB;
extern Current_Task_t Current_Task;
extern SYSTEM_t System;

float Speed_PI_Lookup_Speed_index[10] = {0, 500, 1000, 1500, 2000,
                                         2500, 3000, 3500, 4000, 5000};
// float Speed_PI_Lookup_Kp[10] = {0.00051755, 0.00051755, 0.00051755, 0.00051755, 0.00051755,
//                                 0.00051755, 0.00051755, 0.00051755, 0.00051755, 0.00051755};
// float Speed_PI_Lookup_Ki[10] = {0.000091609, 0.000091609, 0.000091609, 0.000091609, 0.000091609,
//                                 0.000091609, 0.000091609, 0.000091609, 0.000091609, 0.000091609};

float Speed_PI_Lookup_Kp[10] = {9.1755e-04, 8.1755e-04, 6.1755e-04, 6.1755e-04, 6.1755e-04,
                                5.1755e-04, 5.1755e-04, 5.1755e-04, 5.1755e-04, 5.1755e-04};
float Speed_PI_Lookup_Ki[10] = {9.1609e-05, 7.1609e-05, 6.1609e-05, 5.1609e-05, 5.1609e-05,
                                5.1609e-05, 5.1609e-05, 5.1609e-05, 5.1609e-05, 5.1609e-05};
Speed_Ctrl_t Speed_Ctrl = {
    .FREQ_Hz = 1000,
    .IF_Start_Speed_Lookup = {0},
    .IF_Start_Iq_Lookup = {0}};

uint8_t align_done = 0;

void Speed_Ctrl_Init(void)
{
    Speed_Ctrl.spd_ctrl_state = SPEED_CTRL_IDLE;
    Speed_Ctrl.spd_ctrl_timer = 0;
    Speed_Ctrl.pMotor = &PMSM_42JS;
    Speed_Ctrl.Speed_PI.kp = 5.1755e-04;
    Speed_Ctrl.Speed_PI.ki = 9.1609e-05;
    Speed_Ctrl.Speed_PI.Kd = 0.1f;
    Speed_Ctrl.Speed_PI.out_max = 8.0f;
    Speed_Ctrl.Speed_PI.out_min = -8.0f;

    /* IF阶段初始化查表*/
    Speed_Ctrl.IF_Start_Speed_Lookup.x_table = Speed_Ctrl.pMotor->IF_Start_Ramp_Sec;
    Speed_Ctrl.IF_Start_Speed_Lookup.y_table = Speed_Ctrl.pMotor->IF_Start_Speed_RPM;
    Speed_Ctrl.IF_Start_Speed_Lookup.table_size = Speed_Ctrl.pMotor->IF_Start_Profile_Length;
    Speed_Ctrl.IF_Start_Iq_Lookup.x_table = Speed_Ctrl.pMotor->IF_Start_Ramp_Sec;
    Speed_Ctrl.IF_Start_Iq_Lookup.y_table = Speed_Ctrl.pMotor->IF_Start_Iq_A;
    Speed_Ctrl.IF_Start_Iq_Lookup.table_size = Speed_Ctrl.pMotor->IF_Start_Profile_Length;
    /* 速度环PI初始化查表*/
    Speed_Ctrl.Speed_PI_Ki_Lookup.x_table = Speed_PI_Lookup_Speed_index;
    Speed_Ctrl.Speed_PI_Ki_Lookup.y_table = Speed_PI_Lookup_Ki;
    Speed_Ctrl.Speed_PI_Ki_Lookup.table_size = sizeof(Speed_PI_Lookup_Speed_index) / sizeof(float);
    Speed_Ctrl.Speed_PI_Kp_Lookup.x_table = Speed_PI_Lookup_Speed_index;
    Speed_Ctrl.Speed_PI_Kp_Lookup.y_table = Speed_PI_Lookup_Kp;
    Speed_Ctrl.Speed_PI_Kp_Lookup.table_size = sizeof(Speed_PI_Lookup_Speed_index) / sizeof(float);

    Hysteresis_Comp_Init(&Speed_Ctrl.Weak_Control_Hcomp, -0.0f, -1.0f, 50);
    Speed_Ctrl.Weak_Control_Hcomp.enable = 1;
    Speed_Ctrl.Weak_Pi.kp = 0.005f;
    Speed_Ctrl.Weak_Pi.ki = 0.10f;
    Speed_Ctrl.Weak_Pi.Kd = 0.1f;
    // Speed_Ctrl.Weak_Pi.kp = 1.01f;
    // Speed_Ctrl.Weak_Pi.ki = 1.01f;
    Speed_Ctrl.Weak_Pi.out_max = 0.0f;
    Speed_Ctrl.Weak_Pi.out_min = -5.0f;
}

void SPEED_CTRL_IDLE_Task(void);
void SPEED_CTRL_ALIGN_Task();
void SPEED_CTRL_OPEN_Task(void);
void SPEED_CTRL_SWITCH_Task(void);
void SPEED_CTRL_RUN_Task(void);
void Paramater_update(void);

uint32_t speed_cnt = 0;

void Speed_Run(void)
{
    switch (Speed_Ctrl.spd_ctrl_state)
    {
    case SPEED_CTRL_IDLE:
        // Handle idle state
        SPEED_CTRL_IDLE_Task();
        break;
    case SPEED_CTRL_ALIGN:
        // Handle align state
        SPEED_CTRL_ALIGN_Task();
        Speed_Ctrl.spd_ctrl_timer++;
        break;
    case SPEED_CTRL_OPEN:
        // Handle open state
        SPEED_CTRL_OPEN_Task();
        Speed_Ctrl.spd_ctrl_timer++;
        break;
    case SPEED_CTRL_SWITCH:
        // Handle switch state
        SPEED_CTRL_SWITCH_Task();
        Speed_Ctrl.spd_ctrl_timer++;
        break;
    case SPEED_CTRL_RUN:
        // Handle run state
        SPEED_CTRL_RUN_Task();
        Speed_Ctrl.spd_ctrl_timer++;
        break;
    default:
        Speed_Ctrl.spd_ctrl_state = SPEED_CTRL_IDLE;
        break;
    }
}

void Speed_Ctrl_Task(void)
{
    speed_cnt++;
    if (System.system_state == SYSTEM_RUN && Current_Task.Motor_State == MOTOR_RUN)
    {
        Speed_Run();
    }
    else
    {
        Speed_Ctrl.spd_ctrl_state = SPEED_CTRL_IDLE;
        Speed_Ctrl.Speed_Fb = 0;
    }
    Paramater_update();
}

void SPEED_CTRL_IDLE_Task(void)
{
    if (System.system_state == SYSTEM_RUN)
    {
        Speed_Ctrl.spd_ctrl_state = SPEED_CTRL_ALIGN;
        align_done = 0;
        Speed_Ctrl.Speed_Ref = 0;
        Speed_Ctrl.Speed_Switch_Flag = 0;
        Speed_Ctrl.Speed_PI.integral = 0;
        Speed_Ctrl.Weak_Control_Hcomp.comp_out = 0;
        Speed_Ctrl.tick_count_idle = xTaskGetTickCount();
        Speed_Ctrl.Weak_Pi.integral = 0;
        Speed_Ctrl.target_id = 0;
        Speed_Ctrl.target_iq = 0;
        Speed_Ctrl.target_is = 0;
        Speed_Ctrl.Speed_Fb = 0;
    }
}

extern float theta;

void SPEED_CTRL_ALIGN_Task()
{
    // vTaskDelay(100);
    // // Alignment logic can be implemented here if needed
    Speed_Ctrl.target_id = 0.0f;
    Speed_Ctrl.target_iq = 0;
    Speed_Ctrl.Speed_Ref = 0;
    Current_Task.theta = 0; // Align to d-axis
    vTaskDelay(100);
    Speed_Ctrl.target_id = 1.0f;
    align_done = 1;
    vTaskDelay(500);
    Speed_Ctrl.target_id = 0.0f;
    vTaskDelay(10);
    align_done = 0;
    Speed_Ctrl.target_id = 0.0f;
    vTaskDelay(500);
    // Current_Task.theta = 0.17*6;
    // vTaskDelay(5000);
    // Current_Task.theta = 0.17*12;
    // vTaskDelay(5000);
    // Current_Task.theta = 0.17*18;
    // vTaskDelay(5000);
    Speed_Ctrl.spd_ctrl_state = SPEED_CTRL_RUN;
}

void SPEED_CTRL_OPEN_Task(void)
{
    float tick_count = (xTaskGetTickCount() - Speed_Ctrl.tick_count_idle) / 1000.0f;
    Speed_Ctrl.Speed_Ref = Lookup_Table_Linear(tick_count, &Speed_Ctrl.IF_Start_Speed_Lookup);
    Speed_Ctrl.target_iq = Lookup_Table_Linear(tick_count, &Speed_Ctrl.IF_Start_Iq_Lookup);
    Speed_Ctrl.target_id = 0;
    if (Speed_Ctrl.Speed_Ref >= 600)
    {
        Speed_Ctrl.spd_ctrl_state = SPEED_CTRL_SWITCH;
    }
}

void SPEED_CTRL_SWITCH_Task(void)
{
    if (Speed_Ctrl.Speed_Switch_Flag == 0)
    {
        Speed_Ctrl.target_iq -= (SPEED_SWITCH_ID_SUB_STEP);
        if (Speed_Ctrl.target_iq < 0)
        {
            Speed_Ctrl.target_iq = 0;
        }
    }
    else
    {
        Speed_Ctrl.Speed_PI.integral = Speed_Ctrl.target_iq;
        Speed_Ctrl.spd_ctrl_state = SPEED_CTRL_RUN;
    }
}

void SPEED_CTRL_RUN_Task(void)
{
    Speed_Ctrl.Speed_Ref = Oblique_Wave(Speed_Ctrl.Speed_Command, Speed_Ctrl.Speed_Ref, SPEED_ADD_STEP, SPEED_SUB_STEP);
    Hysteresis_Comp_Process(&Speed_Ctrl.Weak_Control_Hcomp, Speed_Ctrl.Voltage_err);
    Speed_Ctrl.target_is = Hal_PI_f32(&Speed_Ctrl.Speed_PI, Speed_Ctrl.Speed_Ref - Speed_Ctrl.Speed_Fb);
    if (Speed_Ctrl.Speed_Ref < 600 && Speed_Ctrl.Speed_Ref >= -600)
    {
        Speed_Ctrl.target_id = Oblique_Wave(0.5f, Speed_Ctrl.target_id, SPEED_ID_ADD_STEP, SPEED_ID_SUB_STEP);
    }
    else
    {
        if (Speed_Ctrl.Weak_Control_Hcomp.comp_out == 1)
        {
            Speed_Ctrl.target_id = Hal_PI_f32(&Speed_Ctrl.Weak_Pi, (Speed_Ctrl.Weak_Control_Hcomp.threshold_high - Speed_Ctrl.Voltage_err));
        }
        else
        {
            Speed_Ctrl.Weak_Pi.integral = 0;
            Speed_Ctrl.target_id = Oblique_Wave(0.0f, Speed_Ctrl.target_id, SPEED_ID_ADD_STEP, SPEED_ID_SUB_STEP);
        }
    }
    arm_sqrt_f32(Current_Task.Ud_Target * Current_Task.Ud_Target + Current_Task.Uq_Target * Current_Task.Uq_Target, &Speed_Ctrl.Vs);
    Speed_Ctrl.Voltage_err = Speed_Ctrl.Vs - 24.0f * WEAK_VOLTAGE_COMPENSATION;
    arm_sqrt_f32(Speed_Ctrl.target_is * Speed_Ctrl.target_is - Speed_Ctrl.target_id * Speed_Ctrl.target_id,
                 &Speed_Ctrl.target_iq);
}

extern struct SMO_Parameter SMO_OB;
extern struct NonFluxObserver_Parameter NonFlux_OB;

void Paramater_update(void)
{
    Speed_Ctrl.Speed_PI.kp = Lookup_Table_Linear(Speed_Ctrl.Speed_Ref, &Speed_Ctrl.Speed_PI_Kp_Lookup);
    Speed_Ctrl.Speed_PI.ki = Lookup_Table_Linear(Speed_Ctrl.Speed_Ref, &Speed_Ctrl.Speed_PI_Ki_Lookup);
    NonFlux_OB.tPLL.PLL_PI.kp = Lookup_Table_Linear(Speed_Ctrl.Speed_Ref, &NonFlux_OB.PLL_Kp_Lookup);
    NonFlux_OB.tPLL.PLL_PI.ki = Lookup_Table_Linear(Speed_Ctrl.Speed_Ref, &NonFlux_OB.PLL_Ki_Lookup);
    // SMO_OB.E_LPF_Coff = Lookup_Table_Linear(Speed_Ctrl.Speed_Fb, &SMO_OB.LPF);
    // SMO_OB.Gain_Add = Lookup_Table_Linear(Speed_Ctrl.Speed_Fb, &SMO_OB.GAIN_LOOKUP);
}