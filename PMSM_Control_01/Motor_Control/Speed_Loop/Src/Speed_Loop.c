#include "Speed_Loop.h"
#include "arm_math.h"
#include "Current_Loop.h"
#include "Motor_Config.h"
#include "Hal_Math.h"
#include "Observer.h"
#include "System_Loop.h"

extern uint8_t MOTOR_Run_flag;
extern float Speed_Command;
extern Motor_Config_t PMSM_42JS_Config;
extern struct NonFluxObserver_Parameter NonFlux_OB;
extern Current_Loop_t Current_Loop;;
extern SYSTEM_t System;

Speed_Loop_t Speed_Loop;

uint8_t align_done = 0;

void Speed_Loop_Init(void)
{
    Speed_Loop.spd_ctrl_state = Speed_Loop_IDLE;
    Speed_Loop.spd_ctrl_timer = 0;
    Speed_Loop.pMotor = &PMSM_42JS_Config;
    Speed_Loop.Speed_PI.kp = 5.1755e-04;
    Speed_Loop.Speed_PI.ki = 9.1609e-05;
    Speed_Loop.Speed_PI.Kd = 0.1f;
    Speed_Loop.Speed_PI.out_max = 8.0f;
    Speed_Loop.Speed_PI.out_min = -8.0f;
    Speed_Loop.FREQ_Hz = 1000;
    Speed_Loop.Speed_Switch_Cnt = 0;

    Hysteresis_Comp_Init(&Speed_Loop.Weak_Control_Hcomp, -0.0f, -1.0f, 50);
    Speed_Loop.Weak_Control_Hcomp.enable = 1;
    Speed_Loop.Weak_Pi.kp = 0.01f;
    Speed_Loop.Weak_Pi.ki = 0.1f;
    Speed_Loop.Weak_Pi.Kd = 0.1f;
    // Speed_Loop.Weak_Pi.kp = 1.01f;
    // Speed_Loop.Weak_Pi.ki = 1.01f;
    Speed_Loop.Weak_Pi.out_max = 0.0f;
    Speed_Loop.Weak_Pi.out_min = -8.0f;
}

void Speed_Loop_IDLE_Task(void);
void Speed_Loop_ALIGN_Task();
void Speed_Loop_OPEN_Task(void);
void Speed_Loop_SWITCH_Task(void);
void Speed_Loop_RUN_Task(void);
void Paramater_update(void);

uint32_t speed_cnt = 0;

void Speed_Run(void)
{
    switch (Speed_Loop.spd_ctrl_state)
    {
    case Speed_Loop_IDLE:
        // Handle idle state
        Speed_Loop_IDLE_Task();
        break;
    case Speed_Loop_ALIGN:
        // Handle align state
        Speed_Loop_ALIGN_Task();
        Speed_Loop.spd_ctrl_timer++;
        break;
    case Speed_Loop_OPEN:
        // Handle open state
        Speed_Loop_OPEN_Task();
        Speed_Loop.spd_ctrl_timer++;
        break;
    case Speed_Loop_SWITCH:
        // Handle switch state
        Speed_Loop_SWITCH_Task();
        Speed_Loop.spd_ctrl_timer++;
        break;
    case Speed_Loop_RUN:
        // Handle run state
        Speed_Loop_RUN_Task();
        Speed_Loop.spd_ctrl_timer++;
        break;
    default:
        Speed_Loop.spd_ctrl_state = Speed_Loop_IDLE;
        break;
    }
}

void Speed_Loop_Task(void)
{
    speed_cnt++;
    if (System.system_state == SYSTEM_RUN && Current_Loop.Motor_State == MOTOR_RUN)
    {
        Speed_Run();
    }
    else
    {
        Speed_Loop.spd_ctrl_state = Speed_Loop_IDLE;
        Speed_Loop.Speed_Fb = 0;
    }
    Paramater_update();
}

void Speed_Loop_IDLE_Task(void)
{
    if (System.system_state == SYSTEM_RUN)
    {
        Speed_Loop.spd_ctrl_state = Speed_Loop_ALIGN;
        align_done = 0;
        Speed_Loop.Speed_Ref = 0;
        Speed_Loop.Speed_Switch_Flag = 0;
        Speed_Loop.Speed_PI.integral = 0;
        Speed_Loop.Weak_Control_Hcomp.comp_out = 0;
        Speed_Loop.tick_count_idle = xTaskGetTickCount();
        Speed_Loop.Weak_Pi.integral = 0;
        Speed_Loop.target_id = 0;
        Speed_Loop.target_iq = 0;
        Speed_Loop.target_is = 0;
        Speed_Loop.Speed_Fb = 0;
    }
}

extern float theta;

void Speed_Loop_ALIGN_Task()
{
    // vTaskDelay(100);
    // // Alignment logic can be implemented here if needed
    Speed_Loop.target_id = 0.5f;
    Speed_Loop.target_iq = 0;
    Speed_Loop.Speed_Ref = 0;
    Current_Loop.theta = 0; // Align to d-axis
    // vTaskDelay(100);
    // Speed_Loop.target_id = 1.0f;
    // align_done = 1;
    // vTaskDelay(500);
    // Speed_Loop.target_id = 0.0f;
    // vTaskDelay(10);
    // align_done = 0;
    // Speed_Loop.target_id = 0.0f;
    // vTaskDelay(500);
    // Current_Loop.theta = 0.17*6;
    // vTaskDelay(5000);
    // Current_Loop.theta = 0.17*12;
    // vTaskDelay(5000);
    // Current_Loop.theta = 0.17*18;
    // vTaskDelay(5000);
    Speed_Loop.spd_ctrl_state = Speed_Loop_RUN;
}

void Speed_Loop_OPEN_Task(void)
{
    float tick_count = (xTaskGetTickCount() - Speed_Loop.tick_count_idle) / 1000.0f;
    Speed_Loop.Speed_Ref = Lookup_Table_Linear(tick_count, &PMSM_42JS_Config.IF_Start_Speed_Lookup);
    Speed_Loop.target_iq = Lookup_Table_Linear(tick_count, &PMSM_42JS_Config.IF_Start_Iq_Lookup);
    Speed_Loop.target_id = 0;
    if (Speed_Loop.Speed_Ref >= 600)
    {
        Speed_Loop.spd_ctrl_state = Speed_Loop_SWITCH;
    }
}

void Speed_Loop_SWITCH_Task(void)
{
    if (Speed_Loop.Speed_Switch_Flag == 0)
    {
        Speed_Loop.target_iq -= (SPEED_SWITCH_ID_SUB_STEP);
        if (Speed_Loop.target_iq < 0)
        {
            Speed_Loop.target_iq = 0;
        }
    }
    else
    {
        Speed_Loop.Speed_PI.integral = Speed_Loop.target_iq;
        Speed_Loop.spd_ctrl_state = Speed_Loop_RUN;
    }
}

void Speed_Loop_RUN_Task(void)
{
    Speed_Loop.Speed_Ref = Oblique_Wave(Speed_Loop.Speed_Command, Speed_Loop.Speed_Ref, SPEED_ADD_STEP, SPEED_SUB_STEP);
    Hysteresis_Comp_Process_Add(&Speed_Loop.Weak_Control_Hcomp, Speed_Loop.Voltage_err);
    Speed_Loop.target_is = Hal_PI_f32(&Speed_Loop.Speed_PI, Speed_Loop.Speed_Ref - Speed_Loop.Speed_Fb);
    if (Speed_Loop.Speed_Ref < 1000 && Speed_Loop.Speed_Ref >= -600)
    {
        Speed_Loop.target_id = Oblique_Wave(0.5f, Speed_Loop.target_id, SPEED_ID_ADD_STEP, SPEED_ID_SUB_STEP);
    }
    else
    {
        if (Speed_Loop.Weak_Control_Hcomp.comp_out == 1)
        {
            Speed_Loop.target_id = Hal_PI_f32(&Speed_Loop.Weak_Pi, (Speed_Loop.Weak_Control_Hcomp.threshold_high - Speed_Loop.Voltage_err));
        }
        else
        {
            Speed_Loop.Weak_Pi.integral = 0;
            Speed_Loop.target_id = Oblique_Wave(0.0f, Speed_Loop.target_id, SPEED_ID_ADD_STEP, SPEED_ID_SUB_STEP);
        }
    }
    arm_sqrt_f32(Current_Loop.Ud_Target * Current_Loop.Ud_Target + Current_Loop.Uq_Target * Current_Loop.Uq_Target, &Speed_Loop.Vs);
    Speed_Loop.Voltage_err = Speed_Loop.Vs - 24.0f * WEAK_VOLTAGE_COMPENSATION;
    arm_sqrt_f32(Speed_Loop.target_is * Speed_Loop.target_is - Speed_Loop.target_id * Speed_Loop.target_id,
                 &Speed_Loop.target_iq);
}

extern struct SMO_Parameter SMO_OB;
extern struct NonFluxObserver_Parameter NonFlux_OB;

void Paramater_update(void)
{
    Speed_Loop.Speed_PI.kp = Lookup_Table_Linear(Speed_Loop.Speed_Ref, &PMSM_42JS_Config.Speed_PI_Kp_Lookup);
    Speed_Loop.Speed_PI.ki = Lookup_Table_Linear(Speed_Loop.Speed_Ref, &PMSM_42JS_Config.Speed_PI_Ki_Lookup);
    
    NonFlux_OB.tPLL.PLL_PI.kp = Lookup_Table_Linear(Speed_Loop.Speed_Ref, &PMSM_42JS_Config.NonFlux_PLL_Kp_Lookup);
    NonFlux_OB.tPLL.PLL_PI.ki = Lookup_Table_Linear(Speed_Loop.Speed_Ref, &PMSM_42JS_Config.NonFlux_PLL_Ki_Lookup);
    NonFlux_OB.gama = Lookup_Table_Linear(Speed_Loop.Speed_Ref, &PMSM_42JS_Config.NonFlux_Gama_Lookup);

    SMO_OB.Gain = Lookup_Table_Linear(Speed_Loop.Speed_Fb, &PMSM_42JS_Config.SMO_Gain_Lookup);
    SMO_OB.tPLL.PLL_PI.kp = Lookup_Table_Linear(Speed_Loop.Speed_Ref, &PMSM_42JS_Config.SMO_PLL_Kp_Lookup);
    SMO_OB.tPLL.PLL_PI.ki = Lookup_Table_Linear(Speed_Loop.Speed_Ref, &PMSM_42JS_Config.SMO_PLL_Ki_Lookup);

    Current_Loop.Id_PI.kp = Lookup_Table_Linear(Speed_Loop.Speed_Ref, &PMSM_42JS_Config.ID_PI_Kp_Lookup);
    Current_Loop.Id_PI.ki = Lookup_Table_Linear(Speed_Loop.Speed_Ref, &PMSM_42JS_Config.ID_PI_Ki_Lookup);
    Current_Loop.Iq_PI.kp = Lookup_Table_Linear(Speed_Loop.Speed_Ref, &PMSM_42JS_Config.IQ_PI_Kp_Lookup);
    Current_Loop.Iq_PI.ki = Lookup_Table_Linear(Speed_Loop.Speed_Ref, &PMSM_42JS_Config.IQ_PI_Ki_Lookup);
}