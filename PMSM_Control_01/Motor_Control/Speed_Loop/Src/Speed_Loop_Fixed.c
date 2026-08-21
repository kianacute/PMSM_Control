#include "Speed_Loop_Fixed.h"
#include "arm_math.h"
#include "Current_Loop_Fixed.h"
#include "Motor_Config_Fixed.h"
#include "Hal_Math.h"
#include "Observer.h"
#include "System_Loop.h"
#include "Motor_Config.h"

extern uint8_t MOTOR_Run_flag;
extern float Speed_Command;
extern Motor_Config_Fixed_t PMSM_42JS_Config;
extern struct NonFluxObserver_Parameter NonFlux_OB;
extern Current_Loop_Fixed_t Current_Loop;
extern SYSTEM_t System;

Speed_Loop_Fixed_t Speed_Loop_Fixed;

void Speed_Run(void);
void Speed_Loop_Fixed_Idle_Task(void);
void Speed_Loop_Fixed_Align_Task();
void Speed_Loop_Fixed_Open_Task(void);
void Speed_Loop_Fixed_Switch_Task(void);
void Speed_Loop_Fixed_Low_Task(void);
void Speed_Loop_Fixed_Middle_Task(void);
void Speed_Loop_Fixed_High_Task(void);
void Speed_Loop_Fixed_Run_Task(void);
void Paramater_update(void);
void Power_Derating_Init(void);
void Power_Derating(float Bus_Current, float Bus_Voltage, float Power_Limit);
void MTPA_Cal(float Is);
void Speed_Input_LPF(void);

void Speed_Loop_Fixed_Init(void)
{
    Speed_Loop_Fixed.spd_ctrl_state = Speed_Loop_Idle;
    Speed_Loop_Fixed.spd_ctrl_timer = 0;
    Speed_Loop_Fixed.Speed_Switch_Cnt = 0;
    Speed_Loop_Fixed.pMotor = &PMSM_42JS_Config;
    Speed_Loop_Fixed.FREQ_Hz = 1000;
    Speed_Loop_Fixed.Speed_Fb_1s = 0;

    /*速度环参数初始化*/
    Speed_Loop_Fixed.Speed_PI.Kd = 0.1f;
    Speed_Loop_Fixed.Speed_PI.out_max = 8.0f;
    Speed_Loop_Fixed.Speed_PI.out_min = -8.0f;

    /*弱磁环参数初始化*/
    Hysteresis_Comp_Init(&Speed_Loop_Fixed.Weak_Control_Hcomp, -0.0f, -1.0f, 50);
    Speed_Loop_Fixed.Weak_Control_Hcomp.enable = 1;
    Speed_Loop_Fixed.Weak_Pi.kp = 0.01f;
    Speed_Loop_Fixed.Weak_Pi.ki = 0.1f;
    Speed_Loop_Fixed.Weak_Pi.Kd = 0.1f;
    // Speed_Loop_Fixed.Weak_Pi.kp = 1.01f;
    // Speed_Loop_Fixed.Weak_Pi.ki = 1.01f;
    Speed_Loop_Fixed.Weak_Pi.out_max = 0.0f;
    Speed_Loop_Fixed.Weak_Pi.out_min = -8.0f;

    /*其他参数*/
    Speed_Loop_Fixed.Align_Finish_Flag = 0;
    Speed_Loop_Fixed.Speed_Ref = 0;
    Speed_Loop_Fixed.Speed_Switch_Flag = 0;
    Speed_Loop_Fixed.Speed_PI.integral = 0;
    Speed_Loop_Fixed.Weak_Control_Hcomp.comp_out = 0;
    Speed_Loop_Fixed.Weak_Pi.integral = 0;
    Speed_Loop_Fixed.target_id = 0;
    Speed_Loop_Fixed.target_iq = 0;
    Speed_Loop_Fixed.target_is = 0;
    Speed_Loop_Fixed.Speed_Fb = 0;

    // LADRC_FirstOrder_Init(&Speed_Loop_Fixed.Speed_LADRC, 0.001f, 100000.0f, 200.0f, 20.0f, 8.0f, -8.0f);

    Hysteresis_Comp_Init(&Speed_Loop_Fixed.Speed_Middle_High_Hcomp, 1000.0f, 800.0f, 1000);
    Speed_Loop_Fixed.Speed_Middle_High_Hcomp.enable = 1;

    Speed_Loop_Fixed.PWM_SWITCH_FREQ = 20000.0f;
    Speed_Loop_Fixed.PWM_CUR_FREQ = 20000.0f;
    Power_Derating_Init();
}

void Speed_Input_LPF()
{
    Speed_Loop_Fixed.Speed_Fb_1s = Speed_Loop_Fixed.Speed_Fb_1s * 0.999f + Speed_Loop_Fixed.Speed_Fb * 0.001f;
}

void Speed_Loop_Fixed_Task(void)
{
    if (System.system_state == SYSTEM_RUN && Current_Loop.Motor_State == MOTOR_RUN)
    {
        Speed_Input_LPF();
        Paramater_update();
        Speed_Run();
        
    }
    else
    {
        Speed_Loop_Fixed.spd_ctrl_state = Speed_Loop_Idle;
        Speed_Loop_Fixed.Speed_Fb = 0;
        Speed_Loop_Fixed.spd_ctrl_timer = 0;
    }
}

void Speed_Run(void)
{
    Speed_Loop_Fixed.spd_ctrl_timer++;
    switch (Speed_Loop_Fixed.spd_ctrl_state)
    {
    case Speed_Loop_Idle:
        // Handle idle state
        Speed_Loop_Fixed_Idle_Task();
        break;
    case Speed_Loop_Align:
        // Handle align state
        Speed_Loop_Fixed_Align_Task();
        break;
    case Speed_Loop_Open:
        // Handle open state
        Speed_Loop_Fixed_Open_Task();
        break;
    case Speed_Loop_Switch:
        // Handle switch state
        Speed_Loop_Fixed_Switch_Task();
        break;
    case Speed_Loop_Low:
        // Handle low state
        Speed_Loop_Fixed_Low_Task();
        break;
    case Speed_Loop_Middle:
        // Handle middle state
        Speed_Loop_Fixed_Middle_Task();
        break;
    case Speed_Loop_High:
        // Handle high state
        Speed_Loop_Fixed_High_Task();
        break;
    default:
        Speed_Loop_Fixed.spd_ctrl_state = Speed_Loop_Idle;
        break;
    }
}

void Speed_Loop_Fixed_Idle_Task(void)
{

    if (System.Run_flag == 1)
    {
        Speed_Loop_Fixed_Init();
        Speed_Loop_Fixed.spd_ctrl_state = Speed_Loop_Align;
    }
}

void Align_Task()
{
    Speed_Loop_Fixed.target_iq = 0;
    Speed_Loop_Fixed.Speed_Ref = 0;
    // vTaskDelay(100);
    // Speed_Loop_Fixed.target_id = 1.0f;
    // align_done = 1;
    // vTaskDelay(500);
    // Speed_Loop_Fixed.target_id = 0.0f;
    // vTaskDelay(10);
    // align_done = 0;
    // Speed_Loop_Fixed.target_id = 0.0f;
    // vTaskDelay(500);
    // Current_Loop.theta = 0.17*6;
    // vTaskDelay(5000);
    // Current_Loop.theta = 0.17*12;
    // vTaskDelay(5000);
    // Current_Loop.theta = 0.17*18;
    // vTaskDelay(5000);
}

void Speed_Loop_Fixed_Align_Task()
{
    Speed_Loop_Fixed.target_id = 0.0f;
    if (System.Run_flag == 1)
    {
#if defined(MOTOR_OPEN_SETUP)
        Speed_Loop_Fixed.spd_ctrl_state = Speed_Loop_Open;
#elif defined(MOTOR_CLOSE_SETUP)
        Speed_Loop_Fixed.spd_ctrl_state = Speed_Loop_Fixed_Low;
#endif
    }
}

void Speed_Loop_Fixed_Open_Task(void)
{
    float tick_count = ((float)(Speed_Loop_Fixed.spd_ctrl_timer) / Speed_Loop_Fixed.FREQ_Hz);
    Speed_Loop_Fixed.Speed_Ref = Lookup_Table_Linear(tick_count, &PMSM_42JS_Config.IF_Start_Speed_Lookup);
    Speed_Loop_Fixed.target_iq = Lookup_Table_Linear(tick_count, &PMSM_42JS_Config.IF_Start_Iq_Lookup);
    Speed_Loop_Fixed.target_id = 0;
    if (Speed_Loop_Fixed.Speed_Ref >= 600)
    {
        // Speed_Loop_Fixed.spd_ctrl_state = Speed_Loop_Fixed_Switch;
    }
}

void Speed_Loop_Fixed_Switch_Task(void)
{
    if (Speed_Loop_Fixed.Speed_Switch_Flag == 0)
    {
        Speed_Loop_Fixed.target_iq -= (SPEED_SWITCH_ID_SUB_STEP);
        if (Speed_Loop_Fixed.target_iq < 0)
        {
            Speed_Loop_Fixed.target_iq = 0;
        }
    }
    else
    {
        Speed_Loop_Fixed.Speed_PI.integral = Speed_Loop_Fixed.target_iq;
        Speed_Loop_Fixed.spd_ctrl_state = Speed_Loop_Low;
    }
}

// void Speed_Loop_Fixed_Low_Task(void)
// {

//     // Speed_Loop_Fixed.target_id = Oblique_Wave(0.5f, Speed_Loop_Fixed.target_id, SPEED_ID_ADD_STEP, SPEED_ID_SUB_STEP);
//     if (Speed_Loop_Fixed.Speed_Ref > MOTOR_SPEED_MIDDLE_THD)
//     {
//         Speed_Loop_Fixed.spd_ctrl_state = Speed_Loop_Middle;
//     }
//     Speed_Loop_Fixed_Run_Task();
// }

// void Speed_Loop_Fixed_Middle_Task(void)
// {
//     // Speed_Loop_Fixed.target_id = Oblique_Wave(0.2f, Speed_Loop_Fixed.target_id, SPEED_ID_ADD_STEP, SPEED_ID_SUB_STEP);
//     Speed_Loop_Fixed_Run_Task();
//     Hysteresis_Comp_Process_Add(&Speed_Loop_Fixed.Speed_Middle_High_Hcomp, Speed_Loop_Fixed.Speed_Ref);
//     if (Speed_Loop_Fixed.Speed_Middle_High_Hcomp.comp_out == 1)
//     {
//         Speed_Loop_Fixed.spd_ctrl_state = Speed_Loop_High;
//     }
// }

// void Speed_Loop_Fixed_High_Task(void)
// {
//     // Speed_Loop_Fixed.target_id = Oblique_Wave(0.0f, Speed_Loop_Fixed.target_id, SPEED_ID_ADD_STEP, SPEED_ID_SUB_STEP);
//     Speed_Loop_Fixed_Run_Task();
//     Hysteresis_Comp_Process_Add(&Speed_Loop_Fixed.Speed_Middle_High_Hcomp, Speed_Loop_Fixed.Speed_Ref);
//     if (Speed_Loop_Fixed.Speed_Middle_High_Hcomp.comp_out == 0)
//     {
//         Speed_Loop_Fixed.spd_ctrl_state = Speed_Loop_Middle;
//     }
// }

// void Speed_Loop_Fixed_Run_Task(void)
// {
//     Power_Derating(Current_Loop.Bus_Current_LPF, Current_Loop_Input_Fixed.Udc_ADISR, PMSM_42JS_Config.motor_param->Power_Limit);
//     Speed_Loop_Fixed.Speed_Ref = Oblique_Wave(Speed_Loop_Fixed.Speed_Command * Speed_Loop_Fixed.Derating_Factor, Speed_Loop_Fixed.Speed_Ref,
//                                         SPEED_ADD_STEP, SPEED_SUB_STEP);
//     // Speed_Loop_Fixed.Speed_Ref = Speed_Loop_Fixed.Speed_Command;
//     Hysteresis_Comp_Process_Add(&Speed_Loop_Fixed.Weak_Control_Hcomp, Speed_Loop_Fixed.Voltage_err);
//     // Speed_Loop_Fixed.target_is = LADRC_FirstOrder_Update(&Speed_Loop_Fixed.Speed_LADRC, Speed_Loop_Fixed.Speed_Ref, Speed_Loop_Fixed.Speed_Fb);
//     Speed_Loop_Fixed.target_is = Hal_PI_f32(&Speed_Loop_Fixed.Speed_PI, Speed_Loop_Fixed.Speed_Ref - Speed_Loop_Fixed.Speed_Fb);
//     if (Speed_Loop_Fixed.Weak_Control_Hcomp.comp_out == 1)
//     {
//         Speed_Loop_Fixed.Flux_Weak_Id = Hal_PI_f32(&Speed_Loop_Fixed.Weak_Pi, (Speed_Loop_Fixed.Weak_Control_Hcomp.threshold_high - Speed_Loop_Fixed.Voltage_err));
//     }
//     else
//     {
//         Speed_Loop_Fixed.Flux_Weak_Id = 0;
//     }
//     MTPA_Cal(Speed_Loop_Fixed.target_is);
//     Speed_Loop_Fixed.target_id = Speed_Loop_Fixed.MTPA_Id + Speed_Loop_Fixed.Flux_Weak_Id;
//     // Speed_Loop_Fixed.target_id = Speed_Loop_Fixed.MTPA_Id;
//     arm_sqrt_f32(Current_Loop.Ud_Target * Current_Loop.Ud_Target + Current_Loop.Uq_Target * Current_Loop.Uq_Target, &Speed_Loop_Fixed.Vs);
//     Speed_Loop_Fixed.Voltage_err = Speed_Loop_Fixed.Vs - Current_Loop_Input_Fixed.Udc_ADISR * WEAK_VOLTAGE_COMPENSATION;
//     if (Speed_Loop_Fixed.target_is > Speed_Loop_Fixed.target_id)
//     {
//         arm_sqrt_f32(Speed_Loop_Fixed.target_is * Speed_Loop_Fixed.target_is - Speed_Loop_Fixed.target_id * Speed_Loop_Fixed.target_id,
//                      &Speed_Loop_Fixed.target_iq);
//     }
//     else
//     {
//         Speed_Loop_Fixed.target_iq = 0;
//     }
// }


// void Power_Derating_Init(void)
// {
//     Speed_Loop_Fixed.Derating_Pi.kp = 1e-5f;
//     Speed_Loop_Fixed.Derating_Pi.ki = 1e-5f;
//     Speed_Loop_Fixed.Derating_Pi.Kd = 0.1f;
//     Speed_Loop_Fixed.Derating_Pi.out_max = 1.0f;
//     Speed_Loop_Fixed.Derating_Pi.out_min = 0.0f;
//     Speed_Loop_Fixed.Derating_Factor = 1.0f;
// }

// void Power_Derating(float Bus_Current, float Bus_Voltage, float Power_Limit)
// {
//     float Error = Power_Limit - (Bus_Current * Bus_Voltage);
//     // if((Error) < 10.0f)
//     {
//         Speed_Loop_Fixed.Derating_Factor = Hal_PI_f32(&Speed_Loop_Fixed.Derating_Pi, Error);
//     }
// }

// void MTPA_Cal(float Is)
// {
//     if (Speed_Loop_Fixed.pMotor->motor_param->Ld_Lq > 1e-4)
//     {
//         float MTPA_tmp = 0;
//         arm_sqrt_f32(Is * Is * Speed_Loop_Fixed.pMotor->motor_param->Ld_Lq * Speed_Loop_Fixed.pMotor->motor_param->Ld_Lq * 8 
//                     + Speed_Loop_Fixed.pMotor->motor_param->Flux_Flux, &MTPA_tmp);
//         Speed_Loop_Fixed.MTPA_Id = (MTPA_tmp - Speed_Loop_Fixed.pMotor->motor_param->flux_linkage_wb) / Speed_Loop_Fixed.pMotor->motor_param->Ld_Lq / 4;
//     }
//     else
//     {
//         Speed_Loop_Fixed.MTPA_Id = 0;
//     }
// }
