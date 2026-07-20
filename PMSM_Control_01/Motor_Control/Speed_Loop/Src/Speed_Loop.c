#include "Speed_Loop.h"
#include "arm_math.h"
#include "Current_Loop.h"
#include "Motor_Config.h"
#include "Hal_Math.h"
#include "Observer.h"
#include "System_Loop.h"
#include "Motor_Config.h"

extern uint8_t MOTOR_Run_flag;
extern float Speed_Command;
extern Motor_Config_t PMSM_42JS_Config;
extern struct NonFluxObserver_Parameter NonFlux_OB;
extern Current_Loop_t Current_Loop;;
extern SYSTEM_t System;

Speed_Loop_t Speed_Loop;

void Speed_Run(void);
void Speed_Loop_Idle_Task(void);
void Speed_Loop_Align_Task();
void Speed_Loop_Open_Task(void);
void Speed_Loop_Switch_Task(void);
void Speed_Loop_Low_Task(void);
void Speed_Loop_Middle_Task(void);
void Speed_Loop_High_Task(void);
void Speed_Loop_Run_Task(void);
void Paramater_update(void);
void Power_Derating_Init(void);
float Power_Derating(float Bus_Current, float Bus_Voltage, float Power_Limit);

void Speed_Loop_Init(void)
{
    Speed_Loop.spd_ctrl_state = Speed_Loop_Idle;
    Speed_Loop.spd_ctrl_timer = 0;    
    Speed_Loop.Speed_Switch_Cnt = 0;
    Speed_Loop.pMotor = &PMSM_42JS_Config;
    Speed_Loop.FREQ_Hz = 1000;

    /*速度环参数初始化*/
    Speed_Loop.Speed_PI.Kd = 0.1f;
    Speed_Loop.Speed_PI.out_max = 8.0f;
    Speed_Loop.Speed_PI.out_min = -8.0f;
    
    /*弱磁环参数初始化*/
    Hysteresis_Comp_Init(&Speed_Loop.Weak_Control_Hcomp, -0.0f, -1.0f, 50);
    Speed_Loop.Weak_Control_Hcomp.enable = 1;
    Speed_Loop.Weak_Pi.kp = 0.01f;
    Speed_Loop.Weak_Pi.ki = 0.1f;
    Speed_Loop.Weak_Pi.Kd = 0.1f;
    // Speed_Loop.Weak_Pi.kp = 1.01f;
    // Speed_Loop.Weak_Pi.ki = 1.01f;
    Speed_Loop.Weak_Pi.out_max = 0.0f;
    Speed_Loop.Weak_Pi.out_min = -8.0f;

    /*其他参数*/
    Speed_Loop.Align_Finish_Flag = 0;
    Speed_Loop.Speed_Ref = 0;
    Speed_Loop.Speed_Switch_Flag = 0;
    Speed_Loop.Speed_PI.integral = 0;
    Speed_Loop.Weak_Control_Hcomp.comp_out = 0;
    Speed_Loop.Weak_Pi.integral = 0;
    Speed_Loop.target_id = 0;
    Speed_Loop.target_iq = 0;
    Speed_Loop.target_is = 0;
    Speed_Loop.Speed_Fb = 0;

    // LADRC_FirstOrder_Init(&Speed_Loop.Speed_LADRC, 0.001f, 100000.0f, 200.0f, 20.0f, 8.0f, -8.0f);

    Hysteresis_Comp_Init(&Speed_Loop.Speed_Middle_High_Hcomp, 1000.0f, 800.0f, 1000);
    Speed_Loop.Speed_Middle_High_Hcomp.enable = 1;

    Power_Derating_Init();
}


void Speed_Loop_Task(void)
{
    if (System.system_state == SYSTEM_RUN && Current_Loop.Motor_State == MOTOR_RUN)
    {
        Paramater_update();
        Speed_Run();  
    }
    else
    {
        Speed_Loop.spd_ctrl_state = Speed_Loop_Idle;
        Speed_Loop.Speed_Fb = 0;
        Speed_Loop.spd_ctrl_timer = 0;
    }
}

void Speed_Run(void)
{
    Speed_Loop.spd_ctrl_timer++;
    switch (Speed_Loop.spd_ctrl_state)
    {
    case Speed_Loop_Idle:
        // Handle idle state
        Speed_Loop_Idle_Task();
        break;
    case Speed_Loop_Align:
        // Handle align state
        Speed_Loop_Align_Task();
        break;
    case Speed_Loop_Open:
        // Handle open state
        Speed_Loop_Open_Task();
        break;
    case Speed_Loop_Switch:
        // Handle switch state
        Speed_Loop_Switch_Task();
        break;
    case Speed_Loop_Low:
        // Handle low state
        Speed_Loop_Low_Task();
        break;
    case Speed_Loop_Middle:
        // Handle middle state
        Speed_Loop_Middle_Task();
        break;
    case Speed_Loop_High:
        // Handle high state
        Speed_Loop_High_Task();
        break;
    default:
        Speed_Loop.spd_ctrl_state = Speed_Loop_Idle;
        break;
    }
}

void Speed_Loop_Idle_Task(void)
{
    
    if (System.Run_flag == 1)
    {
        Speed_Loop_Init();
        Speed_Loop.spd_ctrl_state = Speed_Loop_Align;
    }
}

void Align_Task()
{
    Speed_Loop.target_iq = 0;
    Speed_Loop.Speed_Ref = 0;
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
}

void Speed_Loop_Align_Task()
{
    Speed_Loop.target_id = 0.5f;
    if (System.Run_flag == 1)
    {
        #if defined(MOTOR_OPEN_SETUP)
        Speed_Loop.spd_ctrl_state = Speed_Loop_Open;
        #elif defined(MOTOR_CLOSE_SETUP)
        Speed_Loop.spd_ctrl_state = Speed_Loop_Low;
        #endif
    }
}

void Speed_Loop_Open_Task(void)
{
    float tick_count = ((float)(Speed_Loop.spd_ctrl_timer) / Speed_Loop.FREQ_Hz);
    Speed_Loop.Speed_Ref = Lookup_Table_Linear(tick_count, &PMSM_42JS_Config.IF_Start_Speed_Lookup);
    Speed_Loop.target_iq = Lookup_Table_Linear(tick_count, &PMSM_42JS_Config.IF_Start_Iq_Lookup);
    Speed_Loop.target_id = 0;
    if (Speed_Loop.Speed_Ref >= 600)
    {
        Speed_Loop.spd_ctrl_state = Speed_Loop_Switch;
    }
}

void Speed_Loop_Switch_Task(void)
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
        Speed_Loop.spd_ctrl_state = Speed_Loop_Low;
    }
}

void Speed_Loop_Low_Task(void)
{
   
    Speed_Loop.target_id = Oblique_Wave(0.5f, Speed_Loop.target_id, SPEED_ID_ADD_STEP, SPEED_ID_SUB_STEP);
    if(Speed_Loop.Speed_Ref > 500)
    {
        Speed_Loop.spd_ctrl_state = Speed_Loop_Middle;
    } 
    Speed_Loop_Run_Task();
}

void Speed_Loop_Middle_Task(void)
{
    Speed_Loop.target_id = Oblique_Wave(0.2f, Speed_Loop.target_id, SPEED_ID_ADD_STEP, SPEED_ID_SUB_STEP);
    Speed_Loop_Run_Task();    
    Hysteresis_Comp_Process_Add(&Speed_Loop.Speed_Middle_High_Hcomp, Speed_Loop.Speed_Ref);
    if(Speed_Loop.Speed_Middle_High_Hcomp.comp_out == 1)
    {
        Speed_Loop.spd_ctrl_state = Speed_Loop_High;
    }
}

void Speed_Loop_High_Task(void)
{
    Speed_Loop.target_id = Oblique_Wave(0.0f, Speed_Loop.target_id, SPEED_ID_ADD_STEP, SPEED_ID_SUB_STEP);
    Speed_Loop_Run_Task();
    Hysteresis_Comp_Process_Add(&Speed_Loop.Speed_Middle_High_Hcomp, Speed_Loop.Speed_Ref);
    if(Speed_Loop.Speed_Middle_High_Hcomp.comp_out == 0)
    {
        Speed_Loop.spd_ctrl_state = Speed_Loop_Middle;
    }
}

void Speed_Loop_Run_Task(void)
{
    Power_Derating(Current_Loop.Bus_Current_LPF, Current_Loop_Input.Udc_ADISR, PMSM_42JS_Config.motor_param->Power_Limit);
    Speed_Loop.Speed_Ref = Oblique_Wave(Speed_Loop.Speed_Command * Speed_Loop.Derating_Factor, Speed_Loop.Speed_Ref, 
                            SPEED_ADD_STEP, SPEED_SUB_STEP);
    Hysteresis_Comp_Process_Add(&Speed_Loop.Weak_Control_Hcomp, Speed_Loop.Voltage_err);
    // Speed_Loop.target_is = LADRC_FirstOrder_Update(&Speed_Loop.Speed_LADRC, Speed_Loop.Speed_Ref, Speed_Loop.Speed_Fb);
    Speed_Loop.target_is = Hal_PI_f32(&Speed_Loop.Speed_PI, Speed_Loop.Speed_Ref - Speed_Loop.Speed_Fb);
    if (Speed_Loop.Weak_Control_Hcomp.comp_out == 1)
    {
        Speed_Loop.target_id = Hal_PI_f32(&Speed_Loop.Weak_Pi, (Speed_Loop.Weak_Control_Hcomp.threshold_high - Speed_Loop.Voltage_err));
    }
    arm_sqrt_f32(Current_Loop.Ud_Target * Current_Loop.Ud_Target + Current_Loop.Uq_Target * Current_Loop.Uq_Target, &Speed_Loop.Vs);
    Speed_Loop.Voltage_err = Speed_Loop.Vs - Current_Loop_Input.Udc_ADISR * WEAK_VOLTAGE_COMPENSATION;
    if (Speed_Loop.target_is > Speed_Loop.target_id)
    {
        arm_sqrt_f32(Speed_Loop.target_is * Speed_Loop.target_is - Speed_Loop.target_id * Speed_Loop.target_id,
                 &Speed_Loop.target_iq);  
    }
    else
    {
        Speed_Loop.target_iq = 0;
    }
} 


void Paramater_update(void)
{
    Observer_Param_Lookup_Updata(Speed_Loop.Speed_Ref);

    Speed_Loop.Speed_PI.kp = Lookup_Table_Linear(Speed_Loop.Speed_Ref, &PMSM_42JS_Config.Speed_PI_Kp_Lookup);
    Speed_Loop.Speed_PI.ki = Lookup_Table_Linear(Speed_Loop.Speed_Ref, &PMSM_42JS_Config.Speed_PI_Ki_Lookup);

    Current_Loop.Id_PI.kp = Lookup_Table_Linear(Speed_Loop.Speed_Ref, &PMSM_42JS_Config.ID_PI_Kp_Lookup);
    Current_Loop.Id_PI.ki = Lookup_Table_Linear(Speed_Loop.Speed_Ref, &PMSM_42JS_Config.ID_PI_Ki_Lookup);
    Current_Loop.Iq_PI.kp = Lookup_Table_Linear(Speed_Loop.Speed_Ref, &PMSM_42JS_Config.IQ_PI_Kp_Lookup);
    Current_Loop.Iq_PI.ki = Lookup_Table_Linear(Speed_Loop.Speed_Ref, &PMSM_42JS_Config.IQ_PI_Ki_Lookup);
}

/* ==================================================================
 * 一阶LADRC (线性自抗扰控制)
 * 参考: https://zhuanlan.zhihu.com/p/664345718
 *
 * 一阶系统: dy/dt = f(y,t,d) + b0 * u
 * 其中 f 为总扰动 (内部不确定性 + 外部扰动)
 *
 * 扩张状态: x1=y, x2=f
 *   dx1/dt = x2 + b0*u
 *   dx2/dt = df/dt
 *
 * LESO (线性扩张状态观测器) — 欧拉前向离散化:
 *   e(k) = y(k) - z1(k)
 *   z1(k+1) = z1(k) + h * [beta1*e(k) + z2(k) + b0*u(k)]
 *   z2(k+1) = z2(k) + h * [beta2*e(k)]
 *   其中 beta1=2*wo, beta2=wo^2
 *
 * LSEF (线性误差反馈控制律):
 *   u0 = wc * (ref - z1)
 *   u  = (u0 - z2) / b0
 *
 * 参数整定: wo ≈ (3~10)*wc, b0 = Kt/J
 * ================================================================== */

void LADRC_FirstOrder_Init(LADRC_FirstOrder_t *ladrc, float h, float b0, float wo, float wc, float out_max, float out_min)
{
    ladrc->h = h;
    ladrc->b0 = b0;
    ladrc->wo = wo;
    ladrc->wc = wc;
    ladrc->z1 = 0.0f;
    ladrc->z2 = 0.0f;
    ladrc->u = 0.0f;
    ladrc->out_max = out_max;
    ladrc->out_min = out_min;
}

float LADRC_FirstOrder_Update(LADRC_FirstOrder_t *ladrc, float ref, float y)
{
    float beta1 = 2.0f * ladrc->wo;
    float beta2 = ladrc->wo * ladrc->wo;
    float e = y - ladrc->z1;

    ladrc->z1 += ladrc->h * (beta1 * e + ladrc->z2 + ladrc->b0 * ladrc->u);
    ladrc->z2 += ladrc->h * (beta2 * e);

    float u0 = ladrc->wc * (ref - ladrc->z1);
    ladrc->u = (u0 - ladrc->z2) / ladrc->b0;

    if (ladrc->u > ladrc->out_max)
        ladrc->u = ladrc->out_max;
    else if (ladrc->u < ladrc->out_min)
        ladrc->u = ladrc->out_min;

    return ladrc->u;
}

void Power_Derating_Init(void)
{
    Speed_Loop.Derating_Pi.kp = 1e-5f;
    Speed_Loop.Derating_Pi.ki = 1e-5f;
    Speed_Loop.Derating_Pi.Kd = 0.1f;
    Speed_Loop.Derating_Pi.out_max = 1.0f;
    Speed_Loop.Derating_Pi.out_min = 0.0f;
    Speed_Loop.Derating_Factor = 1.0f;
}


float Power_Derating(float Bus_Current, float Bus_Voltage, float Power_Limit)
{
    float Power = Bus_Current * Bus_Voltage;
    float Error = Power_Limit - Power;
    Speed_Loop.Derating_Factor = Hal_PI_f32(&Speed_Loop.Derating_Pi, Error);
    return Speed_Loop.Derating_Factor;
}


