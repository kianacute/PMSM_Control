#include "Speed_Loop_Float.h"
#include "arm_math.h"
#include "Current_Loop_Float.h"
#include "Motor_Config_Float.h"
#include "Hal_Math_Float.h"
#include "Observer_Float.h"
#include "System_Loop_Float.h"
#include "Motor_Config_Float.h"

Speed_Loop_Float_t Speed_Loop_Float;

void Paramater_update(Motor_Control_t *pControl);
void Power_Derating_Float_Init(Motor_Control_t *pControl);
void Power_Derating_Float(Motor_Control_t *pControl, float Bus_Current, float Bus_Voltage, float Power_Limit);
void MTPA_Cal_Float(Motor_Control_t *pControl, float Is);
void Speed_Input_LPF(void);

void Speed_Loop_Init_Float(Motor_Control_t *pControl)
{
    pControl->Speed_Loop.Status = SPEED_IDLE;
    pControl->Speed_Loop.pSpeed_Loop = (void*)&Speed_Loop_Float;
    Speed_Loop_Float.Speed_Switch_Cnt = 0;
    Speed_Loop_Float.FREQ_Hz = 1000;
    Speed_Loop_Float.Speed_Fb_1s = 0;

    /*速度环参数初始化*/
    Speed_Loop_Float.Speed_PI.Kd = 0.1f;
    Speed_Loop_Float.Speed_PI.out_max = 8.0f;
    Speed_Loop_Float.Speed_PI.out_min = -8.0f;

    /*弱磁环参数初始化*/
    Hysteresis_Comp_Init(&Speed_Loop_Float.Weak_Control_Hcomp, -0.0f, -1.0f, 50);
    Speed_Loop_Float.Weak_Control_Hcomp.enable = 1;
    Speed_Loop_Float.Weak_Pi.kp = 0.01f;
    Speed_Loop_Float.Weak_Pi.ki = 0.1f;
    Speed_Loop_Float.Weak_Pi.Kd = 0.1f;
    // Speed_Loop_Float.Weak_Pi.kp = 1.01f;
    // Speed_Loop_Float.Weak_Pi.ki = 1.01f;
    Speed_Loop_Float.Weak_Pi.out_max = 0.0f;
    Speed_Loop_Float.Weak_Pi.out_min = -8.0f;

    /*其他参数*/
    Speed_Loop_Float.Align_Finish_Flag = 0;
    Speed_Loop_Float.Speed_Ref = 0;
    Speed_Loop_Float.Speed_Switch_Flag = 0;
    Speed_Loop_Float.Speed_PI.integral = 0;
    Speed_Loop_Float.Weak_Control_Hcomp.comp_out = 0;
    Speed_Loop_Float.Weak_Pi.integral = 0;
    Speed_Loop_Float.target_id = 0;
    Speed_Loop_Float.target_iq = 0;
    Speed_Loop_Float.target_is = 0;
    Speed_Loop_Float.Speed_Fb = 0;

    // LADRC_FirstOrder_Init(&Speed_Loop.Speed_LADRC, 0.001f, 100000.0f, 200.0f, 20.0f, 8.0f, -8.0f);

    Hysteresis_Comp_Init(&Speed_Loop_Float.Speed_Middle_High_Hcomp, 1000.0f, 800.0f, 1000);
    Speed_Loop_Float.Speed_Middle_High_Hcomp.enable = 1;

    Speed_Loop_Float.PWM_SWITCH_FREQ = 20000.0f;
    Speed_Loop_Float.PWM_CUR_FREQ = 20000.0f;
    Power_Derating_Float_Init(pControl);
}

void Speed_Input_LPF()
{
    Speed_Loop_Float.Speed_Fb_1s =  Speed_Loop_Float.Speed_Fb_1s * 0.999f + Speed_Loop_Float.Speed_Fb * 0.001f;
}

void SPEED_Idle_Task_FLoat(Motor_Control_t *pControl)
{
    System_Loop_FLoat_t *System_Loop = (System_Loop_FLoat_t *)pControl->System_Loop.pSystem_Loop;
    if (System_Loop->Run_flag == 1)
    {
        Speed_Loop_Init_Float(pControl);
        pControl->Speed_Loop.Status = SPEED_ALIGN;
    }
    else
    {
        pControl->Speed_Loop.Status = SPEED_IDLE;
    }
}

void Align_Task(Motor_Control_t *pControl)
{
    Speed_Loop_Float_t *pSpeed_Loop = (Speed_Loop_Float_t *)pControl->Speed_Loop.pSpeed_Loop;
    pSpeed_Loop->target_iq = 0;
    pSpeed_Loop->Speed_Ref = 0;
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

void Speed_Loop_Align_Task(Motor_Control_t *pControl)
{
    Speed_Loop_Float_t *pSpeed_Loop = (Speed_Loop_Float_t *)pControl->Speed_Loop.pSpeed_Loop;
    System_Loop_FLoat_t *System_Loop = (System_Loop_FLoat_t *)pControl->System_Loop.pSystem_Loop;
    pSpeed_Loop->target_id = 0.0f;
    if (System_Loop->Run_flag == 1)
    {
        #if defined(MOTOR_OPEN_SETUP)
            pControl->Speed_Loop.Status = SPEED_OPEN;
            pSpeed_Loop->IF_Start_Cnt  = pControl->Speed_Loop.Loop_count;
        #elif defined(MOTOR_CLOSE_SETUP)
            pSpeed_Loop->spd_ctrl_state = SPEED_LOW;
        #endif
    }
}

void Speed_Loop_Open_Task(Motor_Control_t *pControl)
{
    Speed_Loop_Float_t *pSpeed_Loop = (Speed_Loop_Float_t *)pControl->Speed_Loop.pSpeed_Loop;
    Motor_Config_t *pMotor_Config = (Motor_Config_t *)pControl->Motor_Config;
    float tick_count = ((float)(pControl->Speed_Loop.Loop_count - pSpeed_Loop->IF_Start_Cnt) / pSpeed_Loop->FREQ_Hz);
    pSpeed_Loop->Speed_Ref = Lookup_Table_Linear(tick_count, &pMotor_Config->IF_Start_Speed_Lookup);
    pSpeed_Loop->target_iq = Lookup_Table_Linear(tick_count, &pMotor_Config->IF_Start_Iq_Lookup);
    pSpeed_Loop->target_id = 0;
    if (pSpeed_Loop->Speed_Ref >= 600)
    {
        // Speed_Loop.spd_ctrl_state = Speed_Loop_Switch;
    }
}

void Speed_Loop_Switch_Task(Motor_Control_t *pControl)
{
    Speed_Loop_Float_t *pSpeed_Loop = (Speed_Loop_Float_t *)pControl->Speed_Loop.pSpeed_Loop;
    if (pSpeed_Loop->Speed_Switch_Flag == 0)
    {
        pSpeed_Loop->target_iq -= (SPEED_SWITCH_ID_SUB_STEP);
        if (pSpeed_Loop->target_iq < 0)
        {
            pSpeed_Loop->target_iq = 0;
        }
    }
    else
    {
        pSpeed_Loop->Speed_PI.integral = pSpeed_Loop->target_iq;
        pControl->Speed_Loop.Status = SPEED_LOW;
    }
}

void Speed_Loop_Low_Task(Motor_Control_t *pControl)
{
    Speed_Loop_Float_t *pSpeed_Loop = (Speed_Loop_Float_t *)pControl->Speed_Loop.pSpeed_Loop;

    // Speed_Loop.target_id = Oblique_Wave(0.5f, Speed_Loop.target_id, SPEED_ID_ADD_STEP, SPEED_ID_SUB_STEP);
    if (pSpeed_Loop->Speed_Ref > MOTOR_SPEED_MIDDLE_THD)
    {
         pControl->Speed_Loop.Status = SPEED_MIDDLE;
    }
    Speed_Loop_Run_Task(pControl);
}

void Speed_Loop_Middle_Task(Motor_Control_t *pControl)
{
    Speed_Loop_Float_t *pSpeed_Loop = (Speed_Loop_Float_t *)pControl->Speed_Loop.pSpeed_Loop;
    // Speed_Loop.target_id = Oblique_Wave(0.2f, Speed_Loop.target_id, SPEED_ID_ADD_STEP, SPEED_ID_SUB_STEP);
    Speed_Loop_Run_Task(pControl);
    Hysteresis_Comp_Process_Add(&pSpeed_Loop->Speed_Middle_High_Hcomp, pSpeed_Loop->Speed_Ref);
    if (pSpeed_Loop->Speed_Middle_High_Hcomp.comp_out == 1)
    {
        pControl->Speed_Loop.Status = SPEED_HIGH;
    }
}

void Speed_Loop_High_Task(Motor_Control_t *pControl)
{
    Speed_Loop_Float_t *pSpeed_Loop = (Speed_Loop_Float_t *)pControl->Speed_Loop.pSpeed_Loop;
    // Speed_Loop.target_id = Oblique_Wave(0.0f, Speed_Loop.target_id, SPEED_ID_ADD_STEP, SPEED_ID_SUB_STEP);
    Speed_Loop_Run_Task(pControl);
    Hysteresis_Comp_Process_Add(&pSpeed_Loop->Speed_Middle_High_Hcomp, pSpeed_Loop->Speed_Ref);
    if (pSpeed_Loop->Speed_Middle_High_Hcomp.comp_out == 0)
    {
        pControl->Speed_Loop.Status = SPEED_MIDDLE;
    }
}

void Speed_Loop_Run_Task(Motor_Control_t *pControl)
{
    Speed_Loop_Float_t *pSpeed_Loop = (Speed_Loop_Float_t *)pControl->Speed_Loop.pSpeed_Loop;
    Current_Loop_Float_t *pCurrent_Loop_Float = (Current_Loop_Float_t *)pControl->Current_Loop.pCurrent_Loop;
    Motor_Control_Input_t *pInput = (Motor_Control_Input_t *)&pControl->Input;
    Motor_Parameter_t *Motor_Param = (Motor_Parameter_t *)pControl->Motor_Config->Motor_Param;

    Power_Derating_FLoat(pCurrent_Loop_Float->Bus_Current_LPF, pInput->Udc_ADISR, Motor_Param->Power_Limit);   
    pSpeed_Loop->Speed_Ref = Oblique_Wave(pSpeed_Loop->Speed_Command * pSpeed_Loop->Derating_Factor, pSpeed_Loop->Speed_Ref,
                                        SPEED_ADD_STEP, SPEED_SUB_STEP);
    // Speed_Loop.Speed_Ref = Speed_Loop.Speed_Command;
    Hysteresis_Comp_Process_Add(&pSpeed_Loop->Weak_Control_Hcomp, pSpeed_Loop->Voltage_err);
    // Speed_Loop.target_is = LADRC_FirstOrder_Update(&Speed_Loop.Speed_LADRC, Speed_Loop.Speed_Ref, Speed_Loop.Speed_Fb);
    pSpeed_Loop->target_is = Hal_PI_f32(&pSpeed_Loop->Speed_PI, pSpeed_Loop->Speed_Ref - pSpeed_Loop->Speed_Fb);
    if (pSpeed_Loop->Weak_Control_Hcomp.comp_out == 1)
    {
        pSpeed_Loop->Flux_Weak_Id = Hal_PI_f32(&pSpeed_Loop->Weak_Pi, (pSpeed_Loop->Weak_Control_Hcomp.threshold_high - pSpeed_Loop->Voltage_err));
    }
    else
    {
        pSpeed_Loop->Flux_Weak_Id = 0;
    }
    MTPA_Cal_FLoat(pSpeed_Loop->target_is);
    pSpeed_Loop->target_id = pSpeed_Loop->MTPA_Id + pSpeed_Loop->Flux_Weak_Id;
    // Speed_Loop.target_id = Speed_Loop.MTPA_Id;
    arm_sqrt_f32(pCurrent_Loop_Float->Ud_Target * pCurrent_Loop_Float->Ud_Target + pCurrent_Loop_Float->Uq_Target * pCurrent_Loop_Float->Uq_Target, &pSpeed_Loop->Vs);
    pSpeed_Loop->Voltage_err = pSpeed_Loop->Vs - pInput->Udc_ADISR * WEAK_VOLTAGE_COMPENSATION;
    if (pSpeed_Loop->target_is > pSpeed_Loop->target_id)
    {
        arm_sqrt_f32(pSpeed_Loop->target_is * pSpeed_Loop->target_is - pSpeed_Loop->target_id * pSpeed_Loop->target_id,
                     &pSpeed_Loop->target_iq);
    }
    else
    {
        pSpeed_Loop->target_iq = 0;
    }
}


void Paramater_update(Motor_Control_t *pControl)
{
    Speed_Loop_Float_t *pSpeed_Loop = (Speed_Loop_Float_t *)pControl->Speed_Loop.pSpeed_Loop;
    Motor_Control_Input_t *pInput = (Motor_Control_Input_t *)&pControl->Input;  
    Motor_Config_t *pMotor_Config = (Motor_Config_t *)pControl->Motor_Config;
    if (pSpeed_Loop->PWM_CUR_FREQ < PWM_SWITH_FREQ_MIN)
    {
        pSpeed_Loop->PWM_CUR_FREQ = PWM_SWITH_FREQ_MIN;
    }
    else if (pSpeed_Loop->PWM_CUR_FREQ > PWM_SWITH_FREQ_MAX)
    {
        pSpeed_Loop->PWM_CUR_FREQ = PWM_SWITH_FREQ_MAX;
    }
    pSpeed_Loop->PWM_CUR_FREQ = Oblique_Wave(pSpeed_Loop->PWM_SWITCH_FREQ, pSpeed_Loop->PWM_CUR_FREQ, PWM_SWITH_FREQ_STEP, PWM_SWITH_FREQ_STEP);
    Observer_Param_Lookup_Updata_Float(pControl, pSpeed_Loop->Speed_Fb_1s, pSpeed_Loop->target_is, 1/pSpeed_Loop->PWM_CUR_FREQ);
    Current_Para_Updata(pControl, pSpeed_Loop->Speed_Fb_1s, 1/pSpeed_Loop->PWM_CUR_FREQ);    
    pSpeed_Loop->Speed_PI.kp = Lookup_Table_Linear(pSpeed_Loop->Speed_Fb_1s, &pMotor_Config->Speed_PI_Kp_Lookup);
    pSpeed_Loop->Speed_PI.ki = Lookup_Table_Linear(pSpeed_Loop->Speed_Fb_1s, &pMotor_Config->Speed_PI_Ki_Lookup);
}                     



void Power_Derating_Float_Init(Motor_Control_t *pControl)
{
    Speed_Loop_Float_t *pSpeed_Loop = (Speed_Loop_Float_t *)pControl->Speed_Loop.pSpeed_Loop;
    pSpeed_Loop->Derating_Pi.kp = 1e-5f;
    pSpeed_Loop->Derating_Pi.ki = 1e-5f;
    pSpeed_Loop->Derating_Pi.Kd = 0.1f;
    pSpeed_Loop->Derating_Pi.out_max = 1.0f;
    pSpeed_Loop->Derating_Pi.out_min = 0.0f;
    pSpeed_Loop->Derating_Factor = 1.0f;
}

void Power_Derating_Float(Motor_Control_t *pControl, float Bus_Current, float Bus_Voltage, float Power_Limit)
{
    Speed_Loop_Float_t *pSpeed_Loop = (Speed_Loop_Float_t *)pControl->Speed_Loop.pSpeed_Loop;
    float Error = Power_Limit - (Bus_Current * Bus_Voltage);
    // if((Error) < 10.0f)
    {
        pSpeed_Loop->Derating_Factor = Hal_PI_f32(&pSpeed_Loop->Derating_Pi, Error);
    }
}

void MTPA_Cal_Float(Motor_Control_t *pControl, float Is)
{
    Speed_Loop_Float_t *pSpeed_Loop = (Speed_Loop_Float_t *)pControl->Speed_Loop.pSpeed_Loop;
    Motor_Parameter_t *pMotor_Param = (Motor_Parameter_t *)pControl->Motor_Config->Motor_Param;
    if (pMotor_Param->Ld_Lq > 1e-4)
    {
        float MTPA_tmp = 0;
        arm_sqrt_f32(Is * Is * pMotor_Param->Ld_Lq * pMotor_Param->Ld_Lq * 8 
                    + pMotor_Param->Flux_Flux, &MTPA_tmp);
        pSpeed_Loop->MTPA_Id = (MTPA_tmp - pMotor_Param->flux_linkage_wb) / pMotor_Param->Ld_Lq / 4;
    }
    else
    {
        pSpeed_Loop->MTPA_Id = 0;
    }
}
