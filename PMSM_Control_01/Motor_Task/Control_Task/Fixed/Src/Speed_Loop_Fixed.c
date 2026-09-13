#include "Speed_Loop_Fixed.h"
#include "arm_math.h"
#include "Current_Loop_Fixed.h"
#include "Hal_Math.h"
#include "Observer_Fixed.h"
#include "System_Loop_Fixed.h"

Speed_Loop_Fixed_t Speed_Loop_Fixed;

void Power_Derating_Init_Fixed(Motor_Control_t *pControl);
void Power_Derating_Fixed(Motor_Control_t *pControl, q31_t Bus_Current, q31_t Bus_Voltage, q31_t Power_Limit);
void MTPA_Cal_Fixed(Motor_Control_t *pControl, q31_t Is);
void Speed_Input_LPF_Fixed(void);
void SPEED_Run_Task_Fixed(Motor_Control_t *pControl);
void PWM_Freq_Update_Fixed(Motor_Control_t *pControl);

void SPEED_Init_Fixed(Motor_Control_t *pControl)
{
    pControl->Speed_Loop.Status = SPEED_IDLE;
    pControl->Speed_Loop.pSpeed_Loop = (void*)&Speed_Loop_Fixed;
    Speed_Loop_Fixed.Speed_Switch_Cnt = 0;
    Speed_Loop_Fixed.FREQ_Hz = 1000;
    Speed_Loop_Fixed.Speed_Fb_1s = 0;

    /*速度环参数初始化*/
    Speed_Loop_Fixed.Speed_PI.Kd = 0.1f;
    Speed_Loop_Fixed.Speed_PI.out_max = 8.0f;
    Speed_Loop_Fixed.Speed_PI.out_min = -8.0f;

    /*弱磁环参数初始化*/
    Hysteresis_Comp_Init_q31(&Speed_Loop_Fixed.Weak_Control_Hcomp, -0.0f, -1.0f, 50);
    Speed_Loop_Fixed.Weak_Control_Hcomp.enable = 1;
    Speed_Loop_Fixed.Weak_Pi.kp = 0.01f * MOTOR_U_BASE / MOTOR_I_BASE;
    Speed_Loop_Fixed.Weak_Pi.ki = 0.1f * MOTOR_U_BASE / MOTOR_I_BASE;
    Speed_Loop_Fixed.Weak_Pi.Kd = 0.1f * MOTOR_U_BASE / MOTOR_I_BASE;
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


    Hysteresis_Comp_Init_q31(&Speed_Loop_Fixed.Speed_Middle_High_Hcomp, 1000.0f, 800.0f, 1000);
    Speed_Loop_Fixed.Speed_Middle_High_Hcomp.enable = 1;

    Speed_Loop_Fixed.PWM_SWITCH_FREQ = 20000.0f;
    Speed_Loop_Fixed.PWM_CUR_FREQ = 20000.0f;
    Power_Derating_Init_Fixed(pControl);
}

void Speed_Input_LPF_Fixed(void)
{
    Speed_Loop_Fixed.Speed_Fb_1s =  Speed_Loop_Fixed.Speed_Fb_1s * 0.999f + Speed_Loop_Fixed.Speed_Fb * 0.001f;
}

void SPEED_Idle_Task_Fixed(Motor_Control_t *pControl)
{
    System_Loop_Fixed_t *System_Loop = (System_Loop_Fixed_t *)pControl->System_Loop.pSystem_Loop;
    Speed_Loop_Fixed_t *Speed_Loop = (Speed_Loop_Fixed_t *)pControl->Speed_Loop.pSpeed_Loop;
    if (System_Loop->Run_flag == 1)
    {
        SPEED_Init_Fixed(pControl);
        pControl->Speed_Loop.Status = SPEED_ALIGN;
        Speed_Loop->Speed_Align_Time_Count = 0;
    }
    else
    {
        pControl->Speed_Loop.Status = SPEED_IDLE;
        Speed_Loop->Speed_Ref = 0;
        Speed_Loop->Speed_Fb = 0;
        Speed_Loop->Speed_Fb_1s = 0;
        Speed_Loop->target_id = 0;
        Speed_Loop->target_iq = 0;
    }
}

void SPEED_Align_Task_Fixed(Motor_Control_t *pControl)
{
    Speed_Loop_Fixed_t *pSpeed_Loop = (Speed_Loop_Fixed_t *)pControl->Speed_Loop.pSpeed_Loop;
    System_Loop_Fixed_t *System_Loop = (System_Loop_Fixed_t *)pControl->System_Loop.pSystem_Loop;
    pSpeed_Loop->target_id = Oblique_Wave_q31(SPEED_ALIGN_ID_A, pSpeed_Loop->target_id, SPEED_ID_ADD_STEP, SPEED_ID_SUB_STEP);
    pSpeed_Loop->Speed_Align_Time_Count ++;
    if ((pSpeed_Loop->Speed_Align_Time_Count > SPEED_ALIGN_TIME_S) && (System_Loop->Run_flag == 1))
    {
        #if defined(MOTOR_OPEN_SETUP)
            pControl->Speed_Loop.Status = SPEED_OPEN;
            pSpeed_Loop->IF_Start_Cnt  = pControl->Speed_Loop.Loop_count;
        #elif defined(MOTOR_CLOSE_SETUP)
            pControl->Speed_Loop.Status = SPEED_LOW;
            // pSpeed_Loop->Speed_Low_Id = SPEED_LOW_ID_TARGET_A;
        #endif
    }
}

void SPEED_Open_Task_Fixed(Motor_Control_t *pControl)
{
    Speed_Loop_Fixed_t *pSpeed_Loop = (Speed_Loop_Fixed_t *)pControl->Speed_Loop.pSpeed_Loop;
    Motor_Config_t *pMotor_Config = (Motor_Config_t *)pControl->Motor_Config;
    float tick_count = ((float)(pControl->Speed_Loop.Loop_count - pSpeed_Loop->IF_Start_Cnt) / pSpeed_Loop->FREQ_Hz);
    pSpeed_Loop->Speed_Ref = Lookup_Table_1D_Linear_q31(tick_count, &pMotor_Config->IF_Start_Speed_Lookup);
    pSpeed_Loop->target_iq = Lookup_Table_1D_Linear_q31(tick_count, &pMotor_Config->IF_Start_Iq_Lookup);
    pSpeed_Loop->target_id = 0;
    if (pSpeed_Loop->Speed_Ref >= SPEED_OPEN2SWITCH_THD_RPM)
    { 
        pControl->Speed_Loop.Status= SPEED_SWITCH;
    }
}

void SPEED_Switch_Task_Fixed(Motor_Control_t *pControl)
{
    Speed_Loop_Fixed_t *pSpeed_Loop = (Speed_Loop_Fixed_t *)pControl->Speed_Loop.pSpeed_Loop;
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

void SPEED_Low_Task_Fixed(Motor_Control_t *pControl)
{
    Speed_Loop_Fixed_t *pSpeed_Loop = (Speed_Loop_Fixed_t *)pControl->Speed_Loop.pSpeed_Loop;
    pSpeed_Loop->Speed_Ref = Oblique_Wave_q31(pSpeed_Loop->Speed_Command * pSpeed_Loop->Derating_Factor, pSpeed_Loop->Speed_Ref,
                                    SPEED_LOW_ADD_STEP, SPEED_SUB_STEP);
    // pSpeed_Loop->Speed_Low_Id = Oblique_Wave_q31(SPEED_LOW_ID_TARGET_A, pSpeed_Loop->Speed_Low_Id, SPEED_ID_ADD_STEP, SPEED_ID_SUB_STEP);
    if (pSpeed_Loop->Speed_Ref > SPEED_MIDDLE_THD_RPM)
    {
         pControl->Speed_Loop.Status = SPEED_MIDDLE;
    }
    SPEED_Run_Task_Fixed(pControl);
}

void SPEED_Middle_Task_Fixed(Motor_Control_t *pControl)
{
    Speed_Loop_Fixed_t *pSpeed_Loop = (Speed_Loop_Fixed_t *)pControl->Speed_Loop.pSpeed_Loop;
    // pSpeed_Loop->Speed_Low_Id = Oblique_Wave_q31(0.0f, pSpeed_Loop->Speed_Low_Id, SPEED_ID_ADD_STEP, SPEED_ID_SUB_STEP);
    pSpeed_Loop->Speed_Ref = Oblique_Wave_q31(pSpeed_Loop->Speed_Command * pSpeed_Loop->Derating_Factor, pSpeed_Loop->Speed_Ref,
                                    SPEED_MIDDLE_HIGH_ADD_STEP, SPEED_SUB_STEP);
    SPEED_Run_Task_Fixed(pControl);
    Hysteresis_Comp_Process_Add_q31(&pSpeed_Loop->Speed_Middle_High_Hcomp, pSpeed_Loop->Speed_Ref);
    if (pSpeed_Loop->Speed_Middle_High_Hcomp.comp_out == 1)
    {
        pControl->Speed_Loop.Status = SPEED_HIGH;
    }
}

void SPEED_High_Task_Fixed(Motor_Control_t *pControl)
{
    Speed_Loop_Fixed_t *pSpeed_Loop = (Speed_Loop_Fixed_t *)pControl->Speed_Loop.pSpeed_Loop;
    // Speed_Loop.target_id = Oblique_Wave(0.0f, Speed_Loop.target_id, SPEED_ID_ADD_STEP, SPEED_ID_SUB_STEP);
    pSpeed_Loop->Speed_Ref = Oblique_Wave_q31(pSpeed_Loop->Speed_Command * pSpeed_Loop->Derating_Factor, pSpeed_Loop->Speed_Ref,
                                    SPEED_MIDDLE_HIGH_ADD_STEP, SPEED_SUB_STEP);
    SPEED_Run_Task_Fixed(pControl);
    Hysteresis_Comp_Process_Add_q31(&pSpeed_Loop->Speed_Middle_High_Hcomp, pSpeed_Loop->Speed_Ref);
    if (pSpeed_Loop->Speed_Middle_High_Hcomp.comp_out == 0)
    {
        pControl->Speed_Loop.Status = SPEED_MIDDLE;
    }
}

void SPEED_Run_Task_Fixed(Motor_Control_t *pControl)
{
    Speed_Loop_Fixed_t *pSpeed_Loop = (Speed_Loop_Fixed_t *)pControl->Speed_Loop.pSpeed_Loop;
    Current_Loop_Fixed_t *pCurrent_Loop_Fixed = (Current_Loop_Fixed_t *)pControl->Current_Loop.pCurrent_Loop;
    Motor_Control_Input_t *pInput = (Motor_Control_Input_t *)&pControl->Input;
    Motor_Parameter_Fixed_t *Motor_Param = (Motor_Parameter_Fixed_t *)pControl->Motor_Config->Motor_Param;

    // Power_Derating_Fixed(pControl, pCurrent_Loop_Fixed->Bus_Current_LPF, pInput->Udc_ADISR, Motor_Param->Power_Max_W);   
    // Speed_Loop.Speed_Ref = Speed_Loop.Speed_Command;
    Hysteresis_Comp_Process_Add_q31(&pSpeed_Loop->Weak_Control_Hcomp, pSpeed_Loop->Voltage_err);
    // Speed_Loop.target_is = LADRC_FirstOrder_Update(&Speed_Loop.Speed_LADRC, Speed_Loop.Speed_Ref, Speed_Loop.Speed_Fb);
    pSpeed_Loop->target_is = Hal_PI_q31(&pSpeed_Loop->Speed_PI, pSpeed_Loop->Speed_Ref - pSpeed_Loop->Speed_Fb);
    if (pSpeed_Loop->Weak_Control_Hcomp.comp_out == 1)
    {
        pSpeed_Loop->Flux_Weak_Id = Hal_PI_q31(&pSpeed_Loop->Weak_Pi, (pSpeed_Loop->Weak_Control_Hcomp.threshold_high - pSpeed_Loop->Voltage_err));
    }
    else
    {
        pSpeed_Loop->Flux_Weak_Id = 0;
    }
    MTPA_Cal_Fixed(pControl,pSpeed_Loop->target_is);
    pSpeed_Loop->target_id = pSpeed_Loop->MTPA_Id + pSpeed_Loop->Flux_Weak_Id + pSpeed_Loop->Speed_Low_Id;
    // Speed_Loop.target_id = Speed_Loop.MTPA_Id;
    arm_sqrt_q31(pCurrent_Loop_Fixed->Ud_Target * pCurrent_Loop_Fixed->Ud_Target + pCurrent_Loop_Fixed->Uq_Target * pCurrent_Loop_Fixed->Uq_Target, &pSpeed_Loop->Vs);
    pSpeed_Loop->Voltage_err = pSpeed_Loop->Vs - pInput->Udc_ADISR * WEAK_VOLTAGE_COMPENSATION;
    if (pSpeed_Loop->target_is > pSpeed_Loop->target_id)
    {
        arm_sqrt_q31(pSpeed_Loop->target_is * pSpeed_Loop->target_is - pSpeed_Loop->target_id * pSpeed_Loop->target_id,
                     &pSpeed_Loop->target_iq);
    }
    else
    {
        pSpeed_Loop->target_iq = 0;
    }
}


void Paramater_update_Fixed(Motor_Control_t *pControl)
{
    Speed_Loop_Fixed_t *pSpeed_Loop = (Speed_Loop_Fixed_t *)pControl->Speed_Loop.pSpeed_Loop;
    Motor_Control_Input_t *pInput = (Motor_Control_Input_t *)&pControl->Input;  
    Motor_Config_t *pMotor_Config = (Motor_Config_t *)pControl->Motor_Config;
    PWM_Freq_Update_Fixed(pControl);
    Observer_Param_Lookup_Updata_Fixed(pControl, pSpeed_Loop->Speed_Ref, pSpeed_Loop->target_is, (MOTOR_WE_BASE/pSpeed_Loop->PWM_CUR_FREQ));
    Current_Para_Updata_Fixed(pControl, pSpeed_Loop->Speed_Ref, (MOTOR_WE_BASE/pSpeed_Loop->PWM_CUR_FREQ));       
    pSpeed_Loop->Speed_PI.kp = Lookup_Table_1D_Linear_q31(pSpeed_Loop->Speed_Ref, &pMotor_Config->Speed_PI_Kp_Lookup);
    pSpeed_Loop->Speed_PI.ki = Lookup_Table_1D_Linear_q31(pSpeed_Loop->Speed_Ref, &pMotor_Config->Speed_PI_Ki_Lookup);
} 

void PWM_Freq_Update_Fixed(Motor_Control_t *pControl)
{
    Speed_Loop_Fixed_t *pSpeed_Loop = (Speed_Loop_Fixed_t *)pControl->Speed_Loop.pSpeed_Loop;
    if (pSpeed_Loop->PWM_CUR_FREQ < PWM_SWITH_FREQ_MIN)
    {
        pSpeed_Loop->PWM_CUR_FREQ = PWM_SWITH_FREQ_MIN;
    }
    else if (pSpeed_Loop->PWM_CUR_FREQ > PWM_SWITH_FREQ_MAX)
    {
        pSpeed_Loop->PWM_CUR_FREQ = PWM_SWITH_FREQ_MAX;
    }
    pSpeed_Loop->PWM_CUR_FREQ = Oblique_Wave_q31(pSpeed_Loop->PWM_SWITCH_FREQ, pSpeed_Loop->PWM_CUR_FREQ, PWM_SWITH_FREQ_STEP, PWM_SWITH_FREQ_STEP);
}


void Power_Derating_Init_Fixed(Motor_Control_t *pControl)
{
    Speed_Loop_Fixed_t *pSpeed_Loop = (Speed_Loop_Fixed_t *)pControl->Speed_Loop.pSpeed_Loop;
    pSpeed_Loop->Derating_Pi.kp = 1e-5f  * (MOTOR_U_BASE * MOTOR_I_BASE);
    pSpeed_Loop->Derating_Pi.ki = 1e-5f  * (MOTOR_U_BASE * MOTOR_I_BASE);
    pSpeed_Loop->Derating_Pi.Kd = 0.1f;
    pSpeed_Loop->Derating_Pi.out_max = 1.0f;
    pSpeed_Loop->Derating_Pi.out_min = 0.0f;
    pSpeed_Loop->Derating_Factor = 1.0f;
}

void Power_Derating_Fixed(Motor_Control_t *pControl, q31_t Bus_Current, q31_t Bus_Voltage, q31_t Power_Limit)
{
    Speed_Loop_Fixed_t *pSpeed_Loop = (Speed_Loop_Fixed_t *)pControl->Speed_Loop.pSpeed_Loop;
    q31_t Error = Power_Limit - (Bus_Current * Bus_Voltage);
    // if((Error) < 10.0f)
    {
        pSpeed_Loop->Derating_Factor = Hal_PI_q31(&pSpeed_Loop->Derating_Pi, Error);
    }
}

void MTPA_Cal_Fixed(Motor_Control_t *pControl, q31_t Is)
{
    Speed_Loop_Fixed_t *pSpeed_Loop = (Speed_Loop_Fixed_t *)pControl->Speed_Loop.pSpeed_Loop;
    Motor_Parameter_Fixed_t *pMotor_Param = (Motor_Parameter_Fixed_t *)pControl->Motor_Config->Motor_Param;
    if (pMotor_Param->Ld_Lq > (1e-4f / MOTOR_L_BASE))
    {
        q31_t MTPA_tmp = 0;
        arm_sqrt_q31(Is * Is * pMotor_Param->Ld_Lq * pMotor_Param->Ld_Lq * 8 
                    + pMotor_Param->Flux_Flux, &MTPA_tmp);
        pSpeed_Loop->MTPA_Id = (MTPA_tmp - pMotor_Param->flux_linkage_wb) / pMotor_Param->Ld_Lq / 4;
    }
    else
    {
        pSpeed_Loop->MTPA_Id = 0;
    }
}
