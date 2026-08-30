#include "Current_Loop_Float.h"
#include "Hal_Math_Float.h"
#include "SVPWM_Float.h"
#include "Observer_Float.h"
#include "Speed_Loop_Float.h"
#include "System_Loop_Float.h"
#include "Motor_Diag.h"
#include "Motor_Control.h"

Current_Loop_Float_t Current_Loop_FLoat;  

const uint8_t rewrite_phase_index[6] = {3, 2, 3, 1, 1, 2};
// const uint8_t rewrite_phase_index[6] = {2, 1, 1, 3, 2, 3};


void Current_Init_Float(Motor_Control_t *pControl)
{
    // Initialization code for current task
    // e.g., setting up filters, initializing variables, etc.
    pControl->Current_Loop.pCurrent_Loop = (void*)&Current_Loop_FLoat;
    OBSERVE_Init(pControl);
    pControl->Current_Loop.Status = MOTOR_IDLE;
    Current_Loop_FLoat.theta = 0.0f;
    Current_Loop_FLoat.FREQ_HZ = MOTOR_CURRENT_LOOP_HZ;
    Current_Loop_FLoat.Loop_time_s = MOTOR_CURRENT_LOOP_CYCLE_TIME_S;
    /* 计算电流环参数 */
    Current_Loop_FLoat.Id_PI.Kd = 0.1f;
    Current_Loop_FLoat.Iq_PI.Kd = 0.1f;

    Current_Loop_FLoat.Speed_fb_1ms = 0.0f;
    Current_Loop_FLoat.theta = 0.0f;
    // Current_Loop_FLoat.Ia_fb = 0.0f;
    // Current_Loop_FLoat.Ib_fb = 0.0f;
    // Current_Loop_FLoat.Ic_fb = 0.0f;
    Current_Loop_FLoat.Id_fb = 0.0f;
    Current_Loop_FLoat.Iq_fb = 0.0f;
    Current_Loop_FLoat.Id_Ref = 0.0f;
    Current_Loop_FLoat.Iq_Ref = 0.0f;
    Current_Loop_FLoat.Id_PI.integral = 0.0f;
    Current_Loop_FLoat.Iq_PI.integral = 0.0f;
    Current_Loop_FLoat.A_Max = 0.0f;
    Current_Loop_FLoat.B_Max = 0.0f;
    Current_Loop_FLoat.C_Max = 0.0f;

    Current_Loop_FLoat.Ia_fb_offset = 0;
    Current_Loop_FLoat.Ib_fb_offset = 0;
    Current_Loop_FLoat.Ic_fb_offset = 0;
    Current_Loop_FLoat.offset_check_cnt = 0;
    Current_Loop_FLoat.Id_PI.integral = 0;
    Current_Loop_FLoat.Iq_PI.integral = 0;

    Current_Loop_FLoat.Dead_Zone_Enable_Flag = 0;
    Current_Loop_FLoat.PWM_FREQ_Coeff = 1.0f;
}                     

void Current_PWM_Switch(uint8_t PWM_Flag)
{
    if (PWM_Flag == PWM_OPEN)
    {
        Bsp_STM32G431_PWM_Enable();
    }
    else if (PWM_Flag == PWM_CLOSE)
    {
        Bsp_STM32G431_PWM_Disable();
    }
    return;
}


inline void Current_Avg_Filt_Float(Motor_Control_t *pControl)
{
    Current_Loop_Float_t *pCurrent_Loop_Float = (Current_Loop_Float_t *)pControl->Current_Loop.pCurrent_Loop;
    Speed_Loop_Float_t *pSpeed_Loop = (Speed_Loop_Float_t *)pControl->Speed_Loop.pSpeed_Loop;
    Motor_Parameter_t *pMotor_Param = (Motor_Parameter_t *)pControl->Motor_Config->Motor_Param;
    struct EffFluxObserver_Parameter *pObserver = (struct EffFluxObserver_Parameter *)pControl->pObserver;
    if (pCurrent_Loop_Float->avg_count >= ((uint32_t)(pCurrent_Loop_Float->FREQ_HZ / pSpeed_Loop->FREQ_Hz)))
    {
        pSpeed_Loop->Speed_Fb = pCurrent_Loop_Float->Speed_fb_1ms / 2 / PI / 
        pMotor_Param->pole_pairs * 60.0f / pCurrent_Loop_Float->avg_count;
        pCurrent_Loop_Float->Speed_fb_1ms = 0;
        pCurrent_Loop_Float->avg_count = 0;
    }
    pCurrent_Loop_Float->avg_count++;
    pCurrent_Loop_Float->Speed_fb_1ms += pObserver->we;
}

void Current_Speed_Switch_Float(Motor_Control_t *pControl)
{
    Current_Loop_Float_t *pCurrent_Loop_Float = (Current_Loop_Float_t *)pControl->Current_Loop.pCurrent_Loop;
    Speed_Loop_Float_t *pSpeed_Loop = (Speed_Loop_Float_t *)pControl->Speed_Loop.pSpeed_Loop;
    Motor_Parameter_t *pMotor_Param = (Motor_Parameter_t *)pControl->Motor_Config->Motor_Param;
    struct EffFluxObserver_Parameter *pObserver = (struct EffFluxObserver_Parameter *)pControl->pObserver;
    switch (pControl->Speed_Loop.Status)
    {
    case SPEED_IDLE:
        pCurrent_Loop_Float->theta = 0;
        break;
    case SPEED_ALIGN:
    {
        break;
    }
    case SPEED_OPEN:
    {
        pCurrent_Loop_Float->theta += pSpeed_Loop->Speed_Ref / 60 * 2 * PI * 
            pMotor_Param->pole_pairs * pCurrent_Loop_Float->Loop_time_s;
        // pCurrent_Loop_Float->theta += 0.002f;
        Limit_2PI(&pCurrent_Loop_Float->theta);
        break;
    }
    case SPEED_SWITCH:
    {
        pCurrent_Loop_Float->theta += pSpeed_Loop->Speed_Ref / 60 * 2 * PI * 
            pMotor_Param->pole_pairs * pCurrent_Loop_Float->Loop_time_s;
        Limit_2PI(&pCurrent_Loop_Float->theta);
        if (MY_ABS(pObserver->theta - pCurrent_Loop_Float->theta) < 0.10)
        {
            pSpeed_Loop->Speed_Switch_Cnt++;
            if (pSpeed_Loop->Speed_Switch_Cnt > 10)
            {
                pSpeed_Loop->Speed_Switch_Flag = 1;
                pCurrent_Loop_Float->theta = pObserver->theta;
            }
        }
        break;
    }
    case SPEED_HIGH:
    case SPEED_MIDDLE:
    case SPEED_LOW:
    {
        pCurrent_Loop_Float->theta = pObserver->theta;
        Limit_2PI(&pCurrent_Loop_Float->theta);
        break;
    }
    default:
        break;
    }
}


/// @brief 三相电流重构，前提：一个桥臂的电流采样值是不准确的，但是其他两个桥臂的电流采样值是准确的
/// @param Ia_fb_raw a相电流原始值
/// @param Ib_fb_raw b相电流原始值
/// @param Ic_fb_raw c相电流原始值
/// @param Ia_fb a相电流重构值
/// @param Ib_fb b相电流重构值
/// @param Ic_fb c相电流重构值
/// @param sector sector值，范围1-6
void Phase_Current_Rewrite_Float(Motor_Control_t *pControl)
{
    Current_Loop_Float_t *pCurrent_Loop_Float = (Current_Loop_Float_t *)pControl->Current_Loop.pCurrent_Loop;
    uint8_t sector_re = rewrite_phase_index[pCurrent_Loop_Float->sector - 1];
    switch (sector_re)
    {
    case 1:
    {
        pCurrent_Loop_Float->Ia_fb = -(pControl->Input.Ib_fb_raw - pCurrent_Loop_Float->Ib_fb_offset +
                                       pControl->Input.Ic_fb_raw - pCurrent_Loop_Float->Ic_fb_offset);
        pCurrent_Loop_Float->Ib_fb = pControl->Input.Ib_fb_raw - pCurrent_Loop_Float->Ib_fb_offset;
        pCurrent_Loop_Float->Ic_fb = pControl->Input.Ic_fb_raw - pCurrent_Loop_Float->Ic_fb_offset;
        break;
    }
    case 2:
    {
        pCurrent_Loop_Float->Ib_fb = -(pControl->Input.Ia_fb_raw - pCurrent_Loop_Float->Ia_fb_offset +
                                       pControl->Input.Ic_fb_raw - pCurrent_Loop_Float->Ic_fb_offset);
        pCurrent_Loop_Float->Ia_fb = pControl->Input.Ia_fb_raw - pCurrent_Loop_Float->Ia_fb_offset;
        pCurrent_Loop_Float->Ic_fb = pControl->Input.Ic_fb_raw - pCurrent_Loop_Float->Ic_fb_offset;
        break;
    }
    case 3:
    {
        pCurrent_Loop_Float->Ic_fb = -(pControl->Input.Ia_fb_raw - pCurrent_Loop_Float->Ia_fb_offset +
                                       pControl->Input.Ib_fb_raw - pCurrent_Loop_Float->Ib_fb_offset);
        pCurrent_Loop_Float->Ia_fb = pControl->Input.Ia_fb_raw - pCurrent_Loop_Float->Ia_fb_offset;
        pCurrent_Loop_Float->Ib_fb = pControl->Input.Ib_fb_raw - pCurrent_Loop_Float->Ib_fb_offset;
        break;
    }
    default:
        pCurrent_Loop_Float->Ia_fb = pControl->Input.Ia_fb_raw - pCurrent_Loop_Float->Ia_fb_offset;
        pCurrent_Loop_Float->Ib_fb = pControl->Input.Ib_fb_raw - pCurrent_Loop_Float->Ib_fb_offset;
        pCurrent_Loop_Float->Ic_fb = pControl->Input.Ic_fb_raw - pCurrent_Loop_Float->Ic_fb_offset;
        break;
    }
    return;
}

/// @brief 检查相电流的最小值和最大值
/// @param A
/// @param B
/// @param C
inline void Phase_Min_Max_Float(Motor_Control_t *pControl, float A, float B, float C)
{
    Current_Loop_Float_t *pCurrent_Loop_Float = (Current_Loop_Float_t *)pControl->Current_Loop.pCurrent_Loop;
    Speed_Loop_Float_t *pSpeed_Loop = (Speed_Loop_Float_t *)pControl->Speed_Loop.pSpeed_Loop;
    if (pCurrent_Loop_Float->Phase_check_cnt > (int)pCurrent_Loop_Float->Phase_check_cnt_THD)
    {
        pCurrent_Loop_Float->Phase_check_cnt = 0;
        pCurrent_Loop_Float->A_Max = 0;
        pCurrent_Loop_Float->C_Max = 0;
        pCurrent_Loop_Float->B_Max = 0;
    }
    else
    {
        pCurrent_Loop_Float->Phase_check_cnt++;
    }
    if (A > pCurrent_Loop_Float->A_Max)
    {
        pCurrent_Loop_Float->A_Max = A;
    }
    if (B > pCurrent_Loop_Float->B_Max)
    {
        pCurrent_Loop_Float->B_Max = B;
    }
    if (C > pCurrent_Loop_Float->C_Max)
    {
        pCurrent_Loop_Float->C_Max = C;
    }
}


void Dead_Zone_Compensation_Float(Motor_Control_t *pControl, float we, float theta)
{
    Current_Loop_Float_t *pCurrent_Loop_Float = (Current_Loop_Float_t *)pControl->Current_Loop.pCurrent_Loop;
    float Ialpha_tmp, Ibeta_tmp;    
    float Ia_pre = 0.0f, Ib_pre = 0.0f, Ic_pre = 0.0f;
    float theta_comp = theta + we * MOTOR_CURRENT_LOOP_CYCLE_TIME_S * (2.0f);

    Ialpha_tmp = pCurrent_Loop_Float->Id_fb * arm_cos_f32(theta_comp) - pCurrent_Loop_Float->Iq_fb * arm_sin_f32(theta_comp);
    Ibeta_tmp = pCurrent_Loop_Float->Id_fb * arm_sin_f32(theta_comp) + pCurrent_Loop_Float->Iq_fb * arm_cos_f32(theta_comp);

    Ia_pre = Ialpha_tmp;
    Ib_pre = -0.5f * Ialpha_tmp + 0.8660254039f * Ibeta_tmp;
    Ic_pre = -(Ia_pre + Ic_pre);
    
    if (pCurrent_Loop_Float->Dead_Zone_Enable_Flag &&
        (pControl->Speed_Loop.Status == SPEED_LOW ||
         pControl->Speed_Loop.Status == SPEED_MIDDLE ||
         pControl->Speed_Loop.Status == SPEED_HIGH))
    {
        if (Ia_pre > MOTOR_DEAD_ZONE_THD)
        {
            pControl->Output.PWM_duty_a = pCurrent_Loop_Float->PWM_duty_a + Dead_TIME_DUTY;
            if (pControl->Output.PWM_duty_a > 1)
            {
                pControl->Output.PWM_duty_a = 1;
            }
        }
        else if (Ia_pre < -MOTOR_DEAD_ZONE_THD)
        {
            pControl->Output.PWM_duty_a = pCurrent_Loop_Float->PWM_duty_a - Dead_TIME_DUTY;
            if (pControl->Output.PWM_duty_a < 0)
            {
                pControl->Output.PWM_duty_a = 0;
            }
        }
        else
        {
            pControl->Output.PWM_duty_a = pCurrent_Loop_Float->PWM_duty_a;
        }
        if (Ib_pre > MOTOR_DEAD_ZONE_THD)
        {
            pControl->Output.PWM_duty_b = pCurrent_Loop_Float->PWM_duty_b + Dead_TIME_DUTY;
            if (pControl->Output.PWM_duty_b > 1)
            {
                pControl->Output.PWM_duty_b = 1;
            }
        }
        else if (Ib_pre < -MOTOR_DEAD_ZONE_THD)
        {
            pControl->Output.PWM_duty_b = pCurrent_Loop_Float->PWM_duty_b - Dead_TIME_DUTY;
            if (pControl->Output.PWM_duty_b < 0)
            {
                pControl->Output.PWM_duty_b = 0;
            }
        }
        else
        {
            pControl->Output.PWM_duty_b = pCurrent_Loop_Float->PWM_duty_b;
        }
        if (Ic_pre > MOTOR_DEAD_ZONE_THD)
        {
            pControl->Output.PWM_duty_c = pCurrent_Loop_Float->PWM_duty_c + Dead_TIME_DUTY;
            if (pControl->Output.PWM_duty_c > 1)
            {
                pControl->Output.PWM_duty_c = 1;
            }
        }
        else if (Ic_pre < -MOTOR_DEAD_ZONE_THD)
        {
            pControl->Output.PWM_duty_c = pCurrent_Loop_Float->PWM_duty_c - Dead_TIME_DUTY;
            if (pControl->Output.PWM_duty_c < 0)
            {
                pControl->Output.PWM_duty_c = 0;
            }
        }
        else
        {
            pControl->Output.PWM_duty_c = pCurrent_Loop_Float->PWM_duty_c;
        }
    }
    else
    {
        pControl->Output.PWM_duty_a = pCurrent_Loop_Float->PWM_duty_a;
        pControl->Output.PWM_duty_b = pCurrent_Loop_Float->PWM_duty_b;
        pControl->Output.PWM_duty_c = pCurrent_Loop_Float->PWM_duty_c;
    }
    return;
}


/// @brief 母线电流重构
/// @param pControl 
inline void MOTOR_Bus_Current_Rewrite_Float(Motor_Control_t *pControl)
{
    Current_Loop_Float_t *pCurrent_Loop_Float = (Current_Loop_Float_t *)pControl->Current_Loop.pCurrent_Loop;
    pCurrent_Loop_Float->Bus_Current = -(pCurrent_Loop_Float->Ia_fb * pCurrent_Loop_Float->PWM_duty_a +
                                        pCurrent_Loop_Float->Ib_fb * pCurrent_Loop_Float->PWM_duty_b +
                                        pCurrent_Loop_Float->Ic_fb * pCurrent_Loop_Float->PWM_duty_c);
    pCurrent_Loop_Float->Bus_Current_LPF = 0.01f * pCurrent_Loop_Float->Bus_Current + 0.99f * pCurrent_Loop_Float->Bus_Current_LPF;
}


void Current_Loop_Run(Motor_Control_t *pControl)
{
    Current_Loop_Float_t *pCurrent_Loop_Float = (Current_Loop_Float_t *)pControl->Current_Loop.pCurrent_Loop;
    Speed_Loop_Float_t *pSpeed_Loop = (Speed_Loop_Float_t *)pControl->Speed_Loop.pSpeed_Loop;
    struct EffFluxObserver_Parameter *pObserver = (struct EffFluxObserver_Parameter *)pControl->pObserver;
    // Encode_ABZ_UpDate();
    Current_Speed_Switch_Float(pControl);
    Current_Avg_Filt_Float(pControl);
    Phase_Current_Rewrite_Float(pControl);

    Phase_Min_Max_Float(pControl, MY_ABS(pCurrent_Loop_Float->Ia_fb), 
                                MY_ABS(pCurrent_Loop_Float->Ib_fb), 
                                 MY_ABS(pCurrent_Loop_Float->Ic_fb));

    arm_clarke_f32(pCurrent_Loop_Float->Ia_fb, pCurrent_Loop_Float->Ib_fb, 
                    &pCurrent_Loop_Float->ialpha_fb, &pCurrent_Loop_Float->ibeta_fb);

    OBSERVE_Updata(pControl, pCurrent_Loop_Float->Ualpha_Ref, pCurrent_Loop_Float->Ubeta_Ref,  \
                    pCurrent_Loop_Float->ialpha_fb, pCurrent_Loop_Float->ibeta_fb);

    pCurrent_Loop_Float->sinVal = arm_sin_f32(pCurrent_Loop_Float->theta);
    pCurrent_Loop_Float->cosVal = arm_cos_f32(pCurrent_Loop_Float->theta);
    arm_park_f32(pCurrent_Loop_Float->ialpha_fb, pCurrent_Loop_Float->ibeta_fb, &pCurrent_Loop_Float->Id_fb,
                 &pCurrent_Loop_Float->Iq_fb, pCurrent_Loop_Float->sinVal, pCurrent_Loop_Float->cosVal);

    pCurrent_Loop_Float->Id_Ref = pSpeed_Loop->target_id;
    pCurrent_Loop_Float->Iq_Ref = pSpeed_Loop->target_iq;
    pCurrent_Loop_Float->Ud_Target = Hal_PI_f32(&pCurrent_Loop_Float->Id_PI, pCurrent_Loop_Float->Id_Ref - pCurrent_Loop_Float->Id_fb);
    pCurrent_Loop_Float->Uq_Target = Hal_PI_f32(&pCurrent_Loop_Float->Iq_PI, pCurrent_Loop_Float->Iq_Ref - pCurrent_Loop_Float->Iq_fb);

    // pCurrent_Loop_Float->Ud_Target = 0.0f;
    // pCurrent_Loop_Float->Uq_Target = 1.0f;

    arm_sqrt_f32(pCurrent_Loop_Float->Id_fb * pCurrent_Loop_Float->Id_fb + pCurrent_Loop_Float->Iq_fb * pCurrent_Loop_Float->Iq_fb, 
                    &pCurrent_Loop_Float->Is_fb);
    arm_inv_park_f32(pCurrent_Loop_Float->Ud_Target, pCurrent_Loop_Float->Uq_Target, &pCurrent_Loop_Float->Ualpha_Ref,
                     &pCurrent_Loop_Float->Ubeta_Ref, pCurrent_Loop_Float->sinVal, pCurrent_Loop_Float->cosVal);
    SVPWM_Calculate_f32(2, pControl->Input.Udc_ADISR, pCurrent_Loop_Float->Ualpha_Ref, pCurrent_Loop_Float->Ubeta_Ref,
                    &pCurrent_Loop_Float->PWM_duty_a, &pCurrent_Loop_Float->PWM_duty_b, &pCurrent_Loop_Float->PWM_duty_c, &pCurrent_Loop_Float->sector);

    pCurrent_Loop_Float->Id_PI.out_max = pControl->Input.Udc_ADISR * WEAK_VOLTAGE_COMPENSATION * 1.02f;
    pCurrent_Loop_Float->Id_PI.out_min = -pCurrent_Loop_Float->Id_PI.out_max;
    if (pCurrent_Loop_Float->Id_PI.out_max > pCurrent_Loop_Float->Ud_Target)
    {
        arm_sqrt_f32(pCurrent_Loop_Float->Id_PI.out_max * pCurrent_Loop_Float->Id_PI.out_max -
                         pCurrent_Loop_Float->Ud_Target * pCurrent_Loop_Float->Ud_Target,
                     &pCurrent_Loop_Float->Iq_PI.out_max);
    }
    else
    {
        pCurrent_Loop_Float->Iq_PI.out_max = 0.0f;
    }
    pCurrent_Loop_Float->Iq_PI.out_min = -pCurrent_Loop_Float->Iq_PI.out_max;
    Dead_Zone_Compensation_Float(pControl, pObserver->we, pObserver->theta);
    MOTOR_Bus_Current_Rewrite_Float(pControl);
    pControl->Output.PWM_HZ_Coeff = pCurrent_Loop_Float->PWM_FREQ_Coeff;
}


void Current_Para_Updata_Float(Motor_Control_t *pControl, float speed, float Ts)
{
    Current_Loop_Float_t *pCurrent_Loop_Float = (Current_Loop_Float_t *)pControl->Current_Loop.pCurrent_Loop;
    Speed_Loop_Float_t *pSpeed_Loop = (Speed_Loop_Float_t *)pControl->Speed_Loop.pSpeed_Loop;
    Motor_Config_t *Motor_Config = (Motor_Config_t *)pControl->Motor_Config;
    pCurrent_Loop_Float->Loop_time_s = Ts;
    pCurrent_Loop_Float->FREQ_HZ = 1.0f / Ts;
    pCurrent_Loop_Float->PWM_FREQ_Coeff = pCurrent_Loop_Float->FREQ_HZ / MOTOR_CURRENT_LOOP_HZ;
    pCurrent_Loop_Float->Id_PI.kp = Lookup_Table_1D_Linear_f32(pSpeed_Loop->Speed_Fb_1s, &Motor_Config->ID_PI_Kp_Lookup)
                                     * pControl->Output.PWM_HZ_Coeff;
    pCurrent_Loop_Float->Iq_PI.ki = pCurrent_Loop_Float->Id_PI.ki = Lookup_Table_1D_Linear_f32(pSpeed_Loop->Speed_Ref,
                                     &Motor_Config->ID_PI_Ki_Lookup);
    pCurrent_Loop_Float->Iq_PI.kp = Lookup_Table_1D_Linear_f32(pSpeed_Loop->Speed_Fb_1s, &Motor_Config->IQ_PI_Kp_Lookup) 
                                     * pControl->Output.PWM_HZ_Coeff;
    pCurrent_Loop_Float->Phase_check_cnt_THD = (uint32_t)(pCurrent_Loop_Float->FREQ_HZ / (pSpeed_Loop->FREQ_Hz) * 60);
}



void MOTOR_IDLE_TASK_Float(Motor_Control_t *pControl)
{
    Current_Loop_Float_t *pCurrent_Loop_Float = (Current_Loop_Float_t *)pControl->Current_Loop.pCurrent_Loop;
    System_Loop_FLoat_t *pSystem_Loop = (System_Loop_FLoat_t *)pControl->System_Loop.pSystem_Loop;
    // Code for MOTOR_IDLE state
    if (pSystem_Loop->Run_flag == 1)
    {
        pControl->Current_Loop.Status = MOTOR_READY;
    }
    else
    {
        Current_PWM_Switch(PWM_CLOSE);
        pControl->Current_Loop.Status = MOTOR_IDLE;
    }
    return;
}

void MOTOR_READY_TASK_Float(Motor_Control_t *pControl)
{
    // Code for MOTOR_READY state
    Current_Loop_Float_t *pCurrent_Loop_Float = (Current_Loop_Float_t *)pControl->Current_Loop.pCurrent_Loop;
    Speed_Loop_Float_t *pSpeed_Loop = (Speed_Loop_Float_t *)pControl->Speed_Loop.pSpeed_Loop;
    System_Loop_FLoat_t *pSystem_Loop = (System_Loop_FLoat_t *)pControl->System_Loop.pSystem_Loop;
    if (pSystem_Loop->Run_flag == 1)
    {
        if (Motor_Diag_Fault_Flag != 0)
        {
            pControl->Current_Loop.Status = MOTOR_FAULT;
        }
        else
        {
            Current_Init_Float(pControl);
            pControl->Current_Loop.Status = MOTOR_OFFSET_CHECK;
        }
    }
    else
    {
        pControl->Current_Loop.Status = MOTOR_WAIT;
    }
    return;
}

void MOTOR_OFFSET_CHECK_TASK_Float(Motor_Control_t *pControl)
{
    Current_Loop_Float_t *pCurrent_Loop_Float = (Current_Loop_Float_t *)pControl->Current_Loop.pCurrent_Loop;
    Speed_Loop_Float_t *pSpeed_Loop = (Speed_Loop_Float_t *)pControl->Speed_Loop.pSpeed_Loop;
    System_Loop_FLoat_t *pSystem_Loop = (System_Loop_FLoat_t *)pControl->System_Loop.pSystem_Loop;
    if (pSystem_Loop->Run_flag == 1)
    {
        if (Motor_Diag_Fault_Flag != 0)
        {
            pControl->Current_Loop.Status = MOTOR_FAULT;
        }
        else
        {
            // Code for MOTOR_OFFSET_CHECK state
            pCurrent_Loop_Float->offset_check_cnt++;
            pCurrent_Loop_Float->Ia_fb_offset += pControl->Input.Ia_fb_raw; // Accumulate ADC1 injected channel 1 value
            pCurrent_Loop_Float->Ib_fb_offset += pControl->Input.Ib_fb_raw; // Accumulate ADC2 injected channel 1 value
            pCurrent_Loop_Float->Ic_fb_offset += pControl->Input.Ic_fb_raw; // Accumulate ADC1 injected channel 2 value
            if (pCurrent_Loop_Float->offset_check_cnt >= MOTOR_ADC_OFFSET_SAMPLE_CNT)
            {
                pCurrent_Loop_Float->Ia_fb_offset /= (float)pCurrent_Loop_Float->offset_check_cnt; // Calculate average for ADC1 injected channel 1
                pCurrent_Loop_Float->Ib_fb_offset /= (float)pCurrent_Loop_Float->offset_check_cnt; // Calculate average for ADC2 injected channel 1
                pCurrent_Loop_Float->Ic_fb_offset /= (float)pCurrent_Loop_Float->offset_check_cnt; // Calculate average for ADC1 injected channel 2
                if (pCurrent_Loop_Float->Ia_fb_offset > MOTOR_PHASE_LOCK_THRESHOLD || pCurrent_Loop_Float->Ia_fb_offset < -MOTOR_PHASE_LOCK_THRESHOLD ||
                    pCurrent_Loop_Float->Ib_fb_offset > MOTOR_PHASE_LOCK_THRESHOLD || pCurrent_Loop_Float->Ib_fb_offset < -MOTOR_PHASE_LOCK_THRESHOLD ||
                    pCurrent_Loop_Float->Ic_fb_offset > MOTOR_PHASE_LOCK_THRESHOLD || pCurrent_Loop_Float->Ic_fb_offset < -MOTOR_PHASE_LOCK_THRESHOLD)
                {
                    Motor_Diag_Fault_Flag |= MOTOR_CURRENT_OFFSET_OVER_FLAG_MASK;
                    pControl->Current_Loop.Status = MOTOR_FAULT;
                }
                else
                {
                    pControl->Current_Loop.Status = MOTOR_RUN;
                    Current_PWM_Switch(PWM_OPEN);
                }
            }
        }
    }
    else
    {
        pControl->Current_Loop.Status = MOTOR_WAIT;
    }
    return;
}

void MOTOR_RUN_TASK_Float(Motor_Control_t *pControl)
{
    Current_Loop_Float_t *pCurrent_Loop_Float = (Current_Loop_Float_t *)pControl->Current_Loop.pCurrent_Loop;
    Speed_Loop_Float_t *pSpeed_Loop = (Speed_Loop_Float_t *)pControl->Speed_Loop.pSpeed_Loop;
    System_Loop_FLoat_t *pSystem_Loop = (System_Loop_FLoat_t *)pControl->System_Loop.pSystem_Loop;
    /* Code for MOTOR_RUN state */
    if (pSystem_Loop->Run_flag == 0 && pSpeed_Loop->Speed_Fb < 3000.0f)
    {
        pControl->Current_Loop.Status = MOTOR_WAIT;
    }
    else
    {
        if (Motor_Diag_Fault_Flag != 0)
        {
            pControl->Current_Loop.Status = MOTOR_FAULT;
        }
        else
        {
            Current_Loop_Run(pControl);
            Bsp_STM32G431_PWM_SetDuty();
        }
    }
    return;
}

void MOTOR_FAULT_TASK_Float(Motor_Control_t *pControl)
{
    Current_Loop_Float_t *pCurrent_Loop_Float = (Current_Loop_Float_t *)pControl->Current_Loop.pCurrent_Loop;
    Speed_Loop_Float_t *pSpeed_Loop = (Speed_Loop_Float_t *)pControl->Speed_Loop.pSpeed_Loop;
    /* Code for MOTOR_FAULT state */
    Current_PWM_Switch(PWM_CLOSE);
    if (Motor_Diag_Fault_Flag == 0)
    {
        pControl->Current_Loop.Status = MOTOR_WAIT;
        pCurrent_Loop_Float->Motor_Wait_Cnt = 0;
    }
    else
    {
        pControl->Current_Loop.Status = MOTOR_FAULT;
    }
}

void MOTOR_WAIT_TASK_Float(Motor_Control_t *pControl)
{
    Current_Loop_Float_t *pCurrent_Loop_Float = (Current_Loop_Float_t *)pControl->Current_Loop.pCurrent_Loop;
    Speed_Loop_Float_t *pSpeed_Loop = (Speed_Loop_Float_t *)pControl->Speed_Loop.pSpeed_Loop;
    /* Code for MOTOR_WAIT state */
    Current_PWM_Switch(PWM_CLOSE);
    pCurrent_Loop_Float->Motor_Wait_Cnt++;
    if (pCurrent_Loop_Float->Motor_Wait_Cnt > ((uint32_t)(pCurrent_Loop_Float->FREQ_HZ * 5))) // 5s
    {
        pControl->Current_Loop.Status = MOTOR_IDLE;
        pCurrent_Loop_Float->Motor_Wait_Cnt = 0;
    }
    else
    {
        pControl->Current_Loop.Status = MOTOR_WAIT;
    }
    return;
}

