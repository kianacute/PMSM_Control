#include "arm_math.h"
#include "Current_Task.h"
#include "Hal_Math.h"
#include "SVPWM.h"
#include "adc.h"
#include "tim.h"
#include "Motor_parameter.h"
#include "Observer.h"
#include "usart.h"
#include "speed_ctrl.h"
#include "Vofa.h"
#include "system.h"

extern MOTOR_t PMSM_42JS;
extern struct SMO_Parameter SMO_OB;
extern struct NonFluxObserver_Parameter NonFlux_OB;
extern struct Encoder_Parameter Encode_ABZ;
extern struct HFSWInjection_Parameter HFSW_OB;
extern Speed_Ctrl_t Speed_Ctrl;
extern SYSTEM_t System;

const uint8_t rewrite_phase_index[6] = {3, 2, 3, 1, 1, 2};
// const uint8_t rewrite_phase_index[6] = {2, 1, 1, 3, 2, 3};

adc_adjustment_t adc_adjustment = {0};

Current_Task_t Current_Task =
    {
        .FREQ_HZ = 20000,
        .Loop_time_s = 1.0f / 20000.0f,
        .pMotor = &PMSM_42JS,
        .Id_PI = {
            .kp = 1.0f,
            .ki = 0.1f,
            .integral = 0.0f,
            .prev_error = 0.0f,
            .out_min = -100.0f,
            .out_max = 100.0f},
        .Iq_PI = {.kp = 1.0f, .ki = 0.1f, .integral = 0.0f, .prev_error = 0.0f, .out_min = -100.0f, .out_max = 100.0f},
};

void Current_Task_Init(void)
{
    // Initialization code for current task
    // e.g., setting up filters, initializing variables, etc.
    Current_Task.theta = 0.0f;
    Current_Task.pMotor = &PMSM_42JS;
    float lp = 10.0f;
    /* 计算电流环参数 */
    Current_Task.Id_PI.kp = Current_Task.pMotor->phase_inductance_d * Current_Task.Loop_time_s * 2 * PI / lp;
    Current_Task.Iq_PI.kp = Current_Task.pMotor->phase_inductance_q * Current_Task.Loop_time_s * 2 * PI / lp;
    Current_Task.Id_PI.ki = Current_Task.pMotor->phase_resistance_ohm * 2 * PI / lp; /* 离散化，其中开关周期被抵消 */
    Current_Task.Iq_PI.ki = Current_Task.pMotor->phase_resistance_ohm * 2 * PI / lp; /* 离散化，其中开关周期被抵消 */
    Current_Task.Id_PI.Kd = 0.1f;
    Current_Task.Iq_PI.Kd = 0.1f;
    Speed_Ctrl.Speed_Switch_Cnt = 0;
    Speed_Ctrl.Speed_Switch_Flag = 0;
    Current_Task.count = 0;
    Current_Task.Speed_fb_1ms = 0.0f;
    SMO_Observer_Init();
    Encode_ABZ_Init();
    Nonlinear_FluxObserver_Init();
    HFSWInjection_Init();
}


int32_t MOTOR_IDLE_TASK(void);
int32_t MOTOR_READY_TASK(void);
int32_t MOTOR_OFFSET_CHECK_TASK(void);
int32_t MOTOR_RUN_TASK(void);
void Current_Task_Switch(void);

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        adc_adjustment.ADC_j1 = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1); // Read injected channel value
        adc_adjustment.ADC_j2 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1); // Read another injected channel value
        adc_adjustment.ADC_j3 = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2); // Read another injected channel value
        adc_adjustment.ADC_j4 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2); // Read another injected channel value
        Current_Task_Switch();    
        Current_Task.Udc_ADISR = Current_Task.Udc_ADISR * 0.9f + adc_adjustment.ADC_j4 * 1.1f * 0.019842f * 0.1f; // Update Udc_1ms with the latest ADC value
    }
    else if (hadc->Instance == ADC2)
    {
        
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
void Phase_Current_Rewrite(float32_t Ia_fb_raw, float32_t Ib_fb_raw, float32_t Ic_fb_raw,
                           float32_t *Ia_fb, float32_t *Ib_fb, float32_t *Ic_fb, uint8_t sector)
{
    uint8_t sector_re = rewrite_phase_index[sector - 1];
    switch (sector_re)
    {
    case 1:
    {
        *Ia_fb = -(Ib_fb_raw + Ic_fb_raw);
        *Ib_fb = Ib_fb_raw;
        *Ic_fb = Ic_fb_raw;
        break;
    }
    case 2:
    {
        *Ib_fb = -(Ia_fb_raw + Ic_fb_raw);
        *Ia_fb = Ia_fb_raw;
        *Ic_fb = Ic_fb_raw;
        break;
    }
    case 3:
    {
        *Ic_fb = -(Ia_fb_raw + Ib_fb_raw);
        *Ia_fb = Ia_fb_raw;
        *Ib_fb = Ib_fb_raw;
        break;
    }
    default:
        *Ia_fb = Ia_fb_raw;
        *Ib_fb = Ib_fb_raw;
        *Ic_fb = Ic_fb_raw;
        break;
    }
    return;
}

/// @brief 这种死区补偿方式会引入振动，总之就是更加不稳定
/// @param Id   
/// @param Iq   
/// @param we   
/// @param theta    
/// @param Duty_A_Raw   
/// @param Duty_B_Raw   
/// @param Duty_C_Raw   
/// @param Duty_A_Comp  
/// @param Duty_B_Comp  
/// @param Duty_C_Comp  
void Dead_Zone_Compensation(float Id, float Iq, float we, float theta,
                            float Duty_A_Raw, float Duty_B_Raw, float Duty_C_Raw,
                            float *Duty_A_Comp, float *Duty_B_Comp, float *Duty_C_Comp)
{
    float Ialpha_tmp, Ibeta_tmp;
    float theta_comp = theta + we * MOTOR_CURRENT_LOOP_CYCLE_TIME_S * (2.0f);
    arm_inv_park_f32(Id, Iq, &Ialpha_tmp, &Ibeta_tmp, arm_sin_f32(theta_comp), arm_cos_f32(theta_comp));
    float Ia_pre, Ib_pre, Ic_pre;

    arm_inv_clarke_f32(Ialpha_tmp, Ibeta_tmp, &Ia_pre, &Ib_pre);
    if (we > (2000000/60*6.24f*4))
    {
        if (Ia_pre > 0.1f)
        {
            *Duty_A_Comp = Duty_A_Raw + Dead_TIME_DUTY;
            if (*Duty_A_Comp > 1)
            {
                *Duty_A_Comp = 1;
            }
        }
        else if (Ia_pre < -0.1f)
        {
            *Duty_A_Comp = Duty_A_Raw - Dead_TIME_DUTY;
            if (*Duty_A_Comp < 0)
            {
                *Duty_A_Comp = 0;
            }
        }
        else
        {
            *Duty_A_Comp = Duty_A_Raw;
        }
        if (Ib_pre > 0.1f)
        {
            *Duty_B_Comp = Duty_B_Raw + Dead_TIME_DUTY;
            if (*Duty_B_Comp > 1)
            {
                *Duty_B_Comp = 1;
            }
        }
        else if (Ib_pre < -0.1f)
        {
            *Duty_B_Comp = Duty_B_Raw - Dead_TIME_DUTY;
            if (*Duty_B_Comp < 0)
            {
                *Duty_B_Comp = 0;
            }
        }
        else 
        {
            *Duty_B_Comp = Duty_B_Raw;
        }
        if (Ic_pre > 0.1f)
        {
            *Duty_C_Comp = Duty_C_Raw + Dead_TIME_DUTY;
            if (*Duty_C_Comp > 1)
            {
                *Duty_C_Comp = 1;
            }
        }
        else if (Ic_pre < -0.1f)
        {
            *Duty_C_Comp = Duty_C_Raw - Dead_TIME_DUTY;
            if (*Duty_C_Comp < 0)
            {
                *Duty_C_Comp = 0;
            }
        }
        else 
        {
            *Duty_C_Comp = Duty_C_Raw;
        }
    }
    else
    {
        *Duty_A_Comp = Duty_A_Raw;
        *Duty_B_Comp = Duty_B_Raw;
        *Duty_C_Comp = Duty_C_Raw;
    }
    return;
}

void Speed_Switch(void)
{
    switch (Speed_Ctrl.spd_ctrl_state)
    {
    case SPEED_CTRL_IDLE:
        Current_Task.theta = 0;
        break;
    case SPEED_CTRL_ALIGN:
    {
        // theta = 0;
        extern uint8_t align_done;
        if (align_done == 1)
        {
            Encode_ABZ_Get_Offset();
        }
        // Current_Task.theta = Limit_2PI(HFSW_OB.tPLL.theta);
    }
    case SPEED_CTRL_OPEN:
    {
        Current_Task.theta += Speed_Ctrl.Speed_Ref / 60 * 2 * PI * Current_Task.pMotor->pole_pairs * Current_Task.Loop_time_s;
        Current_Task.theta = Limit_2PI(Current_Task.theta);
        break;
    }
    case SPEED_CTRL_SWITCH:
    {
        Current_Task.theta += Speed_Ctrl.Speed_Ref / 60 * 2 * PI * Current_Task.pMotor->pole_pairs * Current_Task.Loop_time_s;
        Current_Task.theta = Limit_2PI(Current_Task.theta);
        if (my_abs(NonFlux_OB.tPLL.theta - Current_Task.theta) < 0.10)
        {
            Speed_Ctrl.Speed_Switch_Cnt++;
            if (Speed_Ctrl.Speed_Switch_Cnt > 10)
            {
                Speed_Ctrl.Speed_Switch_Flag = 1;
                Current_Task.theta = NonFlux_OB.tPLL.theta;
            }
        }
        break;
    }
    case SPEED_CTRL_RUN:
    {
        Current_Task.theta = Limit_2PI(NonFlux_OB.tPLL.theta);
    }
    default:
    }
}

void Current_Task_Run(float32_t ia_fb, float32_t ib_fb, float32_t ic_fb, float32_t *PWM_duty_a, float32_t *PWM_duty_b, float32_t *PWM_duty_c, float Udc)
{
    if (Current_Task.avg_count >= (Current_Task.FREQ_HZ / Speed_Ctrl.FREQ_Hz))
    {
        Current_Task.avg_count = 0;
        Speed_Ctrl.Speed_Fb = Current_Task.Speed_fb_1ms / 2 / PI / Current_Task.pMotor->pole_pairs * 60 / 20.0f;
        Current_Task.Speed_fb_1ms = 0;
    }
    Current_Task.avg_count++;
    Current_Task.Speed_fb_1ms += NonFlux_OB.tPLL.we;
    // Speed_Ctrl.Speed_Fb = SMO_OB.tPLL.we / 2 / PI / Current_Task.pMotor->pole_pairs * 60;
    Encode_ABZ_UpDate();

    // if ((HFSW_OB.hfj_cnt % HFSW_OB.PSR) >= HFSW_OB.PSR / 2)
    // {
    //     HFSW_OB.U_hfj = 2.4f;
    // }
    // else
    // {
    //     HFSW_OB.U_hfj = -2.4f;
    // }
    // HFSW_OB.hfj_cnt++;
    Speed_Switch();
    Phase_Current_Rewrite(ia_fb, ib_fb, ic_fb, &Current_Task.Ia_fb, &Current_Task.Ib_fb, &Current_Task.Ic_fb, Current_Task.sector);
    arm_clarke_f32(Current_Task.Ia_fb, Current_Task.Ib_fb, &Current_Task.ialpha_fb, &Current_Task.ibeta_fb);
    // SMO_Observer(&SMO_OB, Current_Task.Ualpha_Ref, Current_Task.Ubeta_Ref, Current_Task.ialpha_fb, Current_Task.ibeta_fb);
    Nonlinear_FluxObserver_Update(&NonFlux_OB, Current_Task.Ualpha_Ref, Current_Task.Ubeta_Ref,
                                  Current_Task.ialpha_fb, Current_Task.ibeta_fb);

    // HFSWInjection_NSF(&HFSW_OB, Current_Task.Id_fb);
    // HFSWInjection_Update(&HFSW_OB, Current_Task.ialpha_fb, Current_Task.ibeta_fb, HFSW_OB.U_hfj);

    Current_Task.sinVal = arm_sin_f32(Current_Task.theta);
    Current_Task.cosVal = arm_cos_f32(Current_Task.theta);
    arm_park_f32(Current_Task.ialpha_fb, Current_Task.ibeta_fb, &Current_Task.Id_fb,
                 &Current_Task.Iq_fb, Current_Task.sinVal, Current_Task.cosVal);

    Current_Task.Id_Ref = Speed_Ctrl.target_id;
    Current_Task.Iq_Ref = Speed_Ctrl.target_iq;
    Current_Task.Ud_Target = Hal_PI_f32(&Current_Task.Id_PI, Current_Task.Id_Ref - Current_Task.Id_fb);
    Current_Task.Uq_Target = Hal_PI_f32(&Current_Task.Iq_PI, Current_Task.Iq_Ref - Current_Task.Iq_fb);

    arm_inv_park_f32(Current_Task.Ud_Target, Current_Task.Uq_Target, &Current_Task.Ualpha_Ref,
                     &Current_Task.Ubeta_Ref, Current_Task.sinVal, Current_Task.cosVal);
    SVPWM_Calculate(1, Udc, Current_Task.Ualpha_Ref, Current_Task.Ubeta_Ref,
                    &Current_Task.PWM_duty_a, &Current_Task.PWM_duty_b, &Current_Task.PWM_duty_c, &Current_Task.sector);

    Current_Task.Id_PI.out_max = Udc * WEAK_VOLTAGE_COMPENSATION * 1.02f;
    Current_Task.Id_PI.out_min = -Current_Task.Id_PI.out_max;
    arm_sqrt_f32(Current_Task.Id_PI.out_max * Current_Task.Id_PI.out_max - Current_Task.Ud_Target * Current_Task.Ud_Target,
                 &Current_Task.Iq_PI.out_max);
    Current_Task.Iq_PI.out_min = -Current_Task.Iq_PI.out_max;
    Dead_Zone_Compensation(Current_Task.Id_fb, Current_Task.Iq_fb, SMO_OB.tPLL.we, SMO_OB.tPLL.theta,
                           Current_Task.PWM_duty_a, Current_Task.PWM_duty_b, Current_Task.PWM_duty_c,
                           &Current_Task.PWM_Duty_A_Comp, &Current_Task.PWM_Duty_B_Comp, &Current_Task.PWM_Duty_C_Comp);
    Current_Task.PWM_Duty_A_Comp *= PWM_MAX_DUTY;
    Current_Task.PWM_Duty_B_Comp *= PWM_MAX_DUTY;
    Current_Task.PWM_Duty_C_Comp *= PWM_MAX_DUTY;
}

void Current_PWM_Switch(uint8_t PWM_Flag)
{
    if (PWM_Flag == PWM_OPEN)
    {
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
        // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);   /* 通道4的PWM必须开着，否则无法触发ADC注入中断 */
    }
    else if (PWM_Flag == PWM_CLOSE)
    {
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
        HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
        // HAL_TIM_PWM_Stop( &htim1, TIM_CHANNEL_4);
    }
    return;
}

void Current_Task_Switch(void)
{
    // Code to switch current task states
    if (System.system_state == SYSTEM_RUN)
    {
        switch (Current_Task.Motor_State)
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
            default:
                break;
        }
    }
    else
    {
        Current_Task.Motor_State = MOTOR_IDLE;
        Current_PWM_Switch(PWM_CLOSE);
    }
    Current_Task.count++;
}

int32_t MOTOR_IDLE_TASK(void)
{
    // Code for MOTOR_IDLE state
    if (System.system_state == SYSTEM_RUN)
    {
        Current_Task.Motor_State = MOTOR_READY;
        SMO_OB.tPLL.PLL_PI.integral = 0;
        NonFlux_OB.tPLL.PLL_PI.integral = 0;
        Current_Task.Id_PI.integral = 0;
        Current_Task.Iq_PI.integral = 0;
        Current_Task_Init();
    }
    else
    {
        Current_PWM_Switch(PWM_CLOSE);
        Current_Task.Motor_State = MOTOR_IDLE;
    }
    return 0;
}

int32_t MOTOR_READY_TASK(void)
{
    // Code for MOTOR_READY state

    if (System.system_state == SYSTEM_RUN)
    {
        Current_Task.Motor_State = MOTOR_OFFSET_CHECK;
        adc_adjustment.ADC_A_offset = 0;
        adc_adjustment.ADC_B_offset = 0;
        adc_adjustment.ADC_C_offset = 0;
        adc_adjustment.Motor_ADC_OFFSET_CNT = 0;
        Current_Task.Id_PI.integral = 0;
        Current_Task.Iq_PI.integral = 0;
    }
    else
    {
        Current_Task.Motor_State = MOTOR_IDLE;
    }
    return 0;
}

int32_t MOTOR_OFFSET_CHECK_TASK(void)
{
    // Code for MOTOR_OFFSET_CHECK state
    adc_adjustment.Motor_ADC_OFFSET_CNT++;
    adc_adjustment.ADC_A_offset += adc_adjustment.ADC_j1; // Accumulate ADC1 injected channel 1 value
    adc_adjustment.ADC_B_offset += adc_adjustment.ADC_j2; // Accumulate ADC2 injected channel 1 value
    adc_adjustment.ADC_C_offset += adc_adjustment.ADC_j3; // Accumulate ADC1 injected channel 2 value
    if (adc_adjustment.Motor_ADC_OFFSET_CNT >= MOTOR_ADC_OFFSET_SAMPLE_CNT)
    {
        adc_adjustment.ADC_A_offset /= adc_adjustment.Motor_ADC_OFFSET_CNT; // Calculate average for ADC1 injected channel 1
        adc_adjustment.ADC_B_offset /= adc_adjustment.Motor_ADC_OFFSET_CNT; // Calculate average for ADC2 injected channel 1
        adc_adjustment.ADC_C_offset /= adc_adjustment.Motor_ADC_OFFSET_CNT; // Calculate average for ADC1 injected channel 2
        adc_adjustment.Motor_ADC_OFFSET_CNT = 0;
        Current_Task.Motor_State = MOTOR_RUN;
        Current_PWM_Switch(PWM_OPEN);
    }
    return 0;
}

int32_t MOTOR_RUN_TASK(void)
{
    /* Code for MOTOR_RUN state */
    Current_Task.Ia_fb_raw = (adc_adjustment.ADC_j1 - adc_adjustment.ADC_A_offset) * ADC_OPAMP_GAIN; // Adjust ADC1 injected channel 1 value
    Current_Task.Ib_fb_raw = (adc_adjustment.ADC_j2 - adc_adjustment.ADC_B_offset) * ADC_OPAMP_GAIN; // Adjust ADC2 injected channel 1 value
    Current_Task.Ic_fb_raw = (adc_adjustment.ADC_j3 - adc_adjustment.ADC_C_offset) * ADC_OPAMP_GAIN; // Adjust ADC1 injected channel 2 value
    Current_Task_Run(Current_Task.Ia_fb_raw, Current_Task.Ib_fb_raw, Current_Task.Ic_fb_raw,
                     &Current_Task.PWM_duty_a, &Current_Task.PWM_duty_b, &Current_Task.PWM_duty_c, Current_Task.Udc_ADISR);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, Current_Task.PWM_Duty_A_Comp); // Set PWM duty cycle for Channel 1
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, Current_Task.PWM_Duty_B_Comp); // Set PWM duty cycle for Channel 2
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, Current_Task.PWM_Duty_C_Comp); // Set PWM duty cycle for Channel 3

    return 0;
}