#include "arm_math.h"
#include "Current_Loop.h"
#include "Hal_Math.h"
#include "SVPWM.h"
#include "Motor_Config.h"
#include "Observer.h"
#include "Speed_Loop.h"
#include "System_Loop.h"

extern Motor_Config_t PMSM_42JS_Config;
extern struct SMO_Parameter SMO_OB;
extern struct NonFluxObserver_Parameter NonFlux_OB;
extern struct Encoder_Parameter Encode_ABZ;
extern struct HFSWInjection_Parameter HFSW_OB;
extern Speed_Loop_t Speed_Loop;
extern SYSTEM_t System;
extern struct EMF_Cal_Parameter EMF_Cal;



Current_Loop_Input_t Current_Loop_Input;
Current_Loop_Output_t Current_Loop_Output;

const uint8_t rewrite_phase_index[6] = {3, 2, 3, 1, 1, 2};
// const uint8_t rewrite_phase_index[6] = {2, 1, 1, 3, 2, 3};

Current_Loop_t Current_Loop;

void Current_Loop_Init(void)
{
    // Initialization code for current task
    // e.g., setting up filters, initializing variables, etc.
    Current_Loop.theta = 0.0f;
    Current_Loop.pMotor = &PMSM_42JS_Config;
    Current_Loop.FREQ_HZ = 20000;
    Current_Loop.Loop_time_s = 1.0f / 20000.0f;
    /* 计算电流环参数 */
    Current_Loop.Id_PI.Kd = 0.1f;
    Current_Loop.Iq_PI.Kd = 0.1f;

    Speed_Loop.Speed_Switch_Cnt = 0;
    Speed_Loop.Speed_Switch_Flag = 0;
    Current_Loop.Loop_count = 0;
    Current_Loop.Speed_fb_1ms = 0.0f;
    SMO_Observer_Init();
    Encode_ABZ_Init();
    Nonlinear_FluxObserver_Init();
    HFSWInjection_Init();
}


int32_t MOTOR_IDLE_TASK(void);
int32_t MOTOR_READY_TASK(void);
int32_t MOTOR_OFFSET_CHECK_TASK(void);
int32_t MOTOR_RUN_TASK(void);
void Current_Loop_Switch(void);


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


void Phase_Min_Max(float A, float B, float C)
{
    if( Current_Loop.Phase_check_cnt > (int)(20000/(Speed_Loop.Speed_Ref/60.0f*4.0f + 1)) )
    {
        Current_Loop.Phase_check_cnt = 0;
        Current_Loop.A_Max = 0;
        Current_Loop.A_Min = 0; 
        Current_Loop.B_Max = 0;
        Current_Loop.B_Min = 0;
        Current_Loop.C_Max = 0;
        Current_Loop.C_Min = 0;
    }
    else
    {
        Current_Loop.Phase_check_cnt++;
    }
    if (A > Current_Loop.A_Max)
    {
        Current_Loop.A_Max = A;
    }
    if (A < Current_Loop.A_Min)
    {
        Current_Loop.A_Min = A;
    }
    if (B > Current_Loop.B_Max)
    {
        Current_Loop.B_Max = B;
    }
    if (B < Current_Loop.B_Min)
    {
        Current_Loop.B_Min = B;
    }
    if (C > Current_Loop.C_Max)
    {
        Current_Loop.C_Max = C;
    }
    if (C < Current_Loop.C_Min)
    {
        Current_Loop.C_Min = C;
    }
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

float angle_comp;

void Speed_Switch(void)
{
    switch (Speed_Loop.spd_ctrl_state)
    {
    case Speed_Loop_IDLE:
        Current_Loop.theta = 0;
        break;
    case Speed_Loop_ALIGN:
    {
        // theta = 0;
        extern uint8_t align_done;
        if (align_done == 1)
        {
            Encode_ABZ_Get_Offset();
        }
        // Current_Loop.theta = Limit_2PI(HFSW_OB.tPLL.theta);
        break;
    }
    case Speed_Loop_OPEN:
    {
        Current_Loop.theta += Speed_Loop.Speed_Ref / 60 * 2 * PI * Current_Loop.pMotor->motor_param->pole_pairs * Current_Loop.Loop_time_s;
        Current_Loop.theta = Limit_2PI(Current_Loop.theta);
        break;
    }
    case Speed_Loop_SWITCH:
    {
        Current_Loop.theta += Speed_Loop.Speed_Ref / 60 * 2 * PI * Current_Loop.pMotor->motor_param->pole_pairs * Current_Loop.Loop_time_s;
        Current_Loop.theta = Limit_2PI(Current_Loop.theta);
        if (my_abs(SMO_OB.tPLL.theta - Current_Loop.theta) < 0.10)
        {
            Speed_Loop.Speed_Switch_Cnt++;
            if (Speed_Loop.Speed_Switch_Cnt > 10)
            {
                Speed_Loop.Speed_Switch_Flag = 1;
                Current_Loop.theta = SMO_OB.tPLL.theta;
            }
        }
        break;
    }
    case Speed_Loop_RUN:
    {
        Current_Loop.theta = Limit_2PI(NonFlux_OB.tPLL.theta + angle_comp);
    }
    default:
    }
}

void Current_Avg_Filt()
{
    if (Current_Loop.avg_count >= (Current_Loop.FREQ_HZ / Speed_Loop.FREQ_Hz))
    {
        Current_Loop.avg_count = 0;
        Speed_Loop.Speed_Fb = Current_Loop.Speed_fb_1ms / 2 / PI / Current_Loop.pMotor->motor_param->pole_pairs * 60 / 20.0f;
        Current_Loop.Speed_fb_1ms = 0;
    }
    Current_Loop.avg_count++;
    Current_Loop.Speed_fb_1ms += NonFlux_OB.tPLL.we;    
}

void Current_Loop_Run(void)
{

    // Speed_Loop.Speed_Fb = SMO_OB.tPLL.we / 2 / PI / Current_Loop.pMotor->pole_pairs * 60;
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
    Current_Avg_Filt();
    Phase_Current_Rewrite(Current_Loop_Input.Ia_fb_raw - Current_Loop.Ia_fb_offset, 
                          Current_Loop_Input.Ib_fb_raw - Current_Loop.Ib_fb_offset, 
                          Current_Loop_Input.Ic_fb_raw - Current_Loop.Ic_fb_offset,
                          &Current_Loop.Ia_fb, &Current_Loop.Ib_fb, &Current_Loop.Ic_fb,
                          Current_Loop.sector);

    Phase_Min_Max(my_abs(Current_Loop.Ia_fb), my_abs(Current_Loop.Ib_fb), my_abs(Current_Loop.Ic_fb));

    arm_clarke_f32(Current_Loop.Ia_fb, Current_Loop.Ib_fb, &Current_Loop.ialpha_fb, &Current_Loop.ibeta_fb);
    // SMO_Observer(&SMO_OB, Current_Loop.Ualpha_Ref, Current_Loop.Ubeta_Ref, Current_Loop.ialpha_fb, Current_Loop.ibeta_fb);
    Nonlinear_FluxObserver_Update(&NonFlux_OB, Current_Loop.Ualpha_Ref, Current_Loop.Ubeta_Ref,
                                  Current_Loop.ialpha_fb, Current_Loop.ibeta_fb);

    // HFSWInjection_NSF(&HFSW_OB, Current_Loop.Id_fb);
    // HFSWInjection_Update(&HFSW_OB, Current_Loop.ialpha_fb, Current_Loop.ibeta_fb, HFSW_OB.U_hfj);

    Current_Loop.sinVal = arm_sin_f32(Current_Loop.theta);
    Current_Loop.cosVal = arm_cos_f32(Current_Loop.theta);
    arm_park_f32(Current_Loop.ialpha_fb, Current_Loop.ibeta_fb, &Current_Loop.Id_fb,
                 &Current_Loop.Iq_fb, Current_Loop.sinVal, Current_Loop.cosVal);

    Current_Loop.Id_Ref = Speed_Loop.target_id;
    Current_Loop.Iq_Ref = Speed_Loop.target_iq;
    Current_Loop.Ud_Target = Hal_PI_f32(&Current_Loop.Id_PI, Current_Loop.Id_Ref - Current_Loop.Id_fb);
    Current_Loop.Uq_Target = Hal_PI_f32(&Current_Loop.Iq_PI, Current_Loop.Iq_Ref - Current_Loop.Iq_fb);
    arm_sqrt_f32(Current_Loop.Id_fb * Current_Loop.Id_fb + Current_Loop.Iq_fb * Current_Loop.Iq_fb, &Current_Loop.Is_fb);
    arm_inv_park_f32(Current_Loop.Ud_Target, Current_Loop.Uq_Target, &Current_Loop.Ualpha_Ref,
                     &Current_Loop.Ubeta_Ref, Current_Loop.sinVal, Current_Loop.cosVal);
    SVPWM_Calculate(1, Current_Loop_Input.Udc_ADISR, Current_Loop.Ualpha_Ref, Current_Loop.Ubeta_Ref,
                    &Current_Loop.PWM_duty_a, &Current_Loop.PWM_duty_b, &Current_Loop.PWM_duty_c, &Current_Loop.sector);

    Current_Loop.Id_PI.out_max = Current_Loop_Input.Udc_ADISR * WEAK_VOLTAGE_COMPENSATION * 1.02f;
    Current_Loop.Id_PI.out_min = -Current_Loop.Id_PI.out_max;
    arm_sqrt_f32(Current_Loop.Id_PI.out_max * Current_Loop.Id_PI.out_max - Current_Loop.Ud_Target * Current_Loop.Ud_Target,
                 &Current_Loop.Iq_PI.out_max);
    Current_Loop.Iq_PI.out_min = -Current_Loop.Iq_PI.out_max;
    Dead_Zone_Compensation(Current_Loop.Id_fb, Current_Loop.Iq_fb, SMO_OB.tPLL.we, SMO_OB.tPLL.theta,
                           Current_Loop.PWM_duty_a, Current_Loop.PWM_duty_b, Current_Loop.PWM_duty_c,
                           &Current_Loop_Output.PWM_duty_a, &Current_Loop_Output.PWM_duty_b, &Current_Loop_Output.PWM_duty_c);
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

void Current_Loop_Switch(void)
{
    // Code to switch current task states
    if (System.system_state == SYSTEM_RUN)
    {
        switch (Current_Loop.Motor_State)
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
        MOTOR_IDLE_TASK();
        Current_Loop.Motor_State = MOTOR_IDLE;
        Current_PWM_Switch(PWM_CLOSE);
    }
    Current_Loop.Loop_count++;
}

int32_t MOTOR_IDLE_TASK(void)
{
    // Code for MOTOR_IDLE state
    if (System.system_state == SYSTEM_RUN)
    {
        Current_Loop.Motor_State = MOTOR_READY;
        SMO_OB.tPLL.PLL_PI.integral = 0;
        NonFlux_OB.tPLL.PLL_PI.integral = 0;
        Current_Loop.Id_PI.integral = 0;
        Current_Loop.Iq_PI.integral = 0;
        EMF_Cal.EMF = 0;
        Current_Loop.A_Max = 0;
        Current_Loop.A_Min = 0; 
        Current_Loop.B_Max = 0;
        Current_Loop.B_Min = 0;
        Current_Loop.C_Max = 0;
        Current_Loop.C_Min = 0;
    }
    else
    {
        Current_PWM_Switch(PWM_CLOSE);
        Current_Loop.Motor_State = MOTOR_IDLE;
        Current_Loop.theta = 0.0f;
        // Current_Loop.Ia_fb = 0.0f;
        // Current_Loop.Ib_fb = 0.0f;
        // Current_Loop.Ic_fb = 0.0f;      
        Current_Loop.Id_fb = 0.0f;
        Current_Loop.Iq_fb = 0.0f;
        Current_Loop.Id_Ref = 0.0f;
        Current_Loop.Iq_Ref = 0.0f;
    }
    return 0;
}

int32_t MOTOR_READY_TASK(void)
{
    // Code for MOTOR_READY state

    if (System.system_state == SYSTEM_RUN)
    {
        Current_Loop.Motor_State = MOTOR_OFFSET_CHECK;
        Current_Loop.Ia_fb_offset = 0;
        Current_Loop.Ib_fb_offset = 0;
        Current_Loop.Ic_fb_offset = 0;
        Current_Loop.offset_check_cnt = 0;
        Current_Loop.Id_PI.integral = 0;
        Current_Loop.Iq_PI.integral = 0;
    }
    else
    {
        Current_Loop.Motor_State = MOTOR_IDLE;
    }
    return 0;
}

int32_t MOTOR_OFFSET_CHECK_TASK(void)
{
    // Code for MOTOR_OFFSET_CHECK state
    Current_Loop.offset_check_cnt++;
    Current_Loop.Ia_fb_offset += Current_Loop_Input.Ia_fb_raw; // Accumulate ADC1 injected channel 1 value
    Current_Loop.Ib_fb_offset += Current_Loop_Input.Ib_fb_raw; // Accumulate ADC2 injected channel 1 value
    Current_Loop.Ic_fb_offset += Current_Loop_Input.Ic_fb_raw; // Accumulate ADC1 injected channel 2 value
    if (Current_Loop.offset_check_cnt >= MOTOR_ADC_OFFSET_SAMPLE_CNT)
    {
        Current_Loop.Ia_fb_offset /= (float)Current_Loop.offset_check_cnt; // Calculate average for ADC1 injected channel 1
        Current_Loop.Ib_fb_offset /= (float)Current_Loop.offset_check_cnt; // Calculate average for ADC2 injected channel 1
        Current_Loop.Ic_fb_offset /= (float)Current_Loop.offset_check_cnt; // Calculate average for ADC1 injected channel 2
        Current_Loop.Motor_State = MOTOR_RUN;
        Current_PWM_Switch(PWM_OPEN);
    }
    return 0;
}

int32_t MOTOR_RUN_TASK(void)
{
    /* Code for MOTOR_RUN state */
    Current_Loop_Run();
    Bsp_STM32G431_PWM_SetDuty();    
    return 0;
}