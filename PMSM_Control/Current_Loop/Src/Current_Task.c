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

extern MOTOR_t PMSM_42JS;
extern struct SMO_Parameter SMO_OB;

const uint8_t rewrite_phase_index[6] = {3, 2, 3, 1, 1, 2};
// const uint8_t rewrite_phase_index[6] = {2, 1, 1, 3, 2, 3};

float Ud_Target, Uq_Target;
float Id_Ref, Iq_Ref;
float Id_fb, Iq_fb;
float Ualpha_Ref, Ubeta_Ref, sinVal, cosVal;
float theta;
float ialpha_fb, ibeta_fb;
float Ia_fb_raw, Ib_fb_raw, Ic_fb_raw;
float Ia_fb, Ib_fb, Ic_fb;
float32_t PWM_duty_a, PWM_duty_b, PWM_duty_c;
float32_t SMO_we, SMO_rpm, SMO_theta;

uint32_t Speed_Switch_Cnt;
uint8_t sector;

enum Motor_State Motor_State;
uint8_t PWM_OPEN_Flag = 0;

extern float target_rpm, target_iq, target_id;

extern float Udc_1ms;

uint8_t Speed_Switch_Flag;

Hal_PI_t Id_PI = {
    .kp = 1.0f,
    .ki = 0.1f,
    .integral = 0.0f,
    .prev_error = 0.0f,
    .out_min = -100.0f,
    .out_max = 100.0f};

Hal_PI_t Iq_PI = {
    .kp = 1.0f,
    .ki = 0.1f,
    .integral = 0.0f,
    .prev_error = 0.0f,
    .out_min = -100.0f,
    .out_max = 100.0f
};

static MOTOR_t *pMotor;

void Current_Task_Init(void)
{
    // Initialization code for current task
    // e.g., setting up filters, initializing variables, etc.
    theta = 0.0f;
    pMotor = &PMSM_42JS;
    float lp = 10.0f;
    /* 计算电流环参数 */
    Id_PI.kp = pMotor->phase_inductance_d * MOTOR_CURRENT_LOOP_HZ * 2 * PI / lp;
    Iq_PI.kp = pMotor->phase_inductance_q * MOTOR_CURRENT_LOOP_HZ * 2 * PI / lp;
    Id_PI.ki = pMotor->phase_resistance_ohm * 2 * PI / lp; /* 离散化，其中开关周期被抵消 */
    Iq_PI.ki = pMotor->phase_resistance_ohm * 2 * PI / lp; /* 离散化，其中开关周期被抵消 */
    Speed_Switch_Cnt = 0;
    Speed_Switch_Flag = 0;
    SMO_Observer_Init();
    Encode_ABZ_Init();
    Nonlinear_FluxObserver_Init();
}

extern enum SpeedCtrlState_t spd_ctrl_state;
extern float Speed_Ref, Speed_Fb;
float Encode_theta, Encode_rpm;
extern struct NonFluxObserver_Parameter NonFlux_OB;

extern struct Encoder_Parameter Encode_ABZ;

inline float my_abs(float num)
{
    if (num < 0)
    {
        return -num;
    }
    return num;
}

void Speed_Switch(void)
{
    switch (spd_ctrl_state)
    {
    case SPEED_CTRL_IDLE:
        theta = 0;
        break;
    case SPEED_CTRL_ALIGN:
    {
        // theta = 0;
        extern uint8_t align_done;
        if (align_done == 1)
        {
            Encode_ABZ_Get_Offset();
        }
    }
    case SPEED_CTRL_OPEN:
    {
        theta += Speed_Ref / 60 * 2 * PI * pMotor->pole_pairs * MOTOR_CURRENT_LOOP_CYCLE_TIME_S;
        theta = Limit_2PI(theta);
        break;
    }
    case SPEED_CTRL_SWITCH:
    {
        theta += Speed_Ref / 60 * 2 * PI * pMotor->pole_pairs * MOTOR_CURRENT_LOOP_CYCLE_TIME_S;
        theta = Limit_2PI(theta);
        if (my_abs(NonFlux_OB.tPLL.theta - theta) < 0.10)
        {
            Speed_Switch_Cnt++;
            if (Speed_Switch_Cnt > 10)
            {
                Speed_Switch_Flag = 1;
                theta = NonFlux_OB.tPLL.theta;
            }
        }
        break;
    }
    case SPEED_CTRL_RUN:
    {
        theta = Limit_2PI(NonFlux_OB.tPLL.theta);
    }
    default:
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
    switch(sector_re)
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

void Current_Task_Run(float32_t ia_fb, float32_t ib_fb, float32_t ic_fb, float32_t *PWM_duty_a, float32_t *PWM_duty_b, float32_t *PWM_duty_c, float Udc)
{
    // Main code for current task
    // e.g., reading ADC values, processing current measurements, etc.
    Encode_ABZ_UpDate();
    Speed_Switch();
    Phase_Current_Rewrite(ia_fb, ib_fb, ic_fb, &Ia_fb, &Ib_fb, &Ic_fb, sector);
    arm_clarke_f32(Ia_fb, Ib_fb, &ialpha_fb, &ibeta_fb);
    SMO_Observer(&SMO_OB, Ualpha_Ref, Ubeta_Ref, ialpha_fb, ibeta_fb);
    Nonlinear_FluxObserver_Update(&NonFlux_OB, Ualpha_Ref, Ubeta_Ref, ialpha_fb, ibeta_fb);
    Speed_Fb = NonFlux_OB.tPLL.we / 2 / PI / pMotor->pole_pairs * 60;
    // Speed_Fb = Encode_ABZ.speed_rpm;
    sinVal = arm_sin_f32(theta);
    cosVal = arm_cos_f32(theta);
    arm_park_f32(ialpha_fb, ibeta_fb, &Id_fb, &Iq_fb, sinVal, cosVal);
    Id_Ref = target_id;
    Iq_Ref = target_iq;
    Ud_Target = Hal_PI_f32(&Id_PI, Id_Ref - Id_fb);
    Uq_Target = Hal_PI_f32(&Iq_PI, Iq_Ref - Iq_fb);
    arm_inv_park_f32(Ud_Target, Uq_Target, &Ualpha_Ref, &Ubeta_Ref, sinVal, cosVal);
    SVPWM_Calculate(PWM_MAX_DUTY, Udc, Ualpha_Ref, Ubeta_Ref, PWM_duty_a, PWM_duty_b, PWM_duty_c, &sector);
}

volatile int32_t ADC_j1, ADC_j2, ADC_j3;
int32_t ADC_A_offset, ADC_B_offset, ADC_C_offset;
uint32_t Motor_ADC_OFFSET_CNT;

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

extern uint8_t MOTOR_Run_flag;

int32_t MOTOR_IDLE_TASK(void)
{
    // Code for MOTOR_IDLE state
    if (MOTOR_Run_flag == 1)
    {
        Motor_State = MOTOR_READY;
    }
    else
    {
        Current_PWM_Switch(PWM_CLOSE);
        Motor_State = MOTOR_IDLE;
    }
    return 0;
}

int32_t MOTOR_READY_TASK(void)
{
    // Code for MOTOR_READY state

    if (MOTOR_Run_flag == 1)
    {
        Motor_State = MOTOR_OFFSET_CHECK;
        ADC_A_offset = 0;
        ADC_B_offset = 0;
        ADC_C_offset = 0;
        Motor_ADC_OFFSET_CNT = 0;
        Id_PI.integral = 0;
        Iq_PI.integral = 0;
    }
    else
    {
    }
    return 0;
}

int32_t MOTOR_OFFSET_CHECK_TASK(void)
{
    // Code for MOTOR_OFFSET_CHECK state
    Motor_ADC_OFFSET_CNT++;
    ADC_A_offset += ADC_j1; // Accumulate ADC1 injected channel 1 value
    ADC_B_offset += ADC_j2; // Accumulate ADC2 injected channel 1 value
    ADC_C_offset += ADC_j3; // Accumulate ADC1 injected channel 2 value
    if (Motor_ADC_OFFSET_CNT >= MOTOR_ADC_OFFSET_SAMPLE_CNT)
    {
        ADC_A_offset /= Motor_ADC_OFFSET_CNT; // Calculate average for ADC1 injected channel 1
        ADC_B_offset /= Motor_ADC_OFFSET_CNT; // Calculate average for ADC2 injected channel 1
        ADC_C_offset /= Motor_ADC_OFFSET_CNT; // Calculate average for ADC1 injected channel 2
        Motor_ADC_OFFSET_CNT = 0;
        Motor_State = MOTOR_RUN;
        Current_PWM_Switch(PWM_OPEN);
    }
    return 0;
}

int32_t MOTOR_RUN_TASK(void)
{
    /* Code for MOTOR_RUN state */

    Ia_fb_raw = (ADC_j1 - ADC_A_offset) * ADC_OPAMP_GAIN; // Adjust ADC1 injected channel 1 value
    Ib_fb_raw = (ADC_j2 - ADC_B_offset) * ADC_OPAMP_GAIN; // Adjust ADC2 injected channel 1 value
    Ic_fb_raw = (ADC_j3 - ADC_C_offset) * ADC_OPAMP_GAIN; // Adjust ADC1 injected channel 2 value
    Current_Task_Run(Ia_fb_raw, Ib_fb_raw, Ic_fb_raw, &PWM_duty_a, &PWM_duty_b, &PWM_duty_c, Udc_1ms);
    if (Speed_Ref < 600 && MOTOR_Run_flag == 0)
    {
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0); // Set PWM duty cycle for Channel 1
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0); // Set PWM duty cycle for Channel 2
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0); // Set PWM duty cycle for Channel 3
        Motor_State = MOTOR_IDLE;
    }
    else
    {
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, PWM_duty_a); // Set PWM duty cycle for Channel 1
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, PWM_duty_b); // Set PWM duty cycle for Channel 2
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, PWM_duty_c); // Set PWM duty cycle for Channel 3
    }

    return 0;
}

typedef struct vofa_buffer_
{
    float32_t data[8];
    uint8_t tail[4];
} vofa_buffer_t;

uint32_t adc_cnt = 0;

void Current_Task_Switch(void)
{
    // Code to switch current task states
    switch (Motor_State)
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

    adc_cnt++;
    extern float theta;
    extern float Ia_fb, Ib_fb, Ic_fb;
    extern uint32_t adc_cnt;
    extern float target_rpm, target_iq;
    extern float32_t PWM_duty_a, PWM_duty_b, PWM_duty_c;
    extern float Id_fb, Iq_fb;
    extern float32_t SMO_we, SMO_rpm, SMO_theta;
    extern struct SMO_Parameter SMO_OB;
    extern float Speed_Ref, Speed_Fb;
    extern enum SpeedCtrlState_t spd_ctrl_state;
    extern enum Motor_State Motor_State;
    extern uint8_t Speed_Switch_Flag;
    extern uint8_t MOTOR_Run_flag;
    extern float Encode_theta, Encode_rpm;
    extern int16_t Encode_ABZ_offset;
    extern uint32_t adc_cnt;
    extern vofa_buffer_t vofa_buffer;
    extern float Udc_1ms;
    vofa_buffer.data[0] = Speed_Ref;
    vofa_buffer.data[1] = Speed_Fb;
    vofa_buffer.data[2] = (float32_t)(Udc_1ms);
    vofa_buffer.data[3] = (float32_t)(Id_fb);
    vofa_buffer.data[4] = (float32_t)(Iq_fb);
    vofa_buffer.data[5] = (float32_t)(Ia_fb);
    vofa_buffer.data[6] = (float32_t)(Ib_fb);
    vofa_buffer.data[7] = (float32_t)(Ic_fb);
    HAL_UART_Transmit_DMA(&huart3, (uint8_t *)&vofa_buffer, sizeof(vofa_buffer));
}

extern uint32_t my_task2_cnt;

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        ADC_j1 = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1); // Read injected channel value
        ADC_j2 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1); // Read another injected channel value
        ADC_j3 = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2); // Read another injected channel value
        Current_Task_Switch();
        my_task2_cnt++;
    }
    else if (hadc->Instance == ADC2)
    {
    }
}
