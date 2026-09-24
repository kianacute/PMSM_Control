#include "Bsp_STM32G431.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "fdcan.h"
#include "usart.h"
#include "gpio.h"
#include "tim.h"
#include "arm_math.h"
#include "adc.h"
#include "opamp.h"
#include "Motor_Diag.h"
#include "System_Diag.h"
#include "Motor_Control.h"
#include "SVPWM_Fixed.h"
#include "SVPWM_Float.h"

extern Motor_Control_t PMSM_42J;   

    
TickType_t lasttick = 0;
uint16_t adc_v24;

uint8_t load_send_buffer[500];
extern volatile uint32_t CPU_RunTime;
adc_adjustment_t adc_adjustment = {0};

/* Profiler 全局变量 (ISR-aware 运行时间统计) */
volatile uint32_t g_isr_accumulated_cycles = 0;
volatile uint32_t g_isr_nest_level = 0;
Profiler_Slot_t g_profiler_slots[PROFILER_SLOT_COUNT] = {0};

void my_task1(void *argument)
{
    uint32_t isr_before, start, net_cycles;
    TickType_t lasttick = 0;
    for (;;)
    {
        /* ---- ISR-aware 任务时间测量 ---- */
        // Profiler_TaskBegin(&isr_before, &start);

        lasttick = xTaskGetTickCount();
        Speed_Loop_Task(&PMSM_42J);
        // Motor_Diag_Task(&PMSM_42J);

        // net_cycles = Profiler_TaskEnd(isr_before, start);

        /* 记录到 profiler 槽位 + 更新兼容变量 (us) */
        // Profiler_Record(CPU_TASK1_INDEX, net_cycles);

        vTaskDelayUntil(&lasttick, 1); /* 每1ms执行一次 */
    }
}

void my_task2(void *argument)
{
    TickType_t lasttick = 0;
    for (;;)
    {
        lasttick = xTaskGetTickCount();
        SYSTEM_LOOP_Task(&PMSM_42J);
        vTaskDelayUntil(&lasttick, 10); // 每10ms执行一次
    }
}

extern void MOTOR_Run_flag_UPDOWN(void);
extern uint8_t MOTOR_Run_flag;


q15_t sin_output1, cos_output1, input1;
q15_t sin_output2, cos_output2, input2;
q15_t id_t, iq_t;
uint16_t add_output1 = 100, add_output2;
q15_t TT1, TT2, TT3;
uint8_t sector_tmp;

void my_task3(void *argument)
{
    // extern uint8_t sector;

    for (;;)
    {
        lasttick = xTaskGetTickCount();
        // memset(load_send_buffer, 0, sizeof(load_send_buffer)); // 信息缓冲区清零
        // vTaskGetRunTimeStats((char *)&load_send_buffer);
        // vTaskList((char *)&load_send_buffer);  //获取任务运行时间信息
        // sprintf((char *)load_send_buffer, "adc: %d  %d\r\n", cup_adc_1, cup_adc_2);
        // HAL_UART_Transmit_DMA(&huart3, (uint8_t *)load_send_buffer, strlen((char *)load_send_buffer));
        // if(MOTOR_Run_flag == 1)
        // {
        //     vTaskDelayUntil(&lasttick, 10000); // 每5000ms执行一次
        // }
        // else
        // {
        //     vTaskDelayUntil(&lasttick, 100); // 每100ms执行一次
        // }
        // MOTOR_Run_flag_UPDOWN();

        // arm_sin_cos_q15(input1, &sin_output1, &cos_output1);

        // sin_output1 = arm_cos_q15(input1);
        // cos_output1 = arm_cos_q15(input1 + 21845u);
        // sin_output2 = arm_sin_q15(input1 - 21845u);
        uint32_t cb_start = DWT->CYCCNT;

        // SVPWM_Calculate_q31(4000 * 2, 9142, sin_output2, cos_output2,
        //                 &TT1, &TT2, &TT3, &sector_tmp);

        // arm_clarke_q15(sin_output1, cos_output1, &sin_output2, &cos_output2);
        // arm_park_q15(sin_output2, cos_output2, &id_t, &iq_t, arm_sin_q15(input1), arm_cos_q15(input1));
        arm_inv_park_q15(id_t, iq_t, &sin_output2, &cos_output2, arm_sin_q15(input1), arm_cos_q15(input1));
        arm_inv_clarke_q15(sin_output2, cos_output2, &sin_output1, &cos_output1, &input2);

        uint32_t cb_elapsed = DWT->CYCCNT - cb_start;
        Profiler_Record(CPU_ADC_INT_INDEX, cb_elapsed);
        vTaskDelayUntil(&lasttick, 10); // 每100ms执行一次


        input1 = input1 + add_output1;

    }
}

void my_task4(void *argument)
{
    for (;;)
    {
        // memset(send_buffer, 0, 100); // 信息缓冲区清零
        // vTaskGetRunTimeStats((char *)&send_buffer);
        // // vTaskList((char *)&send_buffer);  //获取任务运行时间信息
        // HAL_UART_Transmit_DMA(&huart3, (uint8_t *)send_buffer, strlen((char *)send_buffer));
        lasttick = xTaskGetTickCount();
        System_Diag_Task();
        vTaskDelayUntil(&lasttick, 10); // 每10ms执行一次
        // osDelay(1);
    }
}

int Bsp_Init(void)
{
    HAL_OPAMP_Start(&hopamp1);
    HAL_OPAMP_Start(&hopamp2);
    HAL_OPAMP_Start(&hopamp3);
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC);
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC);
    __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_JEOC);

    HAL_ADCEx_InjectedStart_IT(&hadc1); // Start ADC1 injected conversion with interrupt
    HAL_ADCEx_InjectedStart_IT(&hadc2); // Start ADC2 injected conversion

    __HAL_TIM_SET_AUTORELOAD(&htim1, 4000 - 1);            // Set the auto-reload value for TIM1
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 4000 - 5); // Set initial compare value for TIM1 Channel 4

    HAL_TIM_Base_Start(&htim1); // Start TIM1 base timer
    // __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE); // Enable update interrupt for TIM1

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    // HAL_TIM_Base_Start(&htim16);
    // __HAL_TIM_ENABLE_IT(&htim16, TIM_IT_UPDATE);  //使能更新中断
    
    xTaskCreate(my_task1, "Speed_Ctrl_Task", 256, NULL, osPriorityRealtime, NULL);
    xTaskCreate(my_task2, "SYSTEM_Task", 256, NULL, osPriorityHigh, NULL);
    xTaskCreate(my_task3, "MOTOR_Run_Task", 128, NULL, osPriorityNormal, NULL);
    // xTaskCreate(my_task4, "System_Diag_Task", 256, NULL, osPriorityAboveNormal, NULL);
    Profiler_Init();
    Motor_Control_Init(&PMSM_42J);
    Bsp_STM32G431_PWM_Disable();
    return 0;
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        /* DWT 周期计数器 — 不受中断优先级影响, 精度 6.25ns @160MHz */
        // uint32_t cb_start = DWT->CYCCNT;
        // adc_adjustment.ADC_j1 = hadc1.Instance->JDR1; // Read injected channel value
        // adc_adjustment.ADC_j2 = hadc2.Instance->JDR1; // Read another injected channel value
        // adc_adjustment.ADC_j3 = hadc1.Instance->JDR2; // Read another injected channel value
        // adc_adjustment.ADC_j4 = hadc2.Instance->JDR2; // Read another injected channel value
        PMSM_42J.Input.Ia_fb_raw = ((hadc1.Instance->JDR1 - ADC_VDDA_REF / 2) << 3); // Adjust ADC1 injected channel 1 value
        PMSM_42J.Input.Ib_fb_raw = ((hadc2.Instance->JDR1 - ADC_VDDA_REF / 2) << 3); // Adjust ADC2 injected channel 1 value
        PMSM_42J.Input.Ic_fb_raw = ((hadc1.Instance->JDR2 - ADC_VDDA_REF / 2) << 3); // Adjust ADC1 injected channel 2 value
        PMSM_42J.Input.Udc_ADISR = ((hadc2.Instance->JDR2) << 3);

        /*调用电流环切换函数*/
        Current_Loop_Task(&PMSM_42J);

        if(PMSM_42J.Current_Loop.PWM_OPEN_Flag_z != PMSM_42J.Current_Loop.PWM_OPEN_Flag)
        {
            if (PMSM_42J.Current_Loop.PWM_OPEN_Flag == PWM_OPEN)
            {
                Bsp_STM32G431_PWM_Enable();
            }
            else
            {
                Bsp_STM32G431_PWM_Disable();
            }
        }
        Bsp_STM32G431_PWM_SetDuty();

        // uint32_t cb_elapsed = DWT->CYCCNT - cb_start;
        // Profiler_Record(CPU_ADC_INT_INDEX, cb_elapsed);
    }
    else if (hadc->Instance == ADC2)
    {
    }
}

inline void Bsp_STM32G431_PWM_Enable(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

inline void Bsp_STM32G431_PWM_Disable(void)
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
}

void Bsp_STM32G431_PWM_SetDuty()
{
    __HAL_TIM_SET_AUTORELOAD(&htim1, PWM_MAX_DUTY/PMSM_42J.Output.PWM_HZ_Coeff - 1);            // Set the auto-reload value for TIM1
    // __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, PWM_MAX_DUTY/PMSM_42J.Output.PWM_HZ_Coeff - 5); // Set initial compare value for TIM1 Channel 4
    // __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, PMSM_42J.Output.PWM_duty_a*PWM_MAX_DUTY/PMSM_42J.Output.PWM_HZ_Coeff);
    // __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, PMSM_42J.Output.PWM_duty_b*PWM_MAX_DUTY/PMSM_42J.Output.PWM_HZ_Coeff);
    // __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, PMSM_42J.Output.PWM_duty_c*PWM_MAX_DUTY/PMSM_42J.Output.PWM_HZ_Coeff);
    htim1.Instance->CCR4 = PWM_MAX_DUTY/PMSM_42J.Output.PWM_HZ_Coeff - 5;
    htim1.Instance->CCR1 = PMSM_42J.Output.PWM_duty_a*PWM_MAX_DUTY/PMSM_42J.Output.PWM_HZ_Coeff;
    htim1.Instance->CCR2 = PMSM_42J.Output.PWM_duty_b*PWM_MAX_DUTY/PMSM_42J.Output.PWM_HZ_Coeff;
    htim1.Instance->CCR3 = PMSM_42J.Output.PWM_duty_c*PWM_MAX_DUTY/PMSM_42J.Output.PWM_HZ_Coeff;
    // __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, PMSM_42J.Output.PWM_duty_a*PWM_MAX_DUTY);
    // __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, PMSM_42J.Output.PWM_duty_b*PWM_MAX_DUTY);
    // __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, PMSM_42J.Output.PWM_duty_c*PWM_MAX_DUTY);
    // __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, PMSM_42J.Output.PWM_duty_d);
}