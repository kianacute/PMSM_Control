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
#include "Current_Loop.h"
#include "Speed_Loop.h"
#include "Observer.h"
#include "Motor_Diag.h"
#include "System_Diag.h"
#include "System_Loop.h"

TickType_t lasttick = 0;
uint16_t adc_v24;

uint8_t load_send_buffer[500];
extern volatile uint32_t CPU_RunTime;
adc_adjustment_t adc_adjustment = {0};

extern Current_Loop_Input_t Current_Loop_Input;
extern Current_Loop_Output_t Current_Loop_Output;

void my_task1(void *argument)
{
    for (;;)
    {
        // memset(send_buffer, 0, 100); // 信息缓冲区清零
        // vTaskGetRunTimeStats((char *)&send_buffer);
        // // vTaskList((char *)&send_buffer);  //获取任务运行时间信息
        // HAL_UART_Transmit_DMA(&huart3, (uint8_t *)send_buffer, strlen((char *)send_buffer));
        lasttick = xTaskGetTickCount();
        Speed_Loop_Task();
        Motor_Diag_Task();
        vTaskDelayUntil(&lasttick, 1); // 每1ms执行一次
        // osDelay(1);
    }
}

void my_task2(void *argument)
{
    for (;;)
    {
        lasttick = xTaskGetTickCount();
        SYSTEM_Task();
        vTaskDelayUntil(&lasttick, 1); // 每1ms执行一次
    }
}

extern void MOTOR_Run_flag_UPDOWN(void);
extern uint8_t MOTOR_Run_flag;

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
        if(MOTOR_Run_flag == 1)
        {
            vTaskDelayUntil(&lasttick, 10000); // 每5000ms执行一次
        }
        else
        {
            vTaskDelayUntil(&lasttick, 100); // 每100ms执行一次
        }
        MOTOR_Run_flag_UPDOWN();
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

    HAL_TIM_Base_Start(&htim16);
    __HAL_TIM_ENABLE_IT(&htim16, TIM_IT_UPDATE);  //使能更新中断
    
    xTaskCreate(my_task1, "Speed_Ctrl_Task", 256, NULL, osPriorityRealtime, NULL);
    xTaskCreate(my_task2, "SYSTEM_Task", 256, NULL, osPriorityRealtime, NULL);
    // xTaskCreate(my_task3, "MOTOR_Run_Task", 16, NULL, osPriorityNormal, NULL);
    xTaskCreate(my_task4, "System_Diag_Task", 256, NULL, osPriorityAboveNormal, NULL);
    SYSTEM_Init();
    return 0;
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        adc_adjustment.ADC_j1 = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1) ; // Read injected channel value
        adc_adjustment.ADC_j2 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1); // Read another injected channel value
        adc_adjustment.ADC_j3 = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2); // Read another injected channel value
        adc_adjustment.ADC_j4 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2); // Read another injected channel value
        Current_Loop_Input.Ia_fb_raw = (adc_adjustment.ADC_j1 - ADC_VDDA_REF/2) * ADC_OPAMP_GAIN; // Adjust ADC1 injected channel 1 value
        Current_Loop_Input.Ib_fb_raw = (adc_adjustment.ADC_j2 - ADC_VDDA_REF/2) * ADC_OPAMP_GAIN; // Adjust ADC2 injected channel 1 value
        Current_Loop_Input.Ic_fb_raw = (adc_adjustment.ADC_j3 - ADC_VDDA_REF/2) * ADC_OPAMP_GAIN; // Adjust ADC1 injected channel 2 value
        Current_Loop_Input.Udc_ADISR = adc_adjustment.ADC_j4 * 1.1f * 0.019842f;

        /*调用电流环切换函数*/
        Current_Loop_Switch();
    }
    else if (hadc->Instance == ADC2)
    {
    }
}

void Bsp_STM32G431_PWM_Enable(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

void Bsp_STM32G431_PWM_Disable(void)
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
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, Current_Loop_Output.PWM_duty_a*PWM_MAX_DUTY);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, Current_Loop_Output.PWM_duty_b*PWM_MAX_DUTY);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, Current_Loop_Output.PWM_duty_c*PWM_MAX_DUTY);
    // __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, Current_Loop_Output.PWM_duty_d);
}