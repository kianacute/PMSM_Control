/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : app_freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fdcan.h"
#include "usart.h"
#include "gpio.h"
#include "tim.h"
#include "arm_math.h"
#include <main.h>
#include "Current_Task.h"
#include "adc.h"
#include "speed_ctrl.h"
#include "opamp.h"
#include "Observer.h"
#include "Can.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

typedef struct vofa_buffer_
{
    float32_t data[8];
    uint8_t tail[4];
} vofa_buffer_t;

vofa_buffer_t vofa_buffer =
    {
        .tail[0] = 0x00,
        .tail[1] = 0x00,
        .tail[2] = 0x80,
        .tail[3] = 0x7F,
};

TickType_t lasttick = 0; 
uint16_t adc_v24;
float Udc_1ms;

void my_task1(void *argument)
{ 
    for (;;)
    {
        // memset(send_buffer, 0, 100); // 信息缓冲区清零
        // vTaskGetRunTimeStats((char *)&send_buffer);
        // // vTaskList((char *)&send_buffer);  //获取任务运行时间信息
        // HAL_UART_Transmit_DMA(&huart3, (uint8_t *)send_buffer, strlen((char *)send_buffer));
        
        lasttick = xTaskGetTickCount();
        Speed_Ctrl_Task();
        HAL_ADC_Start(&hadc2);
        HAL_ADC_PollForConversion(&hadc2, 1);
        if (HAL_ADC_GetState(&hadc2) & HAL_ADC_STATE_REG_EOC)
        {
            adc_v24 = HAL_ADC_GetValue(&hadc2); // Read ADC value
            Udc_1ms = adc_v24 * 1.1f * 0.019842f; // 
        }
        vTaskDelayUntil(&lasttick, 1); // 每1ms执行一次
        // osDelay(1);
    }
}

uint32_t my_task2_cnt; 


extern uint32_t adc_cnt;
uint8_t printf_buffer[100];

void my_task2(void *argument)
{
    // extern uint8_t sector;
    for (;;)
    {
        lasttick = xTaskGetTickCount();
        // Current_Task_Run(&vofa_buffer.data[0], &vofa_buffer.data[1], &vofa_buffer.data[2],
        // &vofa_buffer.data[3], &vofa_buffer.data[4]);
        
        //Can_Task_Run();
        // sprintf((char *)printf_buffer, "adc_v24: %d\r\n", adc_cnt);
        // HAL_UART_Transmit_DMA(&huart3, (uint8_t *)printf_buffer, strlen((char *)printf_buffer));
        vTaskDelayUntil(&lasttick, 1000); // 每1000ms执行一次
    }
}

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
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
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 4000 - 2); // Set initial compare value for TIM1 Channel 4

    HAL_TIM_Base_Start(&htim1); // Start TIM1 base timer

    Current_Task_Init();
    // __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE); // Enable update interrupt for TIM1

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);


    Speed_Ctrl_Init();
    Current_Task_Init();
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    xTaskCreate(my_task1, "my_task1", 128, NULL, osPriorityNormal, NULL);
    xTaskCreate(my_task2, "my_task2", 256, NULL, osPriorityNormal, NULL);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
    /* Infinite loop */
    for (;;)
    {
        fdcan_transmit_data();
        osDelay(1000);
    }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

