#ifndef __BSP_STM32G431_H__
#define __BSP_STM32G431_H__
#include "stdint.h"
#include "Profiler.h"

#define ADC_OPAMP_GAIN          (0.02197265625f)
#define PWM_MAX_DUTY            (4000.0f)
#define ADC_VDDA_REF            ((float)4096/2)


/* Profiler 槽位索引
 * 基于 DWT CYCCNT 做 ISR-aware 任务/中断运行时间统计
 * 任务净时间: 扣除 ISR 累积时间后的纯任务代码耗时
 * 使用 Profiler_CyclesToUs() 可将 CPU cycles 转换为 us (160MHz)
 */
#define CPU_ADC_INT_INDEX       (0u)   /* ADC ISR -- FOC 电流环耗时           */
#define CPU_TASK1_INDEX         (1u)   /* my_task1 -- 速度环 + 电机诊断       */
#define CPU_TASK2_INDEX         (2u)   /* my_task2 -- 系统状态机              */
#define CPU_TASK3_INDEX         (3u)   /* my_task3 -- 预留                    */
#define CPU_TASK4_INDEX         (4u)   /* my_task4 -- 系统诊断 (10ms)         */

typedef struct adc_adjustment
{
    float ADC_j1, ADC_j2, ADC_j3;
    uint32_t ADC_j4;
} adc_adjustment_t;

int Bsp_Init(void);
void Bsp_STM32G431_PWM_Enable(void);
void Bsp_STM32G431_PWM_Disable(void);
void Bsp_STM32G431_PWM_SetDuty(void);

#endif /* __BSP_STM32G431_H__ */