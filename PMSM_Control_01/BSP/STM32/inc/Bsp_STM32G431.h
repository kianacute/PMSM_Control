#ifndef __BSP_STM32G431_H__
#define __BSP_STM32G431_H__
#include "stdint.h"

#define ADC_OPAMP_GAIN          (0.02197265625f)
#define PWM_MAX_DUTY            (8000.0f)
#define ADC_VDDA_REF            ((float)4096)

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