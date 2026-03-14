#ifndef __CURRENT_TASK_H__
#define __CURRENT_TASK_H__

#include <stdint.h>

#define MOTOR_ADC_OFFSET_SAMPLE_CNT (100U)
#define PWM_OPEN                    (1U)
#define PWM_CLOSE                   (0U)
#define ADC_OPAMP_GAIN              (0.02197265625f)
#define PWM_MAX_DUTY                (8000.0f)


enum Motor_State{
    MOTOR_IDLE = 0,
    MOTOR_READY,
    MOTOR_OFFSET_CHECK,
    MOTOR_RUN,
    MOTOR_default,
};


void Current_Task_Init(void);
void current_task_run(void);

#endif // SVPWM_H