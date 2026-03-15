#ifndef __CURRENT_TASK_H__
#define __CURRENT_TASK_H__

#include <stdint.h>
#include "Hal_Math.h"
#include "Motor_parameter.h"

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

typedef struct Current_Task
{
    // Define any necessary variables and structures for the current task
    uint32_t FREQ_HZ;
    float Loop_time_s;
    MOTOR_t *pMotor;
    Hal_PI_t Id_PI;
    Hal_PI_t Iq_PI;
    float Ud_Target, Uq_Target;
    float Id_Ref, Iq_Ref;
    float Id_fb, Iq_fb;
    float Ualpha_Ref, Ubeta_Ref, sinVal, cosVal;
    float theta;
    float ialpha_fb, ibeta_fb;
    float Ia_fb_raw, Ib_fb_raw, Ic_fb_raw;
    float Ia_fb, Ib_fb, Ic_fb;
    float PWM_duty_a, PWM_duty_b, PWM_duty_c;
    uint8_t sector;
    enum Motor_State Motor_State;
    uint8_t PWM_OPEN_Flag;
    float Speed_fb_1ms;
    uint8_t avg_count;
    uint64_t count;
    uint64_t timestamp;
}Current_Task_t;


typedef struct adc_adjustment
{
    int32_t ADC_j1, ADC_j2, ADC_j3;
    int32_t ADC_A_offset, ADC_B_offset, ADC_C_offset;
    uint32_t Motor_ADC_OFFSET_CNT;
}adc_adjustment_t;


void Current_Task_Init(void);
void current_task_run(void);

#endif // SVPWM_H