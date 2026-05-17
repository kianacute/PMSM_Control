#ifndef __SYSTEM_H__
#define __SYSTEM_H__

#include "Hal_Math.h"
#include "Motor_parameter.h"

#define SYSTEM_HZ               (1000.0f)

#define SYSTEM_LV_INIT_TIME        (3.0f * SYSTEM_HZ)
#define SYSTEM_HV_STANDY_TIME   (0.1f * SYSTEM_HZ)
#define SYSTEM_WAIT_TIME      (3.0f * SYSTEM_HZ)

enum SYSTEM_State_t
{
    SYSTEM_LV_STANDY = 0,
    SYSTEM_HV_STANDY,
    SYSTEM_CMD_STANDY,
    SYSTEM_RUN,
    SYSTEM_FAULT,
    SYSTEM_WAIT,
};

typedef struct SYSTEM_Ctrl
{
    uint32_t FREQ_Hz;                               // 循环周期
    enum SYSTEM_State_t system_state;  
    uint32_t Fault_cnt;                          // 系统状态
} SYSTEM_t;

void SYSTEM_Init(void);
void SYSTEM_Task(void);

#endif // __SYSTEM_H__
