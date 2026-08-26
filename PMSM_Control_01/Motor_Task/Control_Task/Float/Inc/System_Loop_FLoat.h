#ifndef __SYSTEM_H__
#define __SYSTEM_H__

#include "Hal_Math_Float.h"
#include "Motor_Config_Float.h"

#define SYSTEM_HZ               (1000.0f)

#define SYSTEM_LV_INIT_TIME         (uint32_t)(3.0f * SYSTEM_HZ)
#define SYSTEM_HV_STANDY_TIME       (uint32_t)(0.1f * SYSTEM_HZ)
#define SYSTEM_WAIT_TIME            (uint32_t)(3.0f * SYSTEM_HZ)

enum SYSTEM_State_t
{
    SYSTEM_LV_STANDY = 0,
    SYSTEM_HV_STANDY,
    SYSTEM_RUN,
    SYSTEM_FAULT,
    SYSTEM_WAIT,
};

typedef struct SYSTEM_Ctrl
{
    uint32_t FREQ_Hz;                            // 循环周期
    enum SYSTEM_State_t system_state;  
    uint32_t Fault_cnt;                          // 系统状态
    uint8_t Run_flag;                            // 在Run状态下管理运行指令
} System_Loop_FLoat_t;

void SYSTEM_Init_Float(Motor_Control_t *pControl);
void SYSTEM_LV_Standy_Float(Motor_Control_t *pControl);
void SYSTEM_HV_Standy_Float(Motor_Control_t *pControl);
void SYSTEM_Run_Float(Motor_Control_t *pControl);
void SYSTEM_Fault_Float(Motor_Control_t *pControl);
void SYSTEM_Wait_Float(Motor_Control_t *pControl);

#endif // __SYSTEM_H__
