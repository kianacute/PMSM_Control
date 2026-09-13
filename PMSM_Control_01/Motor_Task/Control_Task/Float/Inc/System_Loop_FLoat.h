#ifndef __SYSTEM_LOOP_FLOAT_H__
#define __SYSTEM_LOOP_FLOAT_H__

#include "Hal_Math.h"
#include "Motor_Control.h"

typedef struct SYSTEM_Loop_Float
{
    uint32_t FREQ_Hz;                            // 循环周期
    uint32_t Fault_cnt;                          // 系统状态
    uint8_t Run_flag;                            // 在Run状态下管理运行指令
    Hysteresis_Comp_TypeDef_f32_t System_Hv_Comp; // 系统高压滞回比较器
} System_Loop_Float_t;


#endif // __SYSTEM_LOOP_FLOAT_H__
