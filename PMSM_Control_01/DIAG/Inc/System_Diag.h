#ifndef __SYSTEM_DIAG_H__
#define __SYSTEM_DIAG_H__
#include "system.h"
#include "Hal_Math.h"

/*高压传感器*/
#define HV_OVER_THRESHOLD       (32.0f)
#define HV_OVER_RE_THRESHOLD    (30.0f)
#define HV_LOW_THRESHOLD        (13.0f)
#define HV_LOW_RE_THRESHOLD     (12.0f)
#define HV_POWER_THRESHOLD      (13.0f)
#define HV_GND_THRESHOLD        (3.0f)
#define HV_UPLIMIT              (50.0f)
#define HV_LOWLIMIT             (00.0f)
//延迟时间
#define HV_DELAY_TIME           (100u) // 100个周期

void System_Diag_Init(void);
void System_Diag_Task(void);


#endif // __SYSTEM_DIAG_H__