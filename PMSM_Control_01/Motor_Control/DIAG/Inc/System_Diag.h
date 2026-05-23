#ifndef __SYSTEM_DIAG_H__
#define __SYSTEM_DIAG_H__

#include "System_Loop.h"
#include "Hal_Math.h"
#include "Diag_List.h"

/*高压传感器*/
#define HV_OVER_THRESHOLD       (32.0f)
#define HV_OVER_RE_THRESHOLD    (30.0f)
#define HV_LOW_THRESHOLD        (13.0f)
#define HV_LOW_RE_THRESHOLD     (12.0f)
#define HV_POWER_THRESHOLD      (13.0f)
#define HV_GND_THRESHOLD        (3.0f)
#define HV_UPLIMIT              (50.0f)
#define HV_LOWLIMIT             (00.0f)
#define HV_DELAY_TIME           (100u)

typedef struct {
    Diag_Node_t node;
    Sensor_Hysteresis_Comp_TypeDef hcomp;
} Sensor_Diag_Item_t;

void System_Diag_Init(void);
void System_Diag_Task(void);

#endif
