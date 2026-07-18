#ifndef __MOTOR_DIAG_H__
#define __MOTOR_DIAG_H__

#include "Hal_Math.h"
#include "Diag_List.h"

#define MOTOR_OVER_SPEED_THRESHOLD       (5000.0f)
#define MOTOR_OVER_SPEED_THRESHOLD_DELAY (1000u)

#define MOTOR_PHASE_LOCK_THRESHOLD       (2.5f)

typedef struct {
    Diag_Node_t node;
    Hysteresis_Comp_TypeDef hcomp;
} Motor_Diag_Item_t;

extern uint8_t Motor_Diag_Fault_Flag;

void Motor_Diag_Init(void);
void Motor_Diag_Task(void);

#endif
