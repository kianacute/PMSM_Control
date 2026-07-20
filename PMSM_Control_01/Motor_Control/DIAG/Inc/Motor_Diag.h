#ifndef __MOTOR_DIAG_H__
#define __MOTOR_DIAG_H__

#include "Hal_Math.h"
#include "Diag_List.h"

#define MOTOR_OVER_SPEED_THRESHOLD       (5000.0f)
#define MOTOR_OVER_SPEED_THRESHOLD_DELAY (1000u)

#define MOTOR_PHASE_LOCK_THRESHOLD       (2.5f)

#define MOTOR_SPEED_OVER_FLAG_MASK                  ((uint64_t)0x01 << 0)
#define MOTOR_BLOCK_DETECT_FLAG_MASK                ((uint64_t)0x01 << 1)
#define MOTOR_PHASE_A_LOCK_FLAG_MASK                ((uint64_t)0x01 << 2)
#define MOTOR_PHASE_B_LOCK_FLAG_MASK                ((uint64_t)0x01 << 3)
#define MOTOR_PHASE_C_LOCK_FLAG_MASK                ((uint64_t)0x01 << 4)
#define MOTOR_CURRENT_OFFSET_OVER_FLAG_MASK         ((uint64_t)0x01 << 5)


typedef struct {
    Diag_Node_t node;
    Hysteresis_Comp_TypeDef hcomp;
} Motor_Diag_Item_t;

extern uint64_t Motor_Diag_Fault_Flag;

void Motor_Diag_Init(void);
void Motor_Diag_Task(void);

#endif
