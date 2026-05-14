#include "Motor_Diag.h"
#include "speed_ctrl.h"
#include "Current_Task.h"

extern Speed_Ctrl_t Speed_Ctrl;

static Diag_Node_t *Motor_Diag_Head = NULL;

static Motor_Diag_Item_t Motor_Over_Speed_Diag;
static Motor_Diag_Item_t Motor_Phase_Lock;
static Motor_Diag_Item_t Motor_Block_Detect_Diag;

static void Over_Speed_Update(Diag_Node_t *node)
{
    Motor_Diag_Item_t *item = (Motor_Diag_Item_t *)node;
    Hysteresis_Comp_Process_Add(&item->hcomp, Speed_Ctrl.Speed_Fb);
}

void MOTOR_PHASE_LOCK(void)
{

}

void MOTOR_BLOCK_DETECT(void)
{

}

void Motor_Diag_Init(void)
{
    Hysteresis_Comp_Init(&Motor_Over_Speed_Diag.hcomp, MOTOR_OVER_SPEED_THRESHOLD, 0, MOTOR_OVER_SPEED_THRESHOLD_DELAY);
    Diag_List_Register(&Motor_Diag_Head, &Motor_Over_Speed_Diag.node, Over_Speed_Update);

    Hysteresis_Comp_Init(&Motor_Phase_Lock.hcomp, 0, 0, 0);
    Hysteresis_Comp_Init(&Motor_Block_Detect_Diag.hcomp, 0, 0, 0);
}

void Motor_Diag_Task(void)
{
    Diag_List_Traverse(Motor_Diag_Head);
}
