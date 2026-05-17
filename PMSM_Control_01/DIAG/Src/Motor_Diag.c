#include "Motor_Diag.h"
#include "speed_ctrl.h"
#include "Current_Task.h"
#include "Observer.h"
#include "system.h"

extern Speed_Ctrl_t Speed_Ctrl;
extern SYSTEM_t System ;
extern struct EMF_Cal_Parameter EMF_Cal;

static Diag_Node_t *Motor_Diag_Head = NULL;

static Motor_Diag_Item_t Motor_Over_Speed_Diag;
static Motor_Diag_Item_t Motor_Phase_Lock;
static Motor_Diag_Item_t Motor_Block_Detect_Diag;

uint8_t Motor_Diag_Fault_Flag = 0;

static void MOTOR_Over_Speed_Update(Diag_Node_t *node)
{
    Motor_Diag_Item_t *item = (Motor_Diag_Item_t *)node;
    Hysteresis_Comp_Process_Add(&item->hcomp, Speed_Ctrl.Speed_Fb);
    if(item->hcomp.comp_out == 1)
    {
        Motor_Diag_Fault_Flag |= 0x01; // 设置过速故障标志
    }
}

static void MOTOR_PHASE_LOCK(void)
{

}

float emf_err = 0.0f;

/*堵转故障*/
static void MOTOR_BLOCK_DETECT(Diag_Node_t *node)
{
    Motor_Diag_Item_t *item = (Motor_Diag_Item_t *)node;
    if(System.system_state == SYSTEM_WAIT)
    {
        item->hcomp.reset = 1; // 系统未运行时复位比较器
    }
    else
    {
        item->hcomp.reset = 0; // 系统运行时正常工作
    }
    if(System.system_state == SYSTEM_RUN)
    {
        item->hcomp.enable = 0; // 系统运行时使能比较器
    }
    emf_err = my_abs(Speed_Ctrl.Speed_Fb * Speed_Ctrl.pMotor->flux_rpm_per_v / 1000.0f - EMF_Cal.EMF);
    Hysteresis_Comp_Process_Add(&item->hcomp, emf_err);
    if(item->hcomp.comp_out == 1)
    {
        Motor_Diag_Fault_Flag |= 0x02; // 设置堵转故障标志
    }
}

void Motor_Diag_Init(void)
{
    Hysteresis_Comp_Init(&Motor_Over_Speed_Diag.hcomp, MOTOR_OVER_SPEED_THRESHOLD, 0, MOTOR_OVER_SPEED_THRESHOLD_DELAY);
    Diag_List_Register(&Motor_Diag_Head, &Motor_Over_Speed_Diag.node, MOTOR_Over_Speed_Update);

    Hysteresis_Comp_Init(&Motor_Phase_Lock.hcomp, 0, 0, 0);

    Hysteresis_Comp_Init(&Motor_Block_Detect_Diag.hcomp, 2.0f, 0.5f, 1000);
    Diag_List_Register(&Motor_Diag_Head, &Motor_Block_Detect_Diag.node, MOTOR_BLOCK_DETECT);
}

void Motor_Diag_Task(void)
{
    Diag_List_Traverse(Motor_Diag_Head);
}
