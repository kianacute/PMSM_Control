#include "Motor_Diag.h"
#include "Speed_Loop.h"
#include "Current_Loop.h"
#include "Observer.h"
#include "System_Loop.h"

extern Speed_Loop_t Speed_Loop;
extern SYSTEM_t System ;
extern struct EMF_Cal_Parameter EMF_Cal;
extern Current_Loop_t Current_Loop;

static Diag_Node_t *Motor_Diag_Head = NULL;

static Motor_Diag_Item_t Motor_Over_Speed_Diag;
static Motor_Diag_Item_t Motor_Phase_A_Lock, Motor_Phase_B_Lock, Motor_Phase_C_Lock;
static Motor_Diag_Item_t Motor_Block_Detect_Diag;

uint8_t Motor_Diag_Fault_Flag = 0;

static void MOTOR_Over_Speed_Update(Diag_Node_t *node)
{
    Motor_Diag_Item_t *item = (Motor_Diag_Item_t *)node;
    Hysteresis_Comp_Process_Add(&item->hcomp, Speed_Loop.Speed_Fb);
    if(item->hcomp.comp_out == 1)
    {
        Motor_Diag_Fault_Flag |= 0x01; // 设置过速故障标志
    }
}

static void MOTOR_PHASE_LOCK(Diag_Node_t *node)
{
    Motor_Diag_Item_t *item = (Motor_Diag_Item_t *)node;
    if(Current_Loop.Motor_State == MOTOR_WAIT || Current_Loop.Motor_State == MOTOR_IDLE)
    {
        Motor_Phase_A_Lock.hcomp.reset = 1;
        Motor_Phase_B_Lock.hcomp.reset = 1;
        Motor_Phase_C_Lock.hcomp.reset = 1;
    }
    else
    {
        Motor_Phase_A_Lock.hcomp.reset = 0;
        Motor_Phase_B_Lock.hcomp.reset = 0;
        Motor_Phase_C_Lock.hcomp.reset = 0;
    }
    if(Current_Loop.Motor_State == MOTOR_RUN)
    {
        Motor_Phase_A_Lock.hcomp.enable = 1;
        Motor_Phase_B_Lock.hcomp.enable = 1;
        Motor_Phase_C_Lock.hcomp.enable = 1;
        uint32_t delay_time = -Speed_Loop.Speed_Ref * 0.24f + 750;
        if(delay_time < 10)
        {
            delay_time = 10;
        }
        Motor_Phase_A_Lock.hcomp.delay_time = delay_time;
        Motor_Phase_B_Lock.hcomp.delay_time = delay_time;
        Motor_Phase_C_Lock.hcomp.delay_time = delay_time;
    }

    float err = my_abs(Current_Loop.A_Max - 0);
    Hysteresis_Comp_Process_Sub(&Motor_Phase_A_Lock.hcomp, err);
    if(Motor_Phase_A_Lock.hcomp.comp_out == 1)
    {
        Motor_Diag_Fault_Flag |= 0x04; // 设置A相锁定故障标志
    }

    err = my_abs(Current_Loop.B_Max - 0);
    Hysteresis_Comp_Process_Sub(&Motor_Phase_B_Lock.hcomp, err);
    if(Motor_Phase_B_Lock.hcomp.comp_out == 1)
    {
        Motor_Diag_Fault_Flag |= 0x08; // 设置B相锁定故障标志
    }
    err = my_abs(Current_Loop.C_Max - 0);
    Hysteresis_Comp_Process_Sub(&Motor_Phase_C_Lock.hcomp, err);
    if(Motor_Phase_C_Lock.hcomp.comp_out == 1)
    {
        Motor_Diag_Fault_Flag |= 0x10; // 设置C相锁定故障标志
    }
}

float emf_err = 0.0f;

/*堵转故障*/
static void MOTOR_BLOCK_DETECT(Diag_Node_t *node)
{
    Motor_Diag_Item_t *item = (Motor_Diag_Item_t *)node;
    if(Current_Loop.Motor_State == MOTOR_WAIT || Current_Loop.Motor_State == MOTOR_IDLE)
    {
        item->hcomp.reset = 1; // 系统未运行时复位比较器
    }
    else
    {
        item->hcomp.reset = 0; // 系统运行时正常工作
    }
    if(Current_Loop.Motor_State == MOTOR_RUN)
    {
        item->hcomp.enable = 1; // 系统运行时使能比较器
    }
    emf_err = my_abs(Speed_Loop.Speed_Fb * Speed_Loop.pMotor->motor_param->flux_rpm_per_v / 1000.0f - EMF_Cal.EMF);
    Hysteresis_Comp_Process_Add(&item->hcomp, emf_err);
    if(item->hcomp.comp_out == 1)
    {
        Motor_Diag_Fault_Flag |= 0x02; // 设置堵转故障标志
    }
}

void Motor_Diag_Init(void)
{
    Hysteresis_Comp_Init(&Motor_Over_Speed_Diag.hcomp, MOTOR_OVER_SPEED_THRESHOLD, 0, MOTOR_OVER_SPEED_THRESHOLD_DELAY);
    // Diag_List_Register(&Motor_Diag_Head, &Motor_Over_Speed_Diag.node, MOTOR_Over_Speed_Update);

    Hysteresis_Comp_Init(&Motor_Phase_A_Lock.hcomp, 0.5, 0.1, 500);
    Hysteresis_Comp_Init(&Motor_Phase_B_Lock.hcomp, 0.5, 0.1, 500);
    Hysteresis_Comp_Init(&Motor_Phase_C_Lock.hcomp, 0.5, 0.1, 500);
    Diag_List_Register(&Motor_Diag_Head, &Motor_Phase_A_Lock.node, MOTOR_PHASE_LOCK);

    Hysteresis_Comp_Init(&Motor_Block_Detect_Diag.hcomp, 2.0f, 0.5f, 1000);
    Diag_List_Register(&Motor_Diag_Head, &Motor_Block_Detect_Diag.node, MOTOR_BLOCK_DETECT);
}

void Motor_Diag_Task(void)
{
    Diag_List_Traverse(Motor_Diag_Head);
}
