#include "System_Diag.h"
#include "Speed_Loop.h"
#include "Current_Loop.h"
#include "System_Loop.h"

extern Current_Loop_t Current_Loop;
extern SYSTEM_t System;
extern Current_Loop_Input_t Current_Loop_Input;
extern Current_Loop_Output_t Current_Loop_Output;

static Diag_Node_t *System_Diag_Head = NULL;

uint8_t System_Diag_Fault_Flag = 0;

static Sensor_Diag_Item_t Hv_Sensor = {
    .hcomp = {
        .enable = 1,
        .reset = 0,
        .threshold_over = HV_OVER_THRESHOLD,
        .threshold_over_re = HV_OVER_RE_THRESHOLD,
        .threshold_power = HV_POWER_THRESHOLD,
        .threshold_low = HV_LOW_THRESHOLD,
        .threshold_low_re = HV_LOW_RE_THRESHOLD,
        .threshold_gnd = HV_GND_THRESHOLD,
        .analog_input_uplimit = HV_UPLIMIT,
        .analog_input_lowlimit = HV_LOWLIMIT,
        .delay_time = HV_DELAY_TIME,
        .delay_cnt = 0,
        .status = SENSOR_STATUS_NORMAL
    }
};

static void Hv_Sensor_Update(Diag_Node_t *node)
{
    Sensor_Diag_Item_t *item = (Sensor_Diag_Item_t *)node;
    if(System.system_state == SYSTEM_WAIT)
    {
        item->hcomp.reset = 1; // 系统未运行时复位比较器
    }
    else
    {
        item->hcomp.reset = 0; // 系统运行时正常工作
    }
    if(System.system_state >= SYSTEM_RUN)
    {
        item->hcomp.enable = 1; // 系统运行时使能比较器
    }
    Sensor_Hysteresis_Comp_Process(&item->hcomp, Current_Loop_Input.Udc_ADISR);
    if (item->hcomp.status != 0)
    {
        System_Diag_Fault_Flag |= 0x01; // 设置高压过压故障标志
    }
}

void System_Diag_Init(void)
{
    Diag_List_Register(&System_Diag_Head, &Hv_Sensor.node, Hv_Sensor_Update);
}

void System_Diag_Task(void)
{
    if (System.system_state >= SYSTEM_RUN)
    {
        Diag_List_Traverse(System_Diag_Head);
    }
}
