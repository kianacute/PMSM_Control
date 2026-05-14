#include "System_Diag.h"
#include "speed_ctrl.h"
#include "Current_Task.h"
#include "system.h"

extern Current_Task_t Current_Task;
extern SYSTEM_t System;

static Diag_Node_t *System_Diag_Head = NULL;

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
    Sensor_Hysteresis_Comp_Process(&item->hcomp, Current_Task.Udc_ADISR);
}

void System_Diag_Init(void)
{
    Diag_List_Register(&System_Diag_Head, &Hv_Sensor.node, Hv_Sensor_Update);
}

void System_Diag_Task(void)
{
    if (System.system_state >= SYSTEM_CMD_STANDY)
    {
        Diag_List_Traverse(System_Diag_Head);
    }
}
