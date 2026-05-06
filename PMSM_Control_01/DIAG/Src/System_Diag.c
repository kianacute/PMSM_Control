#include "System_Diag.h"
#include "speed_ctrl.h"
#include "Current_Task.h"
#include "system.h"

extern Speed_Ctrl_t Speed_Ctrl;
extern Current_Task_t Current_Task;
extern SYSTEM_t System;


Sensor_Hysteresis_Comp_TypeDef Hv_Sensor = {
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
};


void System_Diag_Init(void)
{

}

void System_Diag_Task(void)
{
    if(System.system_state >= SYSTEM_CMD_STANDY)
    {
        Sensor_Hysteresis_Comp_Process(&Hv_Sensor, Current_Task.Udc_ADISR);
    }
}
