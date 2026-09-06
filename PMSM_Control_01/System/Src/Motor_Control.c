#include "Motor_Control.h"
#include "Motor_Lookup_Tables.h"

Motor_Control_t PMSM_42J; 
Motor_Parameter_t PMSM_42JS_Parameter;
Motor_Config_t PMSM_42JS_Config;

void Motor_Parameter_Init(Motor_Control_t *pMotor_Control)
{
    // 电机基本参数数值来自 Motor_Parameters.csv，由 generate.py 生成到
    // Motor_Lookup_Tables.h 的宏定义中，请勿在此处直接改数值
    PMSM_42JS_Parameter.Pn = MOTOR_PN;
    PMSM_42JS_Parameter.Current_Max_A = 1.0f;
    PMSM_42JS_Parameter.Bus_Voltage_Max = 1.0f;
    PMSM_42JS_Parameter.Flux_Vkrpm = MOTOR_FLUX_VKRPM / MOTOR_FLUX_BASE;
    PMSM_42JS_Parameter.Rs = MOTOR_RS / MOTOR_R_BASE;
    PMSM_42JS_Parameter.Ld = MOTOR_LD / MOTOR_L_BASE;
    PMSM_42JS_Parameter.Lq = MOTOR_LQ / MOTOR_L_BASE;
    PMSM_42JS_Parameter.Power_Max_W = MOTOR_POWER_MAX_W / MOTOR_POWER_BASE;
    PMSM_42JS_Parameter.Speed_Max_Rpm = 1.0f;
    PMSM_42JS_Parameter.Ls = (PMSM_42JS_Parameter.Ld + PMSM_42JS_Parameter.Lq) / 2;
    PMSM_42JS_Parameter.flux_linkage_wb = MOTOR_FLUX_VS / MOTOR_FLUX_BASE;
    PMSM_42JS_Parameter.Flux_Flux  = PMSM_42JS_Parameter.flux_linkage_wb * PMSM_42JS_Parameter.flux_linkage_wb;
    PMSM_42JS_Parameter.Ld_Lq = PMSM_42JS_Parameter.Ld - PMSM_42JS_Parameter.Lq;
    PMSM_42JS_Parameter.One_per_Flux = 1 / PMSM_42JS_Parameter.flux_linkage_wb;
}

void Motor_Config_Init(Motor_Control_t *pMotor_Control)
{
    Motor_Parameter_Init(pMotor_Control);
    pMotor_Control->Motor_Config = (Motor_Config_t*)&PMSM_42JS_Config;
    pMotor_Control->Motor_Config->Motor_Param = &PMSM_42JS_Parameter;

    // IF启动参数查表初始化
    PMSM_42JS_Config.IF_Start_Iq_Lookup.x_table = IF_Start_Ramp_Sec;
    PMSM_42JS_Config.IF_Start_Iq_Lookup.y_table = IF_Start_Iq_A;
    PMSM_42JS_Config.IF_Start_Iq_Lookup.table_size = sizeof(IF_Start_Iq_A) / sizeof(float);
    PMSM_42JS_Config.IF_Start_Speed_Lookup.x_table = IF_Start_Ramp_Sec;
    PMSM_42JS_Config.IF_Start_Speed_Lookup.y_table = IF_Start_Speed_RPM;
    PMSM_42JS_Config.IF_Start_Speed_Lookup.table_size = sizeof(IF_Start_Speed_RPM) / sizeof(float);

    // 电流环参数查表初始化
    PMSM_42JS_Config.ID_PI_Kp_Lookup.x_table = Current_Lookup_Speed_index;
    PMSM_42JS_Config.ID_PI_Kp_Lookup.y_table = Current_ID_PI_Kp_Lookup_1D;
    PMSM_42JS_Config.ID_PI_Kp_Lookup.table_size = sizeof(Current_ID_PI_Kp_Lookup_1D) / sizeof(float);
    PMSM_42JS_Config.ID_PI_Ki_Lookup.x_table = Current_Lookup_Speed_index;
    PMSM_42JS_Config.ID_PI_Ki_Lookup.y_table = Current_ID_PI_Ki_Lookup_1D;
    PMSM_42JS_Config.ID_PI_Ki_Lookup.table_size = sizeof(Current_ID_PI_Ki_Lookup_1D) / sizeof(float);
    PMSM_42JS_Config.IQ_PI_Kp_Lookup.x_table = Current_Lookup_Speed_index;
    PMSM_42JS_Config.IQ_PI_Kp_Lookup.y_table = Current_IQ_PI_Kp_Lookup_1D;
    PMSM_42JS_Config.IQ_PI_Kp_Lookup.table_size = sizeof(Current_IQ_PI_Kp_Lookup_1D) / sizeof(float);
    PMSM_42JS_Config.IQ_PI_Ki_Lookup.x_table = Current_Lookup_Speed_index;
    PMSM_42JS_Config.IQ_PI_Ki_Lookup.y_table = Current_IQ_PI_Ki_Lookup_1D;
    PMSM_42JS_Config.IQ_PI_Ki_Lookup.table_size = sizeof(Current_IQ_PI_Ki_Lookup_1D) / sizeof(float);

    // 速度环参数查表初始化
    PMSM_42JS_Config.Speed_PI_Kp_Lookup.x_table = Speed_Loop_Speed_Index;
    PMSM_42JS_Config.Speed_PI_Kp_Lookup.y_table = Speed_Loop_Speed_PI_Kp_1D;
    PMSM_42JS_Config.Speed_PI_Kp_Lookup.table_size = sizeof(Speed_Loop_Speed_PI_Kp_1D) / sizeof(float);
    PMSM_42JS_Config.Speed_PI_Ki_Lookup.x_table = Speed_Loop_Speed_Index;
    PMSM_42JS_Config.Speed_PI_Ki_Lookup.y_table = Speed_Loop_Speed_PI_Ki_1D;
    PMSM_42JS_Config.Speed_PI_Ki_Lookup.table_size = sizeof(Speed_Loop_Speed_PI_Ki_1D) / sizeof(float);

    // 磁链观测器参数查表初始化
    PMSM_42JS_Config.NonFlux_PLL_Kp_Lookup.x_table = NonFlux_Lookup_Speed_index;
    PMSM_42JS_Config.NonFlux_PLL_Kp_Lookup.y_table = NonFlux_PLL_Kp_Lookup_1D;
    PMSM_42JS_Config.NonFlux_PLL_Kp_Lookup.table_size = sizeof(NonFlux_PLL_Kp_Lookup_1D) / sizeof(float);
    PMSM_42JS_Config.NonFlux_PLL_Ki_Lookup.x_table = NonFlux_Lookup_Speed_index;
    PMSM_42JS_Config.NonFlux_PLL_Ki_Lookup.y_table = NonFlux_PLL_Ki_Lookup_1D;
    PMSM_42JS_Config.NonFlux_PLL_Ki_Lookup.table_size = sizeof(NonFlux_PLL_Ki_Lookup_1D) / sizeof(float);
    PMSM_42JS_Config.NonFlux_Gama_Lookup.x_table = NonFlux_Lookup_Speed_index;
    PMSM_42JS_Config.NonFlux_Gama_Lookup.y_table = NonFlux_Gama_Lookup_1D;
    PMSM_42JS_Config.NonFlux_Gama_Lookup.table_size = sizeof(NonFlux_Gama_Lookup_1D) / sizeof(float);
    PMSM_42JS_Config.EfFlux_Gama_Lookup.x_table = NonFlux_Lookup_Speed_index;
    PMSM_42JS_Config.EfFlux_Gama_Lookup.y_table = EfFlux_Gama_Lookup_1D;
    PMSM_42JS_Config.EfFlux_Gama_Lookup.table_size = sizeof(EfFlux_Gama_Lookup_1D) / sizeof(float);

    // SMO观测器参数查表初始化
    PMSM_42JS_Config.SMO_PLL_Kp_Lookup.x_table = SMO_Lookup_Speed_index;
    PMSM_42JS_Config.SMO_PLL_Kp_Lookup.y_table = SMO_PLL_Kp_Lookup_1D;
    PMSM_42JS_Config.SMO_PLL_Kp_Lookup.table_size = sizeof(SMO_PLL_Kp_Lookup_1D) / sizeof(float);
    PMSM_42JS_Config.SMO_PLL_Ki_Lookup.x_table = SMO_Lookup_Speed_index;
    PMSM_42JS_Config.SMO_PLL_Ki_Lookup.y_table = SMO_PLL_Ki_Lookup_1D;
    PMSM_42JS_Config.SMO_PLL_Ki_Lookup.table_size = sizeof(SMO_PLL_Ki_Lookup_1D) / sizeof(float);
    PMSM_42JS_Config.SMO_Gain_Lookup.x_table = SMO_Lookup_Speed_index;
    PMSM_42JS_Config.SMO_Gain_Lookup.y_table = SMO_Gain_Lookup_1D;
    PMSM_42JS_Config.SMO_Gain_Lookup.table_size = sizeof(SMO_Gain_Lookup_1D) / sizeof(float);

    // 角度补偿2D查表初始化
    PMSM_42JS_Config.EfFlux_Angle_Comp.x_table = NonFlux_Lookup_Speed_index;
    PMSM_42JS_Config.EfFlux_Angle_Comp.y_table = NonFlux_Lookup_Is_index;
    PMSM_42JS_Config.EfFlux_Angle_Comp.z_table = (const float *)EFFlux_Angle_Comp_table_2D;
    PMSM_42JS_Config.EfFlux_Angle_Comp.nx_size = sizeof(NonFlux_Lookup_Speed_index) / sizeof(float);
    PMSM_42JS_Config.EfFlux_Angle_Comp.ny_size = sizeof(NonFlux_Lookup_Is_index) / sizeof(float);
}


void Motor_Control_Init(Motor_Control_t *pControl)
{
    Current_Init(pControl);
    SPEED_Init(pControl);
    SYSTEM_Init(pControl);
    Motor_Config_Init(pControl);
}

void Current_Loop_Task(Motor_Control_t *pControl)
{
    // Code to switch current task states
    if (pControl->System_Loop.Status == SYSTEM_RUN)
    {
        switch (pControl->Current_Loop.Status)
        {
        case MOTOR_IDLE:
            // Handle idle state
            MOTOR_IDLE_TASK(pControl);
            break;
        case MOTOR_READY:
            // Handle ready state
            MOTOR_READY_TASK(pControl);
            break;
        case MOTOR_OFFSET_CHECK:
            // Handle offset check state
            MOTOR_OFFSET_CHECK_TASK(pControl);
            break;
        case MOTOR_RUN:
            // Handle run state
            MOTOR_RUN_TASK(pControl);
            break;
        case MOTOR_FAULT:
            // Handle fault state
            // Add fault handling code here
            MOTOR_FAULT_TASK(pControl);
            break;
        case MOTOR_WAIT:
            // Handle wait state
            MOTOR_WAIT_TASK(pControl);
            break;
        default:
            break;
        }
    }
    else
    {
        MOTOR_IDLE_TASK(pControl);
        pControl->Current_Loop.Status = MOTOR_IDLE;
    }
    pControl->Current_Loop.Loop_count++;
}

void Speed_Loop_Task(Motor_Control_t *pControl)
{
    if (pControl->System_Loop.Status == SYSTEM_RUN && pControl->Current_Loop.Status == MOTOR_RUN)
    {
        Paramater_update_Float(pControl);
        switch (pControl->Speed_Loop.Status)
        {
        case SPEED_IDLE:
            // Handle idle state
            SPEED_Idle_Task(pControl);
            break;
        case SPEED_ALIGN:
            // Handle align state
            SPEED_Align_Task(pControl);
            break;
        case SPEED_OPEN:
            // Handle open state
            SPEED_Open_Task(pControl);
            break;
        case SPEED_SWITCH:
            // Handle switch state
            SPEED_Switch_Task(pControl);
            break;
        case SPEED_LOW:
            // Handle low state
            SPEED_Low_Task(pControl);
            break;
        case SPEED_MIDDLE:
            // Handle middle state
            SPEED_Middle_Task(pControl);
            break;
        case SPEED_HIGH:
            // Handle high state
            SPEED_High_Task(pControl);
            break;
        default:
            pControl->Speed_Loop.Status = SPEED_IDLE;
            break;
        }
    }
    else
    {
        pControl->Speed_Loop.Status = SPEED_IDLE;
        SPEED_Idle_Task(pControl);
    }
    pControl->Speed_Loop.Loop_count++;
}

void SYSTEM_LOOP_Task(Motor_Control_t *pControl)
{
    pControl->System_Loop.Loop_count++;
    switch (pControl->System_Loop.Status)
    {
    case SYSTEM_LV_STANDY:
        SYSTEM_LV_Standy(pControl);
        break;
    case SYSTEM_HV_STANDY:
        SYSTEM_HV_Standy(pControl);
        break;
    case SYSTEM_RUN:
        SYSTEM_Run(pControl);
        break;
    case SYSTEM_FAULT:
        SYSTEM_Fault(pControl);
        break;
    case SYSTEM_WAIT:
        SYSTEM_Wait(pControl);
        break;
    default:
        break;
    }
}
