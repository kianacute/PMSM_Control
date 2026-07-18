#include "Motor_Config.h"


/*IF启动参数查表*/
extern const float IF_Start_Ramp_Sec[10];
extern const float IF_Start_Iq_A[10];
extern const float IF_Start_Speed_RPM[10];

// 速度环参数查表
extern const float Speed_Loop_Speed_Index[10];
extern const float Speed_Loop_Speed_PI_Kp_1D[10];
extern const float Speed_Loop_Speed_PI_Ki_1D[10];

// 电流环参数查表
extern const float Current_Lookup_Speed_index[10];
extern const float Current_ID_PI_Kp_Lookup_1D[10];
extern const float Current_IQ_PI_Kp_Lookup_1D[10];
extern const float Current_ID_PI_Ki_Lookup_1D[10];
extern const float Current_IQ_PI_Ki_Lookup_1D[10];

// 磁链观测器参数查表
extern const float NonFlux_Lookup_Speed_index[10];
extern const float NonFlux_Gama_Lookup_1D[10];
extern const float NonFlux_PLL_Kp_Lookup_1D[10];
extern const float NonFlux_PLL_Ki_Lookup_1D[10];

// 滑模观测器参数查表
extern const float SMO_Lookup_Speed_index[10];
extern const float SMO_Gain_Lookup_1D[10];
extern const float SMO_PLL_Kp_Lookup_1D[10];
extern const float SMO_PLL_Ki_Lookup_1D[10];


Motor_Parameter_t PMSM_42JS_Parameter;
Motor_Config_t PMSM_42JS_Config;

void Motor_Parameter_Init(void)
{
    PMSM_42JS_Parameter.pole_pairs = 2;
    PMSM_42JS_Parameter.max_rpm = 5000.0f;
    PMSM_42JS_Parameter.max_current_a = 20.0f;
    PMSM_42JS_Parameter.voltage_limit_v = 50.0f;
    PMSM_42JS_Parameter.flux_rpm_per_v = 4.3f;
    PMSM_42JS_Parameter.Rs = 0.4f;
    PMSM_42JS_Parameter.rs_identified = 0;
    PMSM_42JS_Parameter.Ld = 0.00050f;
    PMSM_42JS_Parameter.Lq = 0.00059f;
    PMSM_42JS_Parameter.Ls = (PMSM_42JS_Parameter.Ld + PMSM_42JS_Parameter.Lq) / 2;
    PMSM_42JS_Parameter.flux_rpm_per_v = 1.3f;
    PMSM_42JS_Parameter.flux_linkage_wb = (PMSM_42JS_Parameter.flux_rpm_per_v / PMSM_42JS_Parameter.pole_pairs
                         / 100.0f / PI * 3.0f); 
    /*磁链计算参考文章：https://www.zhihu.com/question/606311981/answer/3091158625 */
}

void Motor_Config_Init(void)
{    
    Motor_Parameter_Init();
    PMSM_42JS_Config.motor_param = &PMSM_42JS_Parameter;
    // IF启动参数查表初始化
    PMSM_42JS_Config.IF_Start_Iq_Lookup.x_table = IF_Start_Ramp_Sec;
    PMSM_42JS_Config.IF_Start_Iq_Lookup.y_table = IF_Start_Iq_A;
    PMSM_42JS_Config.IF_Start_Iq_Lookup.table_size = sizeof(IF_Start_Ramp_Sec) / sizeof(float);

    PMSM_42JS_Config.IF_Start_Speed_Lookup.x_table = IF_Start_Ramp_Sec;
    PMSM_42JS_Config.IF_Start_Speed_Lookup.y_table = IF_Start_Speed_RPM;
    PMSM_42JS_Config.IF_Start_Speed_Lookup.table_size = sizeof(IF_Start_Speed_RPM) / sizeof(float);
    // 电流环参数查表初始化
    PMSM_42JS_Config.ID_PI_Kp_Lookup.x_table = Current_Lookup_Speed_index;
    PMSM_42JS_Config.ID_PI_Kp_Lookup.y_table = Current_ID_PI_Kp_Lookup_1D;
    PMSM_42JS_Config.ID_PI_Kp_Lookup.table_size = sizeof(Current_Lookup_Speed_index) / sizeof(float);

    PMSM_42JS_Config.IQ_PI_Kp_Lookup.x_table = Current_Lookup_Speed_index;
    PMSM_42JS_Config.IQ_PI_Kp_Lookup.y_table = Current_IQ_PI_Kp_Lookup_1D;
    PMSM_42JS_Config.IQ_PI_Kp_Lookup.table_size = sizeof(Current_Lookup_Speed_index) / sizeof(float);

    PMSM_42JS_Config.ID_PI_Ki_Lookup.x_table = Current_Lookup_Speed_index;
    PMSM_42JS_Config.ID_PI_Ki_Lookup.y_table = Current_ID_PI_Ki_Lookup_1D;
    PMSM_42JS_Config.ID_PI_Ki_Lookup.table_size = sizeof(Current_Lookup_Speed_index) / sizeof(float);

    PMSM_42JS_Config.IQ_PI_Ki_Lookup.x_table = Current_Lookup_Speed_index;
    PMSM_42JS_Config.IQ_PI_Ki_Lookup.y_table = Current_IQ_PI_Ki_Lookup_1D;
    PMSM_42JS_Config.IQ_PI_Ki_Lookup.table_size = sizeof(Current_Lookup_Speed_index) / sizeof(float);
    // 速度环参数查表初始化
    PMSM_42JS_Config.Speed_PI_Kp_Lookup.x_table = Speed_Loop_Speed_Index;
    PMSM_42JS_Config.Speed_PI_Kp_Lookup.y_table = Speed_Loop_Speed_PI_Kp_1D;
    PMSM_42JS_Config.Speed_PI_Kp_Lookup.table_size = sizeof(Speed_Loop_Speed_Index) / sizeof(float);

    PMSM_42JS_Config.Speed_PI_Ki_Lookup.x_table = Speed_Loop_Speed_Index;
    PMSM_42JS_Config.Speed_PI_Ki_Lookup.y_table = Speed_Loop_Speed_PI_Ki_1D;
    PMSM_42JS_Config.Speed_PI_Ki_Lookup.table_size = sizeof(Speed_Loop_Speed_Index) / sizeof(float);
    // 磁链观测器查表初始化
    PMSM_42JS_Config.NonFlux_PLL_Kp_Lookup.x_table = NonFlux_Lookup_Speed_index;
    PMSM_42JS_Config.NonFlux_PLL_Kp_Lookup.y_table = NonFlux_PLL_Kp_Lookup_1D;
    PMSM_42JS_Config.NonFlux_PLL_Kp_Lookup.table_size = sizeof(NonFlux_Lookup_Speed_index) / sizeof(float);

    PMSM_42JS_Config.NonFlux_PLL_Ki_Lookup.x_table = NonFlux_Lookup_Speed_index;
    PMSM_42JS_Config.NonFlux_PLL_Ki_Lookup.y_table = NonFlux_PLL_Ki_Lookup_1D;
    PMSM_42JS_Config.NonFlux_PLL_Ki_Lookup.table_size = sizeof(NonFlux_Lookup_Speed_index) / sizeof(float);

    PMSM_42JS_Config.NonFlux_Gama_Lookup.x_table = NonFlux_Lookup_Speed_index;
    PMSM_42JS_Config.NonFlux_Gama_Lookup.y_table = NonFlux_Gama_Lookup_1D;
    PMSM_42JS_Config.NonFlux_Gama_Lookup.table_size = sizeof(NonFlux_Lookup_Speed_index) / sizeof(float);

    // SMO观测器查表初始化
    PMSM_42JS_Config.SMO_PLL_Kp_Lookup.x_table = SMO_Lookup_Speed_index;
    PMSM_42JS_Config.SMO_PLL_Kp_Lookup.y_table = SMO_PLL_Kp_Lookup_1D;
    PMSM_42JS_Config.SMO_PLL_Kp_Lookup.table_size = sizeof(SMO_Lookup_Speed_index) / sizeof(float);

    PMSM_42JS_Config.SMO_PLL_Ki_Lookup.x_table = SMO_Lookup_Speed_index;
    PMSM_42JS_Config.SMO_PLL_Ki_Lookup.y_table = SMO_PLL_Ki_Lookup_1D;
    PMSM_42JS_Config.SMO_PLL_Ki_Lookup.table_size = sizeof(SMO_Lookup_Speed_index) / sizeof(float);

    PMSM_42JS_Config.SMO_Gain_Lookup.x_table = SMO_Lookup_Speed_index;
    PMSM_42JS_Config.SMO_Gain_Lookup.y_table = SMO_Gain_Lookup_1D;
    PMSM_42JS_Config.SMO_Gain_Lookup.table_size = sizeof(SMO_Lookup_Speed_index) / sizeof(float);
    
}
