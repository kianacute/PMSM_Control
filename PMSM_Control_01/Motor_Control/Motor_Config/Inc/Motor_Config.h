#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H


#include <stdint.h>
#include "Hal_Math.h"

#define MOTOR_POLE_PAIRS       4U
#define MOTOR_MAX_RPM          6000.0f
#define MOTOR_MAX_CURRENT_A    20.0f
#define MOTOR_VOLTAGE_LIMIT_V  48.0f
#define MOTOR_PHASE_RESISTANCE_OHM  0.2f    
#define MOTOR_PHASE_INDUCTANCE_D_H    0.00050f
#define MOTOR_PHASE_INDUCTANCE_Q_H    0.00059f
#define MOTOR_FLUX_RPM_PER_V      4.3f
#define MOTOR_FLUX_LINKAGE_Wb      (MOTOR_FLUX_RPM_PER_V / MOTOR_POLE_PAIRS / 1000.0f * 7.79f * 1.414f)

typedef struct Motor_Parameter
{
    float pole_pairs;      // 电机极对数
    float Rs;              // 定子电阻
    float Ld, Lq, Ls;      // 定子电感
    float flux_rpm_per_v;  // 反电动势系数，单位为Vs/rpm
    float flux_linkage_wb; // 磁链, 单位为Wb
    float max_rpm;         // 最大转速
    float max_current_a;   // 最大电流
    float voltage_limit_v; // 电压限制，单位为V
    uint8_t rs_identified; // Rs离线辨识标志: 0=未辨识, 1=已辨识成功
} Motor_Parameter_t;

typedef struct Motor_Config
{
    Motor_Parameter_t *motor_param; // 电机参数

    // IF启动参数
    Lookup_Table_t IF_Start_Speed_Lookup;           // 启动速度查表
    Lookup_Table_t IF_Start_Iq_Lookup;              // 启动Iq查表

    // 电流环查表参数
    Lookup_Table_t ID_PI_Kp_Lookup;  
    Lookup_Table_t IQ_PI_Kp_Lookup; 
    Lookup_Table_t ID_PI_Ki_Lookup; 
    Lookup_Table_t IQ_PI_Ki_Lookup; 

    // 速度环查表参数
    Lookup_Table_t Speed_PI_Kp_Lookup; // 速度PI比例增益查表
    Lookup_Table_t Speed_PI_Ki_Lookup; // 速度PI积分增益查表    

    //磁链观测器查表参数
    Lookup_Table_t NonFlux_PLL_Kp_Lookup;
    Lookup_Table_t NonFlux_PLL_Ki_Lookup;
    Lookup_Table_t NonFlux_Gama_Lookup;
    

    //SMO观测器查表参数
    Lookup_Table_t SMO_PLL_Kp_Lookup;
    Lookup_Table_t SMO_PLL_Ki_Lookup;
    Lookup_Table_t SMO_Gain_Lookup;

} Motor_Config_t;

/*current parameters*/
#define MOTOR_CURRENT_LOOP_HZ               (20000.0f)
#define MOTOR_CURRENT_LOOP_CYCLE_TIME_S     (1.0f/ MOTOR_CURRENT_LOOP_HZ)
#define MOTOR_SPEED_LOOP_HZ                 (1000.0f)
#define MOTOR_SPEED_LOOP_CYCLE_TIME_S       (1.0f/ MOTOR_SPEED_LOOP_HZ)
#define Dead_TIME_DUTY                      (120.0f/160.0f*1e-6/MOTOR_CURRENT_LOOP_CYCLE_TIME_S)
// #define Dead_TIME_DUTY         (0/MOTOR_CURRENT_LOOP_HZ)


// #define U_base   25.0f
// #define I_base   10.0f
// #define F_BASE   20000.0f
// #define L_BASE   (U_base/(I_base*F_BASE))
// #define T_BASE   (1.0f/F_BASE)
// #define R_BASE   (U_base/I_base)

void Motor_Config_Init(void);

#endif // MOTOR_CONFIG_H
