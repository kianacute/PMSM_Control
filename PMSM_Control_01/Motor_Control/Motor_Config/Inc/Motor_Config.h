#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H


#include <stdint.h>
#include "Hal_Math.h"

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
    float Power_Limit; // 功率限制参数
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

#define MOTOR_DEAD_ZONE_THD                 (0.5f)

/*电机启动方式*/
// #define MOTOR_OPEN_SETUP                    // 电机开环启动
#define MOTOR_CLOSE_SETUP                     // 电机闭环启动

/*电机观测器选择*/
// #define MOTOR_SMO_OBSERVER                 // 滑模观测器
// #define MOTOR_NONFLUX_OBSERVER             // 非磁链观测器
#define MOTOR_EFFECTIVE_FLUX_OBSERVER         // 有效磁链观测器
// #define MOTOR_ENCODER_OBSERVER             // 编码器观测器
// #define MOTOR_HFI_OBSERVER                 // 高频注入观测器

// #define U_base   25.0f
// #define I_base   10.0f
// #define F_BASE   20000.0f
// #define L_BASE   (U_base/(I_base*F_BASE))
// #define T_BASE   (1.0f/F_BASE)
// #define R_BASE   (U_base/I_base)

void Motor_Config_Init(void);

#endif // MOTOR_CONFIG_H
