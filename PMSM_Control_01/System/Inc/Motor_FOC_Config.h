#ifndef __MOTOR_FOC_CONFIG_H__
#define __MOTOR_FOC_CONFIG_H__

#define MOTOR_CONTROL_FLOAT         // Use floating-point control for motor FOC
// #define MOTOR_CONTROL_FIXED         // Use fixed-point control for motor FOC
#include "Motor_Lookup_Tables.h"


/*电机参数*/


/*current parameters*/
#define MOTOR_CURRENT_LOOP_HZ               (20000.0f)
#define MOTOR_CURRENT_LOOP_CYCLE_TIME_S     (1.0f/ MOTOR_CURRENT_LOOP_HZ)
#define MOTOR_SPEED_LOOP_HZ                 (1000.0f)
#define MOTOR_SPEED_LOOP_CYCLE_TIME_S       (1.0f/ MOTOR_SPEED_LOOP_HZ)


// 电机参数宏（MOTOR_PN/MOTOR_CURRENT_MAX_A/...）由 generate.py 从
// Motor_Parameters.csv 生成到 Motor_Lookup_Tables.h（文件顶部，无条件定义），
// 本文件第 6 行已 include 该文件。

#define MOTOR_U_BASE        (MOTOR_BUS_VOLTAGE_MAX)                                        //母线电压采样最大值
#define MOTOR_I_BASE        (MOTOR_CURRENT_MAX_A)                                          //相电流采样最大值
#define MOTOR_FREQ_BASE     (MOTOR_SPEED_MAX_RPM/60.0f*MOTOR_PN)                            //最大电频率
#define MOTOR_WE_BASE       (MOTOR_FREQ_BASE*3.14159265358979f*2.0f)
#define MOTOR_L_BASE        (MOTOR_U_BASE/(MOTOR_I_BASE*MOTOR_WE_BASE))                     //电感基值
#define MOTOR_T_BASE        (1.0f/MOTOR_WE_BASE)                                            //时间基值
#define MOTOR_R_BASE        (MOTOR_U_BASE/MOTOR_I_BASE)                                     //电阻基值
#define MOTOR_FLUX_BASE     (MOTOR_U_BASE/MOTOR_WE_BASE)                                    //磁链基值
#define MOTOR_POWER_BASE    (MOTOR_U_BASE*MOTOR_I_BASE)                                     //功率基值
#define MOTOR_RPM_BASE      (MOTOR_SPEED_MAX_RPM)                                           //速度基值


/*电流环参数*/

#define MOTOR_ADC_OFFSET_SAMPLE_CNT                 (100U)
#define WEAK_VOLTAGE_COMPENSATION                   (2.0f/3.0f)
#define PWM_OPEN                                    (1U)
#define PWM_CLOSE                                   (0U)
#define Dead_TIME_S                                 ((120.0f/160.0f*1e-6f)/MOTOR_T_BASE)
#define MOTOR_DEAD_ZONE_THD_A                       (0.5f/MOTOR_I_BASE)
#define CUREENT_LOOP_DEAD_ZONE_ENABLE               (1U)
#define CURRENT_PID_OUTPUT_LIMIT                    (1.02f)
#define CURRENT_STOP_SPEED_RPM                      (3000.0f / MOTOR_RPM_BASE)

/*速度环参数*/

#define SPEED_LOW_ADD_STEP                          (200.0f / 1000.0f / MOTOR_RPM_BASE)
#define SPEED_LOW_ID_TARGET_A                       (0.5f/MOTOR_I_BASE)
#define SPEED_MIDDLE_HIGH_ADD_STEP                  (200.0f / 1000.0f / MOTOR_RPM_BASE)
#define SPEED_SUB_STEP                              (1000.0f / 1000.0f / MOTOR_RPM_BASE)
#define SPEED_ID_ADD_STEP                           (1.0f / 1000.0f / MOTOR_I_BASE)
#define SPEED_ID_SUB_STEP                           (1.0f / 1000.0f / MOTOR_I_BASE)
#define SPEED_SWITCH_ID_SUB_STEP                    (0.001f / MOTOR_I_BASE)
#define SPEED_MIDDLE_THD_RPM                        (500.0f/MOTOR_RPM_BASE)
#define SPEED_ALIGN_ID_A                            (1.0f/MOTOR_I_BASE)
#define SPEED_ALIGN_TIME_S                          (2u * MOTOR_SPEED_LOOP_HZ)

#define PWM_SWITH_FREQ_MAX                          (20000.0f)
#define PWM_SWITH_FREQ_MIN                          (1000.0f)
#define PWM_SWITH_FREQ_STEP                         (2000.0f/1000.0f)

#define SPEED_OPEN2SWITCH_THD_RPM                   (500.0f/MOTOR_RPM_BASE)


/*系统环参数*/

#define SYSTEM_HZ                                   (1000.0f)
#define SYSTEM_LV_INIT_TIME_S                       (uint32_t)(3.0f * SYSTEM_HZ)
#define SYSTEM_HV_STANDY_TIME_S                     (uint32_t)(0.1f * SYSTEM_HZ)
#define SYSTEM_HV_STANDY_THD_V                      (12.0f / MOTOR_BUS_VOLTAGE_MAX)
#define SYSTEM_WAIT_TIME_S                          (uint32_t)(3.0f * SYSTEM_HZ)


/*位置观测器选择*/
// #define MOTOR_SMO_OBSERVER                   // 滑模观测器
// #define MOTOR_NONFLUX_OBSERVER               // 非磁链观测器
#define MOTOR_EFFECTIVE_FLUX_OBSERVER           // 有效磁链观测器
// #define MOTOR_ENCODER_OBSERVER               // 编码器观测器
// #define MOTOR_HFI_OBSERVER                   // 高频注入观测器


/*电机启动方式*/
// #define MOTOR_OPEN_SETUP                        // 电机开环启动
#define MOTOR_CLOSE_SETUP                    // 电机闭环启动

/*System Paramater*/




#endif

