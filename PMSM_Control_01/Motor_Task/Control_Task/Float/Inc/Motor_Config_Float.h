#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H


#include <stdint.h>
#include "Hal_Math_Float.h"
#include "Motor_Control.h"

/*current parameters*/
#define MOTOR_CURRENT_LOOP_HZ               (20000.0f)
#define MOTOR_CURRENT_LOOP_CYCLE_TIME_S     (1.0f/ MOTOR_CURRENT_LOOP_HZ)
#define MOTOR_SPEED_LOOP_HZ                 (1000.0f)
#define MOTOR_SPEED_LOOP_CYCLE_TIME_S       (1.0f/ MOTOR_SPEED_LOOP_HZ)
#define Dead_TIME_DUTY                      (120.0f/160.0f*1e-6/MOTOR_CURRENT_LOOP_CYCLE_TIME_S)
// #define Dead_TIME_DUTY         (0/MOTOR_CURRENT_LOOP_HZ)

#define MOTOR_DEAD_ZONE_THD                 (0.5f)

/*电机启动方式*/
#define MOTOR_OPEN_SETUP                    // 电机开环启动
// #define MOTOR_CLOSE_SETUP                     // 电机闭环启动

/*电机观测器选择*/
// #define MOTOR_SMO_OBSERVER                 // 滑模观测器
// #define MOTOR_NONFLUX_OBSERVER             // 非磁链观测器
#define MOTOR_EFFECTIVE_FLUX_OBSERVER      // 有效磁链观测器
// #define MOTOR_ENCODER_OBSERVER             // 编码器观测器
// #define MOTOR_HFI_OBSERVER                 // 高频注入观测器


#define MOTOR_SPEED_MIDDLE_THD       (500.0f)

#define U_BASE   86.0f
#define I_BASE   45.0f
#define F_BASE   20000.0f
#define L_BASE   (U_BASE/(I_BASE*F_BASE))
#define T_BASE   (1.0f/F_BASE)
#define R_BASE   (U_BASE/I_BASE)

#endif // MOTOR_CONFIG_H
