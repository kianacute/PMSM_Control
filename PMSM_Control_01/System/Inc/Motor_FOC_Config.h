#ifndef __MOTOR_FOC_CONFIG_H__
#define __MOTOR_FOC_CONFIG_H__

#define MOTOR_CONTROL_FLOAT         // Use floating-point control for motor FOC
// #define MOTOR_CONTROL_FIXED         // Use fixed-point control for motor FOC


/*电流环参数*/

#define MOTOR_ADC_OFFSET_SAMPLE_CNT                 (100U)
#define WEAK_VOLTAGE_COMPENSATION                   (2.0f/3.0f)
#define PWM_OPEN                                    (1U)
#define PWM_CLOSE                                   (0U)


/*速度环参数*/

#define SPEED_ADD_STEP (1000 / 1000.0f)
#define SPEED_SUB_STEP (1000 / 1000.0f)
#define SPEED_ID_ADD_STEP (1.0 / 1000.0f)
#define SPEED_ID_SUB_STEP (1.0 / 1000.0f)
#define SPEED_SWITCH_ID_SUB_STEP (0.001f)

#define PWM_SWITH_FREQ_MAX  (20000.0f)
#define PWM_SWITH_FREQ_MIN  (1000.0f)
#define PWM_SWITH_FREQ_STEP    (2000.0f/1000.0f)


/*位置观测器选择*/
// #define MOTOR_SMO_OBSERVER                 // 滑模观测器
// #define MOTOR_NONFLUX_OBSERVER             // 非磁链观测器
#define MOTOR_EFFECTIVE_FLUX_OBSERVER         // 有效磁链观测器
// #define MOTOR_ENCODER_OBSERVER             // 编码器观测器
// #define MOTOR_HFI_OBSERVER                 // 高频注入观测器


/*System Paramater*/

#define SYSTEM_HZ               (1000.0f)
#define SYSTEM_LV_INIT_TIME         (uint32_t)(3.0f * SYSTEM_HZ)
#define SYSTEM_HV_STANDY_TIME       (uint32_t)(0.1f * SYSTEM_HZ)
#define SYSTEM_WAIT_TIME            (uint32_t)(3.0f * SYSTEM_HZ)

/*电机参数*/


/*current parameters*/
#define MOTOR_CURRENT_LOOP_HZ               (20000.0f)
#define MOTOR_CURRENT_LOOP_CYCLE_TIME_S     (1.0f/ MOTOR_CURRENT_LOOP_HZ)
#define MOTOR_SPEED_LOOP_HZ                 (1000.0f)
#define MOTOR_SPEED_LOOP_CYCLE_TIME_S       (1.0f/ MOTOR_SPEED_LOOP_HZ)
#define Dead_TIME_DUTY                      (120.0f/160.0f*1e-6f/MOTOR_CURRENT_LOOP_CYCLE_TIME_S)
// #define Dead_TIME_DUTY         (0/MOTOR_CURRENT_LOOP_HZ)

#define MOTOR_DEAD_ZONE_THD                 (0.5f)

/*电机启动方式*/
// #define MOTOR_OPEN_SETUP                    // 电机开环启动
#define MOTOR_CLOSE_SETUP                     // 电机闭环启动


#define MOTOR_SPEED_MIDDLE_THD       (500.0f)

#define U_BASE   86.0f
#define I_BASE   45.0f
#define F_BASE   20000.0f
#define L_BASE   (U_BASE/(I_BASE*F_BASE))
#define T_BASE   (1.0f/F_BASE)
#define R_BASE   (U_BASE/I_BASE)


#endif

