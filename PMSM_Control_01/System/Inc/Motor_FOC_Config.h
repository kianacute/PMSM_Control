#ifndef __MOTOR_FOC_CONFIG_H__
#define __MOTOR_FOC_CONFIG_H__

#define MOTOR_CONTROL_FLOAT         // Use floating-point control for motor FOC
#define MOTOR_CONTROL_FIXED         // Use fixed-point control for motor FOC



/*System Paramater*/

#define SYSTEM_HZ               (1000.0f)
#define SYSTEM_LV_INIT_TIME         (uint32_t)(3.0f * SYSTEM_HZ)
#define SYSTEM_HV_STANDY_TIME       (uint32_t)(0.1f * SYSTEM_HZ)
#define SYSTEM_WAIT_TIME            (uint32_t)(3.0f * SYSTEM_HZ)


#endif