#ifndef MOTOR_PARAMETER_H
#define MOTOR_PARAMETER_H


#include <stdint.h>


#define MOTOR_POLE_PAIRS       4U
#define MOTOR_MAX_RPM          6000.0f
#define MOTOR_MAX_CURRENT_A    30.0f
#define MOTOR_VOLTAGE_LIMIT_V  48.0f
#define MOTOR_KV_RPM_PER_V     120.0f
#define MOTOR_PHASE_RESISTANCE_OHM  0.2f    
#define MOTOR_PHASE_INDUCTANCE_D_H    0.00050f
#define MOTOR_PHASE_INDUCTANCE_Q_H    0.00059f
#define MOTOR_FLUX_RPM_per_V      4.3f
#define MOTOR_FLUX_LINKAGE_Wb      (MOTOR_FLUX_RPM_per_V / MOTOR_POLE_PAIRS / 1000.0f * 7.79f * 1.414f)


typedef struct MOTOR
{
    float pole_pairs;
    float max_rpm;
    float max_current_a;
    float voltage_limit_v;
    float kv_rpm_per_v;
    float phase_resistance_ohm;
    float phase_inductance_d;
    float phase_inductance_q;
    float phase_inductance_s;
    float flux_linkage_wb;
    float *IF_Start_Iq_A;
    float *IF_Start_Ramp_Sec;
    float *IF_Start_Speed_RPM;
    uint32_t IF_Start_Profile_Length;
} MOTOR_t;

/*current parameters*/
#define  MOTOR_CURRENT_LOOP_HZ  20000.0f
#define MOTOR_CURRENT_LOOP_CYCLE_TIME_S   (1.0f/ MOTOR_CURRENT_LOOP_HZ)
#define MOTOR_SPEED_LOOP_HZ    1000.0f
#define MOTOR_SPEED_LOOP_CYCLE_TIME_S   (1.0f/ MOTOR_SPEED_LOOP_HZ)

#endif // MOTOR_PARAMETER_H
