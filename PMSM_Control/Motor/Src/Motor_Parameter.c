#include "Motor_parameter.h"

const float PMSM_42JS_IF_Start_Ramp_Sec[10] = {0.0f, 0.5f, 1.0f, 1.5f, 2.0f, 2.5f, 3.0f, 3.5f, 4.0f, 5.0f};
const float PMSM_42JS_IF_Start_Iq_A[10] = {0.0f, 0.1f, 0.5f, 1.0f, 1.5f, 2.0f, 3.0f, 3.0f, 3.0f, 3.0f};
const float PMSM_42JS_IF_Start_Speed_RPM[10] = {0.0f, 20.0f, 50.0f, 100.0f, 250.0f, 400.0f, 600.0f, 600.0f, 600.0f, 600.0f}; 

MOTOR_t PMSM_42JS = {
    .pole_pairs = MOTOR_POLE_PAIRS,
    .max_rpm = MOTOR_MAX_RPM,
    .max_current_a = MOTOR_MAX_CURRENT_A,
    .voltage_limit_v = MOTOR_VOLTAGE_LIMIT_V,
    .kv_rpm_per_v = MOTOR_KV_RPM_PER_V,
    .phase_resistance_ohm = MOTOR_PHASE_RESISTANCE_OHM,
    .phase_inductance_d = MOTOR_PHASE_INDUCTANCE_D_H,
    .phase_inductance_q = MOTOR_PHASE_INDUCTANCE_Q_H,
    .phase_inductance_s = (MOTOR_PHASE_INDUCTANCE_D_H + MOTOR_PHASE_INDUCTANCE_Q_H) / 2,
    .flux_linkage_wb = MOTOR_FLUX_LINKAGE_Wb,
    .IF_Start_Iq_A = (float *)&PMSM_42JS_IF_Start_Iq_A,
    .IF_Start_Ramp_Sec = (float *)&PMSM_42JS_IF_Start_Ramp_Sec,
    .IF_Start_Speed_RPM = (float *)&PMSM_42JS_IF_Start_Speed_RPM,
    .IF_Start_Profile_Length = sizeof(PMSM_42JS_IF_Start_Iq_A) / sizeof(float),
};

