#ifndef SVPWM_H
#define SVPWM_H

#include <stdint.h>
#include "arm_math.h"

void SVPWM_Init_f32(void);
void SVPWM_Calculate_f32(float T_s, float V_dc, float U_alpha, float U_beta,
                     float* T_a, float* T_b, float* T_c, uint8_t* N);

#endif // SVPWM_H