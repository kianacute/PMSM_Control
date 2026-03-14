#ifndef SVPWM_H
#define SVPWM_H

#include <stdint.h>

void SVPWM_Init(void);
void SVPWM_Calculate(float T_s, float V_dc, float U_alpha, float U_beta,
                     float* T_a, float* T_b, float* T_c, uint8_t* N);

#endif // SVPWM_H