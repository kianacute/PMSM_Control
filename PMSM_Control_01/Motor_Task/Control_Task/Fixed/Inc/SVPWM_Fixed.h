#ifndef __SVPWM_FIXED_H__
#define __SVPWM_FIXED_H__

#include <stdint.h>
#include "arm_math.h"

void SVPWM_Init_q31(void);
void SVPWM_Calculate_q31(q15_t T_s, q15_t V_dc, q15_t U_alpha, q15_t U_beta,
                     q15_t* T_a, q15_t* T_b, q15_t* T_c, uint8_t* N);

#endif // __SVPWM_FIXED_H   