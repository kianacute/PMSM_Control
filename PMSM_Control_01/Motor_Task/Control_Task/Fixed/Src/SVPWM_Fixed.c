#include "SVPWM_Fixed.h"
#include "Hal_Math_Fixed.h"

void SVPWM_Init_q31(void)
{
    // Initialization code for SVPWM
}

void SVPWM_Calculate_q31(q15_t T_s, q15_t V_dc, q15_t U_alpha, q15_t U_beta,
                     q15_t* T_a, q15_t* T_b, q15_t* T_c, uint8_t* N)
{
    uint8_t sector = 0;
    q31_t X, Y, Z;
    X = (q31_t)U_beta;  if (X > 0) sector |= 1;
    Y = MATH_SQRT_3_PER_2_Q15 * U_alpha - (U_beta << 14);  if (Y > 0) sector |= 2;
    Z = -MATH_SQRT_3_PER_2_Q15 * U_alpha - (U_beta << 14);   if (Z > 0) sector |= 4;    
    *N = sector;
    
    q31_t factor = ((56756 * (q31_t)T_s)) / (q31_t)V_dc;  
    X = factor * (q31_t)U_beta;
    Y = factor * (((MATH_SQRT_3_PER_2_Q15 * U_alpha) >> 15) + ((16384 * (q31_t)U_beta) >> 15));
    Z = factor * (((-MATH_SQRT_3_PER_2_Q15 * U_alpha) >> 15) + ((16384 * (q31_t)U_beta) >> 15));
    q31_t T1, T2;

    // 根据扇区选择时间
    switch (sector)
    {
        case 1: T1 = Z; T2 = Y; break;
        case 2: T1 = Y; T2 = -X; break;
        case 3: T1 = -Z; T2 = X; break;
        case 4: T1 = -X; T2 = Z; break;
        case 5: T1 = X; T2 = -Y; break;
        case 6: T1 = -Y; T2 = -Z; break;
        default: T1 = 0; T2 = 0; break;
    }
    
    // 饱和处理
    // q31_t T_sum = T1 + T2;
    // if (T_sum > (T_s << 15))
    // {
    //     T1 = (T1 / T_sum) * T_s;
    //     T2 = (T2 / T_sum) * T_s;
    // }
    // else if (T_sum < 0)
    // {
    //     T1 = 0;
    //     T2 = 0;
    // }
    
    q31_t T0 = (q31_t)(T_s << 15) - T1 - T2;
    
    // 计算七段式SVPWM的比较点
    q31_t T0_half = T0 >> 2;
    q31_t Ta = T0_half;
    q31_t Tb = Ta + (T1 >> 1);
    q31_t Tc = Tb + (T2 >> 1);

    Ta = Ta >> 15;
    Tb = Tb >> 15;
    Tc = Tc >> 15;
    
    // 根据扇区分配比较值
    switch (sector)
    {
        case 1: *T_a = Tb; *T_b = Ta; *T_c = Tc; break;
        case 2: *T_a = Ta; *T_b = Tc; *T_c = Tb; break;
        case 3: *T_a = Ta; *T_b = Tb; *T_c = Tc; break;
        case 4: *T_a = Tc; *T_b = Tb; *T_c = Ta; break;
        case 5: *T_a = Tc; *T_b = Ta; *T_c = Tb; break;
        case 6: *T_a = Tb; *T_b = Tc; *T_c = Ta; break;
        default: *T_a = 0; *T_b = 0; *T_c = 0; break;
    }
}