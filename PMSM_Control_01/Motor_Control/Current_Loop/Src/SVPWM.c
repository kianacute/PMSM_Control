#include "SVPWM.h"

void SVPWM_Init(void)
{
    // Initialization code for SVPWM
}

// 修正后的SVPWM实现
static inline uint8_t SVPWM_Sector_Predict(float U_alpha, float U_beta)
{
    float V1 = U_beta;
    float V2 = 0.8660254f * U_alpha - 0.5f * U_beta;
    float V3 = -0.8660254f * U_alpha - 0.5f * U_beta;
    uint8_t N = 0;
    if (V1 > 0.0f) N |= 1;
    if (V2 > 0.0f) N |= 2;
    if (V3 > 0.0f) N |= 4;
    return N;
}

void SVPWM_Calculate(float T_s, float V_dc, float U_alpha, float U_beta,
                     float* T_a, float* T_b, float* T_c, uint8_t* N)
{
    int8_t sector = SVPWM_Sector_Predict(U_alpha, U_beta);
    *N = sector;
    float factor = 1.7320508f * T_s / V_dc;  
    float X = factor * U_beta;
    float Y = factor * (0.8660254f * U_alpha + 0.5f * U_beta);
    float Z = factor * (-0.8660254f * U_alpha + 0.5f * U_beta);
    float T1, T2;
    
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
    float T_sum = T1 + T2;
    if (T_sum > T_s)
    {
        T1 = (T1 / T_sum) * T_s;
        T2 = (T2 / T_sum) * T_s;
    }
    else if (T_sum < 0)
    {
        T1 = 0;
        T2 = 0;
    }
    
    float T0 = T_s - T1 - T2;
    
    // 计算七段式SVPWM的比较点
    float T0_half = T0 / 4.0f;
    float Ta = T0_half;
    float Tb = Ta + T1 / 2.0f;
    float Tc = Tb + T2 / 2.0f;
    
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