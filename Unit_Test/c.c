#include <stdio.h>
#include <stdint.h>

#define q31_t int32_t
#define q15_t int16_t

void SVPWM_Calculate_q31(q15_t T_s, q15_t V_dc, q15_t U_alpha, q15_t U_beta,
                         q15_t *T_a, q15_t *T_b, q15_t *T_c, uint8_t *N)
{
    int8_t sector = 0;
    q31_t V1 = U_beta;
    q31_t V2 = 28377 * (q31_t)U_alpha - 16384 * (q31_t)U_beta;
    q31_t V3 = -28377 * (q31_t)U_alpha - 16384 * (q31_t)U_beta;
    printf("V1: 0x%x, V2: 0x%x, V3: 0x%x\n", V1, V2, V3);
    if (V1 > 0)
        sector |= 1;
    if (V2 > 0)
        sector |= 2;
    if (V3 > 0)
        sector |= 4;
    *N = sector;
}

int main()
{
    q15_t T_s = 1000;    // 采样周期
    q15_t V_dc = 300;    // 直流母线电压
    q15_t U_alpha = 100; // α轴电压分量
    q15_t U_beta = -200; // β轴电压分量
    q15_t T_a, T_b, T_c;
    uint8_t N;

    int16_t a1 = -32768; // 将浮点数转换为q15_t格式
    int32_t a2 = 1.5 * 32768;  // 将浮点数转换为q15_t格式
    int32_t b1 = a1 * a2;      // 计算乘积
    int16_t c1 = -100;
    int32_t c2 = -100;

    // SVPWM_Calculate_q31(T_s, V_dc, U_alpha, U_beta, &T_a, &T_b, &T_c, &N);

    printf("T_a: 0x%x size: %d\n", c1, sizeof(c1));
    printf("T_b: 0x%x size: %d\n", c2, sizeof(c2));
    printf("T_c: 0x%x size: %d\n", b1, sizeof(b1));
    printf("\n\n");
    printf("T_a: %d\n", a1);
    printf("T_b: %d\n", a2);
    printf("T_c: %d\n", b1);
    // printf("Sector: %d\n", N);
    return 0;
}