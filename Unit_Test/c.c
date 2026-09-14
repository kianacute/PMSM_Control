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

q15_t clip_q31_to_q15(q31_t x)
{
return ((q31_t) (x >> 16) != ((q31_t) x >> 15)) ?
    ((0x7FFF ^ ((q15_t) (x >> 31)))) : (q15_t) x;
}

int main()
{
    q15_t T_s = 0x7fff;    // 采样周期
    q15_t V_dc = 300;    // 直流母线电压
    q15_t U_alpha = 100; // α轴电压分量
    q15_t U_beta = -200; // β轴电压分量
    q15_t T_a, T_b, T_c;
    uint8_t N;

    int16_t a1 = 0x7f00; // 将浮点数转换为q15_t格式
    int16_t a2 = 0x7f00;  // 将浮点数转换为q15_t格式
    int16_t a3 = 0x0001;  // 将浮点数转换为q15_t格式
    int32_t b1 = a1 * a2;      // 计算乘积
    int16_t c1 = -100;
    int32_t c2 = -100;

    b1 = a1 + a2;
    c1 = a1 + a2;

    a3 = clip_q31_to_q15(b1);
    // SVPWM_Calculate_q31(T_s, V_dc, U_alpha, U_beta, &T_a, &T_b, &T_c, &N);

    printf("a1: 0x%x size: %d\n", a1, sizeof(a1));
    printf("a2: 0x%x size: %d\n", a2, sizeof(a2));
    printf("a3: 0x%x size: %d\n", a3, sizeof(a3));
    printf("b1: 0x%x size: %d\n", b1, sizeof(b1));
    printf("c1: 0x%x size: %d\n", c1, sizeof(c1));
    printf("\n---------------------------------------------------");
    printf("a1: %d\n", a1);
    printf("a2: %d\n", a2);
    printf("a3: %d\n", a3);
    printf("b1: %d\n", b1);
    printf("c1: %d\n", c1);
    // printf("Sector: %d\n", N);
    return 0;
}