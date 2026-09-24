#ifndef __HAL_MATH_H__
#define __HAL_MATH_H__

#include "arm_math.h"

#define MY_ABS(x) (((x) > 0) ? (x) : (-(x)))
#define Limit_2PI(theta)                   \
    {                                      \
        while ((*theta) > 6.28318530718f)  \
        {                                  \
            (*theta) -= 6.28318530718f;    \
        }                                  \
        while ((*theta) < -6.28318530718f) \
        {                                  \
            (*theta) += 6.28318530718f;    \
        }                                  \
    }

typedef struct Hal_PI_f32
{
    float32_t kp;         // Proportional gain
    float32_t ki;         // Integral gain
    float32_t Kd;         // 抗饱和 gain
    float32_t integral;   // Integral term
    float32_t prev_error; // Previous error term
    float32_t out_min;    // Minimum output limit
    float32_t out_max;    // Maximum output limit
    float32_t output_raw; // Output value
    float32_t output;     // Output value
} Hal_PI_f32_t;

typedef struct Lookup_Table
{
    const float *x_table;
    const float *y_table;
    uint32_t table_size;
} Lookup_Table_1D_f32_t;

typedef struct Lookup_Table_2D
{
    const float *x_table; // x轴坐标数组，长度 nx
    const float *y_table; // y轴坐标数组，长度 ny
    const float *z_table; // z数据矩阵，按行主序存储，大小 nx * ny
    uint32_t nx_size;     // x轴点数
    uint32_t ny_size;     // y轴点数
} Lookup_Table_2D_f32_t;

typedef struct
{
    // 输入接口
    uint8_t enable;       // 比较器使能
    uint8_t reset;        // 比较器复位
    float threshold_high; // 滞回点1(上限阈值)
    float threshold_low;  // 滞回点2(下限阈值)
    uint32_t delay_time;  // 延迟时间

    // 内部状态
    uint32_t delay_cnt; // 延迟计数器
    uint8_t comp_out;   // 最终输出
} Hysteresis_Comp_TypeDef_f32_t;

static inline float32_t Hal_PI_f32(Hal_PI_f32_t *controller, float error)
{
    controller->integral += controller->ki * error -
                            controller->Kd * (controller->output_raw - controller->output); // 抗饱和项
    controller->output_raw = controller->kp * error +                                       // 比例项
                             controller->integral;

    // Clamp output to min/max limits
    if (controller->output_raw > controller->out_max)
    {
        controller->output = controller->out_max;
    }
    else if (controller->output_raw < controller->out_min)
    {
        controller->output = controller->out_min;
    }
    else
    {
        controller->output = controller->output_raw;
    }

    // Update previous error
    controller->prev_error = error;

    return controller->output;
}

inline float32_t Hal_LPF_f32(float coff, float input);

inline float Lookup_Table_1D_Linear_f32(float x, Lookup_Table_1D_f32_t *table);

inline float Lookup_Table_2D_Linear_f32(float x, float y, Lookup_Table_2D_f32_t *table);

inline int Binary_Search_f32(const float *arr, uint32_t n, float target);

static inline float Oblique_Wave_f32(float end_value, float cur_value, float Sub_Step, float Add_Step)
{
    float cur = cur_value;
    if (cur_value < end_value)
    {
        cur += Add_Step;
        if (cur > end_value)
        {
            cur = end_value;
        }
    }
    else if (cur_value > end_value)
    {
        cur -= Sub_Step;
        if (cur < end_value)
        {
            cur = end_value;
        }
    }
    return cur;
}

inline void Hysteresis_Comp_Init_f32(Hysteresis_Comp_TypeDef_f32_t *hcomp, float th_h, float th_l, uint32_t delay);

inline void Hysteresis_Comp_Process_Add_f32(Hysteresis_Comp_TypeDef_f32_t *hcomp, float analog_input);

inline void Hysteresis_Comp_Process_Sub_f32(Hysteresis_Comp_TypeDef_f32_t *hcomp, float analog_input);


/* ============================================================================
 * sin/cos 合表查找 (弧度制, float)
 *
 * 原理:
 *   - 一张覆盖 [0, 2pi) 的 2048 点正弦表 (Flash 8KB, 定义在 Hal_Math_FLoat.c)
 *   - 表长取 2 的幂, 角度折叠用一条 & (N-1) 完成, 无需象限分支
 *   - cos(theta) = sin(theta + pi/2), 即表索引 + N/4 (掩码回绕)
 *   - 相邻两点线性插值 (编译器合成 FMA)
 *
 * 精度: 角度量化 2pi/2048 = 0.176 度; 插值后最大误差约 1.2e-6, 优于 arm_sin_f32
 * 速度: 一次索引同时输出 sin 和 cos, 比 arm_sin_f32 + arm_cos_f32 两次调用快约 3~5 倍
 * 输入: theta 为弧度, 推荐范围 (-2*pi, 2*pi], 负角度自动折叠; 更大正角度亦可
 *       (掩码折叠), 超出 float32 可精确表示的范围后折叠失效, 建议先用 Limit_2PI
 * ==========================================================================*/
#define SINCOS_LUT_SIZE     (2048u)      /* 表长, 必须为 2 的幂 */
#define SINCOS_LUT_MASK     (SINCOS_LUT_SIZE - 1u)
#define SINCOS_LUT_SIZE_F   (2048.0f)
#define SINCOS_LUT_SCALE    (325.94932345220165f)   /* 2048 / (2*pi) */

extern const float g_SinCos_LUT_Sin[SINCOS_LUT_SIZE];  /* sin(2*pi*i/2048), 定义于 Hal_Math_FLoat.c */

/* @brief 内部辅助: 角度折叠 + 取表索引和小数部分 (theta -> [0, N) 坐标)
 * @param theta   弧度, 推荐范围 (-2*pi, 2*pi]
 * @param pIndex  输出掩码后的表索引 [0, N)
 * @param pFract  输出插值小数部分 [0, 1)
 * @note  仅被本文件内的三个查表函数调用, static inline 无调用开销
 */
static inline void SinCos_FoldIndex_f32(float theta, uint32_t *pIndex, float *pFract)
{
    float findex = theta * SINCOS_LUT_SCALE;   /* theta*N/(2*pi), 表索引坐标 [0, N) */
    if (findex < 0.0f)                         /* 负角度: 加一个周期 (theta > -2*pi) */
    {
        findex += SINCOS_LUT_SIZE_F;
    }

    uint32_t i0 = (uint32_t)findex;            /* 截断取整 */
    *pFract = findex - (float)i0;              /* 插值小数部分 [0, 1) */
    *pIndex = i0 & SINCOS_LUT_MASK;            /* 2 的幂掩码 = 对 2*pi 取模 */
}

/* @brief 一次查表同时得到 sin(theta) 和 cos(theta)
 * @param theta    弧度, 推荐范围 (-2*pi, 2*pi], 见文件头说明
 * @param pSinVal  输出 sin(theta)
 * @param pCosVal  输出 cos(theta)
 */
static inline void SinCos_Lookup_f32(float theta, float *pSinVal, float *pCosVal)
{
    uint32_t i0;
    float fract;
    SinCos_FoldIndex_f32(theta, &i0, &fract);

    uint32_t i1  = (i0 + 1u) & SINCOS_LUT_MASK;
    uint32_t ic  = (i0 + (SINCOS_LUT_SIZE >> 2u)) & SINCOS_LUT_MASK; /* +pi/2 得 cos */
    uint32_t ic1 = (ic + 1u) & SINCOS_LUT_MASK;

    float s0 = g_SinCos_LUT_Sin[i0];
    float s1 = g_SinCos_LUT_Sin[i1];
    float c0 = g_SinCos_LUT_Sin[ic];
    float c1 = g_SinCos_LUT_Sin[ic1];

    *pSinVal = s0 + fract * (s1 - s0);         /* 线性插值, 编译器合成 FMA */
    *pCosVal = c0 + fract * (c1 - c0);
}

/* @brief 单独查表得到 sin(theta)
 * @param theta 弧度, 推荐范围 (-2*pi, 2*pi], 见文件头说明
 * @return sin(theta)
 */
static inline float Sin_Lookup_f32(float theta)
{
    uint32_t i0;
    float fract;
    SinCos_FoldIndex_f32(theta, &i0, &fract);

    uint32_t i1 = (i0 + 1u) & SINCOS_LUT_MASK;
    float s0 = g_SinCos_LUT_Sin[i0];
    float s1 = g_SinCos_LUT_Sin[i1];

    return s0 + fract * (s1 - s0);
}

/* @brief 单独查表得到 cos(theta)
 * @param theta 弧度, 推荐范围 (-2*pi, 2*pi], 见文件头说明
 * @return cos(theta)
 */
static inline float Cos_Lookup_f32(float theta)
{
    uint32_t i0;
    float fract;
    SinCos_FoldIndex_f32(theta, &i0, &fract);

    uint32_t ic  = (i0 + (SINCOS_LUT_SIZE >> 2u)) & SINCOS_LUT_MASK; /* +pi/2 得 cos */
    uint32_t ic1 = (ic + 1u) & SINCOS_LUT_MASK;
    float c0 = g_SinCos_LUT_Sin[ic];
    float c1 = g_SinCos_LUT_Sin[ic1];

    return c0 + fract * (c1 - c0);
}

#define MATH_SQRT_3_Q15                             ((q31_t)(1.732051f * 32768.0f))  // sqrt(3)/2 in Q15 format
#define MATH_SQRT_3_PER_2_Q15                       ((q15_t)(0.866025f * 32768.0f))  // sqrt(3)/2 in Q15 format
#define MATH_1_PER_2_Q15                            ((q15_t)(0.5f * 32768.0f))  // 1/2 in Q15 format


typedef struct Hal_PI_f32_q31
{
    q15_t kp;        // Proportional gain
    q15_t ki;        // Integral gain
    q15_t Kd;        // 抗饱和 gain
    q31_t integral;  // Integral term
    q15_t prev_error;// Previous error term
    q15_t out_min;   // Minimum output limit
    q15_t out_max;   // Maximum output limit
    q15_t output_raw;// Output value
    q15_t output;    // Output value
}Hal_PI_q31_t;

typedef struct Lookup_Table_q31
{
    const q31_t *x_table;
    const q31_t *y_table;
    uint32_t table_size;
}Lookup_Table_1D_q31_t;

typedef struct Lookup_Table_2D_q31
{
    const q31_t *x_table;   // x轴坐标数组，长度 nx
    const q31_t *y_table;   // y轴坐标数组，长度 ny
    const q31_t *z_table;   // z数据矩阵，按行主序存储，大小 nx * ny
    uint32_t nx_size;            // x轴点数
    uint32_t ny_size;            // y轴点数
}Lookup_Table_2D_q31_t;

struct PLL
{
    Hal_PI_q31_t PLL_PI;
    q31_t we;
    q31_t theta;
};

typedef struct {
    // 输入接口
    uint8_t enable;          // 比较器使能
    uint8_t reset;           // 比较器复位
    q31_t threshold_high; // 滞回点1(上限阈值)
    q31_t threshold_low;  // 滞回点2(下限阈值)
    uint32_t delay_time;  // 延迟时间

    // 内部状态
    uint32_t delay_cnt;   // 延迟计数器
    uint8_t comp_out;        // 最终输出
} Hysteresis_Comp_TypeDef_q31_t;

inline q31_t Hal_PI_q31(Hal_PI_q31_t* PI_Q31, q31_t error);

inline q31_t Hal_LPF_q31(q31_t coff, q31_t input);

inline q31_t Lookup_Table_1D_Linear_q31(q31_t x, Lookup_Table_1D_q31_t *table);

inline q31_t Lookup_Table_2D_Linear_q31(q31_t x, q31_t y, Lookup_Table_2D_q31_t *table);

inline q31_t Binary_Search_q31(const q31_t* arr, uint32_t n, q31_t target);

inline q31_t Oblique_Wave_q31(q31_t end_value, q31_t cur_value, q31_t Sub_Step, q31_t Add_Step);

inline void Hysteresis_Comp_Init_q31(Hysteresis_Comp_TypeDef_q31_t *hcomp, q31_t th_h, q31_t th_l, uint32_t delay);

inline void Hysteresis_Comp_Process_Add_q31(Hysteresis_Comp_TypeDef_q31_t *hcomp, q31_t analog_input);
inline void Hysteresis_Comp_Process_Sub_q31(Hysteresis_Comp_TypeDef_q31_t *hcomp, q31_t analog_input);


static inline void arm_clarke_q15(q15_t Ia, q15_t Ib, q15_t * pIalpha, q15_t * pIbeta)
{
    q31_t product1, product2;                    /* Temporary variables used to store intermediate results */

    /* Calculating pIalpha from Ia by equation pIalpha = Ia */
    *pIalpha = Ia;

    /* Intermediate product is calculated by (1/(sqrt(3)) * Ia) */
    product1 = (q31_t) (((q31_t) Ia * 18919) >> 15);

    /* Intermediate product is calculated by (2/sqrt(3) * Ib) */
    product2 = (q31_t) (((q31_t) Ib * 37837) >> 15);

    /* pIbeta is calculated by adding the intermediate products */
    *pIbeta = clip_q31_to_q15((q31_t)product1 + (q31_t)product2);
    // *pIbeta = product1 + product2;
}

static inline void arm_inv_clarke_q15(q15_t Ialpha, q15_t Ibeta, q15_t * pIa, q15_t * pIb, q15_t * pIc)
{
    q31_t product1, product2;                    /* Temporary variables used to store intermediate results */

    /* Calculating pIa from Ialpha by equation pIa = Ialpha */
    *pIa = Ialpha;
    // arm_inv_clarke_q31
    /* Intermediate product is calculated by (1/(2*sqrt(3)) * Ia) */
    product1 = (q31_t) (((q31_t) (Ialpha) * (q31_t)(16384)) >> 15);

    /* Intermediate product is calculated by (1/sqrt(3) * pIb) */
    product2 = (q31_t) (((q31_t) (Ibeta) * (q31_t)(28378)) >> 15);

    /* pIb is calculated by subtracting the products */
    *pIb = clip_q31_to_q15((q31_t)product2 - (q31_t)product1);
    
    *pIc = clip_q31_to_q15((-((q31_t)*pIa) - (*pIb)));
}


static inline void arm_park_q15(q15_t Ialpha, q15_t Ibeta, q15_t * pId, q15_t * pIq, q15_t sinVal, q15_t cosVal)
{
    q31_t product1, product2;                    /* Temporary variables used to store intermediate results */
    q31_t product3, product4;                    /* Temporary variables used to store intermediate results */

    /* Intermediate product is calculated by (Ialpha * cosVal) */
    product1 = (q31_t) (((q31_t) (Ialpha) * (q31_t)(cosVal)) >> 15);

    /* Intermediate product is calculated by (Ibeta * sinVal) */
    product2 = (q31_t) (((q31_t) (Ibeta) * (q31_t)(sinVal)) >> 15);

    /* Intermediate product is calculated by (Ialpha * sinVal) */
    product3 = (q31_t) (((q31_t) (Ialpha) * (q31_t)(sinVal)) >> 15);

    /* Intermediate product is calculated by (Ibeta * cosVal) */
    product4 = (q31_t) (((q31_t) (Ibeta) * (q31_t)(cosVal)) >> 15);

    /* Calculate pId by adding the two intermediate products 1 and 2 */
    *pId = clip_q31_to_q15((q31_t)product1 + (q31_t)product2);

    /* Calculate pIq by subtracting the two intermediate products 3 from 4 */
    *pIq = clip_q31_to_q15((q31_t)product4 - (q31_t)product3);
}

static inline void arm_inv_park_q15(q15_t Id, q15_t Iq, q15_t * pIalpha, q15_t * pIbeta, q15_t sinVal, q15_t cosVal)
{
    q31_t product1, product2;                    /* Temporary variables used to store intermediate results */
    q31_t product3, product4;                    /* Temporary variables used to store intermediate results */

    /* Intermediate product is calculated by (Id * cosVal) */
    product1 = (q31_t) (((q31_t) (Id) * (cosVal)) >> 15);

    /* Intermediate product is calculated by (Iq * sinVal) */
    product2 = (q31_t) (((q31_t) (Iq) * (sinVal)) >> 15);


    /* Intermediate product is calculated by (Id * sinVal) */
    product3 = (q31_t) (((q31_t) (Id) * (sinVal)) >> 15);

    /* Intermediate product is calculated by (Iq * cosVal) */
    product4 = (q31_t) (((q31_t) (Iq) * (cosVal)) >> 15);

    /* Calculate pIalpha by using the two intermediate products 1 and 2 */
    *pIalpha = clip_q31_to_q15((q31_t)product1 - (q31_t)product2);

    /* Calculate pIbeta by using the two intermediate products 3 and 4 */
    *pIbeta = clip_q31_to_q15((q31_t)product4 + (q31_t)product3);
}

#endif
