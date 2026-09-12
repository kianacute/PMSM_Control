#ifndef __HAL_MATH_H__
#define __HAL_MATH_H__


#include "arm_math.h"

#define MY_ABS(x) (((x)>0)?(x):(-(x)))
#define Limit_2PI(theta) { while (*theta > 6.283185f) {*theta -= 6.283185f;} while (*theta < -6.283185f) {*theta += 6.283185f;}}

#define MATH_SQRT_3_Q15                             ((q31_t)(1.732051f * 32768.0f))  // sqrt(3)/2 in Q15 format
#define MATH_SQRT_3_PER_2_Q15                       ((q15_t)(0.866025f * 32768.0f))  // sqrt(3)/2 in Q15 format
#define MATH_1_PER_2_Q15                            ((q15_t)(0.5f * 32768.0f))  // 1/2 in Q15 format


typedef struct Hal_PI_f32_q31
{
    q31_t kp;        // Proportional gain
    q31_t ki;        // Integral gain
    q31_t Kd;        // 抗饱和 gain
    q63_t integral;  // Integral term
    q31_t prev_error; // Previous error term
    q31_t out_min;   // Minimum output limit
    q31_t out_max;   // Maximum output limit
    q31_t output_raw;       // Output value
    q31_t output;       // Output value
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


#endif
