#ifndef __HAL_MATH_H__
#define __HAL_MATH_H__


#include "arm_math.h"

#define MY_ABS(x) (((x)>0)?(x):(-(x)))
#define Limit_2PI(theta) { while ((*theta) > 1.0f) {(*theta) -= 1.0f;} \
                           while ((*theta) < -1.0f){(*theta) += 1.0f;}}

typedef struct Hal_PI_f32
{
    float32_t kp;        // Proportional gain
    float32_t ki;        // Integral gain
    float32_t Kd;        // 抗饱和 gain
    float32_t integral;  // Integral term
    float32_t prev_error; // Previous error term
    float32_t out_min;   // Minimum output limit
    float32_t out_max;   // Maximum output limit
    float32_t output_raw;       // Output value
    float32_t output;       // Output value
}Hal_PI_f32_t;

typedef struct Lookup_Table
{
    const float *x_table;
    const float *y_table;
    uint32_t table_size;
}Lookup_Table_f32_t;

typedef struct Lookup_Table_2D
{
    const float *x_table;   // x轴坐标数组，长度 nx
    const float *y_table;   // y轴坐标数组，长度 ny
    const float *z_table;   // z数据矩阵，按行主序存储，大小 nx * ny
    uint32_t nx_size;            // x轴点数
    uint32_t ny_size;            // y轴点数
}Lookup_Table_2D_f32_t;

typedef struct {
    // 输入接口
    uint8_t enable;          // 比较器使能
    uint8_t reset;           // 比较器复位
    float threshold_high; // 滞回点1(上限阈值)
    float threshold_low;  // 滞回点2(下限阈值)
    uint32_t delay_time;  // 延迟时间

    // 内部状态
    uint32_t delay_cnt;   // 延迟计数器
    uint8_t comp_out;        // 最终输出
} Hysteresis_Comp_TypeDef_f32_t;

inline float32_t Hal_PI_f32(Hal_PI_f32_t* controller, float error);

inline float32_t Hal_LPF_f32(float coff, float input);

inline float Lookup_Table_1D_Linear_f32(float x, Lookup_Table_f32_t *table);

inline float Lookup_Table_2D_Linear_f32(float x, float y, Lookup_Table_2D_f32_t *table);

inline int Binary_Search_f32(const float* arr, uint32_t n, float target);

inline float Oblique_Wave_f32(float end_value, float cur_value, float Sub_Step, float Add_Step);

inline void Hysteresis_Comp_Init_f32(Hysteresis_Comp_TypeDef_f32_t *hcomp, float th_h, float th_l, uint32_t delay);

inline void Hysteresis_Comp_Process_Add_f32(Hysteresis_Comp_TypeDef_f32_t *hcomp, float analog_input);
inline void Hysteresis_Comp_Process_Sub_f32(Hysteresis_Comp_TypeDef_f32_t *hcomp, float analog_input);

#endif
