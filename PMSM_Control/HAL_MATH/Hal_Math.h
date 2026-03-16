#ifndef __HAL_MATH_H__
#define __HAL_MATH_H__


#include "arm_math.h"

typedef struct Hal_PI_f32
{
    float32_t kp;        // Proportional gain
    float32_t ki;        // Integral gain
    float32_t integral;  // Integral term
    float32_t prev_error; // Previous error term
    float32_t out_min;   // Minimum output limit
    float32_t out_max;   // Maximum output limit
    float32_t output_raw;       // Output value
    float32_t output;       // Output value
}Hal_PI_t;

typedef struct Lookup_Table
{
    float *x_table;
    float *y_table;
    uint32_t table_size;
}Lookup_Table_t;

struct PLL
{
    Hal_PI_t PLL_PI;
    float we;
    float theta;
};

typedef struct {
    // 输入接口
    uint8_t enable;          // 比较器使能
    uint8_t reset;           // 比较器复位
    float threshold_high; // 滞回点1(上限阈值)
    float threshold_low;  // 滞回点2(下限阈值)
    uint32_t delay_time;  // 延迟时间

    // 内部状态
    uint32_t delay_cnt;   // 延迟计数器
    uint8_t pre_result;      // 预输出结果
    uint8_t comp_out;        // 最终输出
} Hysteresis_Comp_TypeDef;


float32_t Hal_PI_f32(Hal_PI_t* controller, float error);

float32_t Hal_LPF_f32(float coff, float input);

float Lookup_Table_Linear(float x, Lookup_Table_t *table);

int binary_search_float_first(float arr[], uint32_t n, float target);

float Limit_2PI(float theta);

float Oblique_Wave(float end_value, float cur_value, float Sub_Step, float Add_Step);

void PLL_Updata(struct PLL *pPLL, float alpha, float beta, float Discrete_time);

void Hysteresis_Comp_Init(Hysteresis_Comp_TypeDef *hcomp, float th_h, float th_l, uint32_t delay);

void Hysteresis_Comp_Process(Hysteresis_Comp_TypeDef *hcomp, float analog_input);

#endif
