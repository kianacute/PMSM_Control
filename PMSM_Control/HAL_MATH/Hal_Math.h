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

float32_t Hal_PI_f32(Hal_PI_t* controller, float error);

float32_t Hal_LPF_f32(float coff, float input);

float Lookup_Table_Linear(float x, float x_table[], float y_table[], uint32_t table_size);

int binary_search_float_first(float arr[], uint32_t n, float target);

float Limit_2PI(float theta);

float Oblique_Wave(float end_value, float cur_value, float Sub_Step, float Add_Step);

#endif
