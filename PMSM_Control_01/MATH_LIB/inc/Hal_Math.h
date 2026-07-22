#ifndef __HAL_MATH_H__
#define __HAL_MATH_H__


#include "arm_math.h"

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
}Hal_PI_t;

typedef struct Lookup_Table
{
    const float *x_table;
    const float *y_table;
    uint32_t table_size;
}Lookup_Table_t;

typedef struct Lookup_Table_2D
{
    const float *x_table;   // x轴坐标数组，长度 nx
    const float *y_table;   // y轴坐标数组，长度 ny
    const float *z_table;   // z数据矩阵，按行主序存储，大小 nx * ny
    uint32_t nx;            // x轴点数
    uint32_t ny;            // y轴点数
}Lookup_Table_2D_t;

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
    uint8_t comp_out;        // 最终输出
} Hysteresis_Comp_TypeDef;


/*
这一类滞回比较器有留个滞回点，专门针对传感器这类诊断场景，
例如温度传感器，划分为五个状态，正常、过低、过高、电源异常、接地异常，
每个状态都有一个上限和下限，当输入信号超过上限时，状态变为对应的异常状态
*/

enum Sensor_Status
{
    SENSOR_STATUS_NORMAL = 0,
    SENSOR_STATUS_LOW,
    SENSOR_STATUS_OVER,
    SENSOR_STATUS_POWER_ERR,
    SENSOR_STATUS_GND_ERR,
    SENSOR_STATUS_UNKNOWN,
};

typedef struct {
    // 输入接口
    uint8_t enable;          // 比较器使能
    uint8_t reset;           // 比较器复位
    float threshold_over; // 滞回点1(上限阈值)
    float threshold_over_re;  // 滞回点2(下限阈值)
    float threshold_power;      // 对电源阈值
    float threshold_low; // 滞回点1(上限阈值)
    float threshold_low_re;  // 滞回点2(下限阈值)
    float threshold_gnd;      // 对地阈值
    float analog_input_uplimit; // 输入信号上限
    float analog_input_lowlimit; // 输入信号下限
    uint32_t delay_time;  // 延迟时间
    // 内部状态
    uint32_t delay_cnt;   // 延迟计数器
    enum Sensor_Status status;        // 滞回状态位
} Sensor_Hysteresis_Comp_TypeDef;


inline float32_t Hal_PI_f32(Hal_PI_t* controller, float error);

inline float my_abs(float num);

inline float32_t Hal_LPF_f32(float coff, float input);

inline float Lookup_Table_Linear(float x, Lookup_Table_t *table);

inline float Lookup_Table_2D_Linear(float x, float y, Lookup_Table_2D_t *table);

inline int binary_search_float_first(const float* arr, uint32_t n, float target);

inline float Limit_2PI(float theta);

inline float Oblique_Wave(float end_value, float cur_value, float Sub_Step, float Add_Step);

inline void PLL_Update(struct PLL *pPLL, float alpha, float beta, float Discrete_time);

inline void Hysteresis_Comp_Init(Hysteresis_Comp_TypeDef *hcomp, float th_h, float th_l, uint32_t delay);

inline void Hysteresis_Comp_Process_Add(Hysteresis_Comp_TypeDef *hcomp, float analog_input);
inline void Hysteresis_Comp_Process_Sub(Hysteresis_Comp_TypeDef *hcomp, float analog_input);

inline void Sensor_Hysteresis_Comp_Process(Sensor_Hysteresis_Comp_TypeDef *hcomp, float analog_input);

#endif
