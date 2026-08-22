#ifndef __HAL_MATH_H__
#define __HAL_MATH_H__


#include "arm_math.h"

#define MY_ABS(x) (((x)>0)?(x):(-(x)))
#define Limit_2PI(theta) { while (*theta > 6.283185f) {*theta -= 6.283185f;} while (*theta < -6.283185f) {*theta += 6.283185f;}}

typedef struct Hal_PI_f32_Q31
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
}Hal_PI_Q31_t;

typedef struct Lookup_Table_Q31
{
    const q31_t *x_table;
    const q31_t *y_table;
    uint32_t table_size;
}Lookup_Table_Q31_t;

typedef struct Lookup_Table_2D_Q31
{
    const q31_t *x_table;   // x轴坐标数组，长度 nx
    const q31_t *y_table;   // y轴坐标数组，长度 ny
    const q31_t *z_table;   // z数据矩阵，按行主序存储，大小 nx * ny
    uint32_t nx_size;            // x轴点数
    uint32_t ny_size;            // y轴点数
}Lookup_Table_2D_Q31_t;

struct PLL
{
    Hal_PI_Q31_t PLL_PI;
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
    q31_t threshold_over; // 滞回点1(上限阈值)
    q31_t threshold_over_re;  // 滞回点2(下限阈值)
    q31_t threshold_power;      // 对电源阈值
    q31_t threshold_low; // 滞回点1(上限阈值)
    q31_t threshold_low_re;  // 滞回点2(下限阈值)
    q31_t threshold_gnd;      // 对地阈值
    q31_t analog_input_uplimit; // 输入信号上限
    q31_t analog_input_lowlimit; // 输入信号下限
    uint32_t delay_time;  // 延迟时间
    // 内部状态
    uint32_t delay_cnt;   // 延迟计数器
    enum Sensor_Status status;        // 滞回状态位
} Sensor_Hysteresis_Comp_TypeDef;


inline q31_t Hal_PI_Q31(Hal_PI_Q31_t* PI_Q31, q31_t error);

inline q31_t Hal_LPF_Q31(q31_t coff, q31_t input);

inline q31_t Lookup_Table_1D_Linear_Q31(q31_t x, Lookup_Table_Q31_t *table);

inline q31_t Lookup_Table_2D_Linear_Q31(q31_t x, q31_t y, Lookup_Table_2D_Q31_t *table);

inline q31_t Binary_Search_Fixed(const q31_t* arr, uint32_t n, q31_t target);

inline q31_t Oblique_Wave_Q31(q31_t end_value, q31_t cur_value, q31_t Sub_Step, q31_t Add_Step);

inline void PLL_Update(struct PLL *pPLL, float alpha, float beta, float Discrete_time);

inline void Hysteresis_Comp_Init(Hysteresis_Comp_TypeDef *hcomp, float th_h, float th_l, uint32_t delay);

inline void Hysteresis_Comp_Process_Add(Hysteresis_Comp_TypeDef *hcomp, float analog_input);
inline void Hysteresis_Comp_Process_Sub(Hysteresis_Comp_TypeDef *hcomp, float analog_input);

inline void Sensor_Hysteresis_Comp_Process(Sensor_Hysteresis_Comp_TypeDef *hcomp, float analog_input);

#endif
