#include "Hal_Math_Float.h"

/// @brief 离散PI控制器计算函数，积分系数要求乘以采样周期，输出已经限制在out_min和out_max之间
/// @param controller PI控制器对象，包含增益、积分项、输出限制等参数
/// @param error 输入误差
/// @return 输出
float32_t Hal_PI_f32(Hal_PI_f32_t *controller, float error)
{
     controller->integral += controller->ki * error - 
                             controller->Kd * (controller->output_raw - controller->output); // 抗饱和项
     controller->output_raw = controller->kp * error +                               // 比例项
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

/// @brief 二分查找，数组必须满足单调性
/// @param arr 浮点数组，数组下标从0开始
/// @param n 数组长度
/// @param target 目标值
/// @return 查找到小于等于目标值的最后一个元素的下标
int32_t Binary_Search_f32(const float* arr, uint32_t n, float target)
{
     if (n == 0 || arr == NULL)
     {
          return -1;
     }
     int32_t left = 0;
     int32_t right = n - 1;
     int32_t result = -1;
     while (left < right)
     {
          int32_t mid = (left + right + 1) / 2;
          if ((arr[mid] <= target))
          {
               left = mid;
          }
          else
          {
               right = mid - 1;
          }
     }
     result = right;
     return result;
}

/// @brief 一维线性插值函数，输入x和对应的查找表x_table和y_table，输出插值结果
/// @param x 输入值
/// @param x_table 查找表的x坐标数组
/// @param y_table 查找表的y坐标数组
/// @param table_size 查找表的大小
/// @return 插值结果
float Lookup_Table_1D_Linear_f32(float x, Lookup_Table_f32_t *table)
{
     if (x <= table->x_table[0])
     {
          return table->y_table[0];
     }
     else if (x >= table->x_table[table->table_size - 1])
     {
          return table->y_table[table->table_size - 1];
     }
     int32_t idx = Binary_Search_f32(table->x_table, table->table_size, x);
     float x0 = table->x_table[idx];
     float y0 = table->y_table[idx];
     float x1 = table->x_table[idx + 1];
     float y1 = table->y_table[idx + 1];
     return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
}

/// @brief 二维双线性插值查表函数
/// @param x x轴输入值
/// @param y y轴输入值
/// @param table 二维查找表结构体指针，包含x_table、y_table、z_table和各自的维度
/// @return 双线性插值结果
float Lookup_Table_2D_Linear_f32(float x, float y, Lookup_Table_2D_f32_t *table)
{
    // x轴边界限幅
    float x_clamped = x;
    if (x_clamped <= table->x_table[0])
    {
        x_clamped = table->x_table[0];
    }
    else if (x_clamped >= table->x_table[table->nx_size - 1])
    {
        x_clamped = table->x_table[table->nx_size - 1];
    }

    // y轴边界限幅
    float y_clamped = y;
    if (y_clamped <= table->y_table[0])
    {
        y_clamped = table->y_table[0];
    }
    else if (y_clamped >= table->y_table[table->ny_size - 1])
    {
        y_clamped = table->y_table[table->ny_size - 1];
    }

    // 查找x轴和y轴的索引
    int32_t ix = Binary_Search_f32(table->x_table, table->nx_size, x_clamped);
    int32_t iy = Binary_Search_f32(table->y_table, table->ny_size, y_clamped);

    // 确保索引在有效范围内（边界情况下取nx_size-2或ny_size-2）
    if (ix >= (int32_t)(table->nx_size - 1)) ix = table->nx_size - 2;
    if (iy >= (int32_t)(table->ny_size - 1)) iy = table->ny_size - 2;
    if (ix < 0) ix = 0;
    if (iy < 0) iy = 0;

    // z_table按行主序存储: z[i][j] = z_table[i * ny_size + j]
    float x0 = table->x_table[ix];
    float x1 = table->x_table[ix + 1];
    float y0 = table->y_table[iy];
    float y1 = table->y_table[iy + 1];

    float z00 = table->z_table[ix * table->ny_size + iy];
    float z01 = table->z_table[ix * table->ny_size + (iy + 1)];
    float z10 = table->z_table[(ix + 1) * table->ny_size + iy];
    float z11 = table->z_table[(ix + 1) * table->ny_size + (iy + 1)];

    // 先在x方向插值
    float z0 = z00 + (z10 - z00) * (x_clamped - x0) / (x1 - x0);
    float z1 = z01 + (z11 - z01) * (x_clamped - x0) / (x1 - x0);

    // 再在y方向插值
    return z0 + (z1 - z0) * (y_clamped - y0) / (y1 - y0);
}

/// @brief 斜波函数，cur_value向end_value以Sub_Step和Add_Step的速度靠近
/// @param end_value 目标值
/// @param cur_value 当前值
/// @param Sub_Step 减速步长
/// @param Add_Step 加速步长
/// @return 更新后的值
float Oblique_Wave_f32(float end_value, float cur_value, float Add_Step, float Sub_Step)
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

/// @brief 滞回比较器初始化函数
/// @param hcomp 滞回比较器结构体指针
/// @param th_h 高阈值
/// @param th_l 低阈值
/// @param delay 延迟时间，单位为周期数 (例如，delay=5表示需要连续5个周期满足条件才改变输出状态)
/// @author doubao
void Hysteresis_Comp_Init_f32(Hysteresis_Comp_TypeDef_f32_t *hcomp, float th_h, float th_l, uint32_t delay)
{
     hcomp->enable = 0;
     hcomp->reset = 0;
     hcomp->threshold_high = th_h;
     hcomp->threshold_low = th_l;
     hcomp->delay_time = delay;
     hcomp->delay_cnt = 0;
     hcomp->comp_out = 0;
}

/// @brief 滞回比较器核心处理（必须周期性调用）
/// @param hcomp 滞回比较器结构体指针
/// @author doubao
void Hysteresis_Comp_Process_Add_f32(Hysteresis_Comp_TypeDef_f32_t *hcomp, float analog_input)
{
     // 1. 复位优先
     if (hcomp->reset)
     {
          hcomp->comp_out = 0;
          hcomp->delay_cnt = 0;
          return;
     }

     // 2. 未使能
     if (!hcomp->enable)
     {
          hcomp->comp_out = 0;
          hcomp->delay_cnt = 0;
          return;
     }

     // 3. 滞回核心逻辑
     switch (hcomp->comp_out)
     {
     case 0:
          /* code */
          if (analog_input <= hcomp->threshold_high)
          {

               hcomp->delay_cnt = 0;
          }
          else
          {
               hcomp->delay_cnt++;
               if(hcomp->delay_cnt >= hcomp->delay_time)
               {
                    hcomp->comp_out = 1;
               }
          }
          break;
     case 1:
          if (analog_input >= hcomp->threshold_low)
          {
               hcomp->delay_cnt = 0;
          }
          else
          {
               hcomp->delay_cnt++;
               if(hcomp->delay_cnt >= hcomp->delay_time)
               {
                    hcomp->comp_out = 0;
               }
          }
          break;
     default:
          hcomp->comp_out = 0;
          break;
     }
}


void Hysteresis_Comp_Process_Sub_f32(Hysteresis_Comp_TypeDef_f32_t *hcomp, float analog_input)
{
     // 1. 复位优先
     if (hcomp->reset)
     {
          hcomp->comp_out = 0;
          hcomp->delay_cnt = 0;
          return;
     }

     // 2. 未使能
     if (!hcomp->enable)
     {
          hcomp->comp_out = 0;
          hcomp->delay_cnt = 0;
          return;
     }

     // 3. 滞回核心逻辑
     switch (hcomp->comp_out)
     {
     case 0:
          /* code */
          if (analog_input >= hcomp->threshold_low)
          {

               hcomp->delay_cnt = 0;
          }
          else
          {
               hcomp->delay_cnt++;
               if(hcomp->delay_cnt >= hcomp->delay_time)
               {
                    hcomp->comp_out = 1;
               }
          }
          break;
     case 1:
          if (analog_input <= hcomp->threshold_high)
          {
               hcomp->delay_cnt = 0;
          }
          else
          {
               hcomp->delay_cnt++;
               if(hcomp->delay_cnt >= hcomp->delay_time)
               {
                    hcomp->comp_out = 0;
               }
          }
          break;
     default:
          hcomp->comp_out = 0;
          break;
     }
}
