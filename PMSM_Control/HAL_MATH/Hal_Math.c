#include "arm_math.h"
#include "Hal_Math.h"

/// @brief 离散PI控制器计算函数，积分系数要求乘以采样周期，输出已经限制在out_min和out_max之间
/// @param controller PI控制器对象，包含增益、积分项、输出限制等参数
/// @param error 输入误差
/// @return 输出
float32_t Hal_PI_f32(Hal_PI_t *controller, float error)
{
     controller->integral += error;
     controller->output_raw = controller->kp * error +                               // 比例项
                              controller->ki * controller->integral -                // 积分项
                              0.01f * (controller->output_raw - controller->output); // 抗饱和项

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

/// @brief 一维二分查找，数组必须满足单调性，查找到小于等于目标值的最后一个元素的下标
/// @param arr 浮点数组，数组下标从0开始
/// @param n 数组长度
/// @param target 目标值
/// @return 查找到小于等于目标值的最后一个元素的下标
int binary_search_float_first(float arr[], uint32_t n, float target)
{
     if (n == 0 || arr == NULL)
     {
          return -1;
     }
     int left = 0;
     int right = n - 1;
     int result = -1;
     while (left < right)
     {
          int mid = (left + right + 1) / 2;
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
float Lookup_Table_Linear(float x, float x_table[], float y_table[], uint32_t table_size)
{
     if (x <= x_table[0])
     {
          return y_table[0];
     }
     else if (x >= x_table[table_size - 1])
     {
          return y_table[table_size - 1];
     }
     else
     {
          int idx = binary_search_float_first(x_table, table_size, x);
          float x0 = x_table[idx];
          float y0 = y_table[idx];
          float x1 = x_table[idx + 1];
          float y1 = y_table[idx + 1];
          return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
     }
}

/// @brief 将角度限制在0-2PI范围内
/// @param theta 输入角度，单位为弧度
/// @return 限制后的角度，单位为弧度
float Limit_2PI(float theta)
{
     while (theta > 6.283185f)
     {
          theta -= 6.283185f;
     }
     while (theta < -6.283185f)
     {
          theta += 6.283185f;
     }
     return theta;
}

/// @brief 斜波函数，cur_value向end_value以Sub_Step和Add_Step的速度靠近
/// @param end_value 目标值
/// @param cur_value 当前值
/// @param Sub_Step 减速步长
/// @param Add_Step 加速步长
/// @return 更新后的值
float Oblique_Wave(float end_value, float cur_value, float Add_Step, float Sub_Step)
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