#include "arm_math.h"
#include "Hal_Math.h"

/// @brief 离散PI控制器计算函数，积分系数要求乘以采样周期，输出已经限制在out_min和out_max之间
/// @param controller PI控制器对象，包含增益、积分项、输出限制等参数
/// @param error 输入误差
/// @return 输出
float32_t Hal_PI_f32(Hal_PI_t *controller, float error)
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

float my_abs(float num)
{
    if (num < 0)
    {
        return -num;
    }
    return num;
}

/// @brief 一维线性插值函数，输入x和对应的查找表x_table和y_table，输出插值结果
/// @param x 输入值
/// @param x_table 查找表的x坐标数组
/// @param y_table 查找表的y坐标数组
/// @param table_size 查找表的大小
/// @return 插值结果
float Lookup_Table_Linear(float x, Lookup_Table_t *table)
{
     if (x <= table->x_table[0])
     {
          return table->y_table[0];
     }
     else if (x >= table->x_table[table->table_size - 1])
     {
          return table->y_table[table->table_size - 1];
     }
     int idx = binary_search_float_first(table->x_table, table->table_size, x);
     float x0 = table->x_table[idx];
     float y0 = table->y_table[idx];
     float x1 = table->x_table[idx + 1];
     float y1 = table->y_table[idx + 1];
     return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
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

/// @brief PLL更新函数
/// @param pPLL pLL对象指针，包含PI控制器和当前频率、相位等参数
/// @param alpha 正弦信号
/// @param beta 余弦信号
/// @param Discrete_time 离散积分时间,单位为秒
void PLL_Update(struct PLL *pPLL, float alpha, float beta, float Discrete_time)
{
     float deta = alpha * arm_cos_f32(pPLL->theta) - beta * arm_sin_f32(pPLL->theta);
     pPLL->we = Hal_PI_f32(&pPLL->PLL_PI, deta);
     pPLL->theta = (pPLL->theta + pPLL->we * Discrete_time);
     pPLL->theta = Limit_2PI(pPLL->theta);
     return;
}

/// @brief 滞回比较器初始化函数
/// @param hcomp 滞回比较器结构体指针
/// @param th_h 高阈值
/// @param th_l 低阈值
/// @param delay 延迟时间，单位为周期数 (例如，delay=5表示需要连续5个周期满足条件才改变输出状态)
/// @author doubao
void Hysteresis_Comp_Init(Hysteresis_Comp_TypeDef *hcomp, float th_h, float th_l, uint32_t delay)
{
     hcomp->enable = 0;
     hcomp->reset = 0;
     hcomp->threshold_high = th_h;
     hcomp->threshold_low = th_l;
     hcomp->delay_time = delay;
     hcomp->delay_cnt = 0;
     hcomp->pre_result = 0;
     hcomp->comp_out = 0;
}

/// @brief 滞回比较器核心处理（必须周期性调用）
/// @param hcomp 滞回比较器结构体指针
/// @author doubao
void Hysteresis_Comp_Process(Hysteresis_Comp_TypeDef *hcomp, float analog_input)
{
     // 1. 复位优先
     if (hcomp->reset)
     {
          hcomp->comp_out = 0;
          hcomp->delay_cnt = 0;
          hcomp->pre_result = 0;
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
     uint8_t current_pre;
     if (hcomp->comp_out)
     {
          // 当前输出1 → 低于下限才准备变0
          current_pre = (analog_input >= hcomp->threshold_low);
     }
     else
     {
          // 当前输出0 → 高于上限才准备变1
          current_pre = (analog_input >= hcomp->threshold_high);
     }

     // 4. 延迟防抖
     if (current_pre != hcomp->pre_result)
     {
          hcomp->delay_cnt = 0;
          hcomp->pre_result = current_pre;
     }
     else
     {
          if (hcomp->delay_cnt < hcomp->delay_time)
          {
               hcomp->delay_cnt++;
          }
     }

     // 5. 延迟满足才更新输出
     if (hcomp->delay_cnt >= hcomp->delay_time)
     {
          hcomp->comp_out = current_pre;
     }
}