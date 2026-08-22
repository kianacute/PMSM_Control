#include <stdio.h>
#include <stdint.h>

#define q31_t int32_t

typedef struct Lookup_Table_Q31
{
    const q31_t *x_table;
    const q31_t *y_table;
    uint32_t table_size;
}Lookup_Table_Q31_t;

/// @brief 二分查找，数组必须满足单调性
/// @param arr 定点数数组，数组下标从0开始
/// @param n 数组长度
/// @param target 目标值
/// @return 查找到小于等于目标值的最后一个元素的下标
q31_t Binary_Search_Fixed(const q31_t* arr, uint32_t n, q31_t target)
{
     if (n == 0 || arr == NULL)
     {
          return -1;
     }
     q31_t left = 0;
     q31_t right = n - 1;
     q31_t result = -1;
     while (left < right)
     {
          q31_t mid = (left + right + 1) / 2;
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
q31_t Lookup_Table_1D_Linear_Q31(q31_t x, Lookup_Table_Q31_t *table)
{
     if (x <= table->x_table[0])
     {
          return table->y_table[0];
     }
     else if (x >= table->x_table[table->table_size - 1])
     {
          return table->y_table[table->table_size - 1];
     }
     q31_t idx = Binary_Search_Fixed(table->x_table, table->table_size, x);
     q31_t x0 = table->x_table[idx];
     q31_t y0 = table->y_table[idx];
     q31_t x1 = table->x_table[idx + 1];
     q31_t y1 = table->y_table[idx + 1];
     return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
}


int main()
{
     // 定义查找表
     q31_t x_table[] = {0, 1000, 1000, 3000, 4000};
     q31_t y_table[] = {0, 10, 20, 30, 40};
     Lookup_Table_Q31_t table = {x_table, y_table, sizeof(x_table) / sizeof(x_table[0])};

     // 测试插值函数
     q31_t x = 5000; // 输入值
     q31_t result = Lookup_Table_1D_Linear_Q31(x, &table);
     printf("Interpolated result for x = %d: %d\n", x, result);

     return 0;
}

