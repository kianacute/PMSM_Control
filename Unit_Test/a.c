#include "stdio.h"
#include "stdint.h"

typedef struct
{
     // 输入接口
     uint8_t enable;       // 比较器使能
     uint8_t reset;        // 比较器复位
     float threshold_high; // 滞回点1(上限阈值)
     float threshold_low;  // 滞回点2(下限阈值)
     uint32_t delay_time;  // 延迟时间

     // 内部状态
     uint32_t delay_cnt; // 延迟计数器
     uint8_t pre_result; // 预输出结果
     uint8_t comp_out;   // 最终输出
} Hysteresis_Comp_TypeDef;

void Hysteresis_Comp_Process_Add(Hysteresis_Comp_TypeDef *hcomp, float analog_input)
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
     switch (hcomp->comp_out)
     {
     case 0:
          /* code */
          if (analog_input <= hcomp->threshold_high)
          {
               hcomp->pre_result = 0;
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
               hcomp->pre_result = 1;
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


void Hysteresis_Comp_Process_Sub(Hysteresis_Comp_TypeDef *hcomp, float analog_input)
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
     switch (hcomp->comp_out)
     {
     case 0:
          /* code */
          if (analog_input >= hcomp->threshold_low)
          {
               hcomp->pre_result = 0;
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
               hcomp->pre_result = 1;
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

int n, m;

int main()
{
     char ch;
     Hysteresis_Comp_TypeDef hcomp;
     hcomp.enable = 1;
     hcomp.reset = 0;
     hcomp.threshold_high = 8.0f;
     hcomp.threshold_low = 5.0f;
     hcomp.delay_time = 2;
     hcomp.delay_cnt = 0;
     hcomp.pre_result = 0;
     hcomp.comp_out = 0;
     FILE *fp = fopen("input1.csv", "r");
     float analog_input;
     fscanf(fp, "%d", &n);
     printf("Number of inputs: %d\n", n);
     for (int i = 0; i < n; i++)
     {
          while(fscanf(fp, "%c", &ch) != EOF)
          {
               // fseek(fp, 1L, SEEK_CUR);
               if(ch != ',')
               {
                    break;
               }
          } 
          fseek(fp, -1L, SEEK_CUR);
          fscanf(fp, "%f", &analog_input);
          Hysteresis_Comp_Process_Add(&hcomp, analog_input);
          printf("Analog Input: %f, wait cnt: %lu, Comparator Output: %d\n", analog_input, (unsigned long)hcomp.delay_cnt, hcomp.comp_out);
     }
     fclose(fp);
     return 0;
}