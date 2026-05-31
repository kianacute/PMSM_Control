#ifndef __PARAMETER_IDENTIFY_H__
#define __PARAMETER_IDENTIFY_H__

#include <stdint.h>

/* Rs辨识配置参数 */
#define RS_IDENTIFY_VOLTAGE_STEPS    5U      /* 电压测试台阶数 */
#define RS_IDENTIFY_SETTLE_CNT       3000U   /* 电流稳定等待计数 (20kHz * 150ms) */
#define RS_IDENTIFY_SAMPLE_CNT       500U    /* 每台阶采样平均计数 (20kHz * 25ms) */
#define RS_IDENTIFY_MAX_CURRENT_A    8.0f    /* 辨识允许最大电流 (A) */
#define RS_IDENTIFY_ALIGN_VOLTAGE_V  0.5f    /* 转子对齐电压 (V) */
#define RS_IDENTIFY_ALIGN_CNT        4000U   /* 对齐持续计数 (20kHz * 200ms) */

/* Rs辨识状态机 */
enum Rs_Identify_State
{
    RS_IDLE = 0,
    RS_ALIGN,       /* 注入小电压对齐转子 */
    RS_MEASURE,     /* 遍历电压台阶测量 */
    RS_DONE,         /* 辨识完成 */
    RS_WRITE_BACK,   /* 结果回写 */
};

/* Rs辨识控制结构体 */
typedef struct Rs_Identify
{
    enum Rs_Identify_State state;                       /* 当前状态 */
    uint32_t            step_count;                     /* 通用计数器 */
    uint32_t            sample_index;                   /* 当前台阶采样序号 */
    float               V_alpha;                        /* 当前注入的α轴电压 (V) */
    float               I_alpha_sum;                    /* Iα累加值 */
    float               V_samples[RS_IDENTIFY_VOLTAGE_STEPS]; /* 各台阶电压记录 */
    float               I_samples[RS_IDENTIFY_VOLTAGE_STEPS]; /* 各台阶电流记录 */
    uint8_t             current_step;                   /* 当前电压台阶序号 */
    float               identified_Rs;                  /* 辨识结果 (Ω), 负值表示失败 */
} Rs_Identify_t;

/* 全局实例声明 */
extern Rs_Identify_t Rs_Identify;

/* 函数声明 */
void    Rs_Identify_Init(void);
int32_t Rs_Identify_Run(float Ialpha, float Udc);

#endif /* __PARAMETER_IDENTIFY_H__ */
