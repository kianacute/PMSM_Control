#include "Parameter_Identify.h"
#include "Hal_Math.h"
#include <string.h>

Rs_Identify_t Rs_Identify;

/* 电压测试点：占母线电压的比例, 从小到大逐步增加 */
static const float voltage_fractions[RS_IDENTIFY_VOLTAGE_STEPS] = {
    0.04f,  /* 2% Udc */
    0.08f,  /* 4% Udc */
    0.12f,  /* 6% Udc */
    0.16f,  /* 8% Udc */
    0.20f   /* 10% Udc */
};

/**
 * @brief  Rs辨识模块初始化
 * @note   上电或复位时调用，清除所有状态
 */
void Rs_Identify_Init(void)
{
    Rs_Identify.state = RS_IDLE;
    Rs_Identify.step_count = 0;
    Rs_Identify.sample_index = 0;
    Rs_Identify.V_alpha = 0.0f;
    Rs_Identify.I_alpha_sum = 0.0f;
    Rs_Identify.current_step = 0;
    Rs_Identify.identified_Rs = 0.0f;
    (void)memset(Rs_Identify.V_samples, 0, sizeof(Rs_Identify.V_samples));
    (void)memset(Rs_Identify.I_samples, 0, sizeof(Rs_Identify.I_samples));
}

/**
 * @brief  Rs辨识状态机驱动函数
 * @param  Ialpha  当前补偿后的α轴电流 (A)
 * @param  Udc     当前母线电压 (V)
 * @retval 0 正常, -1 辨识失败
 * @note   由电流环ADC中断调用, 每50us执行一次 (20kHz)
 */
int32_t Rs_Identify_Run(float Ialpha, float Udc)
{
    int32_t ret = 0;

    switch (Rs_Identify.state)
    {
    /* ---- 空闲：转对齐 ---- */
    case RS_IDLE:
        Rs_Identify.state = RS_ALIGN;
        Rs_Identify.step_count = 0;
        break;

    /* ---- 转子对齐：注小电压，等转子锁定到α轴 ---- */
    case RS_ALIGN:
    {
        Rs_Identify.V_alpha = RS_IDENTIFY_ALIGN_VOLTAGE_V;
        Rs_Identify.step_count++;
        if (Rs_Identify.step_count >= RS_IDENTIFY_ALIGN_CNT)
        {
            /* 对齐完成，进入测量阶段 */
            Rs_Identify.state = RS_MEASURE;
            Rs_Identify.step_count = 0;
            Rs_Identify.current_step = 0;
            Rs_Identify.sample_index = 0;
            Rs_Identify.I_alpha_sum = 0.0f;
        }
        break;
    }

    /* ---- 多台阶测量 ---- */
    case RS_MEASURE:
    {
        if (Rs_Identify.current_step >= RS_IDENTIFY_VOLTAGE_STEPS)
        {
            /* 所有台阶已完成，跳转到结果计算 */
            Rs_Identify.state = RS_DONE;
        }
        else
        {
            /* 设定当前台阶电压 */
            Rs_Identify.V_alpha = voltage_fractions[Rs_Identify.current_step] * Udc;

            Rs_Identify.step_count++;

            /* 等待电流稳定 */
            if (Rs_Identify.step_count >= RS_IDENTIFY_SETTLE_CNT)
            {
                /* 稳定后开始累加采样 */
                if (Rs_Identify.sample_index < RS_IDENTIFY_SAMPLE_CNT)
                {
                    Rs_Identify.I_alpha_sum += my_abs(Ialpha);  /* 用绝对值, 方向已知 */
                    Rs_Identify.sample_index++;
                }
                else
                {
                    /* 采样完成，记录本台阶 (V, I) */
                    Rs_Identify.V_samples[Rs_Identify.current_step] = Rs_Identify.V_alpha;
                    Rs_Identify.I_samples[Rs_Identify.current_step] =
                        Rs_Identify.I_alpha_sum / (float)RS_IDENTIFY_SAMPLE_CNT;

                    /* 过流检查 */
                    if (Rs_Identify.I_samples[Rs_Identify.current_step] > RS_IDENTIFY_MAX_CURRENT_A)
                    {
                        Rs_Identify.identified_Rs = -1.0f; /* 标记失败 */
                        Rs_Identify.state = RS_DONE;
                        break;
                    }

                    /* 进入下一台阶 */
                    Rs_Identify.current_step++;
                    Rs_Identify.step_count = 0;
                    Rs_Identify.sample_index = 0;
                    Rs_Identify.I_alpha_sum = 0.0f;
                }
            }
        }
        break;
    }

    /* ---- 辨识完成 ---- */
    case RS_DONE:
    {
        /* 仅在首次进入 DONE 时计算（step_count 作标记） */
        if (Rs_Identify.step_count == 0)
        {
            Rs_Identify.step_count = 1; /* 防止重复计算 */

            /* 清除输出电压 */
            Rs_Identify.V_alpha = 0.0f;

            /* 线性回归: Rs = (N·ΣVI - ΣV·ΣI) / (N·ΣI² - (ΣI)²) */
            float sum_V  = 0.0f;
            float sum_I  = 0.0f;
            float sum_VI = 0.0f;
            float sum_I2 = 0.0f;

            for (uint8_t i = 0; i < RS_IDENTIFY_VOLTAGE_STEPS; i++)
            {
                float V = Rs_Identify.V_samples[i];
                float I = Rs_Identify.I_samples[i];
                sum_V  += V;
                sum_I  += I;
                sum_VI += V * I;
                sum_I2 += I * I;
            }

            float N = (float)RS_IDENTIFY_VOLTAGE_STEPS;
            float denominator = N * sum_I2 - sum_I * sum_I;

            if (my_abs(denominator) > 1e-9f)
            {
                Rs_Identify.identified_Rs = (N * sum_VI - sum_V * sum_I) / denominator;
                Rs_Identify.state = RS_WRITE_BACK;

                /* 合理性检查：Rs 应在 0.05Ω ~ 5.0Ω 范围内 */
                if (Rs_Identify.identified_Rs < 0.05f || Rs_Identify.identified_Rs > 5.0f)
                {
                    Rs_Identify.identified_Rs = -2.0f; /* 结果异常 */
                }
            }
            else
            {
                Rs_Identify.identified_Rs = -1.0f; /* 分母为零，辨识失败 */
            }
        }
        /* 保持 V_alpha = 0，不输出电压 */
        break;
    }

    default:
        break;
    }

    return ret;
}
