#include "PLL.h"
#include "arm_math.h"
#include "Hal_Math.h"
#include "Motor_parameter.h"
#include "tim.h"

struct SMO_Parameter SMO_OB = {0};
extern MOTOR_t PMSM_42JS;

const float Speed_index_coeff[10] = {0.0f, 0.02f, 0.05f, 0.10f, 0.20f, 0.30f, 0.40f, 0.60f, 0.80f, 1.00f};
float Speed_index[10] = {0, 20, 50, 100, 250, 500, 600, 600, 600, 600};
float Observer_PLL_Kp[10] = {0.1f, 0.5f, 1.0f, 5.0f, 10.0f, 20.0f, 50.0f, 50.0f, 50.0f, 50.0f};
float Observer_PLL_Ki[10] = {0.01f, 0.05f, 0.1f, 0.5f, 1.0f, 2.0f, 10.0f, 10.0f, 10.0f, 10.0f};

void SMO_Observer_Init(void)
{
    // memset(&SMO, 0, sizeof(SMO));
    SMO_OB.pMotor = &PMSM_42JS;
    SMO_OB.E_LPF_Coff = 0.1;
    SMO_OB.Gain_Min = 2.0f;
    SMO_OB.Gain_Add = 5.0f;
    SMO_OB.SMO_PLL.PLL_PI.kp = 50.1f;
    SMO_OB.SMO_PLL.PLL_PI.ki = 10.1f / 1.0f;
    SMO_OB.SMO_PLL.PLL_PI.out_max = 10000.0f;
    SMO_OB.SMO_PLL.PLL_PI.out_min = -10000.0f;
}

float deta;

int PLL_Updata(struct PLL *pPLL, float alpha, float beta)
{
    deta = alpha * arm_cos_f32(pPLL->theta) - beta * arm_sin_f32(pPLL->theta);
    pPLL->we = Hal_PI_f32(&pPLL->PLL_PI, deta);
    pPLL->theta = (pPLL->theta + pPLL->we * MOTOR_CURRENT_LOOP_CYCLE_TIME_S);
    pPLL->theta = Limit_2PI(pPLL->theta);
    return 0;
}