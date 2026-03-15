#include "Observer.h"
#include "arm_math.h"
#include "Hal_Math.h"
#include "Motor_parameter.h"
#include "tim.h"

struct SMO_Parameter SMO_OB = {0};
extern MOTOR_t PMSM_42JS;

float Speed_index_coeff[10] = {0.0f, 0.02f, 0.05f, 0.10f, 0.20f, 0.30f, 0.40f, 0.60f, 0.80f, 1.00f};
float Speed_index[10] = {0, 20, 50, 100, 250, 500, 600, 600, 600, 600};
float Observer_PLL_Kp[10] = {0.1f, 0.5f, 1.0f, 5.0f, 10.0f, 20.0f, 50.0f, 50.0f, 50.0f, 50.0f};
float Observer_PLL_Ki[10] = {0.01f, 0.05f, 0.1f, 0.5f, 1.0f, 2.0f, 10.0f, 10.0f, 10.0f, 10.0f};


void PLL_PI_Parameter_Init(struct PLL *pPLL, float Speed_Ref)
{
    for (int i = 0; i < 10; i++)
    {
        if (Speed_Ref < Speed_index[i])
        {
            pPLL->PLL_PI.kp = Observer_PLL_Kp[i];
            pPLL->PLL_PI.ki = Observer_PLL_Ki[i];
            break;
        }
    }
}

void PLL_PI_Parameter_Update(struct PLL *pPLL, float Speed_Ref);

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

int SMO_Observer(struct SMO_Parameter *SMO, float32_t Ualpha, float32_t Ubeta,
                 float32_t Ialpha, float32_t Ibeta)
{
    float Est_a, Est_b;
    SMO->we_lpf = SMO->SMO_PLL.we * 0.001f + SMO->we_lpf * 0.999f;
    if (SMO->ia_mat_k > Ialpha)
    {
        Est_a = SMO->Gain_Min + SMO->Gain_Add;
    }
    else
    {
        Est_a = -(SMO->Gain_Min + SMO->Gain_Add);
    }

    if (SMO->ib_mat_k > Ibeta)
    {
        Est_b = SMO->Gain_Min + SMO->Gain_Add;
    }
    else
    {
        Est_b = -(SMO->Gain_Min + SMO->Gain_Add);
    }
    SMO->ia_mat_k1 = SMO->ia_mat_k + ((-SMO->pMotor->phase_resistance_ohm) / SMO->pMotor->phase_inductance_s
                     * SMO->ia_mat_k + Ualpha / SMO->pMotor->phase_inductance_s - Est_a / SMO->pMotor->phase_inductance_s)
                     * MOTOR_CURRENT_LOOP_CYCLE_TIME_S;
    SMO->ib_mat_k1 = SMO->ib_mat_k + ((-SMO->pMotor->phase_resistance_ohm) / SMO->pMotor->phase_inductance_s
                     * SMO->ib_mat_k + Ubeta / SMO->pMotor->phase_inductance_s - Est_b / SMO->pMotor->phase_inductance_s)
                     * MOTOR_CURRENT_LOOP_CYCLE_TIME_S;

    /*观测到的反电动势必须滤波*/
    SMO->E_alpha = Est_a * SMO->E_LPF_Coff + SMO->E_alpha * (1 - SMO->E_LPF_Coff);
    SMO->E_beta = Est_b * SMO->E_LPF_Coff + SMO->E_beta * (1 - SMO->E_LPF_Coff);
    arm_sqrt_f32(SMO->E_alpha * SMO->E_alpha + SMO->E_beta * SMO->E_beta, &SMO->E_Peak);
    /*保留这一拍的数据*/
    SMO->ia_mat_k = SMO->ia_mat_k1;
    SMO->ib_mat_k = SMO->ib_mat_k1;
    PLL_Updata(&SMO->SMO_PLL, -SMO->E_alpha, SMO->E_beta);

    return 0;
}

struct Encoder_Parameter Encode_ABZ;

void Encode_ABZ_Init(void)
{
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); // Start TIM4 in encoder mode
    __HAL_TIM_SET_COUNTER(&htim4, 0);               // Reset the counter to 0
    Encode_ABZ.tPLL.PLL_PI.kp = 8.1f / 1.0f;
    Encode_ABZ.tPLL.PLL_PI.ki = 16.1f / 2000.0f;
    Encode_ABZ.tPLL.PLL_PI.out_max = 10000.0f;  
    Encode_ABZ.tPLL.PLL_PI.out_min = -10000.0f; 
    Encode_ABZ.offset = 0;
    Encode_ABZ.rpm_filt_RP = MOTOR_CURRENT_LOOP_HZ / MOTOR_SPEED_LOOP_HZ;
    Encode_ABZ.rpm_filt_cnt = 0;
    Encode_ABZ.counter = 0;
    Encode_ABZ.value = 0;
    Encode_ABZ.last_value = 0;
    Encode_ABZ.z_flag = 0;
    Encode_ABZ.num_per_coil = 4000;
}

uint16_t read_Encode_ABZ(void)
{
    uint16_t tmp = __HAL_TIM_GET_COUNTER(&htim4);
    return tmp;
}

void Encode_ABZ_Get_Offset(void)
{
    __HAL_TIM_SET_COUNTER(&htim4, 0);
}

float tmp;

void Encode_ABZ_UpDate(void)
{
    Encode_ABZ.value = read_Encode_ABZ() - Encode_ABZ.offset;
    if (Encode_ABZ.value < 0)
    {
        Encode_ABZ.theta = (float32_t)((Encode_ABZ.num_per_coil + Encode_ABZ.value) % Encode_ABZ.num_per_coil)
             * 2.0f * PI /((float)Encode_ABZ.num_per_coil) * MOTOR_POLE_PAIRS;
    }
    else if (Encode_ABZ.value > Encode_ABZ.num_per_coil)
    {
        Encode_ABZ.theta = (float32_t)((-Encode_ABZ.num_per_coil + Encode_ABZ.value) % Encode_ABZ.num_per_coil)
             * 2.0f * PI /((float)Encode_ABZ.num_per_coil) * MOTOR_POLE_PAIRS;
    }
    else
    {
        Encode_ABZ.theta = (float32_t)((Encode_ABZ.value) % Encode_ABZ.num_per_coil)
             * 2.0f * PI / ((float)Encode_ABZ.num_per_coil) * MOTOR_POLE_PAIRS;
    }
        
    Encode_ABZ.theta = Limit_2PI(Encode_ABZ.theta); 
    
    // PLL_Updata(&Encode_ABZ.tPLL, arm_sin_f32(Encode_ABZ.theta), arm_cos_f32(Encode_ABZ.theta), &Encode_ABZ.speed_we, &Encode_ABZ.theta);
    // Encode_ABZ.speed_rpm = Encode_ABZ.speed_we * 60.0f / 2.0f / PI / MOTOR_POLE_PAIRS;
    Encode_ABZ.rpm_filt_cnt++;
    if (!(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4)))
    {

        if (Encode_ABZ.value < Encode_ABZ.last_value)
        {
            Encode_ABZ.counter += (64000 + Encode_ABZ.value - Encode_ABZ.last_value);
        }
        else
        {
            Encode_ABZ.counter += (Encode_ABZ.value - Encode_ABZ.last_value);
        }
    }
    else
    {
        if (Encode_ABZ.value > Encode_ABZ.last_value)
        {
            Encode_ABZ.counter += (-64000 + Encode_ABZ.value - Encode_ABZ.last_value);
        }
        else
        {
            Encode_ABZ.counter += (Encode_ABZ.value - Encode_ABZ.last_value);
        }
    }

    if (Encode_ABZ.rpm_filt_cnt >= Encode_ABZ.rpm_filt_RP)
    {
        Encode_ABZ.speed_rpm = (float32_t)(Encode_ABZ.counter) * 60.0f / ((float)Encode_ABZ.num_per_coil) / (MOTOR_SPEED_LOOP_CYCLE_TIME_S);
        Encode_ABZ.speed_we = Encode_ABZ.speed_rpm * 2.0f * PI / 60.0f * MOTOR_POLE_PAIRS;
        Encode_ABZ.counter = 0;
        Encode_ABZ.rpm_filt_cnt = 0;
    }
    Encode_ABZ.last_value = Encode_ABZ.value;
}

struct NonFluxObserver_Parameter NonFlux_OB = {0};

void Nonlinear_FluxObserver_Init(void)
{
    NonFlux_OB.Flux_alpha = 0.0f;
    NonFlux_OB.Flux_beta = 0.0f;
    NonFlux_OB.tPLL.PLL_PI.kp = 500.1f / 1.0f;
    NonFlux_OB.tPLL.PLL_PI.ki = 16.1f / 40.0f;
    NonFlux_OB.tPLL.PLL_PI.out_max = 10000.0f;
    NonFlux_OB.tPLL.PLL_PI.out_min = -10000.0f;
    NonFlux_OB.gama = 100000.0f;
    NonFlux_OB.pMotor = &PMSM_42JS;
}


int Nonlinear_FluxObserver_Update(struct NonFluxObserver_Parameter *NFO, float32_t Ualpha, float32_t Ubeta,
                                float32_t Ialpha, float32_t Ibeta)
{
    NFO->y_alpha_hat = (Ualpha - NFO->pMotor->phase_resistance_ohm * Ialpha);
    NFO->y_beta_hat = (Ubeta - NFO->pMotor->phase_resistance_ohm * Ibeta);
    NFO->x_alpha_hat += ((NFO->y_alpha_hat + NFO->gama * NFO->Eta_alpha * (NFO->pMotor->flux_linkage_wb
            * NFO->pMotor->flux_linkage_wb - NFO->Flux_hat)) * MOTOR_CURRENT_LOOP_CYCLE_TIME_S);
    NFO->x_beta_hat += ((NFO->y_beta_hat + NFO->gama * NFO->Eta_beta * (NFO->pMotor->flux_linkage_wb 
            * NFO->pMotor->flux_linkage_wb - NFO->Flux_hat)) * MOTOR_CURRENT_LOOP_CYCLE_TIME_S);    
    NFO->Eta_alpha = NFO->x_alpha_hat - NFO->pMotor->phase_inductance_s * Ialpha;
    NFO->Eta_beta = NFO->x_beta_hat - NFO->pMotor->phase_inductance_s * Ibeta;    
    NFO->Flux_hat = NFO->Eta_alpha * NFO->Eta_alpha + NFO->Eta_beta * NFO->Eta_beta;
    NFO->Flux_alpha = NFO->Eta_alpha / NFO->pMotor->flux_linkage_wb;
    NFO->Flux_beta = NFO->Eta_beta / NFO->pMotor->flux_linkage_wb; 
    PLL_Updata(&NFO->tPLL, NFO->Flux_beta, NFO->Flux_alpha); 
    return 0;
}
                                              
