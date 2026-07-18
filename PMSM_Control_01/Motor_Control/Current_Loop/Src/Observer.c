#include "Observer.h"
#include "Motor_Config.h"

struct EMF_Cal_Parameter EMF_Cal = {0};
extern Motor_Config_t PMSM_42JS_Config;

void EMF_CAL_Init(void)
{
    EMF_Cal.pMotor = &PMSM_42JS_Config;
    EMF_Cal.EMF_alpha = 0.0f;
    EMF_Cal.EMF_beta = 0.0f;
    EMF_Cal.ialpha_last = 0.0f;
    EMF_Cal.ibeta_last = 0.0f;
    EMF_Cal.EMF_LPF_Coff = 0.0001f;
}

void EMF_CAL_Updata(struct EMF_Cal_Parameter *EMF_Cal, float32_t Ualpha, float32_t Ubeta,
                    float32_t Ialpha, float32_t Ibeta, float discrete_time)
{

    EMF_Cal->Ls_Ialpha = (EMF_Cal->pMotor->motor_param->Rs * (Ialpha - EMF_Cal->ialpha_last) / discrete_time) * EMF_Cal->EMF_LPF_Coff + EMF_Cal->Ls_Ialpha * (1 - EMF_Cal->EMF_LPF_Coff);
    EMF_Cal->Ls_Ibeta = (EMF_Cal->pMotor->motor_param->Ls * (Ibeta - EMF_Cal->ibeta_last) / discrete_time) * EMF_Cal->EMF_LPF_Coff + EMF_Cal->Ls_Ibeta * (1 - EMF_Cal->EMF_LPF_Coff);

    EMF_Cal->EMF_alpha = Ualpha - Ialpha * EMF_Cal->pMotor->motor_param->Rs - EMF_Cal->Ls_Ialpha;
    EMF_Cal->EMF_beta = Ubeta - Ibeta * EMF_Cal->pMotor->motor_param->Rs - EMF_Cal->Ls_Ibeta;
    EMF_Cal->ialpha_last = Ialpha;
    EMF_Cal->ibeta_last = Ibeta;
    arm_sqrt_f32(EMF_Cal->EMF_alpha * EMF_Cal->EMF_alpha + EMF_Cal->EMF_beta * EMF_Cal->EMF_beta, &EMF_Cal->EMF);
}



#ifdef MOTOR_SMO_OBSERVER

struct SMO_Parameter SMO_OB = {0};

void SMO_Observer_Init(void)
{
    // memset(&SMO, 0, sizeof(SMO));
    SMO_OB.pMotor = &PMSM_42JS_Config;
    SMO_OB.E_LPF_Coff = 0.3f;
    SMO_OB.Gain = 1.0f;
    SMO_OB.tPLL.PLL_PI.kp = 40.1f;
    SMO_OB.tPLL.PLL_PI.ki = 5.1f / 100.0f;
    SMO_OB.tPLL.PLL_PI.out_max = 10000.0f;
    SMO_OB.tPLL.PLL_PI.out_min = -10000.0f;
    SMO_OB.discrete_time = MOTOR_CURRENT_LOOP_CYCLE_TIME_S;
    SMO_OB.tPLL.PLL_PI.integral = 0.0f;
    SMO_OB.ia_mat_k = 0.0f;
}

void SMO_Observer_Updata(struct SMO_Parameter *SMO, float32_t Ualpha, float32_t Ubeta,
                         float32_t Ialpha, float32_t Ibeta)
{
    SMO->E_alpha = (SMO->ia_mat_k - Ialpha) * SMO->Gain;
    SMO->E_beta = (SMO->ib_mat_k - Ibeta) * SMO->Gain;
    SMO->ia_mat_k1 = SMO->ia_mat_k + ((-SMO->pMotor->motor_param->Rs) / SMO->pMotor->motor_param->Ls * SMO->ia_mat_k + Ualpha / SMO->pMotor->motor_param->Ls - SMO->E_alpha / SMO->pMotor->motor_param->Ls) * SMO->discrete_time;
    SMO->ib_mat_k1 = SMO->ib_mat_k + ((-SMO->pMotor->motor_param->Rs) / SMO->pMotor->motor_param->Ls * SMO->ib_mat_k + Ubeta / SMO->pMotor->motor_param->Ls - SMO->E_beta / SMO->pMotor->motor_param->Ls) * SMO->discrete_time;
    arm_sqrt_f32(SMO->E_alpha * SMO->E_alpha + SMO->E_beta * SMO->E_beta, &SMO->E_Peak);
    /*保留这一拍的数据*/
    SMO->ia_mat_k = SMO->ia_mat_k1;
    SMO->ib_mat_k = SMO->ib_mat_k1;
    PLL_Update(&SMO->tPLL, -SMO->E_alpha, SMO->E_beta, SMO->discrete_time);
}

#endif

#ifdef MOTOR_ENCODER_OBSERVER
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
    Encode_ABZ.discrete_time = MOTOR_CURRENT_LOOP_CYCLE_TIME_S;
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

void Encode_ABZ_Updata(void)
{
    Encode_ABZ.value = read_Encode_ABZ() - Encode_ABZ.offset;
    if (Encode_ABZ.value < 0)
    {
        Encode_ABZ.theta = (float32_t)((Encode_ABZ.num_per_coil + Encode_ABZ.value) % Encode_ABZ.num_per_coil) * 2.0f * PI / ((float)Encode_ABZ.num_per_coil) * (float)4.0f;
    }
    else if (Encode_ABZ.value > Encode_ABZ.num_per_coil)
    {
        Encode_ABZ.theta = (float32_t)((-Encode_ABZ.num_per_coil + Encode_ABZ.value) % Encode_ABZ.num_per_coil) * 2.0f * PI / ((float)Encode_ABZ.num_per_coil) * (float)4.0f;
    }
    else
    {
        Encode_ABZ.theta = (float32_t)((Encode_ABZ.value) % Encode_ABZ.num_per_coil) * 2.0f * PI / ((float)Encode_ABZ.num_per_coil) * (float)4.0f;
    }

    Encode_ABZ.theta = Limit_2PI(Encode_ABZ.theta);

    // PLL_Update(&Encode_ABZ.tPLL, arm_sin_f32(Encode_ABZ.theta), arm_cos_f32(Encode_ABZ.theta), &Encode_ABZ.speed_we, &Encode_ABZ.theta);
    // Encode_ABZ.speed_rpm = Encode_ABZ.speed_we * 60.0f / 2.0f / PI / (float)4.0f;
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
        Encode_ABZ.rpm = Encode_ABZ.rpm_last * 0.7 + 0.3 * (float32_t)(Encode_ABZ.counter) * 60.0f /
                                                         ((float)Encode_ABZ.num_per_coil) / (Encode_ABZ.discrete_time * (float)Encode_ABZ.rpm_filt_cnt);
        Encode_ABZ.we = Encode_ABZ.rpm * 2.0f * PI / 60.0f * (float)4.0f;
        Encode_ABZ.rpm_last = Encode_ABZ.rpm;
        Encode_ABZ.counter = 0;
        Encode_ABZ.rpm_filt_cnt = 0;
    }
    Encode_ABZ.last_value = Encode_ABZ.value;
}

#endif

#ifdef MOTOR_NONFLUX_OBSERVER

/*非线性磁链观测器*/

struct NonFluxObserver_Parameter NonFlux_OB = {0};

void Nonlinear_FluxObserver_Init(void)
{
    NonFlux_OB.discrete_time = MOTOR_CURRENT_LOOP_CYCLE_TIME_S;
    NonFlux_OB.Flux_alpha = 0.0f;
    NonFlux_OB.Flux_beta = 0.0f;
    NonFlux_OB.tPLL.PLL_PI.kp = 200.1f / 1.0f;
    NonFlux_OB.tPLL.PLL_PI.ki = 16.1f / 40.0f;
    NonFlux_OB.tPLL.PLL_PI.out_max = 10000.0f;
    NonFlux_OB.tPLL.PLL_PI.out_min = -10000.0f;
    NonFlux_OB.gama = 1000000.0f;
    NonFlux_OB.pMotor = &PMSM_42JS_Config;
    NonFlux_OB.x_alpha_hat = 0.0f;
    NonFlux_OB.x_beta_hat = 0.0f;
    NonFlux_OB.y_alpha_hat = 0.0f;
    NonFlux_OB.y_beta_hat = 0.0f;
    NonFlux_OB.Eta_alpha = 0.0f;
    NonFlux_OB.Eta_beta = 0.0f;
    NonFlux_OB.Flux_hat = 0.0f;
    NonFlux_OB.Flux_alpha = 0.0f;
    NonFlux_OB.Flux_beta = 0.0f;
    NonFlux_OB.tPLL.we = 0.0f;
    NonFlux_OB.tPLL.theta = 0.0f;
    EMF_CAL_Init();
}

void Nonlinear_FluxObserver_Updata(struct NonFluxObserver_Parameter *NFO, float32_t Ualpha, float32_t Ubeta,
                                   float32_t Ialpha, float32_t Ibeta)
{
    NFO->y_alpha_hat = (Ualpha - NFO->pMotor->motor_param->Rs * Ialpha);
    NFO->y_beta_hat = (Ubeta - NFO->pMotor->motor_param->Rs * Ibeta);
    // NFO->y_alpha_hat = (0 - NFO->pMotor->motor_param->Rs * Ialpha);
    // NFO->y_beta_hat = (0 - NFO->pMotor->motor_param->Rs * Ibeta);
    NFO->x_alpha_hat += ((NFO->y_alpha_hat + NFO->gama * NFO->Eta_alpha * (NFO->pMotor->motor_param->flux_linkage_wb * NFO->pMotor->motor_param->flux_linkage_wb - NFO->Flux_hat)) * NFO->discrete_time);
    NFO->x_beta_hat += ((NFO->y_beta_hat + NFO->gama * NFO->Eta_beta * (NFO->pMotor->motor_param->flux_linkage_wb * NFO->pMotor->motor_param->flux_linkage_wb - NFO->Flux_hat)) * NFO->discrete_time);
    NFO->Eta_alpha = NFO->x_alpha_hat - NFO->pMotor->motor_param->Ls * Ialpha;
    NFO->Eta_beta = NFO->x_beta_hat - NFO->pMotor->motor_param->Ls * Ibeta;
    NFO->Flux_hat = NFO->Eta_alpha * NFO->Eta_alpha + NFO->Eta_beta * NFO->Eta_beta;
    NFO->Flux_alpha = NFO->Eta_alpha / NFO->pMotor->motor_param->flux_linkage_wb;
    NFO->Flux_beta = NFO->Eta_beta / NFO->pMotor->motor_param->flux_linkage_wb;
    PLL_Update(&NFO->tPLL, NFO->Flux_beta, NFO->Flux_alpha, NFO->discrete_time);
    EMF_CAL_Updata(&EMF_Cal, Ualpha, Ubeta, Ialpha, Ibeta, NFO->discrete_time);
}

#endif

#ifdef MOTOR_EFFECTIVE_FLUX_OBSERVER

/*有效磁链观测器*/
struct EffFluxObserver_Parameter EffFlux_OB = {0};

void Effective_FluxObserver_Init(void)
{
    EffFlux_OB.discrete_time = MOTOR_CURRENT_LOOP_CYCLE_TIME_S;
    EffFlux_OB.Flux_alpha = 0.0f;
    EffFlux_OB.Flux_beta = 0.0f;
    EffFlux_OB.tPLL.PLL_PI.kp = 200.1f / 1.0f;
    EffFlux_OB.tPLL.PLL_PI.ki = 16.1f / 40.0f;
    EffFlux_OB.tPLL.PLL_PI.out_max = 10000.0f;
    EffFlux_OB.tPLL.PLL_PI.out_min = -10000.0f;
    EffFlux_OB.gama = 200.0f;
    EffFlux_OB.pMotor = &PMSM_42JS_Config;
    EffFlux_OB.x_alpha_hat = 0.0f;
    EffFlux_OB.x_beta_hat = 0.0f;
    EffFlux_OB.y_alpha_hat = 0.0f;
    EffFlux_OB.y_beta_hat = 0.0f;
    EffFlux_OB.Eta_alpha = 0.0f;
    EffFlux_OB.Eta_beta = 0.0f;
    EffFlux_OB.Flux_hat = 0.0f;
    EffFlux_OB.Flux_alpha = 0.0f;
    EffFlux_OB.Flux_beta = 0.0f;
    EffFlux_OB.tPLL.we = 0.0f;
    EffFlux_OB.tPLL.theta = 0.0f;
    EMF_CAL_Init();
}

void Effective_FluxObserver_Updata(struct EffFluxObserver_Parameter *EFO, float32_t Ualpha, float32_t Ubeta,
                                   float32_t Ialpha, float32_t Ibeta)
{
    arm_park_f32(Ialpha, Ibeta, &EFO->Id, &EFO->Iq, EFO->Sin, EFO->Cos);
    EFO->FLux_D = EFO->Id * EFO->pMotor->motor_param->Ld + EFO->pMotor->motor_param->flux_linkage_wb;
    EFO->Flux_Q = EFO->Iq * EFO->pMotor->motor_param->Lq;
    arm_inv_park_f32(EFO->FLux_D, EFO->Flux_Q, &EFO->Flux_alpha, &EFO->Flux_beta, EFO->Sin, EFO->Cos);
    EFO->x_alpha_hat += ((Ualpha + EFO->gama * (EFO->Flux_alpha - EFO->x_alpha_hat)) * EFO->discrete_time);
    EFO->x_beta_hat += ((Ubeta + EFO->gama * (EFO->Flux_beta - EFO->x_beta_hat)) * EFO->discrete_time);
    EFO->y_alpha_hat = EFO->x_alpha_hat - EFO->pMotor->motor_param->Lq * Ialpha;
    EFO->y_beta_hat = EFO->x_beta_hat - EFO->pMotor->motor_param->Lq * Ibeta;
    EFO->Eta_alpha = EFO->y_alpha_hat / EFO->pMotor->motor_param->flux_linkage_wb;
    EFO->Eta_beta = EFO->y_beta_hat / EFO->pMotor->motor_param->flux_linkage_wb;
    PLL_Update(&EFO->tPLL, EFO->Eta_beta, EFO->Eta_alpha, EFO->discrete_time);
    EFO->Sin = arm_sin_f32(EFO->tPLL.theta);
    EFO->Cos = arm_cos_f32(EFO->tPLL.theta);
    EMF_CAL_Updata(&EMF_Cal, Ualpha, Ubeta, Ialpha, Ibeta, EFO->discrete_time);
}

#endif

#ifdef MOTOR_HFSW_OBSERVER

struct HFSWInjection_Parameter HFSW_OB =
    {
        .discrete_time = MOTOR_CURRENT_LOOP_CYCLE_TIME_S,
        .PSR = 2,
        .U_hfj = 2.4f,
        .hfj_cnt = 0,
};

void HFSWInjection_Init(void)
{
    HFSW_OB.U_hfj = 2.4f;
    HFSW_OB.hfj_cnt = 0;
    HFSW_OB.tPLL.PLL_PI.kp = 800.1f / 1.0f;
    HFSW_OB.tPLL.PLL_PI.ki = 16.1f / 40.0f;
    HFSW_OB.tPLL.PLL_PI.out_max = 10000.0f;
    HFSW_OB.tPLL.PLL_PI.out_min = -10000.0f;
}

void HFSWInjection_Updata(struct HFSWInjection_Parameter *HFSW, float Ialpha, float Ibeta, float U_hfj)
{
    HFSW->Ialpha_hfj = (Ialpha - HFSW->Ialpha_last * 2 + HFSW->Ialpha_last_last) / 4.0f;
    HFSW->Ibeta_hfj = (Ibeta - HFSW->Ibeta_last * 2 + HFSW->Ibeta_last_last) / 4.0f;
    if (U_hfj > 0)
    {
        HFSW->alpha = HFSW->Ialpha_hfj - HFSW->Ialpha_hfj_last;
        HFSW->beta = HFSW->Ibeta_hfj - HFSW->Ibeta_hfj_last;
    }
    else
    {
        HFSW->alpha = -(HFSW->Ialpha_hfj - HFSW->Ialpha_hfj_last);
        HFSW->beta = -(HFSW->Ibeta_hfj - HFSW->Ibeta_hfj_last);
    }
    HFSW->Ialpha_last_last = HFSW->Ialpha_last;
    HFSW->Ialpha_last = Ialpha;
    HFSW->Ibeta_last_last = HFSW->Ibeta_last;
    HFSW->Ibeta_last = Ibeta;

    HFSW->Ialpha_hfj_last = HFSW->Ialpha_hfj;
    HFSW->Ibeta_hfj_last = HFSW->Ibeta_hfj;

    PLL_Update(&HFSW->tPLL, -HFSW->alpha, HFSW->beta, HFSW->discrete_time);
}

void HFSWInjection_NSF(struct HFSWInjection_Parameter *HFSW, float id)
{
    HFSW->Id_hfj = (id - HFSW->Id_last * 2 + HFSW->Id_last_last) / 4.0f - HFSW->Id_hfj_last;
    HFSW->Id_last = id;
    HFSW->Id_last_last = HFSW->Id_last;
}

#endif

void Observer_Param_Lookup_Updata(float Speed)
{
#ifdef MOTOR_NONFLUX_OBSERVER
    NonFlux_OB.tPLL.PLL_PI.kp = Lookup_Table_Linear(Speed, &PMSM_42JS_Config.NonFlux_PLL_Kp_Lookup);
    NonFlux_OB.tPLL.PLL_PI.ki = Lookup_Table_Linear(Speed, &PMSM_42JS_Config.NonFlux_PLL_Ki_Lookup);
    NonFlux_OB.gama = Lookup_Table_Linear(Speed, &PMSM_42JS_Config.NonFlux_Gama_Lookup);
#endif

#ifdef MOTOR_EFFECTIVE_FLUX_OBSERVER
    EffFlux_OB.tPLL.PLL_PI.kp = Lookup_Table_Linear(Speed, &PMSM_42JS_Config.NonFlux_PLL_Kp_Lookup);
    EffFlux_OB.tPLL.PLL_PI.ki = Lookup_Table_Linear(Speed, &PMSM_42JS_Config.NonFlux_PLL_Ki_Lookup);
#endif

#ifdef MOTOR_SMO_OBSERVER
    SMO_OB.Gain = Lookup_Table_Linear(Speed, &PMSM_42JS_Config.SMO_Gain_Lookup);
    SMO_OB.tPLL.PLL_PI.kp = Lookup_Table_Linear(Speed, &PMSM_42JS_Config.SMO_PLL_Kp_Lookup);
    SMO_OB.tPLL.PLL_PI.ki = Lookup_Table_Linear(Speed, &PMSM_42JS_Config.SMO_PLL_Ki_Lookup);
#endif
}
