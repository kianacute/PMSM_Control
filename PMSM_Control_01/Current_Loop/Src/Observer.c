#include "Observer.h"
#include "arm_math.h"
#include "Motor_parameter.h"
#include "tim.h"


struct SMO_Parameter SMO_OB = {0};
extern MOTOR_t PMSM_42JS;

struct EMF_Cal_Parameter EMF_Cal = {0};

float Speed_index_coeff[10] = {0.0f, 0.02f, 0.05f, 0.10f, 0.20f, 0.30f, 0.40f, 0.60f, 0.80f, 1.00f};
float Observer_Lookup_Speed_index[10] = {0, 500, 1000, 1500, 2000,
                                         2500, 3000, 3500, 4000, 5000};
float Observer_PLL_Kp[10] =
    {
        148.0737312,
        148.0737312,
        296.1474624,
        444.2211936,
        592.2949249,
        740.3686561,
        888.4423873,
        1036.516118,
        1184.58985,
        1480.737312,
};

float Observer_PLL_Ki[10] =
    {
        0.548311337,
        0.548311337,
        2.193245348,
        4.934802032,
        8.772981391,
        13.70778342,
        19.73920813,
        26.86725551,
        35.09192556,
        54.83113369,
};

float LPK_KP[10] = {0.05f, 0.05f, 0.1f, 0.10f, 0.1f,
                    0.2f, 0.3f, 0.3f, 0.5f, 0.5f};
float SMO_EKF[10] = {1.0f, 2.0f, 4.0f, 6.0f, 8.0f,
                        10.0f, 12.2f, 14.0f, 16.0f, 16.0f};
float SMO_Gain_Lookup[10] = {6.0f, 6.0f, 6.0f, 6.0f, 6.0f,
                       6.0f, 6.0f, 6.0f, 6.0f, 6.0f};
float NorObserver_Gama_Lookup[10] = {1e4, 1e5, 1e6, 1e6, 1e6,
                                     2e6, 3e6, 4e6, 5e6, 5e6};

void SMO_Observer_Init(void)
{
    // memset(&SMO, 0, sizeof(SMO));
    SMO_OB.pMotor = &PMSM_42JS;
    SMO_OB.E_LPF_Coff = 0.3f;
    SMO_OB.Gain = 1.0f;
    SMO_OB.tPLL.PLL_PI.kp = 40.1f;
    SMO_OB.tPLL.PLL_PI.ki = 5.1f / 100.0f;
    SMO_OB.tPLL.PLL_PI.out_max = 10000.0f;
    SMO_OB.tPLL.PLL_PI.out_min = -10000.0f;
    SMO_OB.discrete_time = MOTOR_CURRENT_LOOP_CYCLE_TIME_S;
    SMO_OB.SMO_Gain_Lookup.x_table = Observer_Lookup_Speed_index;
    SMO_OB.SMO_Gain_Lookup.y_table = SMO_Gain_Lookup;
    SMO_OB.SMO_Gain_Lookup.table_size = sizeof(Observer_Lookup_Speed_index) / sizeof(Observer_Lookup_Speed_index[0]);
    SMO_OB.SMO_EKF_Lookup.x_table = Observer_Lookup_Speed_index;
    SMO_OB.SMO_EKF_Lookup.y_table = SMO_EKF;
    SMO_OB.SMO_EKF_Lookup.table_size = sizeof(Observer_Lookup_Speed_index) / sizeof(Observer_Lookup_Speed_index[0]);
    SMO_OB.PLL_Kp_Lookup.x_table = Observer_Lookup_Speed_index;
    SMO_OB.PLL_Kp_Lookup.y_table = Observer_PLL_Kp;
    SMO_OB.PLL_Kp_Lookup.table_size = 10;
    SMO_OB.PLL_Ki_Lookup.x_table = Observer_Lookup_Speed_index;
    SMO_OB.PLL_Ki_Lookup.y_table = Observer_PLL_Ki;
    SMO_OB.PLL_Ki_Lookup.table_size = 10;
}

void EMF_CAL_Init(void)
{
    EMF_Cal.pMotor = &PMSM_42JS;
    EMF_Cal.EMF_alpha = 0.0f;
    EMF_Cal.EMF_beta = 0.0f;
    EMF_Cal.ialpha_last = 0.0f;
    EMF_Cal.ibeta_last = 0.0f;
    EMF_Cal.EMF_LPF_Coff = 0.0001f;
}
 

void EMF_CAL_Update(struct EMF_Cal_Parameter *EMF_Cal, float32_t Ualpha, float32_t Ubeta,
                 float32_t Ialpha, float32_t Ibeta, float discrete_time)
{

    EMF_Cal->Ls_Ialpha = (EMF_Cal->pMotor->phase_inductance_s * (Ialpha - EMF_Cal->ialpha_last) / discrete_time) 
                        * EMF_Cal->EMF_LPF_Coff + EMF_Cal->Ls_Ialpha * (1 - EMF_Cal->EMF_LPF_Coff);
    EMF_Cal->Ls_Ibeta = (EMF_Cal->pMotor->phase_inductance_s * (Ibeta - EMF_Cal->ibeta_last) / discrete_time) 
                        * EMF_Cal->EMF_LPF_Coff + EMF_Cal->Ls_Ibeta * (1 - EMF_Cal->EMF_LPF_Coff);

    EMF_Cal->EMF_alpha = Ualpha - Ialpha * EMF_Cal->pMotor->phase_resistance_ohm - EMF_Cal->Ls_Ialpha;
    EMF_Cal->EMF_beta = Ubeta - Ibeta * EMF_Cal->pMotor->phase_resistance_ohm - EMF_Cal->Ls_Ibeta;
    EMF_Cal->ialpha_last = Ialpha;
    EMF_Cal->ibeta_last = Ibeta;
    arm_sqrt_f32(EMF_Cal->EMF_alpha * EMF_Cal->EMF_alpha + EMF_Cal->EMF_beta * EMF_Cal->EMF_beta, &EMF_Cal->EMF);
}

void SMO_Observer(struct SMO_Parameter *SMO, float32_t Ualpha, float32_t Ubeta,
                 float32_t Ialpha, float32_t Ibeta)
{
    SMO->E_alpha = (SMO->ia_mat_k - Ialpha) * SMO->Gain;
    SMO->E_beta = (SMO->ib_mat_k - Ibeta) * SMO->Gain;
    SMO->ia_mat_k1 = SMO->ia_mat_k + ((-SMO->pMotor->phase_resistance_ohm) / SMO->pMotor->phase_inductance_s 
                    * SMO->ia_mat_k + Ualpha / SMO->pMotor->phase_inductance_s - SMO->E_alpha / SMO->pMotor->phase_inductance_s) 
                    * SMO->discrete_time;
    SMO->ib_mat_k1 = SMO->ib_mat_k + ((-SMO->pMotor->phase_resistance_ohm) / SMO->pMotor->phase_inductance_s 
                    * SMO->ib_mat_k + Ubeta / SMO->pMotor->phase_inductance_s - SMO->E_beta / SMO->pMotor->phase_inductance_s) 
                    * SMO->discrete_time;
    arm_sqrt_f32(SMO->E_alpha * SMO->E_alpha + SMO->E_beta * SMO->E_beta, &SMO->E_Peak);
    /*保留这一拍的数据*/
    SMO->ia_mat_k = SMO->ia_mat_k1;
    SMO->ib_mat_k = SMO->ib_mat_k1;
    PLL_Update(&SMO->tPLL, -SMO->E_alpha, SMO->E_beta, SMO->discrete_time);
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

float tmp;

void Encode_ABZ_UpDate(void)
{
    Encode_ABZ.value = read_Encode_ABZ() - Encode_ABZ.offset;
    if (Encode_ABZ.value < 0)
    {
        Encode_ABZ.theta = (float32_t)((Encode_ABZ.num_per_coil + Encode_ABZ.value) % Encode_ABZ.num_per_coil) * 2.0f * PI / ((float)Encode_ABZ.num_per_coil) * MOTOR_POLE_PAIRS;
    }
    else if (Encode_ABZ.value > Encode_ABZ.num_per_coil)
    {
        Encode_ABZ.theta = (float32_t)((-Encode_ABZ.num_per_coil + Encode_ABZ.value) % Encode_ABZ.num_per_coil) * 2.0f * PI / ((float)Encode_ABZ.num_per_coil) * MOTOR_POLE_PAIRS;
    }
    else
    {
        Encode_ABZ.theta = (float32_t)((Encode_ABZ.value) % Encode_ABZ.num_per_coil) * 2.0f * PI / ((float)Encode_ABZ.num_per_coil) * MOTOR_POLE_PAIRS;
    }

    Encode_ABZ.theta = Limit_2PI(Encode_ABZ.theta);

    // PLL_Update(&Encode_ABZ.tPLL, arm_sin_f32(Encode_ABZ.theta), arm_cos_f32(Encode_ABZ.theta), &Encode_ABZ.speed_we, &Encode_ABZ.theta);
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
        Encode_ABZ.rpm = Encode_ABZ.rpm_last * 0.7 + 0.3 * (float32_t)(Encode_ABZ.counter) * 60.0f /
                                                         ((float)Encode_ABZ.num_per_coil) / (Encode_ABZ.discrete_time * (float)Encode_ABZ.rpm_filt_cnt);
        Encode_ABZ.we = Encode_ABZ.rpm * 2.0f * PI / 60.0f * MOTOR_POLE_PAIRS;
        Encode_ABZ.rpm_last = Encode_ABZ.rpm;
        Encode_ABZ.counter = 0;
        Encode_ABZ.rpm_filt_cnt = 0;
    }
    Encode_ABZ.last_value = Encode_ABZ.value;
}

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
    NonFlux_OB.pMotor = &PMSM_42JS;
    NonFlux_OB.PLL_Kp_Lookup.x_table = Observer_Lookup_Speed_index;
    NonFlux_OB.PLL_Kp_Lookup.y_table = Observer_PLL_Kp;
    NonFlux_OB.PLL_Kp_Lookup.table_size = 10;
    NonFlux_OB.PLL_Ki_Lookup.x_table = Observer_Lookup_Speed_index;
    NonFlux_OB.PLL_Ki_Lookup.y_table = Observer_PLL_Ki;
    NonFlux_OB.PLL_Ki_Lookup.table_size = 10;
    NonFlux_OB.Gama_Lookup.x_table = Observer_Lookup_Speed_index;
    NonFlux_OB.Gama_Lookup.y_table = NorObserver_Gama_Lookup;
    NonFlux_OB.Gama_Lookup.table_size = 10;
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

void Nonlinear_FluxObserver_Update(struct NonFluxObserver_Parameter *NFO, float32_t Ualpha, float32_t Ubeta,
                                  float32_t Ialpha, float32_t Ibeta)
{
    NFO->y_alpha_hat = (Ualpha - NFO->pMotor->phase_resistance_ohm * Ialpha);
    NFO->y_beta_hat = (Ubeta - NFO->pMotor->phase_resistance_ohm * Ibeta);
    // NFO->y_alpha_hat = (0 - NFO->pMotor->phase_resistance_ohm * Ialpha);
    // NFO->y_beta_hat = (0 - NFO->pMotor->phase_resistance_ohm * Ibeta);
    NFO->x_alpha_hat += ((NFO->y_alpha_hat + NFO->gama * NFO->Eta_alpha * (NFO->pMotor->flux_linkage_wb * NFO->pMotor->flux_linkage_wb - NFO->Flux_hat)) * NFO->discrete_time);
    NFO->x_beta_hat += ((NFO->y_beta_hat + NFO->gama * NFO->Eta_beta * (NFO->pMotor->flux_linkage_wb * NFO->pMotor->flux_linkage_wb - NFO->Flux_hat)) * NFO->discrete_time);
    NFO->Eta_alpha = NFO->x_alpha_hat - NFO->pMotor->phase_inductance_s * Ialpha;
    NFO->Eta_beta = NFO->x_beta_hat - NFO->pMotor->phase_inductance_s * Ibeta;
    NFO->Flux_hat = NFO->Eta_alpha * NFO->Eta_alpha + NFO->Eta_beta * NFO->Eta_beta;
    NFO->Flux_alpha = NFO->Eta_alpha / NFO->pMotor->flux_linkage_wb;
    NFO->Flux_beta = NFO->Eta_beta / NFO->pMotor->flux_linkage_wb;
    PLL_Update(&NFO->tPLL, NFO->Flux_beta, NFO->Flux_alpha, NFO->discrete_time);
    EMF_CAL_Update(&EMF_Cal, Ualpha, Ubeta, Ialpha, Ibeta, NFO->discrete_time);
}

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

void HFSWInjection_Update(struct HFSWInjection_Parameter *HFSW, float Ialpha, float Ibeta, float U_hfj)
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
