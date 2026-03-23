#ifndef __Observer_H_
#define __Observer_H_

#include <stdint.h>
#include "Motor_parameter.h"
#include "Hal_Math.h"

struct SMO_Parameter
{
    float discrete_time;
    float Gain_Min;
    float Gain_Add;
    float est_Max;
    float ia_mat_k, ia_mat_k1;
    float ib_mat_k, ib_mat_k1;
    float E_alpha;
    float E_beta;
    float E_Peak;
    float E_LPF_Coff;
    float we_lpf;
    struct PLL tPLL;
    MOTOR_t *pMotor;
    Lookup_Table_t PLL_Kp_Lookup;
    Lookup_Table_t PLL_Ki_Lookup;
};

struct Encoder_Parameter
{
    float discrete_time;
    float theta;          /*电角度*/
    float we;       /*电角速度*/
    float rpm;      /*机械转速*/
    float rpm_last;
    int16_t offset;       /*0电角度偏移*/
    uint32_t rpm_filt_RP; /**/
    uint32_t rpm_filt_cnt;
    int32_t counter;
    uint16_t value;
    uint16_t last_value;
    struct PLL tPLL;
    uint8_t z_flag;
    uint32_t num_per_coil;
    Lookup_Table_t PLL_Kp_Lookup;
    Lookup_Table_t PLL_Ki_Lookup;
};

struct NonFluxObserver_Parameter
{
    float discrete_time;
    float gama;
    struct PLL tPLL;
    float Flux_alpha;
    float Flux_beta;
    float Flux_hat;
    float x_alpha_hat;
    float x_beta_hat;
    float y_alpha_hat;
    float y_beta_hat;
    float Eta_alpha;
    float Eta_beta;
    MOTOR_t *pMotor;
    Lookup_Table_t PLL_Kp_Lookup;
    Lookup_Table_t PLL_Ki_Lookup;
};

struct HFSWInjection_Parameter
{
    float alpha, beta;
    float U_hfj;
    float discrete_time;
    uint32_t PSR;
    uint32_t hfj_cnt;
    float Ialpha_hfj;
    float Ibeta_hfj;
    float Ialpha_hfj_last;
    float Ibeta_hfj_last;
    struct PLL tPLL;
    float Ialpha_last;
    float Ibeta_last;
    float Ialpha_last_last;
    float Ibeta_last_last;
    float Id_last, Id_last_last;
    float Id_hfj, Id_hfj_last;
};

void SMO_Observer_Init(void);
int SMO_Observer(struct SMO_Parameter *SMO, float32_t Ualpha, float32_t Ubeta,
                 float32_t Ialpha, float32_t Ibeta);

void Encode_ABZ_Init(void);
void Encode_ABZ_Get_Offset(void);
void Encode_ABZ_UpDate(void);

void Nonlinear_FluxObserver_Init(void);
int Nonlinear_FluxObserver_Update(struct NonFluxObserver_Parameter *NFO, float32_t Ualpha, float32_t Ubeta,
                                  float32_t Ialpha, float32_t Ibeta);

void HFSWInjection_Init(void);
void HFSWInjection_Update(struct HFSWInjection_Parameter *HFSW, float Ialpha, float Ibeta, float U_hfj);
void HFSWInjection_NSF(struct HFSWInjection_Parameter *HFSW, float id);

#endif // __EST_H__
