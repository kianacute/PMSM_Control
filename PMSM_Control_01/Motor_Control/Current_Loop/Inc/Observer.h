#ifndef __Observer_H_
#define __Observer_H_

#include <stdint.h>
#include "Motor_Config.h"
#include "Hal_Math.h"

struct EMF_Cal_Parameter
{
    float EMF_alpha;
    float EMF_beta;
    float Ls_Ialpha;
    float Ls_Ibeta;
    float ialpha_last;
    float ibeta_last;
    float EMF;
    float EMF_LPF_Coff;
    Motor_Config_t *pMotor;
};

#ifdef MOTOR_SMO_OBSERVER

struct SMO_Parameter
{
    float discrete_time;
    float Gain;
    float est_Max;
    float ia_mat_k, ia_mat_k1;
    float ib_mat_k, ib_mat_k1;
    float E_alpha;
    float E_beta;
    float E_Peak;
    float E_LPF_Coff;
    float we_lpf;
    struct PLL tPLL;
    Motor_Config_t *pMotor;
};

void SMO_Observer_Init(void);
void SMO_Observer_Updata(struct SMO_Parameter *SMO, float32_t Ualpha, float32_t Ubeta,
                         float32_t Ialpha, float32_t Ibeta);
extern struct SMO_Parameter SMO_OB;
#define OBSERVE_Init() SMO_Observer_Init()
#define OBSERVE_Updata(Ualpha, Ubeta, Ialpha, Ibeta) SMO_Observer_Updata(&SMO_OB, Ualpha, Ubeta, Ialpha, Ibeta)
#define OBSERVE_GET_THETA()     (SMO_OB.tPLL.theta)
#define OBSERVE_GET_WE()        (SMO_OB.tPLL.we)

#endif

#ifdef MOTOR_ENCODER_OBSERVER

struct Encoder_Parameter
{
    float discrete_time;
    float theta; /*电角度*/
    float we;    /*电角速度*/
    float rpm;   /*机械转速*/
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

void Encode_ABZ_Init(void);
void Encode_ABZ_Get_Offset(void);
void Encode_ABZ_Updata(void);

#define OBSERVE_Init() Encode_ABZ_Init()
#define OBSERVE_Updata() Encode_ABZ_Updata()

#endif

#ifdef MOTOR_NONFLUX_OBSERVER

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
    Motor_Config_t *pMotor;
    Lookup_Table_t PLL_Kp_Lookup;
    Lookup_Table_t PLL_Ki_Lookup;
    Lookup_Table_t Gama_Lookup;
};

void Nonlinear_FluxObserver_Init(void);
void Nonlinear_FluxObserver_Updata(struct NonFluxObserver_Parameter *NFO, float32_t Ualpha, float32_t Ubeta,
                                   float32_t Ialpha, float32_t Ibeta);

extern struct NonFluxObserver_Parameter NonFlux_OB;
#define OBSERVE_Init() Nonlinear_FluxObserver_Init()
#define OBSERVE_Updata(Ualpha, Ubeta, Ialpha, Ibeta) Nonlinear_FluxObserver_Updata(&NonFlux_OB, Ualpha, Ubeta, Ialpha, Ibeta)
#define OBSERVE_GET_THETA()     (NonFlux_OB.tPLL.theta)
#define OBSERVE_GET_WE()        (NonFlux_OB.tPLL.we)

#endif

#ifdef MOTOR_EFFECTIVE_FLUX_OBSERVER

struct EffFluxObserver_Parameter
{
    float discrete_time;
    float gama;
    struct PLL tPLL;
    float Sin, Cos;
    float Id, Iq;
    float FLux_D, Flux_Q;
    float Flux_alpha;
    float Flux_beta;
    float Flux_hat;
    float x_alpha_hat;
    float x_beta_hat;
    float y_alpha_hat;
    float y_beta_hat;
    float Eta_alpha;
    float Eta_beta;
    Motor_Config_t *pMotor;
    Lookup_Table_t PLL_Kp_Lookup;
    Lookup_Table_t PLL_Ki_Lookup;
    Lookup_Table_t Gama_Lookup;
};

void Effective_FluxObserver_Init(void);
void Effective_FluxObserver_Updata(struct EffFluxObserver_Parameter *EFO, float32_t Ualpha, float32_t Ubeta,
                                   float32_t Ialpha, float32_t Ibeta);

extern struct EffFluxObserver_Parameter EffFlux_OB;
#define OBSERVE_Init() Effective_FluxObserver_Init()
#define OBSERVE_Updata(Ualpha, Ubeta, Ialpha, Ibeta) Effective_FluxObserver_Updata(&EffFlux_OB, Ualpha, Ubeta, Ialpha, Ibeta)
#define OBSERVE_GET_THETA()     (EffFlux_OB.tPLL.theta)
#define OBSERVE_GET_WE()        (EffFlux_OB.tPLL.we)

#endif

#ifdef MOTOR_HFI_OBSERVER

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

void HFSWInjection_Init(void);
void HFSWInjection_Updata(struct HFSWInjection_Parameter *HFSW, float Ialpha, float Ibeta, float U_hfj);
void HFSWInjection_NSF(struct HFSWInjection_Parameter *HFSW, float id);

#endif

void Observer_Param_Lookup_Updata(float Speed);

#endif // __EST_H__
