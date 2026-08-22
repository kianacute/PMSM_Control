// #include "Motor_Config.h"
// #include "Observer.h"


// void Effective_OBSERVE_FIXED_Init(void)
// {  
    
// }

// void Effective_OBSERVE_FIXED_Updata(struct EffFluxObserver_Parameter *EFO, float32_t Ualpha, float32_t Ubeta,
//                                    float32_t Ialpha, float32_t Ibeta)
// {
//     // arm_park_q31(Ialpha, Ibeta, &EFO->Id, &EFO->Iq, EFO->Sin, EFO->Cos);
//     // EFO->FLux_D = EFO->Id * EFO->pMotor->motor_param->Ld + EFO->pMotor->motor_param->flux_linkage_wb;
//     // EFO->Flux_Q = EFO->Iq * EFO->pMotor->motor_param->Lq;
//     // arm_inv_park_q31(EFO->FLux_D, EFO->Flux_Q, &EFO->Flux_alpha, &EFO->Flux_beta, EFO->Sin, EFO->Cos);
//     // EFO->x_alpha_hat += ((Ualpha + EFO->gama * (EFO->Flux_alpha - EFO->x_alpha_hat)) * EFO->discrete_time);
//     // EFO->x_beta_hat += ((Ubeta + EFO->gama * (EFO->Flux_beta - EFO->x_beta_hat)) * EFO->discrete_time);
//     // EFO->y_alpha_hat = EFO->x_alpha_hat - EFO->pMotor->motor_param->Lq * Ialpha;
//     // EFO->y_beta_hat = EFO->x_beta_hat - EFO->pMotor->motor_param->Lq * Ibeta;
//     // EFO->Eta_alpha = EFO->y_alpha_hat * EFO->pMotor->motor_param->One_per_Flux;
//     // EFO->Eta_beta = EFO->y_beta_hat * EFO->pMotor->motor_param->One_per_Flux;
//     // // PLL_Update(&EFO->tPLL, EFO->Eta_beta, EFO->Eta_alpha, EFO->discrete_time);
//     // EFO->tPLL.we = Hal_PI_q31(&EFO->tPLL.PLL_PI, EFO->Eta_beta * EFO->Cos - EFO->Eta_alpha * EFO->Sin);
//     // EFO->tPLL.theta = (EFO->tPLL.theta + EFO->tPLL.we * EFO->discrete_time);
//     // Limit_2PI(&EFO->tPLL.theta);
//     // EFO->Sin = arm_sin_q31(EFO->tPLL.theta);
//     // EFO->Cos = arm_cos_q31(EFO->tPLL.theta);
// }
