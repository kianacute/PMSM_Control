#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__ 

#include "Motor_FOC_Config.h"
#include "Hal_Math.h"

enum Motor_State{
    CURRENT_IDLE = 0,
    CURRENT_READY,
    CURRENT_OFFSET_CHECK,
    CURRENT_RS_IDENTIFY,      
    CURRENT_RUN,
    CURRENT_FAULT,
    CURRENT_WAIT,
};

typedef struct Current_Loop
{
    enum Motor_State Status;
    uint64_t Loop_count;                    // Current loop execution count
    void *pCurrent_Loop;                    // Pointer to the current loop structure (either float or fixed)
    uint8_t PWM_OPEN_Flag, PWM_OPEN_Flag_z;                  //PWM开关标志位
}Current_Loop_t;

enum Speed_LoopState_t
{
    SPEED_IDLE = 0,
    SPEED_ALIGN,
    SPEED_OPEN,
    SPEED_SWITCH,
    SPEED_LOW,
    SPEED_MIDDLE,
    SPEED_HIGH,
};

typedef struct Speed_Loop
{
    enum Speed_LoopState_t Status; // 速度控制状态
    uint64_t Loop_count; // Speed loop execution count
    void *pSpeed_Loop;
} Speed_Loop_t;

enum SYSTEM_State_t
{
    SYSTEM_LV_STANDY = 0,
    SYSTEM_HV_STANDY,
    SYSTEM_RUN,
    SYSTEM_FAULT,
    SYSTEM_WAIT,
};

typedef struct SYSTEM_Loop
{
    enum SYSTEM_State_t Status;  
    uint64_t Loop_count; // System loop execution count
    void *pSystem_Loop;
} System_Loop_t;


#define MOTOR_LOOKUP_TABLES_FLOAT

typedef struct Motor_Parameter_Float
{
    float Pn;              // 电机极对数
    float Rs;              // 定子电阻
    float Ld, Lq, Ls;      // 定子电感
    float Flux_Vkrpm;      // 反电动势系数，单位为V/krpm
    float flux_linkage_wb; // 磁链, 单位为Wb
    float Speed_Max_Rpm;   // 最大转速
    float Current_Max_A;   // 最大电流
    float Bus_Voltage_Max; // 电压限制，单位为V
    float Power_Max_W;     // 功率限制参数
    float Flux_Flux;
    float Ld_Lq;
    float One_per_Flux;
} Motor_Parameter_Float_t;


typedef struct Motor_Parameter_Fixed
{
    q31_t Pn;              // 电机极对数
    q31_t Rs;              // 定子电阻
    q31_t Ld, Lq, Ls;      // 定子电感
    q31_t Flux_Vkrpm;      // 反电动势系数，单位为V/krpm
    q31_t flux_linkage_wb; // 磁链, 单位为Wb
    q31_t Speed_Max_Rpm;   // 最大转速
    q31_t Current_Max_A;   // 最大电流
    q31_t Bus_Voltage_Max; // 电压限制，单位为V
    q31_t Power_Max_W;     // 功率限制参数
    q31_t Flux_Flux;
    q31_t Ld_Lq;
    q31_t One_per_Flux;
} Motor_Parameter_Fixed_t;

struct Motor_Config_Float
{
    Motor_Parameter_Float_t *Motor_Param; // 电机参数

    // IF启动参数
    Lookup_Table_1D_f32_t IF_Start_Speed_Lookup;           // 启动速度查表
    Lookup_Table_1D_f32_t IF_Start_Iq_Lookup;              // 启动Iq查表

    // 电流环查表参数
    Lookup_Table_1D_f32_t ID_PI_Kp_Lookup;  
    Lookup_Table_1D_f32_t IQ_PI_Kp_Lookup; 
    Lookup_Table_1D_f32_t ID_PI_Ki_Lookup; 
    Lookup_Table_1D_f32_t IQ_PI_Ki_Lookup; 

    // 速度环查表参数
    Lookup_Table_1D_f32_t Speed_PI_Kp_Lookup; // 速度PI比例增益查表
    Lookup_Table_1D_f32_t Speed_PI_Ki_Lookup; // 速度PI积分增益查表    

    //磁链观测器查表参数
    Lookup_Table_1D_f32_t NonFlux_PLL_Kp_Lookup;
    Lookup_Table_1D_f32_t NonFlux_PLL_Ki_Lookup;
    Lookup_Table_1D_f32_t NonFlux_Gama_Lookup;
    Lookup_Table_1D_f32_t EfFlux_Gama_Lookup;
    Lookup_Table_2D_f32_t EfFlux_Angle_Comp; 
    
    //SMO观测器查表参数
    Lookup_Table_1D_f32_t SMO_PLL_Kp_Lookup;
    Lookup_Table_1D_f32_t SMO_PLL_Ki_Lookup;
    Lookup_Table_1D_f32_t SMO_Gain_Lookup;
};


struct Motor_Config_Fixed
{
    Motor_Parameter_Fixed_t *Motor_Param; // 电机参数

    // IF启动参数
    Lookup_Table_1D_q31_t IF_Start_Speed_Lookup;           // 启动速度查表
    Lookup_Table_1D_q31_t IF_Start_Iq_Lookup;              // 启动Iq查表

    // 电流环查表参数
    Lookup_Table_1D_q31_t ID_PI_Kp_Lookup;  
    Lookup_Table_1D_q31_t IQ_PI_Kp_Lookup; 
    Lookup_Table_1D_q31_t ID_PI_Ki_Lookup; 
    Lookup_Table_1D_q31_t IQ_PI_Ki_Lookup; 

    // 速度环查表参数
    Lookup_Table_1D_q31_t Speed_PI_Kp_Lookup; // 速度PI比例增益查表
    Lookup_Table_1D_q31_t Speed_PI_Ki_Lookup; // 速度PI积分增益查表    

    //磁链观测器查表参数
    Lookup_Table_1D_q31_t NonFlux_PLL_Kp_Lookup;
    Lookup_Table_1D_q31_t NonFlux_PLL_Ki_Lookup;
    Lookup_Table_1D_q31_t NonFlux_Gama_Lookup;
    Lookup_Table_1D_q31_t EfFlux_Gama_Lookup;
    Lookup_Table_2D_q31_t EfFlux_Angle_Comp; 
    
    //SMO观测器查表参数
    Lookup_Table_1D_q31_t SMO_PLL_Kp_Lookup;
    Lookup_Table_1D_q31_t SMO_PLL_Ki_Lookup;
    Lookup_Table_1D_q31_t SMO_Gain_Lookup;
};

struct Current_Loop_Input_Float
{
    float Ia_fb_raw, Ib_fb_raw, Ic_fb_raw;
    float Udc_ADISR;
};

struct Current_Loop_Output_Float 
{
    float PWM_HZ_Coeff;
    float PWM_duty_a, PWM_duty_b, PWM_duty_c, PWM_duty_d;
};  

struct Current_Loop_Input_Fixed
{
    q15_t Ia_fb_raw, Ib_fb_raw, Ic_fb_raw;
    q15_t Udc_ADISR;
};

struct Current_Loop_Output_Fixed
{
    q15_t PWM_HZ_Coeff;
    q15_t PWM_duty_a, PWM_duty_b, PWM_duty_c, PWM_duty_d;
};


#ifdef MOTOR_CONTROL_FLOAT

typedef struct Current_Loop_Input_Float Motor_Control_Input_t;
typedef struct Current_Loop_Output_Float Motor_Control_Output_t;
typedef struct Motor_Config_Float Motor_Config_t;

#elif defined MOTOR_CONTROL_FIXED

typedef struct Current_Loop_Input_Fixed Motor_Control_Input_t;
typedef struct Current_Loop_Output_Fixed Motor_Control_Output_t;
typedef struct Motor_Config_Fixed Motor_Config_t;

#endif 

typedef struct Motor_Control    
{
    System_Loop_t System_Loop;
    Current_Loop_t Current_Loop;
    Speed_Loop_t Speed_Loop;
    Motor_Control_Input_t Input;
    Motor_Control_Output_t Output;
    Motor_Config_t* Motor_Config;
    void *pObserver; // Pointer to the observer structure (either float or fixed)
} Motor_Control_t;

#ifdef MOTOR_CONTROL_FLOAT

    /*电机状态机*/
    void Current_Init_Float(Motor_Control_t *pControl);
    void Current_IDLE_TASK_Float(Motor_Control_t *pControl);
    void Current_READY_TASK_Float(Motor_Control_t *pControl);
    void Current_OFFSET_CHECK_TASK_Float(Motor_Control_t *pControl);
    void Current_RUN_TASK_Float(Motor_Control_t *pControl);
    void Current_FAULT_TASK_Float(Motor_Control_t *pControl);
    void Current_WAIT_TASK_Float(Motor_Control_t *pControl);

    #define         Current_Init(pMotor_control)                    Current_Init_Float(pMotor_control)
    #define         Current_IDLE_TASK(pMotor_control)               Current_IDLE_TASK_Float(pMotor_control)                  
    #define         Current_READY_TASK(pMotor_control)              Current_READY_TASK_Float(pMotor_control)                  
    #define         Current_OFFSET_CHECK_TASK(pMotor_control)       Current_OFFSET_CHECK_TASK_Float(pMotor_control)                 
    #define         Current_RUN_TASK(pMotor_control)                Current_RUN_TASK_Float(pMotor_control)                  
    #define         Current_FAULT_TASK(pMotor_control)              Current_FAULT_TASK_Float(pMotor_control)                  
    #define         Current_WAIT_TASK(pMotor_control)               Current_WAIT_TASK_Float(pMotor_control)   

    /*速度状态机*/

    void SPEED_Init_Float(Motor_Control_t *pControl);
    void SPEED_Idle_Task_Float(Motor_Control_t *pControl);
    void SPEED_Align_Task_Float(Motor_Control_t *pControl);
    void SPEED_Open_Task_Float(Motor_Control_t *pControl);
    void SPEED_Switch_Task_Float(Motor_Control_t *pControl);
    void SPEED_Low_Task_Float(Motor_Control_t *pControl);
    void SPEED_Middle_Task_Float(Motor_Control_t *pControl);
    void SPEED_High_Task_Float(Motor_Control_t *pControl);

    #define         SPEED_Init(pMotor_control)               SPEED_Init_Float(pMotor_control)
    #define         SPEED_Idle_Task(pMotor_control)          SPEED_Idle_Task_Float(pMotor_control)
    #define         SPEED_Align_Task(pMotor_control)         SPEED_Align_Task_Float(pMotor_control)
    #define         SPEED_Open_Task(pMotor_control)          SPEED_Open_Task_Float(pMotor_control)
    #define         SPEED_Switch_Task(pMotor_control)        SPEED_Switch_Task_Float(pMotor_control)
    #define         SPEED_Low_Task(pMotor_control)           SPEED_Low_Task_Float(pMotor_control)
    #define         SPEED_Middle_Task(pMotor_control)        SPEED_Middle_Task_Float(pMotor_control)
    #define         SPEED_High_Task(pMotor_control)          SPEED_High_Task_Float(pMotor_control)
    #define         SPEED_Run_Task(pMotor_control)           SPEED_Run_Task_Float(pMotor_control)

    /*系统状态机*/
    
    void SYSTEM_Init_Float(Motor_Control_t *pControl);
    void SYSTEM_LV_Standy_Float(Motor_Control_t *pControl);
    void SYSTEM_HV_Standy_Float(Motor_Control_t *pControl);
    void SYSTEM_Run_Float(Motor_Control_t *pControl);
    void SYSTEM_Fault_Float(Motor_Control_t *pControl);
    void SYSTEM_Wait_Float(Motor_Control_t *pControl);

    #define         SYSTEM_Init(pMotor_control)              SYSTEM_Init_Float(pMotor_control)
    #define         SYSTEM_LV_Standy(pMotor_control)         SYSTEM_LV_Standy_Float(pMotor_control)
    #define         SYSTEM_HV_Standy(pMotor_control)         SYSTEM_HV_Standy_Float(pMotor_control)
    #define         SYSTEM_Run(pMotor_control)               SYSTEM_Run_Float(pMotor_control)
    #define         SYSTEM_Fault(pMotor_control)             SYSTEM_Fault_Float(pMotor_control) 
    #define         SYSTEM_Wait(pMotor_control)              SYSTEM_Wait_Float(pMotor_control)


    /*电机参数和控制参数*/

    void Motor_Config_Init(Motor_Control_t *pMotor_Control);
    void Paramater_update_Float(Motor_Control_t *pControl);

#elif defined MOTOR_CONTROL_FIXED

    /*电机状态机*/
    void Current_Init_Fixed(Motor_Control_t *pControl);
    void Current_IDLE_TASK_Fixed(Motor_Control_t *pControl);
    void Current_READY_TASK_Fixed(Motor_Control_t *pControl);
    void Current_OFFSET_CHECK_TASK_Fixed(Motor_Control_t *pControl);
    void Current_RUN_TASK_Fixed(Motor_Control_t *pControl);
    void Current_FAULT_TASK_Fixed(Motor_Control_t *pControl);
    void Current_WAIT_TASK_Fixed(Motor_Control_t *pControl);

    #define         Current_Init(pMotor_control)                    Current_Init_Fixed(pMotor_control)
    #define         Current_IDLE_TASK(pMotor_control)               Current_IDLE_TASK_Fixed(pMotor_control)                  
    #define         Current_READY_TASK(pMotor_control)              Current_READY_TASK_Fixed(pMotor_control)                  
    #define         Current_OFFSET_CHECK_TASK(pMotor_control)       Current_OFFSET_CHECK_TASK_Fixed(pMotor_control)                 
    #define         Current_RUN_TASK(pMotor_control)                Current_RUN_TASK_Fixed(pMotor_control)                  
    #define         Current_FAULT_TASK(pMotor_control)              Current_FAULT_TASK_Fixed(pMotor_control)                  
    #define         Current_WAIT_TASK(pMotor_control)               Current_WAIT_TASK_Fixed(pMotor_control)   

    /*速度状态机*/

    void SPEED_Init_Fixed(Motor_Control_t *pControl);
    void SPEED_Idle_Task_Fixed(Motor_Control_t *pControl);
    void SPEED_Align_Task_Fixed(Motor_Control_t *pControl);
    void SPEED_Open_Task_Fixed(Motor_Control_t *pControl);
    void SPEED_Switch_Task_Fixed(Motor_Control_t *pControl);
    void SPEED_Low_Task_Fixed(Motor_Control_t *pControl);
    void SPEED_Middle_Task_Fixed(Motor_Control_t *pControl);
    void SPEED_High_Task_Fixed(Motor_Control_t *pControl);

    #define         SPEED_Init(pMotor_control)               SPEED_Init_Fixed(pMotor_control)
    #define         SPEED_Idle_Task(pMotor_control)          SPEED_Idle_Task_Fixed(pMotor_control)
    #define         SPEED_Align_Task(pMotor_control)         SPEED_Align_Task_Fixed(pMotor_control)
    #define         SPEED_Open_Task(pMotor_control)          SPEED_Open_Task_Fixed(pMotor_control)
    #define         SPEED_Switch_Task(pMotor_control)        SPEED_Switch_Task_Fixed(pMotor_control)
    #define         SPEED_Low_Task(pMotor_control)           SPEED_Low_Task_Fixed(pMotor_control)
    #define         SPEED_Middle_Task(pMotor_control)        SPEED_Middle_Task_Fixed(pMotor_control)
    #define         SPEED_High_Task(pMotor_control)          SPEED_High_Task_Fixed(pMotor_control)
    #define         SPEED_Run_Task(pMotor_control)           SPEED_Run_Task_Fixed(pMotor_control)

    /*系统状态机*/
    
    void SYSTEM_Init_Fixed(Motor_Control_t *pControl);
    void SYSTEM_LV_Standy_Fixed(Motor_Control_t *pControl);
    void SYSTEM_HV_Standy_Fixed(Motor_Control_t *pControl);
    void SYSTEM_Run_Fixed(Motor_Control_t *pControl);
    void SYSTEM_Fault_Fixed(Motor_Control_t *pControl);
    void SYSTEM_Wait_Fixed(Motor_Control_t *pControl);

    #define         SYSTEM_Init(pMotor_control)              SYSTEM_Init_Fixed(pMotor_control)
    #define         SYSTEM_LV_Standy(pMotor_control)         SYSTEM_LV_Standy_Fixed(pMotor_control)
    #define         SYSTEM_HV_Standy(pMotor_control)         SYSTEM_HV_Standy_Fixed(pMotor_control)
    #define         SYSTEM_Run(pMotor_control)               SYSTEM_Run_Fixed(pMotor_control)
    #define         SYSTEM_Fault(pMotor_control)             SYSTEM_Fault_Fixed(pMotor_control) 
    #define         SYSTEM_Wait(pMotor_control)              SYSTEM_Wait_Fixed(pMotor_control)


    /*电机参数和控制参数*/

    void Motor_Config_Init(Motor_Control_t *pMotor_Control);
    void Paramater_update_Fixed(Motor_Control_t *pControl);


#endif

void Motor_Control_Init(Motor_Control_t *pControl);
void Current_Loop_Task(Motor_Control_t *pControl);
void Speed_Loop_Task(Motor_Control_t *pControl);
void SYSTEM_LOOP_Task(Motor_Control_t *pControl);

#endif

