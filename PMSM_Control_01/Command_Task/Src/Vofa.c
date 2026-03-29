#include "Vofa.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include "speed_ctrl.h"
#include "Current_Task.h"
#include "Observer.h"


typedef struct vofa_buffer_
{
    float data[8];
    uint8_t tail[4];
} vofa_buffer_t;

vofa_buffer_t vofa_buffer =
    {
        .tail[0] = 0x00,
        .tail[1] = 0x00,
        .tail[2] = 0x80,
        .tail[3] = 0x7F,
};

void Vofa_Task_Init(void)
{

}

extern Speed_Ctrl_t Speed_Ctrl;
extern struct Encoder_Parameter Encode_ABZ;
extern Current_Task_t Current_Task;
extern struct NonFluxObserver_Parameter NonFlux_OB;
extern struct SMO_Parameter SMO_OB;

void Vofa_Task_Run(void)
{
    extern vofa_buffer_t vofa_buffer;
    extern float Udc_1ms;
    vofa_buffer.data[0] = Speed_Ctrl.Speed_Ref;
    vofa_buffer.data[1] = Speed_Ctrl.Speed_Fb;
    vofa_buffer.data[2] = (float32_t)(NonFlux_OB.tPLL.theta);
    vofa_buffer.data[3] = (float32_t)(Encode_ABZ.theta);
    vofa_buffer.data[4] = (float32_t)(Current_Task.Id_fb);
    vofa_buffer.data[5] = (float32_t)(Current_Task.Iq_fb);
    vofa_buffer.data[6] = (float32_t)(Speed_Ctrl.target_id);
    vofa_buffer.data[7] = (float32_t)(Speed_Ctrl.target_iq);
    HAL_UART_Transmit_DMA(&huart3, (uint8_t *)&vofa_buffer, sizeof(vofa_buffer));
}