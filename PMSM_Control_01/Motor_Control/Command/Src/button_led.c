#include "button_led.h"
#include "main.h"
#include "usart.h"
#include "tim.h"
#include "stdio.h"
#include "Observer.h"

static uint8_t tmp_buffer[50] = {0};
uint8_t Motor_state;

uint8_t MOTOR_Run_flag;
float Speed_Command;

#define BUTTON_LED_GPIO_PORT GPIOC

#define BUTTON_PIN1 GPIO_PIN_10
#define BUTTON_PIN2 GPIO_PIN_11
#define BUTTON_PIN3 GPIO_PIN_13

#define LED_PIN1 GPIO_PIN_4
#define LED_PIN2 GPIO_PIN_6

void Button_LED_Init(void)
{
    // Initialize button and LED GPIO pins
}

void Button_LED_Task(void)
{
    // Check button state and toggle LED
}
void Button_LED_DeInit(void)
{
    // Deinitialize button and LED GPIO pins
}

extern struct Encoder_Parameter Encode_ABZ;

void MOTOR_Run_flag_UPDOWN(void)
{
    Motor_state = ~Motor_state;
    if (Motor_state == 0)
    {
        MOTOR_Run_flag = 0;
    }
    else
    {
        MOTOR_Run_flag = 1;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == BUTTON_PIN1)
    {
        // Handle button press for BUTTON_PIN1
        MOTOR_Run_flag_UPDOWN();
    }
    else if (GPIO_Pin == BUTTON_PIN2)
    {
        Speed_Command += 100.0f;
    }
    else if (GPIO_Pin == BUTTON_PIN3)
    {
        Speed_Command -= 50.0f;
    }
    else if (GPIO_Pin == GPIO_PIN_8)
    {
        Encode_ABZ.z_flag = 1 - Encode_ABZ.z_flag;
    }
}