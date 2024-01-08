#include "led.h"

void led_on()
{
    HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
}


void led_off()
{
    HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
}


void led_blink()
{
    led_on();
    HAL_Delay(500);
    led_off();
    HAL_Delay(500);
}


void led_blink_number(uint32_t number)
{
    uint32_t i = 0;
    for(i=0;i<number;i++)
    {
        led_blink();
    }
}