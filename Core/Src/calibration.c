#include "main.h"

/// @brief Device calibration function
/*
Consequtively changes PWM output to the following values:

-20degC = 10% PWM
15degC = 55% PWM
50degC = 100% PWM

The idea is to hold the value for 5-10seconds to allow adjustment of the OpAmp output
*/
void run_calibration()
{
    //TIM2 clock is 25MHz
    //PWM freq is 5kHz
    //MX_TIM2_Init();
    //LL_TIM_SetPrescaler(TIM2, 5000);
    //LL_TIM_SetAutoReload(TIM2,65536);
    //LL_TIM_EnableCounter(TIM2);
    
    //TIM2->CCR3 |= 0x1FFF;
    //TIM2->CCER |= TIM_CCER_CC3E;
    //TIM2->PSC = 5000;
    //TIM2->ARR = 0xFFFF;
    //TIM2->CR1 |= TIM_CR1_CEN;

    //LL_TIM_SetPrescaler(TIM2, OUTPUT_PWM_FREQ-1);
    LL_TIM_CC_EnableChannel(TIM2,LL_TIM_CHANNEL_CH3);
    LL_TIM_SetAutoReload(TIM2,OUTPUT_PWM_FREQ-1);
    //LL_TIM_CC_SetUpdate(TIM2,LL_T)
    LL_TIM_OC_SetCompareCH3(TIM2,(uint32_t)(OUTPUT_PWM_FREQ/2));
    LL_TIM_EnableCounter(TIM2);

    uint32_t pwm_outputs[4] = {1, (uint32_t)(OUTPUT_PWM_FREQ*0.2),(uint32_t)(OUTPUT_PWM_FREQ*0.55),(uint32_t)(OUTPUT_PWM_FREQ)};
    while(1)
    {
        for(int i=0;i<4;i++)
        {
            //blink
            LL_GPIO_ResetOutputPin(LED_BLUE_GPIO_Port,LED_BLUE_Pin);
            LL_mDelay(100);
            LL_GPIO_SetOutputPin(LED_BLUE_GPIO_Port,LED_BLUE_Pin);
            LL_mDelay(100);
            LL_TIM_OC_SetCompareCH3(TIM2,pwm_outputs[i]);
            LL_mDelay(5000);
            //TIM2->ARR = pwm_outputs[i];
            //LL_TIM_SetAutoReload(TIM2,pwm_outputs[i]);
        }
        
    }
}
