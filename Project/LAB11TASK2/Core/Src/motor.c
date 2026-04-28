#include "main.h"
#include <math.h>

extern TIM_HandleTypeDef htim3;

void Motor_Set(float control)
{
    if(control > 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, 0);

        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, 1);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, 0);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, 1);

        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, 0);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, 1);
    }

    int pwm = (int)fabs(control);

    if(pwm > 600) pwm = 600;

    if(pwm > 0 && pwm < 120)
        pwm = 120;

    static float pwm_filtered = 0;
    pwm_filtered = 0.8f * pwm_filtered + 0.2f * pwm;
    pwm = pwm_filtered;

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm*2);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm);
}