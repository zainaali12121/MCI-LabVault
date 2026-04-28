#include "main.h"
#include <math.h>
// Timer used for PWM generation (motor speed control)
extern TIM_HandleTypeDef htim3;

/**
 * @brief  Set motor direction and speed based on control signal
 * @param  control PID output (can be positive or negative)
 * 
 * This function:
 * 1. Determines motor direction based on sign of control
 * 2. Converts control magnitude to PWM duty cycle
 * 3. Applies saturation and minimum threshold
 * 4. Smooths PWM using a simple low-pass filter
 * 5. Outputs PWM to motor driver via TIM3
 */
void Motor_Set(float control)
{   // ---------------- Direction Control ----------------
    // Positive: forward, Negative: reverse
    if(control > 0)
    {
        // Motor direction: forward
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, 0);

        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, 1);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, 0);
    }
    else
    {
        // Motor direction: reverse
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, 1);

        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, 0);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, 1);
    }

    // Magnitude Conversion
    // Use absolute value since direction is already handled
    int pwm = (int)fabs(control);

    //  Saturation Limit
    // Prevent excessive speed / protect hardware
    if(pwm > 600) pwm = 600;

      //Minimum Threshold
    // Ensure motor overcomes static friction (dead zone)
    if(pwm > 0 && pwm < 120)
        pwm = 120;

    //PWM Smoothing(Low-pass filter)
    // Reduces sudden jerks in motor response
    static float pwm_filtered = 0;
    pwm_filtered = 0.8f * pwm_filtered + 0.2f * pwm;
    pwm = pwm_filtered;

     //PWM Output
    // Channel 1 and 2 control two motor inputs
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm*2);  //scaled by 2 manually due to mismatch of the two channels in the motor driver
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm);
}
