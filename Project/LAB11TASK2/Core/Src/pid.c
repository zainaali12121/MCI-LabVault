#include "pid.h"
#include <math.h>

/**
 * @brief PID controller gains
 * 
 * Kp: Proportional gain (reacts to current error)
 * Ki:Integral gain (accumulates past error)
 * Kd: Derivative gain (predicts future error)
 */
float Ki = 0.0f;
float Kp = 57.0f;
float Kd = 1.0f;

//system setpoint (target angle for balance)
float setpoint = 1.0f;
float error = 0;     // Current error
float prev_error = 0;   // Previous error (for derivative)
float integral = 0;     // Accumulated error over time
float derivative = 0;   // Rate of change of error
float control = 0;       // Final control output


/**
 * @brief Compute PID control signal
 * @param angle Current measured angle (degrees)
 * @param dt    Time step (seconds)
 * @retval Control signal to be sent to motors
 * 
 * This function implements a standard PID controller:
 * - Proportional: immediate correction
 * - Integral: removes steady-state error
 * - Derivative: dampens oscillations
 */
float PID_Compute(float angle, float dt)
{
    //Error Calculation
    // Difference between current angle and desired setpoint
    error = angle - setpoint;
    // Deadband Handling
    // If error is very small, stop control to avoid jitter
    if(fabs(error) < 0.02f)
    {
        control = 0;
        integral = 0;
        prev_error = 0;
        return 0;
    }
    //Integral Term
    // Accumulate error over time
    integral += error * dt;
    // Anti-windup: limit integral to prevent excessive buildup
    if(integral > 50) integral = 50;
    if(integral < -50) integral = -50;

    //Derivative Term
    // Rate of change of error (predictive action)
    derivative = (error - prev_error) / dt;

     //PID Output
    // Combine all three components
    control = Kp*error + Ki*integral + Kd*derivative;

        // Output Saturation
    // Limit control signal to safe motor range
    if(control > 1000) control = 1000;
    if(control < -1000) control = -1000;

    // Store current error for next iteration
    prev_error = error;

    return control;
}
