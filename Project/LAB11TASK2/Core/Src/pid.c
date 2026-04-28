#include "pid.h"
#include <math.h>

float Ki = 0.0f;
float Kp = 57.0f;
float Kd = 1.0f;

float setpoint = 1.0f;
float error = 0;
float prev_error = 0;
float integral = 0;
float derivative = 0;
float control = 0;

float PID_Compute(float angle, float dt)
{
    error = angle - setpoint;

    if(fabs(error) < 0.02f)
    {
        control = 0;
        integral = 0;
        prev_error = 0;
        return 0;
    }

    integral += error * dt;

    if(integral > 50) integral = 50;
    if(integral < -50) integral = -50;

    derivative = (error - prev_error) / dt;

    control = Kp*error + Ki*integral + Kd*derivative;

    if(control > 1000) control = 1000;
    if(control < -1000) control = -1000;

    prev_error = error;

    return control;
}