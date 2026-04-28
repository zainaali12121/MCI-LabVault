#ifndef PID_H
#define PID_H

extern float Ki;
extern float Kp;
extern float Kd;

extern float setpoint;
extern float error;
extern float prev_error;
extern float integral;
extern float derivative;
extern float control;

float PID_Compute(float angle, float dt);

#endif