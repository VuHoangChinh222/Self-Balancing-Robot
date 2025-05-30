#include "math.h"
#include "pid.h"

float PID_Compute(PID_t *pid, float setpoint, float measured, float dt)
{
    float error = setpoint - measured;
    pid->integral += error * dt;

    // Gioi han tich phan
    if (pid->integral > PID_INTEGRAL_LIMIT)
        pid->integral = PID_INTEGRAL_LIMIT;
    if (pid->integral < -PID_INTEGRAL_LIMIT)
        pid->integral = -PID_INTEGRAL_LIMIT;

    // Tinh dao ham dua tren measured thay vi error
    float derivative = (error - pid->prevMeasurement) / dt;
    pid->prevMeasurement = error; // Luu gi? tri do

    // Gioi han dao ham
    derivative = fminf(fmaxf(derivative, -PID_DERIVATIVE_LIMIT), PID_DERIVATIVE_LIMIT);

    pid->prevError = error;
    return pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
}

void PID_Reset(PID_t *pid)
{
  pid->integral = 0;
  pid->prevError = 0;
  pid->prevMeasurement = 0;
}
