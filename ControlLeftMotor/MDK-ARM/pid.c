#include "pid.h"
#include "math.h"

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float outLim)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0.0f;
    pid->prevMeasurement = 0.0f;
    pid->prevError = 0.0f;
    pid->outputLimit = outLim;
}

float PID_Compute(PID_t *pid, float setpoint, float measured, float dt)
{
    float error = setpoint - measured;
    // Integrate with anti‐windup
    pid->integral += error * dt;
    if (pid->integral > PID_INTEGRAL_LIMIT)
        pid->integral = PID_INTEGRAL_LIMIT;
    if (pid->integral < -PID_INTEGRAL_LIMIT)
        pid->integral = -PID_INTEGRAL_LIMIT;

    // Derivative on measurement (to reduce noise)
    float derivative = -(measured - pid->prevMeasurement) / dt;
    pid->prevMeasurement = measured;
    if (derivative > PID_DERIVATIVE_LIMIT)
        derivative = PID_DERIVATIVE_LIMIT;
    if (derivative < -PID_DERIVATIVE_LIMIT)
        derivative = -PID_DERIVATIVE_LIMIT;

    // PID formula
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

    // Saturate final output
    if (output > pid->outputLimit)
        output = pid->outputLimit;
    if (output < -pid->outputLimit)
        output = -pid->outputLimit;

    pid->prevError = error;
    return output;
}
