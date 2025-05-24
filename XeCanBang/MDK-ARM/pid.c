#include "pid.h"
#include "math.h"
#define PID_INTEGRAL_LIMIT 500.0  // Gioi han t�ch ph�n
#define PID_DERIVATIVE_LIMIT 100.0  // Gioi han gi� tri dao h�m

float PID_Compute(PID_t *pid, float setpoint, float measured, float dt) {
    float error = setpoint - measured;
    pid->integral += error * dt;
    
    // Gioi han t�ch ph�n
    if (pid->integral > PID_INTEGRAL_LIMIT) pid->integral = PID_INTEGRAL_LIMIT;
    if (pid->integral < -PID_INTEGRAL_LIMIT) pid->integral = -PID_INTEGRAL_LIMIT;
    
    // T�nh dao h�m dua tr�n measurement thay v� error
    float derivative = -(measured - pid->prevMeasurement) / dt; 
    pid->prevMeasurement = measured; // Luu gi� tri do
    
    // Gioi han dao h�m
    derivative = fminf(fmaxf(derivative, -PID_DERIVATIVE_LIMIT), PID_DERIVATIVE_LIMIT);
    
    pid->prevError = error;
    return pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
}
