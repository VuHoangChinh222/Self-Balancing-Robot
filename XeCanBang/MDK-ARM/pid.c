#include "pid.h"
#include "math.h"
#define PID_INTEGRAL_LIMIT 500.0  // Gioi han tích phân
#define PID_DERIVATIVE_LIMIT 100.0  // Gioi han giá tri dao hàm

float PID_Compute(PID_t *pid, float setpoint, float measured, float dt) {
    float error = setpoint - measured;
    pid->integral += error * dt;
    
    // Gioi han tích phân
    if (pid->integral > PID_INTEGRAL_LIMIT) pid->integral = PID_INTEGRAL_LIMIT;
    if (pid->integral < -PID_INTEGRAL_LIMIT) pid->integral = -PID_INTEGRAL_LIMIT;
    
    // Tính dao hàm dua trên measurement thay vì error
    float derivative = -(measured - pid->prevMeasurement) / dt; 
    pid->prevMeasurement = measured; // Luu giá tri do
    
    // Gioi han dao hàm
    derivative = fminf(fmaxf(derivative, -PID_DERIVATIVE_LIMIT), PID_DERIVATIVE_LIMIT);
    
    pid->prevError = error;
    return pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
}
