#ifndef PID_H
#define PID_H
#define PID_INTEGRAL_LIMIT 500.0  // Gioi han tích phân
#define PID_DERIVATIVE_LIMIT 100.0  // Gioi han giá tri dao hàm
typedef struct {
    float Kp;
    float Ki;
    float Kd;
		float prevMeasurement;
    float prevError;
    float integral;
} PID_t;

float PID_Compute(PID_t *pid, float setpoint, float measured, float dt);
#endif
