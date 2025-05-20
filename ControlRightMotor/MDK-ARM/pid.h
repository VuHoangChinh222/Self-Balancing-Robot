//#ifndef PID_H
//#define PID_H
//#define PID_INTEGRAL_LIMIT 200.0  // Gioi han tích phân
//#define PID_DERIVATIVE_LIMIT 50.0  // Gioi han giá tri dao hàm
//typedef struct {
//    float Kp;
//    float Ki;
//    float Kd;
//		float prevMeasurement;
//    float prevError;
//    float integral;
//} PID_t;

//float PID_Compute(PID_t *pid, float setpoint, float measured, float dt);
//#endif

#ifndef PID_H
#define PID_H

// Gioi han tich phan va dao ham dau ra
#define PID_INTEGRAL_LIMIT 200.0f
#define PID_DERIVATIVE_LIMIT 50.0f

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prevError;
    float prevMeasurement;
    // Cho anti-windup
    float outputLimit; // Gioi han dau ra
} PID_t;

// Khoi tao PID
void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float outLim);

// TTinh PID co anti-windup va loc dao ham
float PID_Compute(PID_t *pid, float setpoint, float measured, float dt);

#endif

