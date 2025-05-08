#ifndef PID_H
#define PID_H

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
