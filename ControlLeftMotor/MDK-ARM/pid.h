#ifndef PID_H
#define PID_H

#define PID_INTEGRAL_LIMIT 200.0f
#define PID_DERIVATIVE_LIMIT 50.0f

typedef struct
{
  float Kp;
  float Ki;
  float Kd;
  float integral;
  float prevMeasurement;
  float prevError;
  float outputLimit;
} PID_t;

/**
 * @brief  Initialize a PID controller.
 */
void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float outLim);

/**
 * @brief  Compute PID output with anti‐windup and derivative on measurement.
 * @param  setpoint: desired value
 * @param  measured: current value
 * @param  dt: timestep in seconds
 * @return pid output, already saturated to ±outputLimit
 */
float PID_Compute(PID_t *pid, float setpoint, float measured, float dt);

#endif
