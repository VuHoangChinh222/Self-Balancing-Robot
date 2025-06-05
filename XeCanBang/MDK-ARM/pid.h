#ifndef PID_H
#define PID_H
typedef struct
{
  float Kp;
  float Ki;
  float Kd;
  float prevMeasurement;
  float prevError;
  float integral;
} PID_t;

#endif
