#ifndef MPU6050_H_
#define MPU6050_H_

#include <stdint.h>
#include "stm32f1xx_hal.h"

// MPU6050 structure
typedef struct
{
  // Accelerometer
  int16_t Accel_X_RAW;
  int16_t Accel_Y_RAW;
  int16_t Accel_Z_RAW;
  float Ax;
  float Ay;
  float Az;

  // Gyroscope
  int16_t Gyro_X_RAW;
  int16_t Gyro_Y_RAW;
  int16_t Gyro_Z_RAW;
  float Gx;
  float Gy;
  float Gz;

  // Temperature
  float Temperature;

  // Kalman filtered angles
  float KalmanAngleX;
  float KalmanAngleY;
  float KalmanAngleZ;

  // Complementary filtered angles
  float CompAngleX;
  float CompAngleY;
  float CompAngleZ;

  // Magnetometer
  int16_t Mag_X_RAW;
  int16_t Mag_Y_RAW;
  int16_t Mag_Z_RAW;
  float Mx;
  float My;
  float Mz;
  float Heading;

  // Barometer
  float Pressure; // Pa
  float Altitude; // meters
} MPU6050_t;

// Kalman filter structure
typedef struct
{
  float Q_angle;   // Process noise variance for the accelerometer
  float Q_bias;    // Process noise variance for the gyro bias
  float R_measure; // Measurement noise variance
  float angle;     // The angle calculated by the Kalman filter
  float bias;      // The gyro bias calculated by the Kalman filter
  float rate;      // Unbiased rate calculated from the rate and the calculated bias
  float P[2][2];   // Error covariance matrix
} Kalman_t;

// Function prototypes
uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx);
void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);
void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);
void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);
void MPU6050_Read_Mag(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);
void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);
float Kalman_getAngle(Kalman_t *Kalman, float newAngle, float newRate, float dt);
#endif
