#include <math.h>
#include "mpu6050.h"

#define RAD_TO_DEG 57.295779513082320876798154814105
#define ALPHA 0.96f // Complementary filter coefficient

// MPU6050 Registers
#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43
#define MPU6050_ADDR 0xD0

// HMC5883L Registers
#define HMC5883L_ADDR 0x1E
#define HMC5883L_CONFIG_A 0x00
#define HMC5883L_CONFIG_B 0x01
#define HMC5883L_MODE 0x02
#define HMC5883L_OUT_X_H 0x03

const uint16_t i2c_timeout = 100;
static uint32_t timer;

// Kalman filter instances
static Kalman_t KalmanX = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f};

static Kalman_t KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f};

static Kalman_t KalmanZ = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f};

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx)
{
    uint8_t check;
    uint8_t Data;

    // 1. Check device ID
    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, i2c_timeout);
    if (check != 0x68)
        return 1;

    // 2. Wake up MPU6050
    Data = 0x00;
    HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);

    // 3. Set sample rate to 1kHz
    Data = 0x07;
    HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, i2c_timeout);

    // 4. Configure Accelerometer (+/- 2g)
    Data = 0x00;
    HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, i2c_timeout);

    // 5. Configure Gyroscope (+/- 250 deg/s)
    Data = 0x00;
    HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, i2c_timeout);

    // 6. Configure I2C Bypass for HMC5883L access
    Data = 0x02;
    HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x37, 1, &Data, 1, i2c_timeout);

    // 7. Configure HMC5883L
    Data = 0x70; // 8 samples averaged, 15Hz update rate
    HAL_I2C_Mem_Write(I2Cx, HMC5883L_ADDR, HMC5883L_CONFIG_A, 1, &Data, 1, i2c_timeout);

    Data = 0x20; // Gain=1090LSB/Gauss
    HAL_I2C_Mem_Write(I2Cx, HMC5883L_ADDR, HMC5883L_CONFIG_B, 1, &Data, 1, i2c_timeout);

    Data = 0x00; // Continuous measurement mode
    HAL_I2C_Mem_Write(I2Cx, HMC5883L_ADDR, HMC5883L_MODE, 1, &Data, 1, i2c_timeout);

    return 0;
}

void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from ACCEL_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    /*** convert the RAW values into acceleration in 'g'
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 16384.0
         for more details check ACCEL_CONFIG Register              ****/

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / 16384.0;
}

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from GYRO_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    /*** convert the RAW values into dps (?/s)
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 131.0
         for more details check GYRO_CONFIG Register              ****/

    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;
}

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[2];
    int16_t temp;

    // Read 2 BYTES of data starting from TEMP_OUT_H_REG register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, TEMP_OUT_H_REG, 1, Rec_Data, 2, i2c_timeout);

    temp = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
}

// void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
// {
//     uint8_t Rec_Data[14];
//     int16_t temp;

//     // Read Accelerometer and Gyroscope
//     HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, i2c_timeout);

//     // Accelerometer
//     DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
//     DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
//     DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
//     DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0f;
//     DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0f;
//     DataStruct->Az = DataStruct->Accel_Z_RAW / 16384.0f;

//     // Temperature
//     temp = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
//     DataStruct->Temperature = (float)((int16_t)temp / 340.0f + 36.53f);

//     // Gyroscope
//     DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
//     DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
//     DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);
//     DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0f;
//     DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0f;
//     DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0f;

//     // Read Magnetometer
//     uint8_t mag_data[6];
//     HAL_I2C_Mem_Read(I2Cx, HMC5883L_ADDR, HMC5883L_OUT_X_H, 1, mag_data, 6, i2c_timeout);

//     DataStruct->Mag_X_RAW = (int16_t)(mag_data[0] << 8 | mag_data[1]);
//     DataStruct->Mag_Z_RAW = (int16_t)(mag_data[2] << 8 | mag_data[3]);
//     DataStruct->Mag_Y_RAW = (int16_t)(mag_data[4] << 8 | mag_data[5]);

//     DataStruct->Mx = DataStruct->Mag_X_RAW / 1090.0f;
//     DataStruct->My = DataStruct->Mag_Y_RAW / 1090.0f;
//     DataStruct->Mz = DataStruct->Mag_Z_RAW / 1090.0f;

//     // Calculate heading from magnetometer
//     DataStruct->Heading = atan2f(DataStruct->My, DataStruct->Mx) * RAD_TO_DEG;
//     if (DataStruct->Heading < 0)
//         DataStruct->Heading += 360;

//     // Calculate delta time
//     float dt = (float)(HAL_GetTick() - timer) / 1000.0f;
//     timer = HAL_GetTick();

//     // Calculate angles from accelerometer
//     float roll_acc = atan2f(DataStruct->Ay, DataStruct->Az) * RAD_TO_DEG;
//     float pitch_acc = atan2f(-DataStruct->Ax, sqrtf(DataStruct->Ay * DataStruct->Ay +
//                                                     DataStruct->Az * DataStruct->Az)) *
//                       RAD_TO_DEG;
//     float yaw_acc = DataStruct->Heading;

//     // Kalman filter for roll, pitch and yaw
//     DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll_acc, DataStruct->Gx, dt);
//     DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch_acc, DataStruct->Gy, dt);
//     DataStruct->KalmanAngleZ = Kalman_getAngle(&KalmanZ, yaw_acc, DataStruct->Gz, dt);

//     // Complementary filter for comparison
//     DataStruct->CompAngleX = ALPHA * (DataStruct->CompAngleX + DataStruct->Gx * dt) +
//                              (1.0f - ALPHA) * roll_acc;
//     DataStruct->CompAngleY = ALPHA * (DataStruct->CompAngleY + DataStruct->Gy * dt) +
//                              (1.0f - ALPHA) * pitch_acc;
//     DataStruct->CompAngleZ = DataStruct->Heading;
// }

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[14];
    int16_t temp;
    static float yaw = 0;
    static float gz_offset = 0;
    static uint8_t first_run = 1;

    // Calculate gyro Z offset on first run
    if (first_run)
    {
        float sum = 0;
        for (int i = 0; i < 100; i++)
        {
            HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 14, i2c_timeout);
            sum += (float)((int16_t)(Rec_Data[12] << 8 | Rec_Data[13])) / 131.0f;
            HAL_Delay(1);
        }
        gz_offset = sum / 100.0f;
        first_run = 0;
    }

    // 1. Read accelerometer and gyroscope
    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, i2c_timeout);

    // 2. Process accelerometer data
    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0f;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0f;
    DataStruct->Az = DataStruct->Accel_Z_RAW / 16384.0f;

    // 3. Process temperature
    temp = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
    DataStruct->Temperature = (float)((int16_t)temp / 340.0f + 36.53f);

    // 4. Process gyroscope data with offset correction
    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0f;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0f;
    DataStruct->Gz = (DataStruct->Gyro_Z_RAW / 131.0f) - gz_offset;

    // 5. Read magnetometer
    uint8_t mag_data[6];
    HAL_I2C_Mem_Read(I2Cx, HMC5883L_ADDR, HMC5883L_OUT_X_H, 1, mag_data, 6, i2c_timeout);

    DataStruct->Mag_X_RAW = (int16_t)(mag_data[0] << 8 | mag_data[1]);
    DataStruct->Mag_Z_RAW = (int16_t)(mag_data[2] << 8 | mag_data[3]);
    DataStruct->Mag_Y_RAW = (int16_t)(mag_data[4] << 8 | mag_data[5]);

    // 6. Calculate roll and pitch angles from accelerometer
    float roll = atan2f(DataStruct->Ay, DataStruct->Az);
    float pitch = atan2f(-DataStruct->Ax,
                         sqrtf(DataStruct->Ay * DataStruct->Ay +
                               DataStruct->Az * DataStruct->Az));

    // 7. Tilt compensation for magnetometer
    float mx = (float)DataStruct->Mag_X_RAW / 1090.0f;
    float my = (float)DataStruct->Mag_Y_RAW / 1090.0f;
    float mz = (float)DataStruct->Mag_Z_RAW / 1090.0f;

    float mx_comp = mx * cosf(pitch) + mz * sinf(pitch);
    float my_comp = mx * sinf(roll) * sinf(pitch) +
                    my * cosf(roll) -
                    mz * sinf(roll) * cosf(pitch);

    // 8. Calculate heading from compensated magnetometer data
    float heading = atan2f(my_comp, mx_comp) * RAD_TO_DEG;

    // Normalize heading to -180 to 180 degrees
    if (heading < -180.0f)
        heading += 360.0f;
    if (heading > 180.0f)
        heading -= 360.0f;

    // 9. Calculate delta time
    float dt = (float)(HAL_GetTick() - timer) / 1000.0f;
    if (dt > 0.2f)
        dt = 0.2f;
    timer = HAL_GetTick();

    // 10. Update yaw from gyro with threshold
    static const float GYRO_THRESHOLD = 0.5f; // Increased threshold
    if (fabsf(DataStruct->Gz) > GYRO_THRESHOLD)
    {
        yaw += DataStruct->Gz * dt;
    }

    // Normalize yaw to -180 to 180 degrees
    if (yaw > 180.0f)
        yaw -= 360.0f;
    if (yaw < -180.0f)
        yaw += 360.0f;

    // 11. Apply Kalman filter to all angles
    DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll * RAD_TO_DEG, DataStruct->Gx, dt);
    DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch * RAD_TO_DEG, DataStruct->Gy, dt);
    DataStruct->KalmanAngleZ = Kalman_getAngle(&KalmanZ, heading, DataStruct->Gz, dt);

    // 12. Complementary filter for all angles
    DataStruct->CompAngleX = ALPHA * (DataStruct->CompAngleX + DataStruct->Gx * dt) +
                             (1.0f - ALPHA) * (roll * RAD_TO_DEG);
    DataStruct->CompAngleY = ALPHA * (DataStruct->CompAngleY + DataStruct->Gy * dt) +
                             (1.0f - ALPHA) * (pitch * RAD_TO_DEG);

    // Use higher weight for magnetometer in Z angle
    float mag_weight = 0.05f; // Increased to 5%
    DataStruct->CompAngleZ = (1.0f - mag_weight) * yaw + mag_weight * heading;

    // Store final values
    DataStruct->Heading = heading;
    DataStruct->KalmanAngleZ = DataStruct->CompAngleZ; // Use complementary filter result
}

float Kalman_getAngle(Kalman_t *Kalman, float newAngle, float newRate, float dt)
{
    // Discrete Kalman filter time update equations
    Kalman->rate = newRate - Kalman->bias;
    Kalman->angle += dt * Kalman->rate;

    // Update estimation error covariance
    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] -
                             Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    // Measurement update equations
    float S = Kalman->P[0][0] + Kalman->R_measure; // Estimate error
    float K[2];                                    // Kalman gain
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    // Update state estimate (angle and bias)
    float y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    // Update error covariance matrix
    float P00_temp = Kalman->P[0][0];
    float P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
}
