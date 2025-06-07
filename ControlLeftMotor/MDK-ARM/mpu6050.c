#include <math.h>
#include "mpu6050.h"

uint8_t MPU6050_rx;
uint8_t MPU6050_rx_buf[20];
uint8_t MPU6050_tx;
float MPU6050_Gyro_LSB = 32.8;
float MPU6050_Acc_LSB = 4096.0;

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx, uint8_t Gyro_FS, uint8_t Acc_FS, uint8_t DLPF_CFG)
{
	// Save LSB/Unit for both gyro and acc in order to use them later
	switch (Gyro_FS)
	{
	case 0: // 250dps
		MPU6050_Gyro_LSB = 131.0;
		break;
	case 1: // 500dps
		MPU6050_Gyro_LSB = 65.5;
		break;
	case 2: // 1000dps
		MPU6050_Gyro_LSB = 32.8;
		break;
	case 3: // 2000dps
		MPU6050_Gyro_LSB = 16.4;
		break;
	default:
		break;
	}

	switch (Acc_FS)
	{
	case 0: // 2g
		MPU6050_Acc_LSB = 16384.0;
		break;
	case 1: // 4g
		MPU6050_Acc_LSB = 8192.0;
		break;
	case 2: // 8g
		MPU6050_Acc_LSB = 4096.0;
		break;
	case 3: // 16g
		MPU6050_Acc_LSB = 2048.0;
		break;
	default:
		break;
	}

	// Read Who am I
	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &MPU6050_rx, 1, 100);
	MPU6050_tx = 0; // Will return this value if code ends here

	// 0x68 will be returned if sensor accessed correctly
	if (MPU6050_rx == 0x68)
	{
		MPU6050_tx = 0;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &MPU6050_tx, 1, 100);
		HAL_Delay(10);

		MPU6050_tx = 0x00; // Set No Sampling
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &MPU6050_tx, 1, 100);
		HAL_Delay(10);

		MPU6050_tx = DLPF_CFG; // Digital Low Pass Filter Setting
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, CONFIG_REG, 1, &MPU6050_tx, 1, 100);
		HAL_Delay(10);

		MPU6050_tx = Gyro_FS << 3;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &MPU6050_tx, 1, 100);
		HAL_Delay(10);

		MPU6050_tx = Acc_FS << 3;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &MPU6050_tx, 1, 100);
		HAL_Delay(10);

		return 0;
	}
	return 1;
}

void MPU6050_Bypass(I2C_HandleTypeDef *I2Cx)
{
	MPU6050_tx = 0x00;													 //
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x6A, 1, &MPU6050_tx, 1, 100); // Master Disable
	HAL_Delay(10);

	MPU6050_tx = 0x02;													 //
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x37, 1, &MPU6050_tx, 1, 100); // Bypass Enable
	HAL_Delay(10);
}

void MPU6050_Master(I2C_HandleTypeDef *I2Cx)
{
	MPU6050_tx = 0x00;													 //
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x37, 1, &MPU6050_tx, 1, 100); // Disable Bypass
	HAL_Delay(10);

	MPU6050_tx = 0x22;													 //
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x6A, 1, &MPU6050_tx, 1, 100); // Master Enable
	HAL_Delay(10);

	MPU6050_tx = 0x0D;													 //
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x24, 1, &MPU6050_tx, 1, 100); // Master Clock to 400kHz
	HAL_Delay(10);

	MPU6050_tx = 0x00;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &MPU6050_tx, 1, 100);
	HAL_Delay(10);
}

void HMC5883L_Setup(I2C_HandleTypeDef *I2Cx)
{
	MPU6050_tx = 0x18; // Fill Slave0 DO
	HAL_I2C_Mem_Write(I2Cx, HMC5883L_ADDRESS << 1, 0x00, 1, &MPU6050_tx, 1, 100);
	HAL_Delay(10);

	MPU6050_tx = 0x20; // Fill Slave0 DO
	HAL_I2C_Mem_Write(I2Cx, HMC5883L_ADDRESS << 1, 0x01, 1, &MPU6050_tx, 1, 100);
	HAL_Delay(10);

	MPU6050_tx = 0x00;
	HAL_I2C_Mem_Write(I2Cx, HMC5883L_ADDRESS << 1, 0x02, 1, &MPU6050_tx, 1, 100); // Mode: Continuous
	HAL_Delay(10);
}

void MPU6050_Slave_Read(I2C_HandleTypeDef *I2Cx)
{
	MPU6050_tx = HMC5883L_ADDRESS | 0x80; // Access Slave into read mode
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x25, 1, &MPU6050_tx, 1, 100);
	HAL_Delay(10);

	MPU6050_tx = 0x03; // Slave REG for reading to take place
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x26, 1, &MPU6050_tx, 1, 100);
	HAL_Delay(10);

	MPU6050_tx = 0x80 | 0x06; // Number of data bytes
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x27, 1, &MPU6050_tx, 1, 100);
	HAL_Delay(10);
}

uint8_t MPU6050_DataReady(I2C_HandleTypeDef *I2Cx)
{
	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, INT_STATUS_REG, 1, &MPU6050_rx, 1, 100);
	return MPU6050_rx;
}

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, MPU6050_rx_buf, 20, 100);
}

void MPU6050_Read_All_DMA(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
	HAL_I2C_Mem_Read_DMA(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, MPU6050_rx_buf, 20);
}

// void MPU6050_Parsing(MPU6050_t *DataStruct)
// {
//     DataStruct->Accel_X_RAW = -(MPU6050_rx_buf[0] << 8 | MPU6050_rx_buf[1]);
//     DataStruct->Accel_Y_RAW = (MPU6050_rx_buf[2] << 8 | MPU6050_rx_buf[3]);
//     DataStruct->Accel_Z_RAW = (MPU6050_rx_buf[4] << 8 | MPU6050_rx_buf[5]);
//     // Didn't Save Temp Value
//     DataStruct->Gyro_X_RAW = (MPU6050_rx_buf[8] << 8 | MPU6050_rx_buf[9]);
//     DataStruct->Gyro_Y_RAW = -(MPU6050_rx_buf[10] << 8 | MPU6050_rx_buf[11]);
//     DataStruct->Gyro_Z_RAW = -(MPU6050_rx_buf[12] << 8 | MPU6050_rx_buf[13]);

//     DataStruct->Mag_X_RAW = (MPU6050_rx_buf[14] << 8 | MPU6050_rx_buf[15]);
//     DataStruct->Mag_Z_RAW = -(MPU6050_rx_buf[16] << 8 | MPU6050_rx_buf[17]);
//     DataStruct->Mag_Y_RAW = -(MPU6050_rx_buf[18] << 8 | MPU6050_rx_buf[19]);

//     DataStruct->Gyro_X_RAW -= DataStruct->Gyro_X_Offset;
//     DataStruct->Gyro_Y_RAW -= DataStruct->Gyro_Y_Offset;
//     DataStruct->Gyro_Z_RAW -= DataStruct->Gyro_Z_Offset;

//     DataStruct->Mag_X_RAW -= DataStruct->Mag_X_Offset;
//     DataStruct->Mag_Y_RAW -= DataStruct->Mag_Y_Offset;
//     DataStruct->Mag_Z_RAW -= DataStruct->Mag_Z_Offset;

//     DataStruct->Gx = DataStruct->Gyro_X_RAW / MPU6050_Gyro_LSB * D2R;
//     DataStruct->Gy = DataStruct->Gyro_Y_RAW / MPU6050_Gyro_LSB * D2R;
//     DataStruct->Gz = DataStruct->Gyro_Z_RAW / MPU6050_Gyro_LSB * D2R;
//     DataStruct->Ax = DataStruct->Accel_X_RAW / MPU6050_Acc_LSB;
//     DataStruct->Ay = DataStruct->Accel_Y_RAW / MPU6050_Acc_LSB;
//     DataStruct->Az = DataStruct->Accel_Z_RAW / MPU6050_Acc_LSB;
// }

void MPU6050_Parsing_NoOffest(MPU6050_t *DataStruct)
{
	DataStruct->Accel_X_RAW = -(MPU6050_rx_buf[0] << 8 | MPU6050_rx_buf[1]);
	DataStruct->Accel_Y_RAW = (MPU6050_rx_buf[2] << 8 | MPU6050_rx_buf[3]);
	DataStruct->Accel_Z_RAW = (MPU6050_rx_buf[4] << 8 | MPU6050_rx_buf[5]);
	// Didn't Save Temp Value
	DataStruct->Gyro_X_RAW = (MPU6050_rx_buf[8] << 8 | MPU6050_rx_buf[9]);
	DataStruct->Gyro_Y_RAW = -(MPU6050_rx_buf[10] << 8 | MPU6050_rx_buf[11]);
	DataStruct->Gyro_Z_RAW = -(MPU6050_rx_buf[12] << 8 | MPU6050_rx_buf[13]);

	DataStruct->Mag_X_RAW = (MPU6050_rx_buf[14] << 8 | MPU6050_rx_buf[15]);
	DataStruct->Mag_Z_RAW = -(MPU6050_rx_buf[16] << 8 | MPU6050_rx_buf[17]);
	DataStruct->Mag_Y_RAW = -(MPU6050_rx_buf[18] << 8 | MPU6050_rx_buf[19]);

	DataStruct->Gyro_X_RAW -= DataStruct->Gyro_X_Offset;
	DataStruct->Gyro_Y_RAW -= DataStruct->Gyro_Y_Offset;
	DataStruct->Gyro_Z_RAW -= DataStruct->Gyro_Z_Offset;

	DataStruct->Gx = DataStruct->Gyro_X_RAW / MPU6050_Gyro_LSB * D2R;
	DataStruct->Gy = DataStruct->Gyro_Y_RAW / MPU6050_Gyro_LSB * D2R;
	DataStruct->Gz = DataStruct->Gyro_Z_RAW / MPU6050_Gyro_LSB * D2R;
	DataStruct->Ax = DataStruct->Accel_X_RAW / MPU6050_Acc_LSB;
	DataStruct->Ay = DataStruct->Accel_Y_RAW / MPU6050_Acc_LSB;
	DataStruct->Az = DataStruct->Accel_Z_RAW / MPU6050_Acc_LSB;
}

float MPU6050_GetYaw_Absolute(MPU6050_t *DataStruct)
{
	// Bước 1: Chuẩn hóa từ kế
	float norm = sqrt(DataStruct->Mag_X_RAW * DataStruct->Mag_X_RAW +
					  DataStruct->Mag_Y_RAW * DataStruct->Mag_Y_RAW +
					  DataStruct->Mag_Z_RAW * DataStruct->Mag_Z_RAW);

	float mx = DataStruct->Mag_X_RAW / norm;
	float my = DataStruct->Mag_Y_RAW / norm;
	float mz = DataStruct->Mag_Z_RAW / norm;

	// Bước 2: Ước lượng roll và pitch từ gia tốc kế
	float roll = atan2f(DataStruct->Ay, DataStruct->Az);
	float pitch = atan2f(-DataStruct->Ax, sqrtf(DataStruct->Ay * DataStruct->Ay + DataStruct->Az * DataStruct->Az));

	// Bước 3: Bù nhiễu roll và pitch cho từ kế
	float mx2 = mx * cosf(pitch) + mz * sinf(pitch);
	float my2 = mx * sinf(roll) * sinf(pitch) + my * cosf(roll) - mz * sinf(roll) * cosf(pitch);

	// Bước 4: Tính toán yaw (heading)
	float yaw = atan2f(-my2, mx2);

	// Bước 5: Đưa yaw về khoảng 0~2π
	if (yaw < 0)
		yaw += 2 * M_PI;

	// Bước 6: Chuyển sang độ
	return yaw * 180.0f / M_PI;
}

void MPU6050_Parsing(MPU6050_t *DataStruct)
{
	const float GAUSS_PER_LSB = 1090.0f; // Tỷ lệ chuyển đổi cho HMC5883L

	// Đọc dữ liệu thô từ các cảm biến
	DataStruct->Accel_X_RAW = -(int16_t)(MPU6050_rx_buf[0] << 8 | MPU6050_rx_buf[1]);
	DataStruct->Accel_Y_RAW = (int16_t)(MPU6050_rx_buf[2] << 8 | MPU6050_rx_buf[3]);
	DataStruct->Accel_Z_RAW = (int16_t)(MPU6050_rx_buf[4] << 8 | MPU6050_rx_buf[5]);

	DataStruct->Gyro_X_RAW = (int16_t)(MPU6050_rx_buf[8] << 8 | MPU6050_rx_buf[9]);
	DataStruct->Gyro_Y_RAW = -(int16_t)(MPU6050_rx_buf[10] << 8 | MPU6050_rx_buf[11]);
	DataStruct->Gyro_Z_RAW = -(int16_t)(MPU6050_rx_buf[12] << 8 | MPU6050_rx_buf[13]);

	// Đọc dữ liệu từ kế (HMC5883L)
	DataStruct->Mag_X_RAW = (int16_t)(MPU6050_rx_buf[14] << 8 | MPU6050_rx_buf[15]);  // X
	DataStruct->Mag_Y_RAW = -(int16_t)(MPU6050_rx_buf[16] << 8 | MPU6050_rx_buf[17]); // Y (đảo dấu)
	DataStruct->Mag_Z_RAW = -(int16_t)(MPU6050_rx_buf[18] << 8 | MPU6050_rx_buf[19]); // Z (đảo dấu)

	// Áp dụng offset cho gyroscope và từ kế
	DataStruct->Gyro_X_RAW -= DataStruct->Gyro_X_Offset;
	DataStruct->Gyro_Y_RAW -= DataStruct->Gyro_Y_Offset;
	DataStruct->Gyro_Z_RAW -= DataStruct->Gyro_Z_Offset;

	DataStruct->Mag_X_RAW -= DataStruct->Mag_X_Offset;
	DataStruct->Mag_Y_RAW -= DataStruct->Mag_Y_Offset;
	DataStruct->Mag_Z_RAW -= DataStruct->Mag_Z_Offset;

	// Tính toán gia tốc (g) và vận tốc góc (rad/s)
	DataStruct->Gx = DataStruct->Gyro_X_RAW / MPU6050_Gyro_LSB * D2R;
	DataStruct->Gy = DataStruct->Gyro_Y_RAW / MPU6050_Gyro_LSB * D2R;
	DataStruct->Gz = DataStruct->Gyro_Z_RAW / MPU6050_Gyro_LSB * D2R;

	DataStruct->Ax = DataStruct->Accel_X_RAW / MPU6050_Acc_LSB;
	DataStruct->Ay = DataStruct->Accel_Y_RAW / MPU6050_Acc_LSB;
	DataStruct->Az = DataStruct->Accel_Z_RAW / MPU6050_Acc_LSB;

	// Tính Roll và Pitch từ gia tốc kế (đơn vị radian)
	float roll_rad = atan2(DataStruct->Ay, DataStruct->Az);
	float pitch_rad = atan2(-DataStruct->Ax,
							sqrt(DataStruct->Ay * DataStruct->Ay +
								 DataStruct->Az * DataStruct->Az));

	// Chuyển đổi dữ liệu từ kế sang Gauss
	float mx = DataStruct->Mag_X_RAW / GAUSS_PER_LSB;
	float my = DataStruct->Mag_Y_RAW / GAUSS_PER_LSB;
	float mz = DataStruct->Mag_Z_RAW / GAUSS_PER_LSB;

	// Bù nghiêng (Tilt Compensation)
	float cos_roll = cos(roll_rad);
	float sin_roll = sin(roll_rad);
	float cos_pitch = cos(pitch_rad);
	float sin_pitch = sin(pitch_rad);

	// Công thức bù nghiêng cho GY-87
	float mx_comp = mx * cos_pitch + mz * sin_pitch;
	float my_comp = mx * sin_roll * sin_pitch +
					my * cos_roll -
					mz * sin_roll * cos_pitch;

	// Tính Yaw và chuẩn hóa (0-360 độ)
	float yaw_rad = atan2(my_comp, mx_comp);
	DataStruct->Yaw = yaw_rad * R2D; // Chuyển sang độ

	// Đảm bảo Yaw trong khoảng 0-360 độ
	if (DataStruct->Yaw < 0)
	{
		DataStruct->Yaw += 360.0f;
	}

	// Lưu Roll và Pitch (đơn vị độ)
	DataStruct->Roll = roll_rad * R2D;
	DataStruct->Pitch = pitch_rad * R2D;
}