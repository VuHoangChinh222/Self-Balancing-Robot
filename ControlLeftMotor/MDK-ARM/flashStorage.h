#ifndef FLASH_STORAGE_H
#define FLASH_STORAGE_H

#include "stm32f1xx_hal.h"
#include "pid.h"

// Định nghĩa địa chỉ lưu trong Flash
#define FLASH_SECTOR_ADDRESS 0x0801FC00 // Sector cuối của Flash
#define FLASH_PAGE_SIZE_CUSTOM 0x400    // 1KB cho STM32F103

// Cấu trúc lưu thông số PID
typedef struct
{
    float position_kp;
    float position_ki;
    float position_kd;
    float speed_kp;
    float speed_ki;
    float speed_kd;
    float pitch_kp;
    float pitch_ki;
    float pitch_kd;
    float yaw_kp;
    float yaw_ki;
    float yaw_kd;
    uint32_t checksum; // Để kiểm tra tính toàn vẹn dữ liệu
} PIDSettings_t;

HAL_StatusTypeDef Flash_Write_PID(PID_t *position, PID_t *speed, PID_t *pitch, PID_t *yaw);
HAL_StatusTypeDef Flash_Read_PID(PID_t *position, PID_t *speed, PID_t *pitch, PID_t *yaw);
void Flash_Load_Default_PID(PID_t *position, PID_t *speed, PID_t *pitch, PID_t *yaw);

#endif
