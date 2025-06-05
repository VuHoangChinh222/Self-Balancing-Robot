#ifndef FLASH_STORAGE_H
#define FLASH_STORAGE_H

#include "stm32f1xx_hal.h"
#include "pid.h"

// Ð?nh nghia d?a ch? luu trong Flash
#define FLASH_SECTOR_ADDRESS 0x0801FC00 // Sector cu?i c?a Flash
#define FLASH_PAGE_SIZE_CUSTOM 0x400           // 1KB cho STM32F103

// C?u trúc luu thông s? PID
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
    uint32_t checksum; // Ð? ki?m tra tính toàn v?n d? li?u
} PIDSettings_t;

HAL_StatusTypeDef Flash_Write_PID(PID_t *position, PID_t *speed, PID_t *pitch, PID_t *yaw);
HAL_StatusTypeDef Flash_Read_PID(PID_t *position, PID_t *speed, PID_t *pitch, PID_t *yaw);
void Flash_Load_Default_PID(PID_t *position, PID_t *speed, PID_t *pitch, PID_t *yaw);

#endif
