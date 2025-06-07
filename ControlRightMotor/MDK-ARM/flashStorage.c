#include "flashStorage.h"
#include <string.h>

static uint32_t Calculate_Checksum(PIDSettings_t *settings)
{
    uint32_t sum = 0;
    uint32_t *data = (uint32_t *)settings;
    // Tính tổng tất cả các giá trị trừ checksum
    for (int i = 0; i < (sizeof(PIDSettings_t) / 4 - 1); i++)
    {
        sum += data[i];
    }
    return sum;
}

HAL_StatusTypeDef Flash_Write_PID(PID_t *position, PID_t *speed, PID_t *pitch, PID_t *yaw)
{
    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PAGEError;
    PIDSettings_t settings;

    // Đóng gói dữ liệu
    settings.position_kp = position->Kp;
    settings.position_ki = position->Ki;
    settings.position_kd = position->Kd;
    settings.speed_kp = speed->Kp;
    settings.speed_ki = speed->Ki;
    settings.speed_kd = speed->Kd;
    settings.pitch_kp = pitch->Kp;
    settings.pitch_ki = pitch->Ki;
    settings.pitch_kd = pitch->Kd;
    settings.yaw_kp = yaw->Kp;
    settings.yaw_ki = yaw->Ki;
    settings.yaw_kd = yaw->Kd;

    // Tính checksum
    settings.checksum = Calculate_Checksum(&settings);

    // Mở khóa Flash
    HAL_FLASH_Unlock();

    // Xóa sector
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = FLASH_SECTOR_ADDRESS;
    EraseInitStruct.NbPages = 1;
    status = HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);
    if (status != HAL_OK)
    {
        HAL_FLASH_Lock();
        return status;
    }

    // Ghi dữ liệu
    uint32_t *source = (uint32_t *)&settings;
    uint32_t address = FLASH_SECTOR_ADDRESS;

    for (uint32_t i = 0; i < sizeof(PIDSettings_t) / 4; i++)
    {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, source[i]);
        if (status != HAL_OK)
        {
            HAL_FLASH_Lock();
            return status;
        }
        address += 4;
    }

    HAL_FLASH_Lock();
    return HAL_OK;
}

HAL_StatusTypeDef Flash_Read_PID(PID_t *position, PID_t *speed, PID_t *pitch, PID_t *yaw)
{
    PIDSettings_t settings;
    uint32_t *source = (uint32_t *)FLASH_SECTOR_ADDRESS;
    uint32_t *dest = (uint32_t *)&settings;

    // Đọc dữ liệu từ Flash
    for (uint32_t i = 0; i < sizeof(PIDSettings_t) / 4; i++)
    {
        dest[i] = source[i];
    }

    // Kiểm tra checksum
    uint32_t calculated_checksum = Calculate_Checksum(&settings);
    if (calculated_checksum != settings.checksum)
    {
        Flash_Load_Default_PID(position, speed, pitch, yaw);
        return HAL_ERROR;
    }

    // Cập nhật các giá trị PID
    position->Kp = settings.position_kp;
    position->Ki = settings.position_ki;
    position->Kd = settings.position_kd;

    speed->Kp = settings.speed_kp;
    speed->Ki = settings.speed_ki;
    speed->Kd = settings.speed_kd;

    pitch->Kp = settings.pitch_kp;
    pitch->Ki = settings.pitch_ki;
    pitch->Kd = settings.pitch_kd;

    yaw->Kp = settings.yaw_kp;
    yaw->Ki = settings.yaw_ki;
    yaw->Kd = settings.yaw_kd;

    return HAL_OK;
}

void Flash_Load_Default_PID(PID_t *position, PID_t *speed, PID_t *pitch, PID_t *yaw)
{
    // Giá trị mặc định cho các bộ PID
    position->Kp = 0.0f;
    position->Ki = 0.0f;
    position->Kd = 0.0f;

    speed->Kp = 0.0f;
    speed->Ki = 0.0f;
    speed->Kd = 0.0f;

    pitch->Kp = 6.0f;
    pitch->Ki = 0.0f;
    pitch->Kd = 0.0f;

    yaw->Kp = 0.0f;
    yaw->Ki = 0.0f;
    yaw->Kd = 0.0f;
}
