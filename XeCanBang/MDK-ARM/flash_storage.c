#include "flash_storage.h"
#include "stm32f1xx_hal.h"

#define FLASH_USER_START_ADDR  ((uint32_t)0x0801FC00)

void SavePIDToFlash(PID_t *pid) {
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef eraseInit;
    uint32_t pageError;
    eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInit.PageAddress = FLASH_USER_START_ADDR;
    eraseInit.NbPages = 1;

    HAL_FLASHEx_Erase(&eraseInit, &pageError);

    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_USER_START_ADDR, *(uint32_t*)&pid->Kp);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_USER_START_ADDR+4, *(uint32_t*)&pid->Ki);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_USER_START_ADDR+8, *(uint32_t*)&pid->Kd);

    HAL_FLASH_Lock();
}

void LoadPIDFromFlash(PID_t *pid) {
    float tempKp = *(float*)FLASH_USER_START_ADDR;
    float tempKi = *(float*)(FLASH_USER_START_ADDR+4);
    float tempKd = *(float*)(FLASH_USER_START_ADDR+8);

    // Kiem tra du lieu có hop le không
    if (tempKp < 0 || tempKp > 10 ||
        tempKi < 0 || tempKi > 5 ||
        tempKd < 0 || tempKd > 5 ) {
        // Neu du lieu không hop le, dat giá tri mac dinh
        pid->Kp = 0.5;
        pid->Ki = 0.01;
        pid->Kd = 0.03;
    } else {
        // Neu du lieu hop le, se dung giá tri tu flash
        pid->Kp = tempKp;
        pid->Ki = tempKi;
        pid->Kd = tempKd;
    }
}
