#include "74165.h"

uint8_t Read_74165(void)
{
  uint8_t received_data = 0;

  // Bước 1: Xác lập trạng thái ban đầu
  HAL_GPIO_WritePin(PIN_CP_GPIO_Port, PIN_CP_Pin, GPIO_PIN_RESET); // CLK low
  HAL_GPIO_WritePin(PIN_PL_GPIO_Port, PIN_PL_Pin, GPIO_PIN_SET);   // PL high

  // Bước 2: Tạo xung LOAD (PL) để nạp dữ liệu vào thanh ghi
  HAL_GPIO_WritePin(PIN_PL_GPIO_Port, PIN_PL_Pin, GPIO_PIN_RESET); // PL low
  for (volatile uint16_t i = 0; i < 20; i++)
    ;                                                            // Short delay
  HAL_GPIO_WritePin(PIN_PL_GPIO_Port, PIN_PL_Pin, GPIO_PIN_SET); // PL high
  for (volatile uint16_t i = 0; i < 20; i++)
    ; // Short delay

  // Bước 3: Đọc 8 bit (MSB first)
  for (uint8_t i = 0; i < 8; i++)
  {
    // Đọc dữ liệu trước khi tạo xung clock
    if (HAL_GPIO_ReadPin(PIN_Q7_GPIO_Port, PIN_Q7_Pin))
    {
      received_data |= (0x80 >> i); // MSB first
    }

    // Tạo xung clock
    HAL_GPIO_WritePin(PIN_CP_GPIO_Port, PIN_CP_Pin, GPIO_PIN_SET); // CLK high
    for (volatile uint16_t j = 0; j < 20; j++)
      ;                                                              // Short delay
    HAL_GPIO_WritePin(PIN_CP_GPIO_Port, PIN_CP_Pin, GPIO_PIN_RESET); // CLK low
    for (volatile uint16_t j = 0; j < 20; j++)
      ; // Short delay
  }

  return received_data;
}

uint8_t Get6Buttons(uint8_t result_full)
{
  // Chỉ lấy 6 bit thấp và đảo bit vì nút nhấn active LOW
  return (~result_full & 0x3F);
}