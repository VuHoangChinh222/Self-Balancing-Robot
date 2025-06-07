/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "string.h"
#include <stdio.h>
#include <stdlib.h>
#include "74165.h"
#include "ssd1306.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for ReceiveUART */
osThreadId_t ReceiveUARTHandle;
const osThreadAttr_t ReceiveUART_attributes = {
    .name = "ReceiveUART",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for SendCAN */
osThreadId_t SendCANHandle;
const osThreadAttr_t SendCAN_attributes = {
    .name = "SendCAN",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for ReadMPU6050 */
osThreadId_t ReadMPU6050Handle;
const osThreadAttr_t ReadMPU6050_attributes = {
    .name = "ReadMPU6050",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* USER CODE BEGIN PV */

//------------------------------------------------------------------------------------------
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);

/* USER CODE BEGIN PFP */
void CAN_Send_Extended(int16_t x, int16_t y);
void SendAllPIDViaCAN(void);
void DisplayPIDMenu(uint8_t position);
void DisplayPIDParams(uint8_t pidType, uint8_t position);
void DisplayFlashSaveSuccess(void);
//----------------------------------------------------------
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct
{
  int16_t x;
  int16_t y;
} JoystickData_t;
JoystickData_t joystickData = {0, 0};
osMutexId_t joystickMutexHandle;
#define UART_BUFFER_SIZE 32
#define CAN_SEND_INTERVAL 5 // Gửi mỗi 5ms
#define CAN_RETRY_COUNT 3   // Số lần thử lại nếu gửi thất bại
// Biến nhận tín hiệu UART2
typedef struct
{
  char buffer[64];
  uint8_t idx;
  uint32_t lastReceiveTime;
  uint8_t isNewData;
} UART_Buffer_t;
uint8_t u8_RxData;
// Khai báo là volatile vì được truy cập từ interrupt
volatile UART_Buffer_t uartBuffer = {0};
// CAN
CAN_TxHeaderTypeDef TxHeader = {
    .StdId = 0x446,
    .RTR = CAN_RTR_DATA,
    .IDE = CAN_ID_STD,
    .DLC = 3,
    .TransmitGlobalTime = DISABLE};

uint8_t TxData[3];
uint32_t TxMailbox;

#define UART_TIMEOUT 2000

// Khai bao cac bien cho IC 74165
// Button
uint8_t blackButtonData = 0x3F;
typedef enum
{
  BTN_SELECT = 0,
  BTN_UP = 1,
  BTN_DOWN = 2,
  BTN_SAVE = 3,
  BTN_EXIT = 4,
  BTN_OPENMENU = 5
} TypeButtonBlack;
volatile uint8_t blackButtonStates = 0; // Bien luu trang thai cac nut
#define PID_MODE_SELECT 0               // Mode chọn loại PID
#define PID_MODE_PARAMS 1               // Mode xem thông số PID
#define PID_MODE_EDIT 2                 // Mode chỉnh sửa giá trị
uint8_t pidMode = PID_MODE_SELECT;      // Mode hiện tại
uint8_t paramPosition = 0;              // Vị trí trong menu thông số (0: P, 1: I, 2: D)
PID_t position_pid = {0};
PID_t speed_pid = {0};
PID_t pitch_pid = {0};
PID_t yaw_pid = {0};
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_CAN_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);

  HAL_UART_Receive_IT(&huart2, &u8_RxData, 1);
  //  HAL_UART_Receive_DMA(&huart2, &u8_RxData, 1);

  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }

  if (Flash_Read_PID(&position_pid, &speed_pid, &pitch_pid, &yaw_pid) != HAL_OK)
  {
    // Nếu chưa có dữ liệu hợp lệ thì dùng giá trị mặc định
    Flash_Load_Default_PID(&position_pid, &speed_pid, &pitch_pid, &yaw_pid);

    // Và ghi giá trị mặc định đó vào Flash
    Flash_Write_PID(&position_pid, &speed_pid, &pitch_pid, &yaw_pid);
    SendAllPIDViaCAN();
  }
  else
  {
    SendAllPIDViaCAN();
  }

  // Khoi tao IC74165 cho nhung nut nhan mau den
  SSD1306_Init();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  joystickMutexHandle = osMutexNew(NULL);
  if (joystickMutexHandle == NULL)
  {
    Error_Handler();
  }
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ReceiveUART */
  ReceiveUARTHandle = osThreadNew(StartTask02, NULL, &ReceiveUART_attributes);

  /* creation of SendCAN */
  SendCANHandle = osThreadNew(StartTask03, NULL, &SendCAN_attributes);

  /* creation of ReadMPU6050 */
  ReadMPU6050Handle = osThreadNew(StartTask04, NULL, &ReadMPU6050_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // MPU6050_Read_All(&hi2c1, &MPU6050);
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = ENABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  CAN_FilterTypeDef canfilterconfig;
  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 18; // which filter bank to use from the assigned ones
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canfilterconfig.FilterIdHigh = 0x211 << 5;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0x211 << 5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 40; // how many filters to assign to the CAN1 (master can)

  HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

  /* USER CODE END CAN_Init 2 */
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PIN_PL_Pin | PIN_CP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PIN_PL_Pin PIN_CP_Pin */
  GPIO_InitStruct.Pin = PIN_PL_Pin | PIN_CP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PIN_Q7_Pin */
  GPIO_InitStruct.Pin = PIN_Q7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PIN_Q7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // Kích hoạt ngắt EXTI
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    uartBuffer.lastReceiveTime = HAL_GetTick();

    if (u8_RxData == '\n' || u8_RxData == '\r')
    {
      if (uartBuffer.idx > 0)
      {
        uartBuffer.buffer[uartBuffer.idx] = '\0';
        uartBuffer.isNewData = 1;
      }
      uartBuffer.idx = 0;
    }
    else if (uartBuffer.idx < sizeof(uartBuffer.buffer) - 1)
    {
      uartBuffer.buffer[uartBuffer.idx++] = u8_RxData;
    }

    HAL_UART_Receive_IT(&huart2, &u8_RxData, 1);
  }
}

void CAN_Send_Extended(int16_t x, int16_t y)
{
  if (x <= 100 && x >= -100)
  {
    // Gửi giá trị X,Y
    TxHeader.StdId = 0x447;
    TxHeader.DLC = 3;

    // Gửi X
    TxData[0] = x > 0 ? 1 : 0;
    x = abs(x);
    TxData[1] = (x >> 8) & 0xFF;
    TxData[2] = x & 0xFF;

    if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
    {
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      Error_Handler();
    }
  }
  osDelay(1);
  // Gửi giá trị Y
  if (y <= 100 && y >= -100)
  {
    TxHeader.StdId = 0x448;
    TxHeader.DLC = 3;
    TxData[0] = y > 0 ? 1 : 0;
    y = abs(y);
    TxData[1] = (y >> 8) & 0xFF;
    TxData[2] = y & 0xFF;

    if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
    {
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      Error_Handler();
    }
  }
  osDelay(1);
}

void SendAllPIDViaCAN(void)
{
  PID_t *pidList[4] = {&pitch_pid, &speed_pid, &position_pid, &yaw_pid};
  uint16_t baseCanID = 0x500; // bạn có thể tùy chỉnh ID base này

  for (int i = 0; i < 4; i++)
  {
    uint16_t canID = baseCanID + i; // Mỗi PID có 1 ID riêng
    float Kp = pidList[i]->Kp;
    float Ki = pidList[i]->Ki;
    float Kd = pidList[i]->Kd;

    // Gửi từng thông số PID (dạng float 4 byte)
    uint8_t data[8];
    memcpy(&data[0], &Kp, sizeof(float));
    memcpy(&data[4], &Ki, sizeof(float));

    CAN_TxHeaderTypeDef header;
    header.StdId = canID; // Ví dụ: 0x500 (Pitch), 0x501 (Speed), ...
    header.RTR = CAN_RTR_DATA;
    header.IDE = CAN_ID_STD;
    header.DLC = 8; // 2 float: Kp, Ki
    header.TransmitGlobalTime = DISABLE;
    uint32_t mailbox;
    if (HAL_CAN_AddTxMessage(&hcan, &header, data, &mailbox) != HAL_OK)
    {
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      Error_Handler();
    }

    HAL_Delay(2);

    // Gửi tiếp Kd (vì không đủ 3 float trong 8 byte)
    memcpy(&data[0], &Kd, sizeof(float));
    memset(&data[4], 0, 4); // phần sau không cần thiết, để 0

    header.StdId = canID + 0x10; // gửi Kd bằng ID kế tiếp (ví dụ 0x510, 0x511, ...)

    if (HAL_CAN_AddTxMessage(&hcan, &header, data, &mailbox) != HAL_OK)
    {
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      Error_Handler();
    }
    HAL_Delay(2);
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
 * @brief Function implementing the ReceiveUART thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  char processBuffer[50]; // Thêm khai báo này
  /* Infinite loop */
  for (;;)
  {
    if (uartBuffer.isNewData)
    {
      // Copy buffer để xử lý
      __disable_irq();
      strcpy(processBuffer, (const char *)uartBuffer.buffer);
      uartBuffer.isNewData = 0;
      __enable_irq();

      // Xử lý dữ liệu
      char *x_str = strstr(processBuffer, "X:");
      char *y_str = strstr(processBuffer, "Y:");
      if (x_str && y_str)
      {
        char *comma = strchr(x_str, ',');
        if (comma && comma < y_str)
        {
          *comma = '\0';
          int16_t x = atoi(x_str + 2);
          int16_t y = atoi(y_str + 2);
          x = x > 100 ? 100 : (x < -100 ? -100 : x);
          y = y > 100 ? 100 : (y < -100 ? -100 : y);
          osMutexAcquire(joystickMutexHandle, osWaitForever);
          joystickData.x = x;
          joystickData.y = y;
          osMutexRelease(joystickMutexHandle);
        }
      }
    }

    // Check timeout
    if (HAL_GetTick() - uartBuffer.lastReceiveTime > UART_TIMEOUT)
    {
      osMutexAcquire(joystickMutexHandle, osWaitForever);
      joystickData.x = 0;
      joystickData.y = 0;
      osMutexRelease(joystickMutexHandle);
    }
    CAN_Send_Extended(joystickData.x, joystickData.y);
    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
 * @brief Function implementing the SenCAN thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  uint8_t prev_data = 0x3F;
  uint8_t data_full;
  uint8_t stable_count = 0;
  uint8_t current_button;
  uint8_t menuPosition = 0;
  uint8_t menuState = 0;

  SSD1306_Clear();
  SSD1306_GotoXY(40, 0);
  SSD1306_Puts("^_^", &Font_16x26, 1);
  SSD1306_UpdateScreen();

  for (;;)
  {
    data_full = Read_74165();
    current_button = Get6Buttons(data_full);

    if (current_button == blackButtonData)
    {
      stable_count++;
      if (stable_count >= 3 && current_button != prev_data)
      {
        // Nút OPEN MENU
        if (current_button == (~(1 << BTN_OPENMENU) & 0x3F))
        {
          menuState = !menuState;
          menuPosition = 0;
          pidMode = PID_MODE_SELECT;
          paramPosition = 0;

          SSD1306_Clear();
          if (menuState)
          {
            DisplayPIDMenu(menuPosition);
          }
          else
          {
            SSD1306_GotoXY(40, 0);
            SSD1306_Puts("^_^", &Font_16x26, 1);
          }
          SSD1306_UpdateScreen();
        }

        // Các nút khác chỉ xử lý khi menu đang mở
        else if (menuState)
        {
          // Lấy bộ PID tương ứng
          PID_t *currentPID = NULL;
          switch (menuPosition)
          {
          case 0:
            currentPID = &pitch_pid;
            break;
          case 1:
            currentPID = &speed_pid;
            break;
          case 2:
            currentPID = &position_pid;
            break;
          case 3:
            currentPID = &yaw_pid;
            break;
          }

          // Nút SELECT
          if (current_button == (~(1 << BTN_SELECT) & 0x3F))
          {
            if (pidMode == PID_MODE_SELECT)
            {
              pidMode = PID_MODE_PARAMS;
              paramPosition = 0;
              SSD1306_Clear();
              DisplayPIDParams(menuPosition, paramPosition);
              SSD1306_UpdateScreen();
            }
            else if (pidMode == PID_MODE_PARAMS)
            {
              pidMode = PID_MODE_EDIT;
              SSD1306_Clear();
              DisplayPIDParams(menuPosition, paramPosition);
              SSD1306_UpdateScreen();
            }
          }

          // Nút UP
          else if (current_button == (~(1 << BTN_UP) & 0x3F))
          {
            if (pidMode == PID_MODE_SELECT && menuPosition > 0)
            {
              menuPosition--;
              SSD1306_Clear();
              DisplayPIDMenu(menuPosition);
              SSD1306_UpdateScreen();
            }
            else if (pidMode == PID_MODE_PARAMS && paramPosition > 0)
            {
              paramPosition--;
              SSD1306_Clear();
              DisplayPIDParams(menuPosition, paramPosition);
              SSD1306_UpdateScreen();
            }
            else if (pidMode == PID_MODE_EDIT)
            {
              // Tăng giá trị
              if (paramPosition == 0)
                currentPID->Kp += 0.02f;
              else if (paramPosition == 1)
                currentPID->Ki += 0.02f;
              else if (paramPosition == 2)
                currentPID->Kd += 0.02f;

              SSD1306_Clear();
              DisplayPIDParams(menuPosition, paramPosition);
              SSD1306_UpdateScreen();
            }
          }

          // Nút DOWN
          else if (current_button == (~(1 << BTN_DOWN) & 0x3F))
          {

            if (pidMode == PID_MODE_SELECT && menuPosition < 3)
            {
              menuPosition++;
              SSD1306_Clear();
              DisplayPIDMenu(menuPosition);
              SSD1306_UpdateScreen();
            }
            else if (pidMode == PID_MODE_PARAMS && paramPosition < 2)
            {
              paramPosition++;
              SSD1306_Clear();
              DisplayPIDParams(menuPosition, paramPosition);
              SSD1306_UpdateScreen();
            }
            else if (pidMode == PID_MODE_EDIT)
            {
              // Giảm giá trị (không dưới 0.00)
              if (paramPosition == 0 && currentPID->Kp > 0.00f)
                currentPID->Kp -= 0.01f;
              else if (paramPosition == 1 && currentPID->Ki > 0.00f)
                currentPID->Ki -= 0.01f;
              else if (paramPosition == 2 && currentPID->Kd > 0.00f)
                currentPID->Kd -= 0.01f;

              SSD1306_Clear();
              DisplayPIDParams(menuPosition, paramPosition);
              SSD1306_UpdateScreen();
            }
          }
          // Nút EXIT
          else if (current_button == (~(1 << BTN_EXIT) & 0x3F))
          {
            if (pidMode == PID_MODE_PARAMS)
            {
              pidMode = PID_MODE_SELECT;
              SSD1306_Clear();
              DisplayPIDMenu(menuPosition);
              SSD1306_UpdateScreen();
            }
            else if (pidMode == PID_MODE_EDIT)
            {
              pidMode = PID_MODE_PARAMS;
              SSD1306_Clear();
              DisplayPIDParams(menuPosition, paramPosition);
              SSD1306_UpdateScreen();
            }
          }

          // Nút SAVE
          else if (current_button == (~(1 << BTN_SAVE) & 0x3F))
          {
            if (Flash_Write_PID(&position_pid, &speed_pid, &pitch_pid, &yaw_pid) == HAL_OK)
            {
              SendAllPIDViaCAN();
              DisplayFlashSaveSuccess();
            }
            else
            {
              SSD1306_Clear();
              SSD1306_GotoXY(0, 20);
              SSD1306_Puts("SAVE FAILED!", &Font_7x10, 1);
              SSD1306_UpdateScreen();
              osDelay(1000);
            }

            if (pidMode == PID_MODE_EDIT || pidMode == PID_MODE_PARAMS)
            {
              SSD1306_Clear();
              DisplayPIDParams(menuPosition, paramPosition);
              SSD1306_UpdateScreen();
            }
            else
            {
              SSD1306_Clear();
              DisplayPIDMenu(menuPosition);
              SSD1306_UpdateScreen();
            }
          }
        }

        prev_data = current_button;
      }
    }
    else
    {
      stable_count = 0;
      blackButtonData = current_button;
    }

    osDelay(20);
  }
}

// Hiển thị menu PID
void DisplayPIDMenu(uint8_t position)
{
  SSD1306_Clear();

  // Hiển thị các mục menu - sử dụng mảng char thay vì const char
  char menu_items[4][15] = {
      "PID_Pitch",
      "PID_Speed",
      "PID_Position",
      "PID_Yaw"};

  for (uint8_t i = 0; i < 4; i++)
  {
    SSD1306_GotoXY(0, i * 12);
    if (i == position)
    {
      SSD1306_Puts("->", &Font_7x10, 1);
      SSD1306_GotoXY(15, i * 12);
    }
    else
    {
      SSD1306_GotoXY(15, i * 12);
    }
    SSD1306_Puts(menu_items[i], &Font_7x10, 1);
  }
}

void DisplayPIDParams(uint8_t pidType, uint8_t position)
{
  char str[50];
  PID_t *currentPID = NULL;

  // Không đọc flash nữa!
  switch (pidType)
  {
  case 0:
    currentPID = &pitch_pid;
    break;
  case 1:
    currentPID = &speed_pid;
    break;
  case 2:
    currentPID = &position_pid;
    break;
  case 3:
    currentPID = &yaw_pid;
    break;
  default:
    currentPID = &pitch_pid;
    break;
  }

  SSD1306_GotoXY(0, 0);
  switch (pidType)
  {
  case 0:
    SSD1306_Puts("PID PITCH", &Font_7x10, 1);
    break;
  case 1:
    SSD1306_Puts("PID SPEED", &Font_7x10, 1);
    break;
  case 2:
    SSD1306_Puts("PID POSITION", &Font_7x10, 1);
    break;
  case 3:
    SSD1306_Puts("PID YAW", &Font_7x10, 1);
    break;
  }

  // Kp
  SSD1306_GotoXY(0, 12);
  if (position == 0)
    SSD1306_Puts("->", &Font_7x10, 1);
  SSD1306_GotoXY(15, 12);
  sprintf(str, "Kp = %.2f", currentPID->Kp);
  SSD1306_Puts(str, &Font_7x10, 1);

  // Ki
  SSD1306_GotoXY(0, 24);
  if (position == 1)
    SSD1306_Puts("->", &Font_7x10, 1);
  SSD1306_GotoXY(15, 24);
  sprintf(str, "Ki = %.2f", currentPID->Ki);
  SSD1306_Puts(str, &Font_7x10, 1);

  // Kd
  SSD1306_GotoXY(0, 36);
  if (position == 2)
    SSD1306_Puts("->", &Font_7x10, 1);
  SSD1306_GotoXY(15, 36);
  sprintf(str, "Kd = %.2f", currentPID->Kd);
  SSD1306_Puts(str, &Font_7x10, 1);
}

void DisplayFlashSaveSuccess(void)
{
  SSD1306_Clear();
  SSD1306_GotoXY(20, 16);
  SSD1306_Puts("PID SAVED!", &Font_11x18, 1);
  SSD1306_UpdateScreen();
  osDelay(1000);
}

/* USER CODE BEGIN Header_StartTask04 */
/**
 * @brief Function implementing the ReadMPU6050 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask04 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
