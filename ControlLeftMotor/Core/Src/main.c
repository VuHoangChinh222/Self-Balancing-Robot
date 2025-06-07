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
#include "string.h"
#include "math.h"
#include "pid.h"
#include "mpu6050.h"
#include "flashStorage.h"
#include <stdio.h>
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

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for TaskReceiveCAN */
osThreadId_t TaskReceiveCANHandle;
const osThreadAttr_t TaskReceiveCAN_attributes = {
    .name = "TaskReceiveCAN",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityRealtime,
};
/* Definitions for TaskMControl */
osThreadId_t TaskMControlHandle;
const osThreadAttr_t TaskMControl_attributes = {
    .name = "TaskMControl",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for TaskReadHall */
osThreadId_t TaskReadHallHandle;
const osThreadAttr_t TaskReadHall_attributes = {
    .name = "TaskReadHall",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityRealtime,
};
/* USER CODE BEGIN PV */

//------------------------------------
// MPU9250
MPU6050_t MPU6050;
volatile float angleY = 0; // pitch
volatile float angleZ = 0; // yaw
volatile float gyroY = 0;  // Gy MPU6050 (don vi: deg/s)
volatile float gyroZ = 0;  // Gz
// PID
PID_t position_pid = {0};
PID_t speed_pid = {0};
PID_t pitch_pid = {0};
PID_t yaw_pid = {0};

#define CONTROL_DT 0.001f // 1 ms
#define MAX_SPEED_COMMAND 100.0f
#define JOYSTICK_DEADZONE 5.0f
//--------------------------------------
#define MAX_TILT_ANGLE 50.0f    // Góc nghiêng tối đa cho phép
#define MAX_MOVEMENT_TILT 15.0f // Góc nghiêng tối đa cho di chuyển
#define JOYSTICK_TO_TILT 0.15f  // Hệ số chuyển đổi joystick sang góc nghiêng
//--------------------------------------
#define MAX_GYRO_RATE 150.0f   // Giới hạn tốc độ góc tối đa
#define RECOVERY_KP_SCALE 0.7f // Hệ số giảm Kp khi phát hiện giật
#define RECOVERY_KD_SCALE 1.5f // Hệ số tăng Kd khi phát hiện giật
//--------------------------------------
#define BASE_PWM_FILTER_ALPHA 0.6f // Hệ số làm mượt cơ bản
#define FAST_PWM_FILTER_ALPHA 0.9f // Hệ số làm mượt khi góc lớn
#define ANGLE_THRESHOLD 5.0f       // Ngưỡng góc để thay đổi hệ số
#define MAX_PWM_CHANGE 30.0f       // Giới hạn thay đổi PWM mỗi chu kỳ
//-----------------------------------
#define MAX_PWM_OUTPUT 999.0f
#define COUNTS_PER_REV 90.0f         // 90 bước tương ứng 1 vòng quay
volatile float motorSpeedRPS = 0.0f; // tốc độ động cơ, revs per second
volatile float motorSpeedDPS = 0.0f; // tốc độ động cơ, degrees per second
//  == == PWM State == ==
float pwm_left_filtered = 0.0f;
// Cac bien toan cuc de luu trang thai hall sensor
volatile uint8_t currentState = 0;  // Trang thai hien tai cua hall sensor
volatile int8_t direction = 0;      // Huong quay cua dong co
volatile float positionOffset = 0;  // Vi tri offset
volatile uint8_t lastHallState = 0; // Trang thai hall sensor cu
volatile int32_t hallCounter = 0;   // dem so buoc (co dau: duong la quay thuan, am la quay nguoc)
float anglemotor = 0;               // Goc quay cua dong co
volatile uint8_t u = 0;
volatile uint8_t v = 0;
volatile uint8_t w = 0;
uint8_t checkrunFirtTime = 0; // Bien kiem tra lan dau doc Hall sensor
// CAN
uint8_t RxData[8] = {0};
volatile int16_t joyStickX = 0, joyStickY = 0;
CAN_RxHeaderTypeDef RxHeader;
volatile char *command = "";
// Flash danh dau PID da nhan du
typedef struct
{
  uint8_t hasKpKi;
  uint8_t hasKd;
} PID_Receive_Flags_t;
PID_Receive_Flags_t pidFlags[4] = {0}; // 0: pitch, 1: speed, 2: position, 3: yaw
// UART
uint8_t txBuffer1[50], txBuffer2[50];
volatile uint8_t activeBuffer = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);
void ReceiveCAN(void *argument);
void MotorControl(void *argument);
void ReadHall(void *argument);

/* USER CODE BEGIN PFP */
void updateMotors(float PWM);
void controlLoop(void);
// Ham doc trang thai hall sensor: ghep 3 tin hieu thanh 1 gia tri 3-bit
uint8_t readHallState(void);
// Ham xac dinh huong quay dua vao su chuyen doi trang thai
// Gia su cac trang thai hop le la: 101 (5), 100 (4), 110 (6), 010 (2), 011 (3), 001 (1)
// Forward sequence (quay thuan): 5 -> 4 -> 6 -> 2 -> 3 -> 1 -> 5...
// Reverse sequence (quay nguoc): 5 -> 1 -> 3 -> 2 -> 6 -> 4 -> 5...
int8_t getDirection(uint8_t last, uint8_t current);
// Ham tinh goc quay cua dong co
// Voi gia dinh: 45 bu?c (transition) tuong duong 1 vong quay => moi buoc = 8 do
float getMotorAngle(void);
// Ham nhan tin hieu CAN va xu ly CAN
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
// ==== PWM Smoothing ====
float SmoothPWM(float prev, float target);
// ==== Clamp PWM Output ====
float ClampPWM(float pwm);
void ComputeMotorSpeed(void);
int float_to_str(float value, uint8_t *buffer);
void send_imu_data(float ay, float az, float gy, float gz);
void TryWritePIDToFlashIfReady(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_CAN_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  MPU6050_Init(&hi2c1, 2, 2, 0x03);
  HAL_Delay(100);

  // Bật chế độ Master (nếu dùng đọc từ HMC5883L qua MPU6050)
  MPU6050_Master(&hi2c1);

  // Setup HMC5883L thông qua MPU6050 (MPU6050 làm master, HMC là slave)
  HMC5883L_Setup(&hi2c1);

  // Setup đọc dữ liệu từ HMC5883L qua MPU6050
  MPU6050_Slave_Read(&hi2c1);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  // Doc thong so PID tu bo nho flash
  if (Flash_Read_PID(&position_pid, &speed_pid, &pitch_pid, &yaw_pid) != HAL_OK)
  {
    // Nếu chưa có dữ liệu hợp lệ thì dùng giá trị mặc định
    Flash_Load_Default_PID(&position_pid, &speed_pid, &pitch_pid, &yaw_pid);

    // Và ghi giá trị mặc định đó vào Flash
    Flash_Write_PID(&position_pid, &speed_pid, &pitch_pid, &yaw_pid);
  }
  // Bat yeu cau ngat khi du lieu CAN nhan ve duoc luu tru goi tin o FIFO1 khi dung ID
  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  // Enable CAN RX interrupt
  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  // UART
  HAL_UART_Init(&huart2);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
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

  /* creation of TaskReceiveCAN */
  TaskReceiveCANHandle = osThreadNew(ReceiveCAN, NULL, &TaskReceiveCAN_attributes);

  /* creation of TaskMControl */
  TaskMControlHandle = osThreadNew(MotorControl, NULL, &TaskMControl_attributes);

  /* creation of TaskReadHall */
  TaskReadHallHandle = osThreadNew(ReadHall, NULL, &TaskReadHall_attributes);

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
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
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
  canfilterconfig.FilterBank = 0;
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfilterconfig.FilterIdHigh = 0x0000 << 5; // Chap nhan tat ca cac ID
  canfilterconfig.FilterIdLow = 0x0000;
  canfilterconfig.FilterMaskIdHigh = 0x0000 << 5; // Chap nhan tat ca cac ID
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 0;

  HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);
  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
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
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1800 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000 - 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);
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
  huart2.Init.BaudRate = 1000000;
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
  huart2.Instance->CR3 |= USART_CR3_DMAT;
  /* USER CODE END USART2_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
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
  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BREAK_GPIO_Port, BREAK_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_Pin BREAK_Pin */
  GPIO_InitStruct.Pin = DIR_Pin | BREAK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : HALL_U_Pin */
  GPIO_InitStruct.Pin = HALL_U_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(HALL_U_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : HALL_V_Pin HALL_W_Pin */
  GPIO_InitStruct.Pin = HALL_V_Pin | HALL_W_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  int16_t rawjoyStickX = 0;
  int16_t rawjoyStickY = 0;
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
  {
    switch (RxHeader.StdId)
    {
    case 0x447: // Nhan gia tri joystick X
      rawjoyStickX = RxData[0] ? (((int16_t)RxData[1] << 8) | RxData[2]) : -(((int16_t)RxData[1] << 8) | RxData[2]);
      joyStickX = (rawjoyStickX > 100 || rawjoyStickX < -100) ? 0 : rawjoyStickX;
      break;
    case 0x448: // Nhan gia tri joystick Y
      rawjoyStickY = RxData[0] ? (((int16_t)RxData[1] << 8) | RxData[2]) : -(((int16_t)RxData[1] << 8) | RxData[2]);
      joyStickY = (rawjoyStickY > 100 || rawjoyStickY < -100) ? 0 : rawjoyStickY;
      break;
    case 0x500: // Pitch Kp/Ki
      memcpy(&pitch_pid.Kp, &RxData[0], sizeof(float));
      memcpy(&pitch_pid.Ki, &RxData[4], sizeof(float));
      pidFlags[0].hasKpKi = 1;
      TryWritePIDToFlashIfReady();
      break;

    case 0x501: // Speed Kp/Ki
      memcpy(&speed_pid.Kp, &RxData[0], sizeof(float));
      memcpy(&speed_pid.Ki, &RxData[4], sizeof(float));
      pidFlags[1].hasKpKi = 1;
      TryWritePIDToFlashIfReady();
      break;

    case 0x502: // Position Kp/Ki
      memcpy(&position_pid.Kp, &RxData[0], sizeof(float));
      memcpy(&position_pid.Ki, &RxData[4], sizeof(float));
      pidFlags[2].hasKpKi = 1;
      TryWritePIDToFlashIfReady();
      break;

    case 0x503: // Yaw Kp/Ki
      memcpy(&yaw_pid.Kp, &RxData[0], sizeof(float));
      memcpy(&yaw_pid.Ki, &RxData[4], sizeof(float));
      pidFlags[3].hasKpKi = 1;
      TryWritePIDToFlashIfReady();
      break;

    case 0x510: // Pitch Kd
      memcpy(&pitch_pid.Kd, &RxData[0], sizeof(float));
      pidFlags[0].hasKd = 1;
      TryWritePIDToFlashIfReady();
      break;

    case 0x511: // Speed Kd
      memcpy(&speed_pid.Kd, &RxData[0], sizeof(float));
      pidFlags[1].hasKd = 1;
      TryWritePIDToFlashIfReady();
      break;

    case 0x512: // Position Kd
      memcpy(&position_pid.Kd, &RxData[0], sizeof(float));
      pidFlags[2].hasKd = 1;
      TryWritePIDToFlashIfReady();
      break;

    case 0x513: // Yaw Kd
      memcpy(&yaw_pid.Kd, &RxData[0], sizeof(float));
      pidFlags[3].hasKd = 1;
      TryWritePIDToFlashIfReady();
      break;
    default:
      Error_Handler();
      break;
    }
  }
  else
  {
    Error_Handler();
  }
}

void TryWritePIDToFlashIfReady(void)
{
  for (int i = 0; i < 4; i++)
  {
    if (!pidFlags[i].hasKpKi || !pidFlags[i].hasKd)
      return; // chưa nhận đủ PID
  }

  // Đã đủ → ghi vào Flash
  Flash_Write_PID(&position_pid, &speed_pid, &pitch_pid, &yaw_pid);

  // Reset cờ
  for (int i = 0; i < 4; i++)
  {
    pidFlags[i].hasKpKi = 0;
    pidFlags[i].hasKd = 0;
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    //    HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
    //		HAL_Delay(100);
  }
}

int float_to_str(float value, uint8_t *buffer)
{
  uint8_t *ptr = buffer;

  if (value < 0)
  {
    *ptr++ = '-';
    value = -value;
  }

  int integer = (int)value;
  int fraction = (int)((value - integer) * 1000 + 0.5f);

  if (integer >= 100)
  {
    *ptr++ = '0' + integer / 100;
    integer %= 100;
  }
  *ptr++ = '0' + integer / 10;
  *ptr++ = '0' + integer % 10;
  *ptr++ = '.';
  *ptr++ = '0' + fraction / 100;
  *ptr++ = '0' + (fraction / 10) % 10;
  *ptr++ = '0' + fraction % 10;

  return ptr - buffer;
}

void send_imu_data(float ay, float az, float gy, float gz)
{
  static char uart_buffer[50] = "0"; // Static buffer to avoid stack issues
  int len = 0;

  // Format theo đúng mẫu "ay:12.345,az:23.456,gy:34.567,gz:45.678\n"
  len = sprintf(uart_buffer, "ay:%.3f,az:%.3f,gy:%.3f,gz:%.3f\n",
                ay, az, gy, gz);

  // Kiểm tra trạng thái UART và DMA
  if (huart2.gState == HAL_UART_STATE_READY &&
      huart2.hdmatx->State == HAL_DMA_STATE_READY)
  {
    if (HAL_UART_Transmit_DMA(&huart2, (uint8_t *)uart_buffer, len) != HAL_OK)
    {
      // Fallback to normal transmission if DMA fails
      HAL_UART_Transmit(&huart2, (uint8_t *)uart_buffer, len, 10);
    }
  }
}

void updateMotors(float PWM)
{
  float target_abs_pwm = fminf(fabsf(PWM), MAX_PWM_OUTPUT);
  if (target_abs_pwm >= -1 && target_abs_pwm <= 1)
  {
    HAL_GPIO_WritePin(BREAK_GPIO_Port, BREAK_Pin, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(BREAK_GPIO_Port, BREAK_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, PWM > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)fabsf(target_abs_pwm));
  }
}

uint8_t readHallState(void)
{
  // Doc trang thai hall sensor
  // Ghep 3 trang thai thanh 1 gia tri 3-bit
  u = HAL_GPIO_ReadPin(HALL_U_GPIO_Port, HALL_U_Pin);
  v = HAL_GPIO_ReadPin(HALL_V_GPIO_Port, HALL_V_Pin);
  w = HAL_GPIO_ReadPin(HALL_W_GPIO_Port, HALL_W_Pin);
  return (u << 2) | (v << 1) | w;
}

int8_t getDirection(uint8_t last, uint8_t current)
{
  // Bang chuyen doi trang thai hop le
  // Gia su cac trang thai hop le la: 101 (5), 100 (4), 110 (6), 010 (2), 011 (3), 001 (1)
  const uint8_t validTransitions[12][2] = {
      {5, 4}, {4, 6}, {6, 2}, {2, 3}, {3, 1}, {1, 5}, // Forward
      {5, 1},
      {1, 3},
      {3, 2},
      {2, 6},
      {6, 4},
      {4, 5} // Reverse
  };
  for (int i = 0; i < 12; i++)
  {
    if (last == validTransitions[i][0] && current == validTransitions[i][1])
    {
      return (i < 6) ? 1 : -1;
    }
  }
  return 0; // Trang thai khong hop le
  //	return current!= last ? 1:0;
}

float getMotorAngle(void)
{
  return ((float)hallCounter / 90.0f) * 360.0f; // Hoac: hallCounter * 8.0f;
}

// ==== Clamp PWM Output ====
float ClampPWM(float pwm)
{
  if (pwm > MAX_PWM_OUTPUT)
    return MAX_PWM_OUTPUT;
  if (pwm < -MAX_PWM_OUTPUT)
    return -MAX_PWM_OUTPUT;
  return pwm;
}

// ==== PWM Smoothing ====
float SmoothPWM(float prev, float target)
{
  float currentAngle = angleY;
  float currentGyro = gyroY;
  float alpha = 0;
  float max_change = 0;

  // Điều chỉnh giới hạn thay đổi PWM theo góc nghiêng
  if (fabsf(currentAngle) > 30.0f)
  {
    max_change = MAX_PWM_CHANGE * 4.0f; // Tăng giới hạn nhiều hơn
    alpha = 1.0f;                       // Không làm mượt ở góc lớn
  }
  else if (fabsf(currentAngle) > 15.0f)
  {
    max_change = MAX_PWM_CHANGE * 3.0f;
    alpha = FAST_PWM_FILTER_ALPHA;
  }
  else if (fabsf(currentAngle) > 5.0f)
  {
    max_change = MAX_PWM_CHANGE * 2.0f;
    alpha = (BASE_PWM_FILTER_ALPHA + FAST_PWM_FILTER_ALPHA) / 2;
  }
  else
  {
    max_change = MAX_PWM_CHANGE;
    alpha = BASE_PWM_FILTER_ALPHA;
  }

  float delta = target - prev;
  if (delta > max_change)
    delta = max_change;
  if (delta < -max_change)
    delta = -max_change;

  return prev + alpha * delta;
}
/* Hàm tính tốc độ động cơ — gọi mỗi khi controlLoop chạy */
void ComputeMotorSpeed(void)
{
  static int32_t lastHallCount = 0;
  int32_t deltaCount = hallCounter - lastHallCount;

  // 1) Tính rev/s (revolutions per second)
  motorSpeedRPS = (deltaCount / COUNTS_PER_REV) / CONTROL_DT;

  // 2) Nếu muốn độ/giây (degrees per second)
  motorSpeedDPS = (deltaCount * 360.0f / COUNTS_PER_REV) / CONTROL_DT;

  // Cập nhật cho lần sau
  lastHallCount = hallCounter;
}
void controlLoop(void)
{
  float currentAngle = angleY;
  float currentGyro = gyroY;
  ComputeMotorSpeed();

  // Kiểm tra góc an toàn
  if (fabsf(currentAngle) < MAX_TILT_ANGLE)
  {
    float speed_cmd = 0;   // Tốc độ mong muốn từ PID tốc độ
    float balance_cmd = 0; // Moment cân bằng từ PID góc
    float pwm_output = 0;  // PWM cuối cùng

    static int32_t initialPosition = 0;
    static uint8_t isHolding = 0;
    static uint8_t isInitialized = 0;
    int32_t currentSpeed = motorSpeedRPS * COUNTS_PER_REV;

    // 1. Điều khiển vị trí (tầng ngoài nhất)
    if (fabsf((float_t)joyStickY) > JOYSTICK_DEADZONE)
    {
      // Chế độ điều khiển tốc độ từ joystick
      float velocity_target = (joyStickY / 100.0f) * MAX_SPEED_COMMAND;
      speed_cmd = PID_Compute(&speed_pid, velocity_target, currentSpeed, CONTROL_DT);
      isHolding = 0;
      isInitialized = 0;
    }
    else if (fabsf(currentAngle) < 15.0f)
    {
      // Chế độ giữ vị trí
      if (!isInitialized)
      {
        initialPosition = hallCounter;
        isInitialized = 1;
        isHolding = 1;
        PID_Reset(&position_pid);
        PID_Reset(&speed_pid);
      }

      if (isHolding)
      {
        // PID vị trí -> tốc độ mong muốn
        float position_cmd = PID_Compute(&position_pid, initialPosition, hallCounter, CONTROL_DT);
        position_cmd = fminf(fmaxf(position_cmd, -30.0f), 30.0f);

        // PID tốc độ -> moment cần thiết
        speed_cmd = PID_Compute(&speed_pid, position_cmd, currentSpeed, CONTROL_DT);
        speed_cmd = fminf(fmaxf(speed_cmd, -30.0f), 30.0f);
      }
    }

    // 2. Điều khiển cân bằng (tầng trong cùng)
    //    float base_Kp = 24.0f;
    //    float base_Kd = 0.8f;
    float base_Kp = pitch_pid.Kp;
    float base_Kd = pitch_pid.Kd;
    float Kp = base_Kp;
    float Kd = base_Kd;

    // Điều chỉnh hệ số theo góc nghiêng
    if (fabsf(currentAngle) > 20.0f)
    {
      Kp = base_Kp * 1.5f; // Tăng mạnh khi góc lớn
      Kd = base_Kd * 1.3f;
    }
    else if (fabsf(currentAngle) > 10.0f)
    {
      Kp = base_Kp * 1.2f;
      Kd = base_Kd * 1.1f;
    }
    else if (fabsf(currentAngle) < 5.0f)
    {
      Kp = base_Kp * 0.8f; // Giảm khi gần cân bằng
      Kd = base_Kd * 1.2f;
    }

    // Xử lý chuyển động đột ngột
    if (fabsf(currentGyro) > MAX_GYRO_RATE)
    {
      Kp *= RECOVERY_KP_SCALE;
      Kd *= RECOVERY_KD_SCALE;
    }

    // Tính toán moment cân bằng
    float target_angle = 0;
    if (fabsf((float_t)joyStickY) > JOYSTICK_DEADZONE)
    {
      target_angle = -(joyStickY / 100.0f) * MAX_MOVEMENT_TILT;
    }
    float angleError = currentAngle - target_angle;
    balance_cmd = Kp * angleError + Kd * currentGyro;

    // 3. Tổng hợp các thành phần điều khiển
    float balance_weight = 1.0f;
    float speed_weight = 0.3f;
    pwm_output = -balance_weight * balance_cmd + speed_weight * speed_cmd;

    // 4. Xử lý quay (turning)
    if (fabsf((float_t)joyStickX) <= 100)
    {
      float base_turn_power = 200.0f;
      float angle_factor = 1.0f - (fabsf(currentAngle) / MAX_TILT_ANGLE);
      float turn_cmd = (joyStickX / 100.0f) * base_turn_power * angle_factor;
      pwm_output += turn_cmd;
    }

    // 5. Giới hạn và làm mượt PWM
    pwm_output = ClampPWM(pwm_output);
    pwm_left_filtered = SmoothPWM(pwm_left_filtered, pwm_output);
    pwm_left_filtered = ClampPWM(pwm_left_filtered);

    // 6. Xuất PWM ra động cơ
    updateMotors(pwm_left_filtered);
  }
  else
  {
    // Dừng động cơ khi vượt quá góc an toàn
    updateMotors(0);
    pwm_left_filtered = 0;
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

/* USER CODE BEGIN Header_ReceiveCAN */
/**
 * @brief Function implementing the TaskReceiveCAN thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_ReceiveCAN */
void ReceiveCAN(void *argument)
{
  /* USER CODE BEGIN ReceiveCAN */
  /* Infinite loop */
  for (;;)
  {
    MPU6050_Read_All(&hi2c1, &MPU6050);
    MPU6050_Parsing(&MPU6050);
    angleY = MPU6050.Pitch;
    angleZ = MPU6050.Yaw;
    gyroY = MPU6050.Gy;
    gyroZ = MPU6050.Gz;
    send_imu_data(angleY, angleZ, gyroY, gyroZ);
    osDelay(1);
  }
  /* USER CODE END ReceiveCAN */
}

/* USER CODE BEGIN Header_MotorControl */
/**
 * @brief Function implementing the TaskMControl thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_MotorControl */
void MotorControl(void *argument)
{
  /* USER CODE BEGIN MotorControl */
  /* Infinite loop */
  for (;;)
  {
    controlLoop();
    osDelay(1);
  }
  /* USER CODE END MotorControl */
}

/* USER CODE BEGIN Header_ReadHall */
/**
 * @brief Function implementing the TaskReadHall thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_ReadHall */
void ReadHall(void *argument)
{
  /* USER CODE BEGIN ReadHall */
  /* Infinite loop */
  for (;;)
  {
    // Doc trang thai hall sensor
    currentState = readHallState();
    if (checkrunFirtTime == 0)
    {
      lastHallState = currentState;
      checkrunFirtTime = 1;
    }
    else
      direction = getDirection(lastHallState, currentState);
    if (direction != 0)
    {
      hallCounter += direction;
      lastHallState = currentState;
    }
    // Cap nhat vi tri dong co
    positionOffset = getMotorAngle(); // Gia tri do duoc tu hall sensor
                                      // Tinh sai so vi tri
    osDelay(1);
  }
  /* USER CODE END ReadHall */
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
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_Delay(100);
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