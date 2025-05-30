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

TIM_HandleTypeDef htim1;

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

PID_t PID_Position, PID_SpeedLeft, PID_Pitch, PID_Rotate;

#define CONTROL_DT 0.005f // 10 ms
#define JOYSTICK_DEADZONE 5.0f
//--------------------------------------
#define MAX_GYRO_RATE 150.0f // Giới hạn tốc độ góc tối đa
//--------------------------------------
#define BASE_PWM_FILTER_ALPHA 0.3f // Hệ số làm mượt cơ bản
#define FAST_PWM_FILTER_ALPHA 0.5f // Hệ số làm mượt khi góc lớn
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
volatile uint8_t u = 0;
volatile uint8_t v = 0;
volatile uint8_t w = 0;
uint8_t checkrunFirtTime = 0; // Bien kiem tra lan dau doc Hall sensor
// CAN
uint8_t RxData[4] = {0};
volatile int32_t joyStickX = 0, joyStickY = 0;
CAN_RxHeaderTypeDef RxHeader;
volatile char *command = "";
volatile float angleY = 0;
volatile float gyroY = 0; // Gy MPU6050 (don vi: deg/s)
osMutexId_t joystickXMutexHandle, joystickYMutexHandle;
//-------------------------------------
uint8_t checkBalanceAfterTurnOn = 0; // Bien kiem tra can bang sau khi bat nguon
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM1_Init(void);
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
float GetNonlinearKp(float angle);
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
  MX_CAN_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  // PID cho vi tri
  PID_Position.Kp = 0.0;
  PID_Position.Ki = 0.0;
  PID_Position.Kd = 0.0;
  //  // PID cho toc do dong co trai dua vao joystick Y
  //  PID_SpeedLeft.Kp = 0.15;
  //  PID_SpeedLeft.Ki = 0.001;
  //  PID_SpeedLeft.Kd = 0.05;
  //  // PID can bang
  //  PID_Pitch.Kp = 16.0;
  //  PID_Pitch.Ki = 0.0;
  //  PID_Pitch.Kd = 1.5;
  //  // PID xoay
  //  PID_Rotate.Kp = 1.0;
  //  PID_Rotate.Ki = 0.0;
  //  PID_Rotate.Kd = 0.05;
  // PID cho toc do dong co trai dua vao joystick Y
  PID_SpeedLeft.Kp = 0.0;
  PID_SpeedLeft.Ki = 0.0;
  PID_SpeedLeft.Kd = 0.0;
  // PID can bang
  PID_Pitch.Kp = 13.5;
  PID_Pitch.Ki = 0.0;
  PID_Pitch.Kd = 0.1;
  // PID xoay
  PID_Rotate.Kp = 0.0;
  PID_Rotate.Ki = 0.0;
  PID_Rotate.Kd = 0.0;
  // Bat yeu cau ngat khi du lieu CAN nhan ve duoc luu tru goi tin o FIFO1 khi dung ID
  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  // Enable CAN RX interrupt
  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  joystickXMutexHandle = osMutexNew(NULL);
  if (joystickXMutexHandle == NULL)
  {
    Error_Handler();
  }
  joystickYMutexHandle = osMutexNew(NULL);
  if (joystickYMutexHandle == NULL)
  {
    Error_Handler();
  }
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
  hcan.Init.TimeSeg1 = CAN_BS1_6TQ;
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
  htim1.Init.Prescaler = 3600 - 1;
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

  /*Configure GPIO pins : HALL_U_Pin HALL_V_Pin HALL_W_Pin */
  GPIO_InitStruct.Pin = HALL_U_Pin | HALL_V_Pin | HALL_W_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  int16_t raw_angle = 0;
  int16_t raw_gyroY = 0;
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
  {
    switch (RxHeader.StdId)
    {
    case 0x446: // Nhan goc Y
      // Ghep 2 byte thanh 1 gia tri 16 bit
      raw_angle = ((int16_t)RxData[2] << 8) | RxData[3];
      // Chuyen doi goc Y thanh goc thuc te co dau
      raw_angle = (RxData[1] ? 1 : -1) * raw_angle;
      if (raw_angle > 13000 || raw_angle < -13000)
        raw_angle = 0;
      angleY = (float)raw_angle / 100;
      break;
    case 0x447: // Nhan gia tri joystick X
      // Luu tru gia tri joystick X
      if (RxData[0] != 22)
      {
        joyStickX = 0;
      }
      else
      {
        osMutexAcquire(joystickXMutexHandle, osWaitForever);
        int16_t rawjoyStickX = RxData[1] ? (((int16_t)RxData[2] << 8) | RxData[3]) : -(((int16_t)RxData[2] << 8) | RxData[3]);
        joyStickX = (rawjoyStickX > 100 || rawjoyStickX < -100) ? 0 : rawjoyStickX;
        osMutexRelease(joystickXMutexHandle);
      }
      break;
    case 0x448: // Nhan gia tri joystick Y
      if (RxData[0] != 32)
      {
        joyStickY = 0;
      }
      else
      {
        osMutexAcquire(joystickYMutexHandle, osWaitForever);
        int16_t rawjoyStickY = RxData[1] ? (((int16_t)RxData[2] << 8) | RxData[3]) : -(((int16_t)RxData[2] << 8) | RxData[3]);
        joyStickY = (rawjoyStickY > 100 || rawjoyStickY < -100) ? 0 : rawjoyStickY;
        osMutexRelease(joystickYMutexHandle);
      }
      break;
    case 0x449: // Nhan gia tri gyro Y
      raw_gyroY = ((int16_t)RxData[2] << 8) | RxData[3];
      raw_gyroY = (RxData[1] ? 1 : -1) * raw_gyroY; // Chuyen doi goc Y thanh goc thuc te co dau
      gyroY = (float)raw_gyroY / 100;
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

void updateMotors(float PWM)
{
  if ((PWM >= -2 && PWM <= 2) || angleY == 0)
  {
    HAL_GPIO_WritePin(BREAK_GPIO_Port, BREAK_Pin, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(BREAK_GPIO_Port, BREAK_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, PWM >= 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)fabsf(PWM));
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

  // Tính vòng/giây (RPS - Revolutions Per Second)
  // deltaCount: số xung trong 10ms (CONTROL_DT = 0.01s)
  // deltaCount * 100: số xung trong 1 giây
  // COUNTS_PER_REV = 90 xung/vòng
  motorSpeedRPS = ((float)deltaCount * (1.0f / CONTROL_DT)) / COUNTS_PER_REV;

  // Tính độ/giây (DPS - Degrees Per Second)
  motorSpeedDPS = motorSpeedRPS * 360.0f;

  // Cập nhật cho lần sau
  lastHallCount = hallCounter;
}

float GetNonlinearKp(float angle)
{
  float base_Kp = 20;
  float scale = 1;

  if (fabsf(angle) < 5.0f)
  {
    // Góc nhỏ: tăng Kp để có đủ lực
    scale = 0.8f;
  }
  return base_Kp * scale;
}

void controlLoop(void)
{
  float pwm_output = 0.0f; // Bien luu tru gia tri PWM dau ra
  float currentAngle = angleY;
  float currentGyro = gyroY;
  ComputeMotorSpeed();
  float target_speed = 0.0f; // Toc do muc tieu duoc tinh toan tu joystick Y
  float balance_cmd = 0.0f;  // Lenh can bang (balance command)
  float target_angle = 0.0f; // Goc muc tieu duoc tinh toan tu PID_SpeedLeft
  float turn_cmd = 0.0f;     // Tinh toan lenh xoay (turn command)

  if (checkBalanceAfterTurnOn && currentAngle >= -45 && currentAngle <= 45)
  {
    // 1. Lay tin hieu tu joystick Y
    if (fabsf((float_t)joyStickY) > JOYSTICK_DEADZONE && fabsf((float_t)joyStickY) <= 100)
    {
      target_speed = joyStickY; // Chuyển đổi tín hiệu joystick sang tốc độ mục tiêu
      target_angle = PID_Compute(&PID_SpeedLeft, target_speed, motorSpeedRPS, CONTROL_DT);
    }

    // 2. Tinh toan lenh can bang
    float dynamic_Kp = GetNonlinearKp(currentAngle);
    PID_Pitch.Kp = dynamic_Kp;
    balance_cmd = PID_Compute(&PID_Pitch, target_angle, currentAngle, CONTROL_DT);

    // 3. Xử lý quay (turning) dựa vào joyStickX
    if (fabsf((float_t)joyStickX) > JOYSTICK_DEADZONE && fabsf((float_t)joyStickX) <= 100)
    {
      turn_cmd = PID_Compute(&PID_Rotate, joyStickX, 0.0f, CONTROL_DT);
    }

    // 4. Tong hop lenh can bang va lenh xoay
    pwm_output = balance_cmd + turn_cmd;

    // 5. Gioi han va lam muot PWM
    pwm_output = ClampPWM(pwm_output);
    //    pwm_left_filtered = SmoothPWM(pwm_left_filtered, pwm_output);
    //    pwm_left_filtered = ClampPWM(pwm_left_filtered);

    // 6. Xuat PWM ra dong co
    updateMotors(pwm_output);
  }
  else if (!checkBalanceAfterTurnOn)
  {
    if (currentAngle >= -2 && currentAngle <= 2)
    {
      checkBalanceAfterTurnOn = 1; // Bat dau can bang khi goc trong khoang -2 den 2 do
      return;
    }
    // 1. Lay tin hieu tu joystick Y
    if (fabsf((float_t)joyStickY) > JOYSTICK_DEADZONE && fabsf((float_t)joyStickY) <= 100)
    {
      target_speed = joyStickY; // Chuyển đổi tín hiệu joystick sang tốc độ mục tiêu
      target_angle = PID_Compute(&PID_SpeedLeft, target_speed, motorSpeedRPS, CONTROL_DT);
    }

    // 2. Tinh toan lenh can bang
    balance_cmd = PID_Compute(&PID_Pitch, target_angle, currentAngle, CONTROL_DT);

    //		// Thêm giới hạn tốc độ thay đổi moment
    //		static float last_balance_cmd = 0;
    //		float max_cmd_change = 100.0f;  // Điều chỉnh giá trị này nếu cần

    //		if (balance_cmd - last_balance_cmd > max_cmd_change) {
    //			balance_cmd = last_balance_cmd + max_cmd_change;
    //		}
    //		else if (balance_cmd - last_balance_cmd < -max_cmd_change) {
    //			balance_cmd = last_balance_cmd - max_cmd_change;
    //		}

    //		last_balance_cmd = balance_cmd;

    // 3. Xử lý quay (turning) dựa vào joyStickX
    if (fabsf((float_t)joyStickX) > JOYSTICK_DEADZONE && fabsf((float_t)joyStickX) <= 100)
    {
      turn_cmd = PID_Compute(&PID_Rotate, joyStickX, 0.0f, CONTROL_DT);
    }

    // 4. Tong hop lenh can bang va lenh xoay
    pwm_output = balance_cmd + turn_cmd;

    // 5. Gioi han va lam muot PWM
    pwm_output = ClampPWM(pwm_output);
    //    pwm_left_filtered = SmoothPWM(pwm_left_filtered, pwm_output);
    //    pwm_left_filtered = ClampPWM(pwm_left_filtered);

    // 6. Xuat PWM ra dong co
    updateMotors(pwm_output);
  }
  else
  {
    // Dừng động cơ khi vượt quá góc an toàn
    updateMotors(0);
    pwm_left_filtered = 0;
    PID_Reset(&PID_Position);
    PID_Reset(&PID_SpeedLeft);
    PID_Reset(&PID_Pitch);
    PID_Reset(&PID_Rotate);
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
    osDelay(5);
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
