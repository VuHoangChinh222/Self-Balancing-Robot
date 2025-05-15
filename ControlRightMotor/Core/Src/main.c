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
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TaskReceiveCAN */
osThreadId_t TaskReceiveCANHandle;
const osThreadAttr_t TaskReceiveCAN_attributes = {
  .name = "TaskReceiveCAN",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for TaskMControl */
osThreadId_t TaskMControlHandle;
const osThreadAttr_t TaskMControl_attributes = {
  .name = "TaskMControl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TaskReadHall */
osThreadId_t TaskReadHallHandle;
const osThreadAttr_t TaskReadHall_attributes = {
  .name = "TaskReadHall",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* USER CODE BEGIN PV */
// Bien cho truong hop tôi gioi han can bang
typedef enum
{
  NORMAL_OPERATION,
  SAFETY_STOP,
  WAITING_RECOVERY,
  RECOVERY_IN_PROGRESS
} RobotState;

typedef struct
{
  RobotState state;
  uint32_t stopTime;
  uint32_t recoveryStartTime;
  int32_t lastEncoderPosition;
  uint8_t recoveryAttempts;
  float lastStableAngle;
} BalanceStatus;

BalanceStatus robotStatus = {
    .state = NORMAL_OPERATION,
    .stopTime = 0,
    .recoveryStartTime = 0,
    .lastEncoderPosition = 0,
    .recoveryAttempts = 0,
    .lastStableAngle = 0};
volatile uint32_t stuckTimer = 0;
//------------------------------------

PID_t PID_Position, PID_Pitch, PID_SpeedRight;

#define MAX_SPEED 1000
#define MAX_TURN_RATE 0.5f    // He so quay toi da
#define JOYSTICK_SCALE 0.001f // Scale [-1000,1000] -> [-1,1]

// Them PID tam thoi cho khoi dong
PID_t PID_Startup;
uint8_t isBalanced = 0; // Co kiem tra do can bang

// Thoi gian lay mau
float dt = 0.01; // 10ms

// Lam muot PWM
static float lastPWM = 0;
float alpha = 0.0f; // Hệ số làm mượt, 0 < alpha < 1 (alpha cang lon thi lam muot cang manh)

// Cac bien toan cuc de luu trang thai hall sensor
volatile uint8_t currentState = 0;
volatile int8_t direction = 0;
volatile float positionOffset = 0;
volatile uint8_t lastHallState = 0;
volatile int32_t hallCounter = 0;   // dem so buoc (co dau: duong la quay thuan, am la quay nguoc)
volatile float homePosition = 0.0f; // Luu vi tri ban dau sau khi can bang
float anglemotor = 0;
volatile uint8_t u = 0;
volatile uint8_t v = 0;
volatile uint8_t w = 0;
uint8_t checkrunFirtTime=0; //Bien kiem tra lan dau doc Hall sensor
// CAN
uint8_t RxData[4] = {0};
volatile int32_t joyStickX = 0, joyStickY = 0;
CAN_RxHeaderTypeDef RxHeader;
volatile char *command = "";
volatile int16_t angleY = 0;
volatile int16_t value;
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
void AutoBalanceOnStartup(void);
void checkSafetyStop(int16_t Y);
void updateMotors(float PWM);
void controlLoop(void);
// Ham doc trang thai hall sensor
uint8_t readHallState(void);
// Ham xac dinh huong quay cua dong co
int8_t getDirection(uint8_t last, uint8_t current);
// Ham tinh goc quay cua dong co
// Voi gia dinh co 45 buoc tren 1 vong
float getMotorAngle(void);
// Ham nhan tin hieu CAN va xu ly CAN
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
float getMotorSpeed(void);
void PID_Reset(PID_t *pid);
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
  PID_Position.Kp = 5;
  PID_Position.Ki = 0.0;
  PID_Position.Kd = 2.5;
  PID_Pitch.Kp = 10;
  PID_Pitch.Ki = 0.0;
  PID_Pitch.Kd = 4;
  PID_SpeedRight.Kp = 1.5;
  PID_SpeedRight.Ki = 0.0;
  PID_SpeedRight.Kd = 0.05;
  // PID cho giai doan khoi dong
  PID_Startup.Kp = 10;
  PID_Startup.Ki = 0;
  PID_Startup.Kd = 1.2;
  PID_Startup.integral = 0;
  PID_Startup.prevError = 0;
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
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
  canfilterconfig.FilterIdHigh = 0x0000 << 5;
  canfilterconfig.FilterIdLow = 0x0000;
  canfilterconfig.FilterMaskIdHigh = 0x0000 << 5;
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
  htim1.Init.Prescaler = 7200-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
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

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : DIR_Pin */
  GPIO_InitStruct.Pin = DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : HALL_U_Pin HALL_V_Pin HALL_W_Pin */
  GPIO_InitStruct.Pin = HALL_U_Pin|HALL_V_Pin|HALL_W_Pin;
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
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
  {
    switch (RxHeader.StdId)
    {
    case 0x446: // Receive angle data
      // Ghép 2 byte thành số 16-bit
      raw_angle = ((int16_t)RxData[2] << 8) | RxData[3];
      // Chuyển đổi và áp dụng dấu
      angleY = (RxData[1] ? 1 : -1) * raw_angle;
      break;
    case 0x447: // Receive X and partial Y
      // Store X data
      joyStickX = RxData[0] ? ((RxData[1] << 8) | RxData[2]) : -((RxData[1] << 8) | RxData[2]);
      // Store first part of Y
      joyStickY = RxData[3] ? 1 : -1; // Y sign
      break;
    case 0x448: // Receive remaining Y data
      // Store remaining Y data
      joyStickY *= (RxData[0] << 8) | RxData[1];
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
  uint16_t pwmRight = (uint16_t)fminf(fabsf(PWM), MAX_SPEED);

  float rawPWM = pwmRight;
  float smoothPWM = alpha > 0 ? (alpha * rawPWM + (1 - alpha) * lastPWM) : rawPWM;
  lastPWM = smoothPWM;

  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, PWM > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, smoothPWM);
}

void AutoBalanceOnStartup()
{
  const float targetAngle = 0.0; // Goc muc tieu
  const float tolerance = 1.0;   // Sai so cho phep (+-1 do)

  while (!isBalanced) // Vong lap cho den khi can bang
  {
    float y = angleY;
    float error = targetAngle - y;
    float output = PID_Compute(&PID_Startup, targetAngle, y, dt);
    currentState = readHallState();
    direction = getDirection(lastHallState, currentState);
    // Cap nhan hallCounter neu co su thay doi trang thai hop le
    if (direction != 0)
    {
      hallCounter += direction;
      lastHallState = currentState;
    }
    // Cap nhat dong co
    float motor_balance = 1.0 + (y * 0.02); // Hieu chinh theo huong nghien
    updateMotors(output * motor_balance);
    // Kiem tra dieu kien can bang
    if (fabs(error) < tolerance)
    {
      isBalanced = 1;
      homePosition = getMotorAngle();
      break;
    }
    HAL_Delay(5); // Cho 5ms
  }
}

float getMotorSpeed(void)
{
  static int32_t lastHallCounter = 0;
  static uint32_t lastTick = 0;
  uint32_t now = HAL_GetTick();
  int32_t deltaHall = hallCounter - lastHallCounter;
  uint32_t deltaTime = now - lastTick; // ms

  float speed = 0.0f;
  if (deltaTime > 0)
  {
    // Số vòng quay mỗi giây (rps): (deltaHall/45) vòng trong deltaTime ms
    speed = ((float)deltaHall / 45.0f) * (1000.0f / (float)deltaTime); // vòng/giây
  }

  lastHallCounter = hallCounter;
  lastTick = now;
  return speed;
}

void PID_Reset(PID_t *pid)
{
  pid->integral = 0;
  pid->prevError = 0;
  pid->prevMeasurement = 0;
}

void checkSafetyStop(int16_t Y)
{
  /* if (fabs((float)Y) > 40.0 || !isBalanced)
    {
      updateMotors(0);
      isBalanced = 0; // Reset tr?ng th?i c?n b?ng
    }*/
  float currentAngle = (float)angleY;
  uint32_t currentTime = HAL_GetTick();

  switch (robotStatus.state)
  {
  case NORMAL_OPERATION:
    if (fabs(currentAngle) >= 35)
    {
      robotStatus.state = SAFETY_STOP;
      robotStatus.stopTime = currentTime;
      updateMotors(0);
      robotStatus.lastStableAngle = currentAngle;
    }
    break;

  case SAFETY_STOP:
    if (currentTime - robotStatus.stopTime >= 500)
    {
      if (robotStatus.recoveryAttempts < 3)
      {
        robotStatus.state = RECOVERY_IN_PROGRESS;
        robotStatus.recoveryStartTime = currentTime;
        robotStatus.recoveryAttempts++;
        robotStatus.lastEncoderPosition = hallCounter;
      }
    }
    break;

  case RECOVERY_IN_PROGRESS:
    if (fabs(currentAngle) < 2)
    {
      robotStatus.state = NORMAL_OPERATION;
      robotStatus.recoveryAttempts = 0;
      isBalanced = 1;
      homePosition = getMotorAngle();
      // Reset các PID khi phục hồi thành công
      PID_Reset(&PID_Pitch);
      PID_Reset(&PID_SpeedRight);
      PID_Reset(&PID_Position);
    }
    else if (currentTime - robotStatus.recoveryStartTime > 50 &&
             robotStatus.lastEncoderPosition == hallCounter)
    {
      // Phát hiện kẹt - tăng mô-men
      float recoveryPWM = -0.7 * currentAngle;
      updateMotors(recoveryPWM);
      stuckTimer = currentTime;
    }
    break;

  default:
    robotStatus.state = SAFETY_STOP;
    break;
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
  // B?ng chuy?n d?i h?p l? (Forward v� Reverse)
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
  return 0;
}

float getMotorAngle(void)
{
  return ((float)hallCounter / 45.0f) * 360.0f; // Ho?c: hallCounter * 8.0f;
}

void controlLoop(void)
{
  static float speedRight = 0;
  float currentAngle = (float)angleY;

  // Chỉ xử lý điều khiển khi ở trạng thái bình thường hoặc đang phục hồi
  if (robotStatus.state == NORMAL_OPERATION ||
      robotStatus.state == RECOVERY_IN_PROGRESS)
  {

    float forwardInput = joyStickY * JOYSTICK_SCALE;
    float turnInput = -joyStickX * JOYSTICK_SCALE;

    if (robotStatus.state == RECOVERY_IN_PROGRESS)
    {
      // Giảm độ nhạy điều khiển trong quá trình phục hồi
      forwardInput *= 0.5f;
      turnInput *= 0.5f;
    }

    if (fabs(forwardInput) > 0.1f || fabs(turnInput) > 0.1f)
    {
      float targetSpeedRight = forwardInput;
      targetSpeedRight += turnInput * MAX_TURN_RATE;
      targetSpeedRight = fminf(fmaxf(targetSpeedRight, -1.0f), 1.0f);

      float speedRightOutput = PID_Compute(&PID_SpeedRight, targetSpeedRight, speedRight, dt);
      float balanceTorque = PID_Compute(&PID_Pitch, 0.0f, currentAngle, dt);

      // Điều chỉnh mô-men trong quá trình phục hồi
      if (robotStatus.state == RECOVERY_IN_PROGRESS)
      {
        balanceTorque *= 0.7;
      }

      float pwmRight = balanceTorque + speedRightOutput * MAX_SPEED;
      speedRight = getMotorSpeed();
      updateMotors(pwmRight);
    }
    else
    {
      float positionError = homePosition - positionOffset;
      float balanceTorque = PID_Compute(&PID_Pitch, 0.0f, currentAngle, dt);
      float stabilizeTorque = PID_Compute(&PID_Position, 0.0f, positionError, dt);

      if (robotStatus.state == RECOVERY_IN_PROGRESS)
      {
        balanceTorque *= 0.7;
        stabilizeTorque *= 0.7;
      }

      float pwm = balanceTorque + stabilizeTorque;
      updateMotors(pwm);
    }
  }

  // // Kiểm tra an toàn sau mỗi chu kỳ điều khiển
  // checkSafetyStop(angleY);
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
    checkSafetyStop(angleY);
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
    if (!isBalanced)
    {
      // T? d?ng c�n b?ng l?n d?u
      AutoBalanceOnStartup();
    }
    else
    {
      controlLoop();
    }
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
    // �?c hall sensor
    currentState = readHallState();
    if(checkrunFirtTime==0){
			lastHallState = currentState;
			checkrunFirtTime=1;
		}else
			direction=getDirection(lastHallState,currentState);
    if (direction != 0)
    {
      hallCounter += direction;
      lastHallState = currentState;
    }
    positionOffset = getMotorAngle();
    osDelay(1);
  }
  /* USER CODE END ReadHall */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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

#ifdef  USE_FULL_ASSERT
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
