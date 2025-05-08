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
PID_t PID_Position, PID_Pitch,PID_SpeedLeft;

#define MAX_SPEED 1000
#define MAX_TURN_RATE 0.5f  // He so quay toi da
#define JOYSTICK_SCALE 0.01f  // Scale [-100,100] -> [-10,10]
// Them PID tam thoi cho khoi dong
PID_t PID_Startup;
uint8_t isBalanced = 0; // Co kiem tra do can bang

// Thoi gian lay mau
float dt = 0.05; // 50ms

// Lam muot PWM
float smoothPWM = 0.0001; // PWM mupt (EMA)

// Cac bien toan cuc de luu trang thai hall sensor
volatile uint8_t currentState = 0;
volatile int8_t direction = 0;
volatile float positionOffset = 0;
volatile uint8_t lastHallState = 0;
uint8_t lastState = 0;
volatile int32_t hallCounter = 0;   // dem so buoc (co dau: duong la quay thuan, am la quay nguoc)
volatile float homePosition = 0.0f; // Luu vi tri ban dau sau khi can bang
float anglemotor = 0;

// CAN
uint8_t RxData[4] = {0};
volatile int32_t joyStickX=0,joyStickY=0;
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
// Ham doc trang thai hall sensor: ghep 3 tin hieu thanh 1 gia tri 3-bit
uint8_t readHallState(void);
// H?m x?c d?nh hu?ng quay d?a v?o s? chuy?n d?i tr?ng th?i
// Gi? s? c?c tr?ng th?i h?p l? l?: 101 (5), 100 (4), 110 (6), 010 (2), 011 (3), 001 (1)
// Forward sequence (quay thu?n): 5 -> 4 -> 6 -> 2 -> 3 -> 1 -> 5...
// Reverse sequence (quay ngu?c): 5 -> 1 -> 3 -> 2 -> 6 -> 4 -> 5...
int8_t getDirection(uint8_t last, uint8_t current);
// Ham tinh goc quay cua dong co
// V?i gi? d?nh: 45 bu?c (transition) tuong duong 1 v?ng quay => m?i bu?c = 8 d?
float getMotorAngle(void);
uint8_t readHallStateDebounced(void);
// H?m nh?n t?n hi?u CAN v? x? l? CAN
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
float getMotorSpeed(void);
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
	PID_Position.Kp = 0.08;PID_Position.Ki = 0.0;PID_Position.Kd = 0.0;
	PID_Pitch.Kp = 0.23;PID_Pitch.Ki = 0.1;PID_Pitch.Kd = 0.09;
	PID_SpeedLeft.Kp=0.15;PID_SpeedLeft.Ki=0.01;PID_SpeedLeft.Kd=0.05;
	// PID cho giai do?n kh?i d?ng (tham s? m?m hon)
	PID_Startup.Kp = 0.6; // Gi?m Kp d? tr?nh dao d?ng
	PID_Startup.Ki = 0.02;
	PID_Startup.Kd = 1.2; // Tang Kd d? gi?m overshoot
	PID_Startup.integral = 0;
	PID_Startup.prevError = 0;
	// B?t y?u c?u ng?t khi d? li?u CAN nh?n v? du?c luu tr? g?i tin ? FIFO1 khi d?ng ID
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
  canfilterconfig.FilterIdHigh = 0x0000 << 5; /*0x446<<5*/
  canfilterconfig.FilterIdLow = 0x0000;
  canfilterconfig.FilterMaskIdHigh = 0x0000 << 5; /*0x446<<5*/
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

  /*Configure GPIO pins : HALL_U_Pin HALL_V_Pin */
  GPIO_InitStruct.Pin = HALL_U_Pin|HALL_V_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : HALL_W_Pin */
  GPIO_InitStruct.Pin = HALL_W_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(HALL_W_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIR_Pin */
  GPIO_InitStruct.Pin = DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DIR_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	int16_t raw_angle=0;
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
		switch (RxHeader.StdId) {
				case 0x446: // Receive angle data
						// Ghép 2 byte thành s? 16-bit
						raw_angle = ((int16_t)RxData[2] << 8) | RxData[3];
						// Chuy?n d?i và áp d?ng d?u
						angleY = (RxData[1] ? 1 : -1) * raw_angle;
					break;
				case 0x447: // Receive X and partial Y
						// Store X data
						joyStickX=RxData[0]?((RxData[1]<<8)|RxData[2]):-((RxData[1]<<8)|RxData[2]);
						// Store first part of Y
						joyStickY = RxData[3]?1:-1; // Y sign
					break;
				case 0x448: // Receive remaining Y data
						// Store remaining Y data
						joyStickY*=(RxData[0]<<8)|RxData[1];
					break;	
				default:
						Error_Handler();
					break;
			}
    } else {
        Error_Handler();
    }
}

void updateMotors(float PWM)
{
  uint16_t pwmLeft = (uint16_t)fminf(fabsf(PWM), MAX_SPEED);

  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, PWM < 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwmLeft);
}

void AutoBalanceOnStartup()
{
  const float targetAngle = 0.0; // G?c m?c ti?u
  const float tolerance = 1.0;   // Sai s? cho ph?p (?1 d?)

  while (!isBalanced)
  {
    // ?i?u khi?n PID t?m th?i ch? d?nh cho kh?i d?ng
    float y = angleY;
    float error = targetAngle - y;
    float output = PID_Compute(&PID_Startup, targetAngle, y, dt);
    currentState = readHallStateDebounced();
    direction = getDirection(lastHallState, currentState);
    // C?p nh?t hallCounter n?u c? s? thay d?i h?p l?
    if (direction != 0)
    {
      hallCounter += direction;
      lastHallState = currentState;
    }
    // C?p nh?t d?ng co

    float motor_balance = 1.0 + (y * 0.02); // Hi?u ch?nh theo hu?ng nghi?ng
    updateMotors(output * motor_balance);
    // Ki?m tra di?u ki?n ?n d?nh
    if (fabs(error) < tolerance)
    {
      isBalanced = 1;
      homePosition = getMotorAngle();
      break;
    }
    HAL_Delay(5); // Ch? 5ms
  }
}

void checkSafetyStop(int16_t Y)
{
  if (fabs((float)Y) > 40.0 || !isBalanced)
  {
    updateMotors(0);
    isBalanced = 0; // Reset tr?ng th?i c?n b?ng
  }
}

uint8_t readHallState(void)
{
  // ??c m?c logic t? c?c ch?n GPIO
  uint8_t u = HAL_GPIO_ReadPin(HALL_U_GPIO_Port, HALL_U_Pin);
  uint8_t v = HAL_GPIO_ReadPin(HALL_V_GPIO_Port, HALL_V_Pin);
  uint8_t w = HAL_GPIO_ReadPin(HALL_W_GPIO_Port, HALL_W_Pin);
  return (u << 2) | (v << 1) | w;
}

int8_t getDirection(uint8_t last, uint8_t current)
{
  // B?ng chuy?n d?i h?p l? (Forward v? Reverse)
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
  return 0; // Kh?ng h?p l?
}

uint8_t readHallStateDebounced(void)
{
  uint8_t currentState = readHallState();
  lastState = currentState;
  return lastState;
}

float getMotorAngle(void)
{
  return ((float)hallCounter / 45.0f) * 360.0f; // Ho?c: hallCounter * 8.0f;
}

void controlLoop(void)
{
 // Th?m bi?n d? theo d?i joystick
//  static float targetPosition = 0;
  static float speedLeft = 0;
	// Scale joystick inputs to [-1,1]
  float forwardInput = joyStickY * JOYSTICK_SCALE;  // Forward/Backward
  float turnInput = -joyStickX * JOYSTICK_SCALE;     // Left/Right turn
	if (fabs(forwardInput) > 0.1f || fabs(turnInput) > 0.1f) {
		// T?nh to?n t?c d? m?c ti?u cho m?i b?nh
		float targetSpeedLeft = forwardInput;
		
		// Th?m hi?u ?ng quay
		targetSpeedLeft += turnInput * MAX_TURN_RATE;
		
		// Gi?i h?n t?c d?
		targetSpeedLeft = fminf(fmaxf(targetSpeedLeft, -1.0f), 1.0f);

		
		// PID cho t?c d?
		float speedLeftOutput = PID_Compute(&PID_SpeedLeft, targetSpeedLeft, speedLeft, dt);
		
		// PID cho c?n b?ng
		float balanceTorque = PID_Compute(&PID_Pitch, 0.0f, angleY, dt);
		
		// K?t h?p c?c output
		float pwmLeft = balanceTorque + speedLeftOutput * MAX_SPEED;
		
		// C?p nh?t d?ng co
		updateMotors(pwmLeft);
		// C?p nh?t t?c d? hi?n t?i (t? encoder ho?c hall sensor)
		speedLeft = getMotorSpeed();
  }else {
		// Ch? d? gi? c?n b?ng t?i ch?
		float positionError = homePosition - positionOffset;
		float balanceTorque = PID_Compute(&PID_Pitch, 0.0f, angleY, dt);
		float stabilizeTorque = PID_Compute(&PID_Position, 0.0f, positionError, dt);
		
		float pwm = balanceTorque + stabilizeTorque;
		updateMotors(pwm);
	}  
  checkSafetyStop(angleY);
	
	
/*  // N?u c? t?n hi?u t? joystick
  if (joyStickX != 0 || joyStickY != 0) {
    // T?nh to?n v? tr? m?i d?a tr?n joystick
    float xInput = joyStickX * 0.1f; // H? s? scale cho ph? h?p
    float yInput = joyStickY * 0.1f;
    
    // C?p nh?t v? tr? m?c ti?u
    targetPosition += xInput;
    
    // PID cho c?n b?ng (angleY)
    float balanceTorque = PID_Compute(&PID_Pitch, 0.0f, angleY , dt);
    // ?i?u khi?n d?ng co
    float PWM = balanceTorque + xInput;
    updateMotors(PWM);
  }
  else {
    // Ch? d? gi? v? tr? ban d?u
    float positionError = homePosition - positionOffset;
    float velocitySetpoint = PID_Compute(&PID_Position, 0.0f, positionError, dt);
    float balanceTorque = PID_Compute(&PID_Pitch, 0.0f, angleY, dt);
    
    float PWM = balanceTorque + velocitySetpoint;
    updateMotors(PWM);
  }
  
  checkSafetyStop(angleY);
*/
}

// Th?m h?m d?c t?c d? d?ng co
float getMotorSpeed(void) {
    // T?nh t?c d? t? encoder ho?c hall sensor cho d?ng co tr?i
    static int32_t lastCountLeft = 0;
    float speed = (hallCounter- lastCountLeft) / dt;
    lastCountLeft = hallCounter;
    return speed / MAX_SPEED; // Normalize to [-1,1]
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
  for(;;)
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
  for(;;)
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
  for(;;)
  {
		if (!isBalanced) {
      // T? d?ng c?n b?ng l?n d?u
      AutoBalanceOnStartup();
    }
    else {
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
  for(;;)
  {
		// ??c hall sensor
		currentState = readHallStateDebounced();
		direction = getDirection(lastHallState, currentState);
		if (direction != 0)
		{
			hallCounter += direction;
			lastHallState = currentState;
		}
		// C?p nh?t v? tr? (do du?c t? hall sensor)
		positionOffset = getMotorAngle(); // Gi? tr? do du?c (d?)
																			// T?nh sai s? v? tr? (gi? s? v? tr? m?c ti?u l? 0 d?)
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
