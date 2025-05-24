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

PID_t PID_Position, PID_Pitch, PID_SpeedLeft;

#define MAX_SPEED 1000
#define MAX_TURN_RATE 0.5f    // He so quay toi da
#define JOYSTICK_SCALE 0.001f // Scale [-100,100] -> [-1,1]
// Them PID tam thoi cho khoi dong
PID_t PID_Startup;
uint8_t isBalanced = 0; // Flat kiem tra do can bang

// Thoi gian lay mau
float dt = 0.01;                                // 10ms
#define MAX_TARGET_ANGLE_FROM_POSITION_PID 2.0f // Vi du: +-2 do
// Lam muot PWM
volatile float lastPWM = 0;
// Bien static cho ham updateMotors de xu ly doi chieu muot ma
// s_motor_active_direction: 0 = dung, 1 = quay thuan (PWM > 0), -1 = quay nguoc (PWM < 0)
static int8_t s_motor_active_direction = 0;
#define MIN_PWM_TO_CONSIDER_MOVING 1.0f // Nguong PWM de coi la dong co dang di chuyen

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
uint8_t RxData[4] = {0};
volatile int32_t joyStickX = 0, joyStickY = 0;
CAN_RxHeaderTypeDef RxHeader;
volatile char *command = "";
volatile float angleY = 0;
float test = 0;
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
void AutoBalanceOnStartup(void); // Ham tu dong can bang khi khoi dong
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
  PID_Position.Kp = 0.05;
  PID_Position.Ki = 0.01;
  PID_Position.Kd = 0.0;
  PID_Pitch.Kp = 8.2;
  PID_Pitch.Ki = 0.0;
  PID_Pitch.Kd = 0.3;
  PID_SpeedLeft.Kp = 0.5;
  PID_SpeedLeft.Ki = 0.0;
  PID_SpeedLeft.Kd = 0.005;
  // PID cho giai doan khoi dong
  PID_Startup.Kp = 10;
  PID_Startup.Ki = 0.0;
  PID_Startup.Kd = 1.2;
  PID_Startup.integral = 0;  // Khoi tao tich phan
  PID_Startup.prevError = 0; // Khoi tao gia tri sai so truoc
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
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
  {
    switch (RxHeader.StdId)
    {
    case 0x446: // Nhan goc Y
      // Ghep 2 byte thanh 1 gia tri 16 bit
      raw_angle = ((int16_t)RxData[2] << 8) | RxData[3];
      // Chuyen doi goc Y thanh goc thuc te co dau
      raw_angle = (RxData[1] ? 1 : -1) * raw_angle;
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
        joyStickX = RxData[1] ? ((RxData[2] << 8) | RxData[3]) : -((RxData[2] << 8) | RxData[3]);
      }
      if (joyStickX > 100 || joyStickX < -100)
      {
        joyStickX = 0;
      }
      break;
    case 0x448: // Nhan gia tri joystick Y
      if (RxData[0] != 32)
      {
        joyStickY = 0;
      }
      else
      {
        joyStickY = RxData[1] ? ((RxData[2] << 8) | RxData[3]) : -((RxData[2] << 8) | RxData[3]);
      }
      if (joyStickY > 100 || joyStickY < -100)
      {
        joyStickY = 0;
      }
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
  //  float target_abs_pwm = fminf(fabsf(PWM), MAX_SPEED);
  //  int8_t desired_direction = 0;
  //  if (PWM > 0.01f)
  //    desired_direction = 1;
  //  else if (PWM < -0.01f)
  //    desired_direction = -1;

  //  float smoothed_abs_pwm = target_abs_pwm;

  //  // Kiem tra yeu cau doi chieu khi dong co dang chay theo huong nguoc lai
  //  if (desired_direction != 0 && s_motor_active_direction != 0 && desired_direction != s_motor_active_direction)
  //  {
  //    // Dong co dang chay va co yeu cau doi chieu
  //    if (lastPWM > MIN_PWM_TO_CONSIDER_MOVING)
  //    {
  //      // Dong co dang chay voi toc do dang ke, can giam toc truoc khi doi chieu
  //      // Muc tieu PWM trong chu ky nay la 0 de giam toc
  //      if (smoothed_abs_pwm < MIN_PWM_TO_CONSIDER_MOVING)
  //      {
  //        // Da giam toc du thap, an toan de doi chieu DIR
  //        lastPWM = 0.0f;                                  // Dat lai lastPWM ve 0
  //        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0); // Xuat PWM = 0 trong chu ky nay

  //        s_motor_active_direction = desired_direction; // Cap nhat huong moi
  //        HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, (s_motor_active_direction == 1) ? GPIO_PIN_RESET : GPIO_PIN_SET);
  //        // Ham se return, chu ky tiep theo se bat dau tang toc tu lastPWM = 0 theo huong moi
  //        return;
  //      }
  //      else
  //      {
  //        // Van dang trong qua trinh giam toc
  //        lastPWM = smoothed_abs_pwm;
  //        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)(lastPWM + 0.5f));
  //        // Chua doi chieu DIR, giu nguyen huong hien tai
  //        return;
  //      }
  //    }
  //    else
  //    {
  //      // Dong co chay rat cham hoac da dung, co the doi chieu DIR ngay
  //      lastPWM = 0.0f; // Reset lastPWM de bat dau tang toc tu 0
  //      s_motor_active_direction = desired_direction;
  //      HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, (s_motor_active_direction == 1) ? GPIO_PIN_RESET : GPIO_PIN_SET);
  //      // Tiep tuc xuong logic lam muot PWM de tang toc
  //    }
  //  }
  //  // Khoi dong tu trang thai dung hoac khi PWM muc tieu la 0
  //  else if (desired_direction != 0 && s_motor_active_direction == 0)
  //  {
  //    // Bat dau tu trang thai dung, dat chieu DIR
  //    s_motor_active_direction = desired_direction;
  //    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, (s_motor_active_direction == 1) ? GPIO_PIN_RESET : GPIO_PIN_SET);
  //    // lastPWM nen la 0 hoac gan 0, se tang toc
  //  }

  //  lastPWM = smoothed_abs_pwm;

  //  // Xuat PWM ra timer
  //  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)(lastPWM + 0.5f));

  //  // Cap nhat trang thai s_motor_active_direction neu dong co dung lai
  //  if (desired_direction == 0 && lastPWM < MIN_PWM_TO_CONSIDER_MOVING)
  //  {
  //    s_motor_active_direction = 0;
  //    // Neu PWM muc tieu la 0 va lastPWM da rat thap, dam bao PWM ra la 0
  //    if (fabsf(PWM) < 0.01f)
  //    {
  //      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  //      lastPWM = 0.0f;
  //    }
  //  }
  //  else
  //  {
  //    // Neu desired_direction != 0, s_motor_active_direction da duoc dat o tren
  //  }

  float target_abs_pwm = fminf(fabsf(PWM), MAX_SPEED);
  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, PWM > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)fabsf(target_abs_pwm));
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
    // Cap nhat dong co
    float motor_balance = 1.0 + (y * 0.05); // Hieu chinh theo huong nghien
    updateMotors(output * motor_balance);
    // Kiem tra dieu kien can bang
    if (fabs(error) < tolerance)
    {
      isBalanced = 1;
      break;
    }
    HAL_Delay(10);
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

void controlLoop(void)
{
  // static float speedLeft = 0; // Bien nay duong nhu khong duoc su dung trong logic joystick hien tai
  float currentAngle = angleY;
  float targetAngleForPitch = 0.0f; // Mac dinh goc nghieng muc tieu la 0
  if (currentAngle < -45 || currentAngle > 45)
  {
    updateMotors(0);
    PID_Reset(&PID_Pitch);
    PID_Reset(&PID_Position);
    PID_Reset(&PID_SpeedLeft);
  }
  else
  {
    float forwardInput = 0;
    float turnInput = 0;

    if (joyStickY <= 100 && joyStickY >= -100)
    {
      forwardInput = joyStickY * JOYSTICK_SCALE;
    }
    if (joyStickX <= 100 && joyStickX >= -100)
    {
      turnInput = -joyStickX * JOYSTICK_SCALE;
    }

    // Tinh toan targetAngleForPitch tu PID_Position (khi khong co dieu khien joystick)
    if (!(fabs(forwardInput) > 0.01f || fabs(turnInput) > 0.01f)) // Neu KHONG co dieu khien joystick
    {
      float positionError = 0 - hallCounter; // Muc tieu la hallCounter = 0
      targetAngleForPitch = PID_Compute(&PID_Position, 0.0f, positionError, dt);
      // Gioi han goc nghieng muc tieu de tranh robot nghieng qua nhieu gay mat thang bang
      targetAngleForPitch = fminf(fmaxf(targetAngleForPitch, -MAX_TARGET_ANGLE_FROM_POSITION_PID), MAX_TARGET_ANGLE_FROM_POSITION_PID);
    }
    // Neu co dieu khien joystick, targetAngleForPitch se mac dinh la 0 (robot se co gang dung thang khi di chuyen)
    // Hoac ban co the them logic de joystick cung tao ra mot targetAngleForPitch nho neu muon.

    // Vong lap PID ben trong: Dieu khien goc nghieng
    float balanceTorque = PID_Compute(&PID_Pitch, targetAngleForPitch, currentAngle, dt);

    float finalPwm = balanceTorque;

    // Xu ly dieu khien joystick (neu co)
    // Logic joystick hien tai cua ban tinh ra pwmLeft, co the can xem xet lai cach ket hop
    // voi balanceTorque. Mot cach la joystick se dieu chinh toc do/vi tri muc tieu cho
    // vong lap ngoai, hoac them mot thanh phan PWM nho vao finalPwm.
    // Tam thoi, neu co joystick, se uu tien logic joystick cua ban.
    if (fabs(forwardInput) > 0.01f || fabs(turnInput) > 0.01f) // Neu co dieu khien joystick
    {
      // Logic dieu khien joystick cua ban (can xem xet lai cach ket hop)
      // Hien tai, no tinh speedLeftOutput va cong vao balanceTorque.
      // Co the targetSpeedLeft nen la dau vao cho mot bo PID toc do rieng,
      // hoac dau ra cua PID_Position.
      // De don gian, tam thoi giu nguyen logic cua ban khi co joystick,
      // nhung targetAngleForPitch cho PID_Pitch se la 0.
      float targetSpeedLeft = forwardInput;
      targetSpeedLeft += turnInput * MAX_TURN_RATE;
      targetSpeedLeft = fminf(fmaxf(targetSpeedLeft, -1.0f), 1.0f);

      // Bien speedLeft nen duoc cap nhat hoac la dau ra cua mot cam bien toc do thuc te
      // Neu khong, PID_SpeedLeft se khong hoat dong dung nhu mong doi.
      // Gia su speedLeft la toc do hien tai (can co cam bien hoac uoc luong)
      float currentSpeedApproximation = 0; // CAN THAY THE BANG TOC DO THUC TE HOAC UOC LUONG
      float speedControlPwm = PID_Compute(&PID_SpeedLeft, targetSpeedLeft, currentSpeedApproximation, dt) * MAX_SPEED;

      finalPwm += speedControlPwm;
    }
    test = finalPwm;
    updateMotors(finalPwm);
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
    if (!isBalanced)
    {
      // Tu dong can bang khi khoi dong
      AutoBalanceOnStartup();
    }
    else
    {
      controlLoop();
    }
    osDelay(10);
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
