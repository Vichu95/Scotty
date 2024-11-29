/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  * Type: Lower-level-Controller firmware
  * communicate as slave over SPI
  * communicate with two can-bus-systems and control with this ability three actuators per bus
  * Autor: Dave Mangatter
  * Datum: 16.03.2022
  * Version: 1.02
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "math.h"

//Keypad
#include "MY_Keypad4x4.h"
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Range in the Motor controller
#define MOTOR_P_MIN -12.5f
#define MOTOR_P_MAX 12.5f
#define MOTOR_V_MIN -50.0f
#define MOTOR_V_MAX 50.0f
#define MOTOR_T_MIN -65.0f
#define MOTOR_T_MAX 65.0f
#define MOTOR_KP_MIN 0.0f
#define MOTOR_KP_MAX 500.0f
#define MOTOR_KD_MIN 0.0f
#define MOTOR_KD_MAX 5.0f

// Value limits in ST
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -26.0f
#define V_MAX 26.0f
//#define T_MIN -48.0f
//#define T_MAX 48.0f
#define T_MIN -2.0f
#define T_MAX 2.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f

/// Joint Soft Stops ///P
#define A_LIM_P 1.5f
#define A_LIM_N -1.5f
#define H_LIM_P 2.5f
#define H_LIM_N -2.5f
#define K_LIM_P -1.2f
#define K_LIM_N 6.5f
#define KP_SOFTSTOP 100.0f
#define KD_SOFTSTOP 0.4f

// length of receive/transmit buffers
#define RX_LEN 66
#define TX_LEN 66

// length of outgoing/incoming messages
#define STATE_LEN 30
#define CONTROL_LEN 66


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI1_Init(void);
static void MX_CAN2_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */
//function declaration

void motor_mode(uint8_t ID,CAN_RxHeaderTypeDef*Header,uint8_t*Data);
void exit_mode(uint8_t ID,CAN_RxHeaderTypeDef*Header,uint8_t*Data);
void zero(uint8_t ID,CAN_RxHeaderTypeDef*Header,uint8_t*Data);
void pack_message(uint8_t ID,CAN_RxHeaderTypeDef*Header,uint8_t*Data);
void unpack_replay(uint8_t* Data);
int softstop_joint(float *control,float state, float limit_p, float limit_n);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x, float x_min, float x_max, int bits);

void can_send_receive();
void can_control();

uint32_t xor_checksum(uint32_t*, int);
void spi_send_receive(void);

void delay_us(uint16_t us);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//9*4*2+2*4=68

//structur definition
typedef struct {
    float ab_p[2],hip_p[2],knee_p[2];
    float ab_v[2],hip_v[2],knee_v[2];
    //float ab_t[2],hip_t[2],knee_t[2];
    uint32_t flags[2],checksum;
}spi_tx;

//15*4*2+2*4=68
typedef struct {
    float ab_p[2],hip_p[2],knee_p[2];
    float ab_v[2],hip_v[2],knee_v[2];
    float ab_kp[2],hip_kp[2],knee_kp[2];
    float ab_kd[2],hip_kd[2],knee_kd[2];
    float ab_t[2],hip_t[2],knee_t[2];
    uint32_t flags[2],checksum;
}spi_rx;

typedef struct {
    float ab_t[2],hip_t[2],knee_t[2];
}test;


//spi buffer
uint16_t spi_tx_buffer[TX_LEN];
uint16_t spi_rx_buffer[RX_LEN];

//structurs
spi_tx values;
spi_rx valuesrec;
spi_rx control;
uint32_t currentControlMode = 99;
//spi_tx state={1.12,1.12,3.16,3.16,5.4,5.4,1.12,1.12,3.16,3.16,5.4,5.4,1,1,4};
spi_tx state;
test torque;

//State Variables
uint32_t check;

//to detect a falling edge
uint8_t spi_enable;
uint8_t merker = 0;

//set values
float p_in = 0.0f;
float v_in = 0.0f;
float kp_in = 2.0f;     //stifness
float kd_in = 0.4f;     //damper
float t_in = 0.0f;

//measured values
float p_out = 0.0f;     //position
float v_out = 0.0f;     //velocity
float t_out = 0.0f;     //torque


//steap Variable which change the position of the gear
float p_step = 0.1f;

//Can ID

uint8_t Ab_CAN = 1;
uint8_t Hip_CAN = 2;
uint8_t Knee_CAN = 3;

uint8_t CAN;

//Can variables

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint8_t TxData[8];
uint8_t RxData[8];

uint32_t TxMailbox;

uint8_t TxData2[8];
uint8_t RxData2[8];

uint32_t TxMailbox2;

int datacheck = 2;
int count=2;
int keycontrol=0;
int motormode=0;
volatile spi_enabled = 0;
volatile callback_enabled = 0;

//Time
uint32_t time;
uint32_t time2;

//Keypad
bool mySwitches[16];

Keypad_WiresTypeDef myKeypadStruct;

uint32_t spi_test=0;
uint32_t Error_spi;
uint32_t CallbackError_spi;
uint32_t State_spi;
HAL_StatusTypeDef spierror;



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
	if (RxHeader.DLC == 8)
	{
		datacheck=0;
	}
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData);
	if (RxHeader.DLC == 8)
	{
		datacheck=1;
	}
}


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
  MX_CAN1_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_CAN2_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */

  HAL_CAN_Start(&hcan1);
  HAL_CAN_Start(&hcan2);
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start(&htim8);


  // Activate the notification
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);


  TxHeader.DLC = 8;  // data length
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;


  //Keypad
  myKeypadStruct.IN0_Port=GPIOC;
  myKeypadStruct.IN1_Port=GPIOC;
  myKeypadStruct.IN2_Port=GPIOC;
  myKeypadStruct.IN3_Port=GPIOC;

  myKeypadStruct.OUT0_Port=GPIOB;
  myKeypadStruct.OUT1_Port=GPIOB;
  myKeypadStruct.OUT2_Port=GPIOB;
  myKeypadStruct.OUT3_Port=GPIOB;

  //PINS

  myKeypadStruct.IN0pin=GPIO_PIN_6;
  myKeypadStruct.IN1pin=GPIO_PIN_7;
  myKeypadStruct.IN2pin=GPIO_PIN_8;
  myKeypadStruct.IN3pin=GPIO_PIN_9;

  myKeypadStruct.OUT0pin=GPIO_PIN_12;
  myKeypadStruct.OUT1pin=GPIO_PIN_13;
  myKeypadStruct.OUT2pin=GPIO_PIN_14;
  myKeypadStruct.OUT3pin=GPIO_PIN_15;


  printf("start\n");
  HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t *)spi_tx_buffer, (uint8_t *)spi_rx_buffer, RX_LEN);
//  HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)spi_tx_buffer, (uint8_t *)spi_rx_buffer, RX_LEN);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */





	motor_mode(Ab_CAN, &TxHeader, TxData);
	motor_mode(Hip_CAN, &TxHeader, TxData);
	motor_mode(Knee_CAN, &TxHeader, TxData);



delay_us(1000);

zero(Ab_CAN, &TxHeader, TxData);
zero(Hip_CAN, &TxHeader, TxData);
zero(Knee_CAN, &TxHeader, TxData);

 delay_us(1000);

	// Only CAN
//	count=1;
//	  while (count==1)
//	  {
//
//
//			can_send_receive();
//			time=__HAL_TIM_GET_COUNTER(&htim8);
//	  }




  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	__HAL_TIM_SET_COUNTER(&htim8,0);
	 	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15) == 0 && count==2 && spi_enabled==0){
	//if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15) == 0 ){
		spi_test=1;
		spi_send_receive();
		//can_control();
		//can_send_receive();
		count=1;
	    //count=1;
		time2=__HAL_TIM_GET_COUNTER(&htim8);

	}

	if(count==1){
//		can_control();



		can_send_receive();
		count=2;
		time=__HAL_TIM_GET_COUNTER(&htim8);
	}

	Error_spi=HAL_SPI_GetError(&hspi1);
	State_spi=HAL_SPI_GetState(&hspi1);
	//HAL_Delay(1);

  }


	exit_mode(Ab_CAN, &TxHeader, TxData);
	exit_mode(Hip_CAN, &TxHeader, TxData);
	exit_mode(Knee_CAN, &TxHeader, TxData);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 9;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  CAN_FilterTypeDef canfilterconfig;

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 0;  // which filter bank to use from the assigned ones 18
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canfilterconfig.FilterIdHigh = 0x0000;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0xFFFF;
  canfilterconfig.FilterMaskIdLow = 0xFFFC;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 14;  // how many filters to assign to the CAN1 (master can) 20

  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 9;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  CAN_FilterTypeDef canfilterconfig2;

  canfilterconfig2.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig2.FilterBank = 15;  // which filter bank to use from the assigned ones
  canfilterconfig2.FilterFIFOAssignment = CAN_FILTER_FIFO1;
  canfilterconfig2.FilterIdHigh = 0x0000;
  canfilterconfig2.FilterIdLow = 0;
  canfilterconfig2.FilterMaskIdHigh = 0xFFFF;
  canfilterconfig2.FilterMaskIdLow = 0xFFFC;
  canfilterconfig2.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig2.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig2.SlaveStartFilterBank = 14;  // how many filters to assign to the CAN1 (master can)

  HAL_CAN_ConfigFilter(&hcan2, &canfilterconfig2);

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 180;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xFFFF-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 180;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

//Printfunction

int _write(int file, char *ptr, int len)
{
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		//__io_putchar(*ptr++);
		ITM_SendChar(*ptr++);
	}
	return len;
}



///////////////////////////modes/////////////////////////////////

//Start motor
void motor_mode(uint8_t ID,CAN_RxHeaderTypeDef*Header,uint8_t*Data){
	Header->StdId = ID;
    Data[0] = 0xFF;
    Data[1] = 0xFF;
    Data[2] = 0xFF;
    Data[3] = 0xFF;
    Data[4] = 0xFF;
    Data[5] = 0xFF;
    Data[6] = 0xFF;
    Data[7] = 0xFC;
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, Data, &TxMailbox);
	HAL_CAN_AddTxMessage(&hcan2, &TxHeader, Data, &TxMailbox);
    delay_us(300);
	//wait(100);
    }

//stop motor
void exit_mode(uint8_t ID,CAN_RxHeaderTypeDef*Header,uint8_t*Data){
	Header->StdId = ID;
    Data[0] = 0xFF;
    Data[1] = 0xFF;
    Data[2] = 0xFF;
    Data[3] = 0xFF;
    Data[4] = 0xFF;
    Data[5] = 0xFF;
    Data[6] = 0xFF;
    Data[7] = 0xFD;
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, Data, &TxMailbox);
	HAL_CAN_AddTxMessage(&hcan2, &TxHeader, Data, &TxMailbox);
    delay_us(300);
	//wait(100);
    }

//set motorposition to zero
void zero(uint8_t ID,CAN_RxHeaderTypeDef*Header,uint8_t*Data){
	Header->StdId = ID;
    Data[0] = 0xFF;
    Data[1] = 0xFF;
    Data[2] = 0xFF;
    Data[3] = 0xFF;
    Data[4] = 0xFF;
    Data[5] = 0xFF;
    Data[6] = 0xFF;
    Data[7] = 0xFE;
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, Data, &TxMailbox);
	HAL_CAN_AddTxMessage(&hcan2, &TxHeader, Data, &TxMailbox);
    /*if(CAN==0){
    	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, Data, &TxMailbox);
    }
    if(CAN==1){
    	HAL_CAN_AddTxMessage(&hcan2, &TxHeader, Data, &TxMailbox);
    }*/
    delay_us(300);
	//wait(100);
    }

/////////////////////////pack and unpack//////////////////////////

void pack_message(uint8_t ID,CAN_RxHeaderTypeDef*Header,uint8_t*Data){

		state.flags[0] = 0;
		state.flags[1] = 0;

		if(ID==1){
			state.flags[CAN] |= softstop_joint(&control.ab_p[CAN],state.ab_p[CAN],1.5, -1.5);
			p_in = control.ab_p[CAN];
			v_in = control.ab_v[CAN];
			kp_in = control.ab_kp[CAN];   //stifness
			kd_in = control.ab_kd[CAN];     //damper
			t_in = control.ab_t[CAN];
	    	}
		if(ID==2){
			state.flags[CAN] |= softstop_joint(&control.hip_p[CAN],state.hip_p[CAN],2.5, -2.5)<<1;
			p_in = control.hip_p[CAN];
			v_in = control.hip_v[CAN];
			kp_in = control.hip_kp[CAN];   //stifness
			kd_in = control.hip_kd[CAN];     //damper
			t_in = control.hip_t[CAN];
	    	}
		if(ID==3){
			state.flags[CAN] |= softstop_joint(&control.knee_p[CAN], state.knee_p[CAN],7.3, -0.2)<<2;
			p_in = control.knee_p[CAN];
			v_in = control.knee_v[CAN];
			kp_in = control.knee_kp[CAN];   //stifness
			kd_in = control.knee_kd[CAN];     //damper
			t_in = control.knee_t[CAN];
	    	}

	Header->StdId = ID;

    /// limit data to be within bounds ///
	float p_des = fminf(fmaxf(P_MIN, p_in), P_MAX);
	float v_des = fminf(fmaxf(V_MIN, v_in), V_MAX);
	float kp = fminf(fmaxf(KP_MIN, kp_in), KP_MAX);
	float kd = fminf(fmaxf(KD_MIN, kd_in), KD_MAX);
	float t_ff = fminf(fmaxf(T_MIN, t_in), T_MAX);

    /// convert floats to unsigned ints ///
    uint16_t p_int = float_to_uint(p_des, MOTOR_P_MIN, MOTOR_P_MAX, 16);
    uint16_t v_int = float_to_uint(v_des, MOTOR_V_MIN, MOTOR_V_MAX, 12);
    uint16_t kp_int = float_to_uint(kp, MOTOR_KP_MIN, MOTOR_KP_MAX, 12);
    uint16_t kd_int = float_to_uint(kd, MOTOR_KD_MIN, MOTOR_KD_MAX, 12);
    uint16_t t_int = float_to_uint(t_ff, MOTOR_T_MIN, MOTOR_T_MAX, 12);

    /// pack ints into the can buffer ///
    Data[0] = p_int>>8;
    Data[1] = p_int&0xFF;
    Data[2] = v_int>>4;
    Data[3] = ((v_int&0xF)<<4)|(kp_int>>8);
    Data[4] = kp_int&0xFF;
    Data[5] = kd_int>>4;
    Data[6] = ((kd_int&0xF)<<4)|(t_int>>8);
    Data[7] = t_int&0xff;


//	float t_ffTest = uint_to_float(t_int, MOTOR_T_MIN, MOTOR_T_MAX, 12);
//	float p_out = uint_to_float(p_int, MOTOR_P_MIN, MOTOR_P_MAX, 16);
//	float v_out = uint_to_float(v_int, MOTOR_V_MIN, MOTOR_V_MAX, 12);
//	float kp_out = uint_to_float(kp_int, MOTOR_KP_MIN, MOTOR_KP_MAX, 12);
//	float kd_out = uint_to_float(kd_int, MOTOR_KD_MIN, MOTOR_KD_MAX, 12);
//	float vas_out = uint_to_float(kd_int, MOTOR_KD_MIN, MOTOR_KD_MAX, 12);

    }


void unpack_replay(uint8_t*Data){
	/// unpack ints from can buffer ///

	uint16_t id = Data[0];
	uint16_t p_int = (Data[1]<<8)|Data[2];
	uint16_t v_int = (Data[3]<<4)|(Data[4]>>4);
	uint16_t i_int = ((Data[4]&0xF)<<8)|Data[5];
	/// convert uints to floats ///
	p_out = uint_to_float(p_int, MOTOR_P_MIN, MOTOR_P_MAX, 16);
	v_out = uint_to_float(v_int, MOTOR_V_MIN, MOTOR_V_MAX, 12);
	t_out = uint_to_float(i_int, MOTOR_T_MIN, MOTOR_T_MAX, 12);

	if(id==1){
		state.ab_p[datacheck]=p_out;
		state.ab_v[datacheck]=v_out;
		//state.ab_t[datacheck]=t_out;
		torque.ab_t[datacheck]=t_out;
	}
	if(id==2){
		state.hip_p[datacheck]=p_out;
		state.hip_v[datacheck]=v_out;
		//state.hip_t[datacheck]=t_out;
		torque.hip_t[datacheck]=t_out;
	}
	if(id==3){
		state.knee_p[datacheck]=p_out;
		state.knee_v[datacheck]=v_out;
		//state.knee_t[datacheck]=t_out;
		torque.knee_t[datacheck]=t_out;
    }
}
/////////////////////////////////math/////////////////////////////////////////
void can_send_receive(){


	CAN=0;
	pack_message(Ab_CAN, &TxHeader, TxData);
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
    delay_us(300);
	//wait(100);
	if (datacheck==0){
		unpack_replay(RxData);
	}
	pack_message(Hip_CAN, &TxHeader, TxData);
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
    delay_us(300);
	//wait(100);
	if (datacheck==0){
		unpack_replay(RxData);
	}
	pack_message(Knee_CAN, &TxHeader, TxData);
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
    delay_us(300);
	//wait(100);
	if (datacheck==0){
		unpack_replay(RxData);
	}

//	CAN=1;
//	pack_message(Ab_CAN, &TxHeader, TxData);
//	HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox);
//    delay_us(300);
//	//wait(100);
//	if (datacheck==1){
//		unpack_replay(RxData);
//	}
//	pack_message(Hip_CAN, &TxHeader, TxData);
//	HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox);
//    delay_us(300);
//	//wait(100);
//	if (datacheck==1){
//		unpack_replay(RxData);
//	}
//	pack_message(Knee_CAN, &TxHeader, TxData);
//	HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox);
//    delay_us(300);
//	//wait(100);
//	if (datacheck==1){
//		unpack_replay(RxData);
//	}

}

void can_control(){
	//Kepad	layout
	//S1 start Motor
	//S2 stop Motor
	//S3 set Motorposition to zero
	//S4 start Key control(only for STM32 Board)
	//S9 increase Ab-Motor position
	//S13 decrease Ab-Motor position
	//S10 increase Hip-Motor position
	//S14 decrease Hip-Motor position
	//S11 increase Knee-Motor position
	//S15 decrease Knee-Motor position

	Keypad4x4_ReadKeypad(mySwitches);
		if(mySwitches[3]==true){
			keycontrol=1;
		}
		if(mySwitches[7]==true){
			keycontrol=0;
		}


	if(keycontrol==0){
		if(control.flags[0]==1 && control.flags[1]==1){
			motor_mode(Ab_CAN, &TxHeader, TxData);
			motor_mode(Hip_CAN, &TxHeader, TxData);
			motor_mode(Knee_CAN, &TxHeader, TxData);
			//motormode=1;
			}
		if(control.flags[0]==0 && control.flags[1]==0){
			exit_mode(Ab_CAN, &TxHeader, TxData);
			exit_mode(Hip_CAN, &TxHeader, TxData);
			exit_mode(Knee_CAN, &TxHeader, TxData);
		 	}
		if(control.flags[0]==2 && control.flags[1]==2){
			zero(Ab_CAN, &TxHeader, TxData);
			zero(Hip_CAN, &TxHeader, TxData);
			zero(Knee_CAN, &TxHeader, TxData);
			}
		}

	if(keycontrol==1){
		if(mySwitches[0]==true){
			motor_mode(Ab_CAN, &TxHeader, TxData);
			motor_mode(Hip_CAN, &TxHeader, TxData);
			motor_mode(Knee_CAN, &TxHeader, TxData);
			}
		if(mySwitches[1]==true){
			exit_mode(Ab_CAN, &TxHeader, TxData);
			exit_mode(Hip_CAN, &TxHeader, TxData);
			exit_mode(Knee_CAN, &TxHeader, TxData);
			 }
		if(mySwitches[2]==true){
			zero(Ab_CAN, &TxHeader, TxData);
			zero(Hip_CAN, &TxHeader, TxData);
			zero(Knee_CAN, &TxHeader, TxData);
			}
		if(mySwitches[8]==true){
			control.ab_p[0]=control.ab_p[0]+p_step;
			control.ab_p[1]=control.ab_p[1]+p_step;
			}
		if(mySwitches[12]==true){
			control.ab_p[0]=control.ab_p[0]-p_step;
			control.ab_p[1]=control.ab_p[1]-p_step;
			}
		if(mySwitches[9]==true){
			control.hip_p[0]=control.hip_p[0]-p_step;
			control.hip_p[1]=control.hip_p[1]-p_step;
			}
		if(mySwitches[13]==true){
			control.hip_p[0]=control.hip_p[0]+p_step;
			control.hip_p[1]=control.hip_p[1]+p_step;
			}
		if(mySwitches[10]==true){
			control.knee_p[0]=control.knee_p[0]-p_step;
			control.knee_p[1]=control.knee_p[1]-p_step;
			}
		if(mySwitches[14]==true){
			control.knee_p[0]=control.knee_p[0]+p_step;
			control.knee_p[1]=control.knee_p[1]+p_step;
			}
		}
}

/////////////////////////////////math/////////////////////////////////////////

int float_to_uint(float x, float x_min, float x_max, int bits){
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
    }

float uint_to_float(int x_int, float x_min, float x_max, int bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
    }

////////////////////////softstop//////////////////////////////////
//actuator only can move to a limit position

int softstop_joint(float *control,float state, float limit_p, float limit_n){
  if(*control>=limit_p){
    *control = limit_p;
    v_in = 0.0f;
    kp_in = 0.0f;
    kd_in = KD_SOFTSTOP;
    t_in += KP_SOFTSTOP*(limit_p - state);
    return 1;
    }
  if(*control<=limit_n){
    *control = limit_n;
    v_in = 0.0f;
    kp_in = 0.0f;
    kd_in = KD_SOFTSTOP;
    t_in += KP_SOFTSTOP*(limit_n - state);
    return 1;
    }
  }

////////////////////////delay//////////////////////////////////
void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while ((uint16_t)__HAL_TIM_GET_COUNTER(&htim1) < us);
}

////////////////////////spi//////////////////////////////////
uint32_t xor_checksum(uint32_t* data, int len)
{
    uint32_t t = 0;
    for(int i = 0; i < len; i++)
        t = t ^ data[i];
    return t;
}

void spi_send_receive(void){

	//calculatet the checksum
	//pack the status variables into the tx buffer
	state.checksum = xor_checksum((uint32_t*)&state,14);
	for(int i = 0; i < STATE_LEN ; i++){
		spi_tx_buffer[i] = ((uint16_t*)(&state))[i];
		}


	//SPI transmission and receive
//	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15)==0){
//		spi_enabled = 1;
		//HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)spi_tx_buffer, (uint8_t *)spi_rx_buffer, RX_LEN, HAL_MAX_DELAY); //HAL_MAX_DELAY
//		HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t *)spi_tx_buffer, (uint8_t *)spi_rx_buffer, RX_LEN);
		//HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)spi_tx_buffer, (uint8_t *)spi_rx_buffer, RX_LEN);
//		}

	//unpack the received bytes from rx buffer into †he valuesrec structur
	/*
	for(int i = 0; i < RX_LEN; i++){
		((uint16_t*) &valuesrec)[i] = spi_rx_buffer[i];
		//printf("%d\n", spi_rx_buffer[i]);
		}
	//if the communication has no issues the values will write in the control structure
	if(keycontrol==0){
	check = xor_checksum((uint32_t*)&valuesrec,32);
	if(valuesrec.checksum == check && (valuesrec.flags[0]<=3 ||valuesrec.flags[1]<=3)){
		for(int i = 0; i < CONTROL_LEN; i++){
			((uint16_t*) &control)[i] = ((uint16_t*) &valuesrec)[i];
		    }
		}
	}
*/

}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef * hspi)
{
	//unpack the received bytes from rx buffer into †he valuesrec structur
		callback_enabled = 1;
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15)==0){
			for(int i = 0; i < RX_LEN; i++){
				((uint16_t*) &valuesrec)[i] = spi_rx_buffer[i];
				//printf("%d\n", spi_rx_buffer[i]);
				}
			//if the communication has no issues the values will write in the control structure
			if(keycontrol==0){
			check = xor_checksum((uint32_t*)&valuesrec,32);

			//Retrieve the current control Mode and reset flags to its value
			currentControlMode = (valuesrec.flags[0]>>16);
			valuesrec.flags[0] = (valuesrec.flags[0] & 0xFFFF);
			valuesrec.flags[1] = (valuesrec.flags[0] & 0xFFFF);

			if(valuesrec.checksum == check && (valuesrec.flags[0]<=3 ||valuesrec.flags[1]<=3)){
				for(int i = 0; i < CONTROL_LEN; i++){
					((uint16_t*) &control)[i] = ((uint16_t*) &valuesrec)[i];
					}
				}
			}
		}
		// Disable the SPI //vishnu : I think this actually enables SPI callback for next
		HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t *)spi_tx_buffer, (uint8_t *)spi_rx_buffer, RX_LEN);
//		HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)spi_tx_buffer, (uint8_t *)spi_rx_buffer, RX_LEN);


		spi_enabled = 0;

}

void HAL_SPI_ErrorCallback (SPI_HandleTypeDef* hspi){
	CallbackError_spi=HAL_SPI_GetError(&hspi1);

	HAL_SPI_DeInit(&hspi1);
	HAL_SPI_Init(&hspi1);

	HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t *)spi_tx_buffer, (uint8_t *)spi_rx_buffer, RX_LEN);
}

/* USER CODE END 4 */

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
