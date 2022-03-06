/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PID_v1.h"
#include <string.h>
#include <stdbool.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/*typedef struct HC05_CommandStruct{
	char HC05_AT[4];
	char HC05_UART[10];
	char HC05_ROLE[10];
	char HC05_ADDR[10];

	char HC05_ROLE_SLAVE[11];
	char HC05_ROLE_MASTER[11];
	char HC05_CMODE_STATIC[12];
	char HC05_BIND_ADDR[24];
	char HC05_UART_9600_0_0[19];
}HC05_CommandStruct;

HC05_CommandStruct HC05_Commands = {
		"AT\r\n",
		"AT+UART?\r\n",
		"AT+ROLE?\r\n",
		"AT+ADDR?\r\n",
		"AT+ROLE=0\r\n",
		"AT+ROLE=1\r\n",
		"AT+CMODE=0\r\n",
		"AT+BIND=0021,06,0853a7\r\n",
		"AT+UART=9600,0,0\r\n"
};*/


typedef struct{
	uint16_t  Period_Min;
	uint16_t  Period_Max;
}Servo_Info;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RECEIVED_DATA_ARRAY_SIZE 256
#define TRANSMIT_DATA_ARRAY_SIZE 64
#define SELECT_HC05_MASTER 0x01
#define SELECT_HC05_SLAVE 0x00

#define SERVO_NUM 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
static uint8_t received_data_array[RECEIVED_DATA_ARRAY_SIZE];
static uint8_t data_array[TRANSMIT_DATA_ARRAY_SIZE];
uint16_t rx_len = 0;
uint8_t rx_array_index = 0;
uint8_t skip_first_measurement = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC2_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void HC05_SendCommand(uint8_t HC05_select, char command[]);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// PID KOD BÖLMESİ
double setpoint_0 = 2000;
double setpoint_1 = 2000;

double Kp_0 = 0.3;
double Ki_0 = 0.03;
double Kd_0 = 0.13;

double Kp_1 = 0.3;
double Ki_1 = 0.03;
double Kd_1 = 0.13;

uint16_t x_coordinate = 0;
uint16_t y_coordinate = 0;

double input_0;
double input_1;

double output_0;
double output_1;

PID servo_PID_0(&input_0, &output_0, &setpoint_0, Kp_0, Ki_0, Kd_0, REVERSE);
PID servo_PID_1(&input_1, &output_1, &setpoint_1, Kp_1, Ki_1, Kd_1, REVERSE);

uint32_t servo_0_CCR = 78;
uint32_t servo_1_CCR = 78;

uint8_t object_detected = 0;
uint8_t no_touch_count = 0;


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
  MX_ADC2_Init();
  MX_USART6_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */


  HAL_ADC_Start(&hadc2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  servo_PID_0.SetMode(AUTOMATIC);
  servo_PID_1.SetMode(AUTOMATIC);
  servo_PID_0.SetOutputLimits(61, 95);
  servo_PID_1.SetOutputLimits(61, 95);
  servo_PID_0.SetSampleTime(100);
  servo_PID_1.SetSampleTime(100);

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL = DWT_CTRL_CYCCNTENA_Msk;



	  // 75 TAM ORTA NOKTASI (pervane yatay duruyor)
	  // 78 İSE MASANIN HATALI EĞİMİNİ ENGELLEMEK İÇİN, ASIL ORTA NOKTA BU



  // HC05 CONFIG CODE BLOCK

  // Set huart2 slave.
  /*HAL_GPIO_WritePin(HC05_SLAVE_KEY_GPIO_Port, HC05_SLAVE_KEY_Pin, GPIO_PIN_SET);
  HC05_SendCommand(SELECT_HC05_SLAVE, HC05_Commands.HC05_AT);
  HC05_SendCommand(SELECT_HC05_SLAVE, HC05_Commands.HC05_ROLE_SLAVE);
  HC05_SendCommand(SELECT_HC05_SLAVE, HC05_Commands.HC05_UART_9600_0_0);
  HC05_SendCommand(SELECT_HC05_SLAVE, HC05_Commands.HC05_ADDR);
  for(uint8_t i = 0; i < 4; i++)
  {
	  HC05_Commands.HC05_BIND_ADDR[i+8] = received_data_array[rx_array_index-20+i];
  }
  for(uint8_t i = 0; i < 2; i++)
  {
	  HC05_Commands.HC05_BIND_ADDR[i+13] = received_data_array[rx_array_index-15+i];
  }
  for(uint8_t i = 0; i < 6; i++)
  {
	  HC05_Commands.HC05_BIND_ADDR[i+16] = received_data_array[rx_array_index-12+i];
  }
  HAL_GPIO_WritePin(HC05_SLAVE_KEY_GPIO_Port, HC05_SLAVE_KEY_Pin, GPIO_PIN_RESET);*/

  // Set huart6 master.
  /*HAL_GPIO_WritePin(HC05_MASTER_KEY_GPIO_Port, HC05_MASTER_KEY_Pin, GPIO_PIN_SET);
  HC05_SendCommand(SELECT_HC05_MASTER, HC05_Commands.HC05_AT);
  HC05_SendCommand(SELECT_HC05_MASTER, HC05_Commands.HC05_ROLE_MASTER);
  HC05_SendCommand(SELECT_HC05_MASTER, HC05_Commands.HC05_CMODE_STATIC);
  HC05_SendCommand(SELECT_HC05_MASTER, HC05_Commands.HC05_BIND_ADDR);
  HC05_SendCommand(SELECT_HC05_MASTER, HC05_Commands.HC05_UART_9600_0_0);
  HAL_GPIO_WritePin(HC05_MASTER_KEY_GPIO_Port, HC05_MASTER_KEY_Pin, GPIO_PIN_RESET);*/


  /*while(1)
  {
	  HAL_UART_Transmit(&huart6, (uint8_t*)"Hello", 5, HAL_MAX_DELAY);
	  HAL_Delay(1000);
  }*/


  /*char at_UART[] = "AT+UART=9600,1,0\r\n";
  HAL_UART_Transmit(&huart6, (uint8_t*)at_UART, 18, HAL_MAX_DELAY);
  HAL_UART_Receive(&huart6, &received_data_array[4], 4, 1000);
  char at_UART_sorgu[] = "AT+UART?\r\n";
  HAL_UART_Transmit(&huart6, (uint8_t*)at_UART_sorgu, 10, HAL_MAX_DELAY);
  HAL_UART_Receive(&huart6, &received_data_array[8], 14, 1000);
  char at_ROLE[] = "AT+ROLE=1\r\n";
  HAL_UART_Transmit(&huart6, (uint8_t*)at_ROLE, 11, HAL_MAX_DELAY);
  HAL_UART_Receive(&huart6, &received_data_array[8], 4, 1000);
  HAL_GPIO_WritePin(HC05_KEY_GPIO_Port, HC05_KEY_Pin, GPIO_PIN_RESET);
  */

  //HAL_UARTEx_ReceiveToIdle(&huart6, received_data_array, 100, actual_received_data_count, HAL_MAX_DELAY);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t start_ms_time = HAL_GetTick();
  while (1)
  {
	  uint32_t ms_time = HAL_GetTick() - start_ms_time;
	  uint32_t start_tick = DWT->CYCCNT;
	  // Setting up x read
	  // Setting "right" ports HIGH and setting "left" ports LOW
	  HAL_GPIO_WritePin(SCREEN_TOP_RIGHT_GPIO_Port, SCREEN_TOP_RIGHT_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(SCREEN_BOTTOM_RIGHT_GPIO_Port, SCREEN_BOTTOM_RIGHT_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(SCREEN_TOP_LEFT_GPIO_Port, SCREEN_TOP_LEFT_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(SCREEN_BOTTOM_LEFT_GPIO_Port, SCREEN_BOTTOM_LEFT_Pin, GPIO_PIN_RESET);
	  // Waiting for 25 ms for panel to settle
	  HAL_Delay(25); // 25 ms
	  // Waiting for conversion (polling) and reading x value
	  HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
	  x_coordinate = HAL_ADC_GetValue(&hadc2); // 16 bitlik degisken

	  // Setting up y read
	  // Setting "top" ports HIGH and setting "bottom" ports LOW
	  HAL_GPIO_WritePin(SCREEN_BOTTOM_RIGHT_GPIO_Port, SCREEN_BOTTOM_RIGHT_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(SCREEN_BOTTOM_LEFT_GPIO_Port, SCREEN_BOTTOM_LEFT_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(SCREEN_TOP_LEFT_GPIO_Port, SCREEN_TOP_LEFT_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(SCREEN_TOP_RIGHT_GPIO_Port, SCREEN_TOP_RIGHT_Pin, GPIO_PIN_SET);
	  // Waiting for 25 ms for panel to settle
	  HAL_Delay(25);
	  // Waiting for conversion (polling) and reading y value
	  HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
	  y_coordinate = HAL_ADC_GetValue(&hadc2); // 16 bitlik degisken


	  if(skip_first_measurement == 5)
	  {
		  if(x_coordinate < 1000 || y_coordinate < 1000)
		  {
			  object_detected = 0;
			  no_touch_count++;
			  if(no_touch_count == 100)
			  {
				  // TOP YOKSA MASAYI DÜZLEŞTİRİR
				  htim3.Instance->CCR1 = 78;
				  htim4.Instance->CCR1 = 78;
			  }
		  }
		  else
		  {
			  object_detected = 1;
			  no_touch_count = 0;

			  input_0 = x_coordinate;
			  input_1 = y_coordinate;

			  servo_PID_0.Compute();
			  servo_PID_1.Compute();

			  servo_0_CCR = round(output_0);
			  servo_1_CCR = round(output_1);

			  htim3.Instance->CCR1 = (uint32_t)servo_0_CCR;
			  htim4.Instance->CCR1 = (uint32_t)servo_1_CCR;

		  }

		  data_array[0] = object_detected;

		  uint16_t x_coordinate_temp = x_coordinate;
		  uint8_t digit = 0;
		  uint8_t index = 4;
		  while(index != 0)
		  {
			  digit = x_coordinate_temp % 10;
			  data_array[index] = digit + 0x30; // basamaga, 0x30 (48) ekliyoruz ki ASCII karakterine çevrilsin ve
			  // LCD'de basilsin.
			  index--;
			  x_coordinate_temp = x_coordinate_temp / 10;
		  }

		  uint16_t y_coordinate_temp = y_coordinate;
		  digit = 0;
		  index = 8;
		  while(index != 4)
		  {
			  digit = y_coordinate_temp % 10;
			  data_array[index] = digit + 0x30;
			  index--;
			  y_coordinate_temp = y_coordinate_temp / 10;
		  }
		  /*data_array[1] = (uint8_t)(x_coordinate >> 8);
		  data_array[2] = (uint8_t)x_coordinate;
		  data_array[3] = (uint8_t)(y_coordinate >> 8);
		  data_array[4] = (uint8_t)y_coordinate;*/
		  if(ms_time > 500)
		  {
			  HAL_UART_Transmit(&huart6, data_array, 9, HAL_MAX_DELAY);
			  start_ms_time = HAL_GetTick();
		  }

	  }
	  else
	  {
		  skip_first_measurement++;
	  }
	  uint32_t tick_count = DWT->CYCCNT - start_tick;


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
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1679;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1679;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  huart2.Init.BaudRate = 9600;
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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, HC05_SLAVE_KEY_Pin|SCREEN_TOP_RIGHT_Pin|SCREEN_BOTTOM_RIGHT_Pin|HC05_MASTER_KEY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SCREEN_TOP_LEFT_Pin|SCREEN_BOTTOM_LEFT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Audio_RST_GPIO_Port, Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : HC05_SLAVE_KEY_Pin SCREEN_TOP_RIGHT_Pin SCREEN_BOTTOM_RIGHT_Pin HC05_MASTER_KEY_Pin */
  GPIO_InitStruct.Pin = HC05_SLAVE_KEY_Pin|SCREEN_TOP_RIGHT_Pin|SCREEN_BOTTOM_RIGHT_Pin|HC05_MASTER_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SCREEN_TOP_LEFT_Pin SCREEN_BOTTOM_LEFT_Pin */
  GPIO_InitStruct.Pin = SCREEN_TOP_LEFT_Pin|SCREEN_BOTTOM_LEFT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Audio_RST_Pin */
  GPIO_InitStruct.Pin = Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Audio_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HC05_SendCommand(uint8_t HC05_select, char command[])
{
	UART_HandleTypeDef* huart_ptr = HC05_select ? &huart6 : &huart2;
	//UART_HandleTypeDef* huart_ptr = &huart6;
	HAL_UART_Transmit(huart_ptr, (uint8_t*)command, strlen(command), 1000);
	//HAL_UARTEx_Receive(&huart6, &received_data_array[0], , 1000);
	HAL_UARTEx_ReceiveToIdle(huart_ptr, &received_data_array[rx_array_index], RECEIVED_DATA_ARRAY_SIZE, &rx_len, 1000);
	rx_array_index += (uint8_t)rx_len;
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

