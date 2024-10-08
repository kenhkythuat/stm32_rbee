/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32l0xx_hal.h"
#include "config.h"
#include "string.h"
#include "stdbool.h"
#include "stdio.h"
#include "stdlib.h"
#include <math.h>

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
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char AT_RESET[] = "AT+CRESET\r\n";
char AT_CHECK_A76XX[] = "AT\r\n";
char ATE0[] = "ATE0\r\n";
char AT_CHECK_ESIM[] = "AT+CGREG?\r\n";
char AT_SIGNAL_SIM[] = "AT+CSQ\r\n";
char AT_START_MQTT[] = "AT+CMQTTSTART\r\n";
char AT_ACQUIRE_CLIENT[] = "AT+CMQTTACCQ=0,\"%s\",0\r\n";
//char AT_CONNECT_MQTT[]="AT+CMQTTCONNECT=0,\"%s:%d\",60,1,\"%s\",\"%s\"\r\n";
char AT_CONNECT_MQTT[] = "AT+CMQTTCONNECT=0,\"%s:%d\",60,1,\"%s\",\"%s\"\r\n";

char AT_SET_PUBLISH_TOPIC[] = "AT+CMQTTTOPIC=0,%d\r\n";
char AT_SET_PUBLISH_PAYLOAD[] = "AT+CMQTTPAYLOAD=0,%d\r\n";
char AT_PUBLISH[] = "AT+CMQTTPUB=0,1,60\r\n";
char AT_SUBCRIBE_TOPIC[] = "%s%d\r\n";
char AT_SUBCRIBE[] = "AT+CMQTTSUB=0\r\n";
char AT_COMMAND[200];
char AT_DISCONNECT[] = "AT+CMQTTDISC=0,120";
char BUFFER_TOPPIC_MQTT[200];
char BUFFER_DATA_PAYLOAD_MQTT[100];
//char TOPPIC_PAYLOAD_MQTT[]="{\"WaterLevel\":%.1f}\r\n";
char TOPPIC_PAYLOAD_MQTT[] =
		"{\"WaterLevel\":%.1f,\"_battery_level\":%.1f,\"_gsm_signal_strength\":%d}\r\n";
//char TOPPIC_PAYLOAD_MQTT[]="{\"WaterLevel\":%.1f,\"Battery\":%.1f,\"RSSI\":%.1f}\r\n";
char water[50] = "-123";

int previousTick = 0;
int timeOutConnectA76XX = 40000;
int timeOutConnectMQTT = 40000;
int isATOK = 0;
int isPBDONE = 0;
int payLoadPin, payLoadStatus;

int rxDataCouter = 0;
int numberLoads = 10;
char simcomRxBuffer[100];
int payloadIndex;

uint8_t cnt_1 = 0;
uint8_t rxData;
uint16_t rxIndex;
uint8_t loadflag = 0;
char rxBuffer[50];

uint8_t isConnectSimcomA76xx = 0;
uint8_t isConnectMQTT = 0;
uint8_t sendPayloadStatusToServer;
uint8_t isSleepMode = 0;

//User code sensor distance
uint32_t ICValue = 0;
uint32_t Frequency = 0;
float Duty = 0;
float Distance_water = 0;
uint8_t cnt_start_sleep = 0;
// Flash
float SaveDataWater = 0;
float isDistance = 0;
int intDataWater = 0;
int floatDataWater = 0;
int Negative = 0;
float absChange = 0;
int big_change;

// Query signal quality
float SignalStrength = 0;
// BatteryLevel
uint32_t adcValue = 0;
float BatteryLevel = 0;
float PercentageBattery = 0;
uint32_t arrayBattery[50];
uint32_t temp;
uint8_t times = 10;
//int time_Period=0;
int Filter_Baterry_Values();
int MediumBattery();

uint32_t ADC_VAL;
float Vin = 0;
uint8_t test1 = 0;
uint8_t A7677S_DONE = 0;
uint8_t A7670C_DONE = 0;
// VL53L1
char rx_VL53L1[8];
int distance;
unsigned char data_vl53l1[3] = { 0xA5, 0x15, 0xBA };
unsigned char save_mode[3] = { 0xA5, 0x25, 0xCA };
uint32_t medium_distance[30];
uint32_t temp_distance;
float water_level;
int test123=0;

#if wt53r_ttl
//unsigned char data_w53r_ttl[8] = { 0x50, 0x06, 0x00, 0x03, 0x00, 0x03, 0x34, 0x4A};
unsigned char save_mode_1hz[8] = { 0x50, 0x06, 0x00, 0x03, 0x00, 0x03, 0x34, 0x4A};
unsigned char save_mode_0_2hz[8] = { 0x50, 0x06, 0x00, 0x03, 0x00, 0x06, 0xF4, 0x49};
char rx_wt53r_ttl[80];
char data_w53r_ttl[5];
#endif
int rssi = -99;
void RunningSendData();

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void turnOnA76XX(void);	//reset module sim A76XX
void sendingToSimcomA76xx(char *cmd);
void Filter_Value(void);
int connectMQTT();
int Sleep_Stm32_A7672S();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Flash_Erase(uint32_t address) {
	HAL_FLASH_Unlock();
	FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase = 1;
	EraseInitStruct.NbPages = 1;
	EraseInitStruct.PageAddress = address;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	uint32_t pagerr;
	HAL_FLASHEx_Erase(&EraseInitStruct, &pagerr);
	HAL_FLASH_Lock();
}
void Flash_Write_Init(uint32_t address, int value) {
	HAL_FLASH_Unlock();
	HAL_FLASH_Program(FLASH_TYPEPROGRAMDATA_WORD, address, value);
	HAL_FLASH_Lock();
}
int Flash_Read_Init(uint32_t address) {
	return *(__IO uint16_t*) (address);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == htim6.Instance) {
		// cnt_1++;
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) // Chon ngat tai channel 1 | cubeMX (PWM input on CH1)
			{
		// Read the IC value
		ICValue = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1); // Do chu ky PWM tu TIM1

		if (ICValue != 0) {
			// calculate the Duty Cycle
			Duty = (float) (HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2)
					* 100) / ICValue; // Do do rong xung muc cao
#if tube_length==110
			if (tube_length == 110) {
				Distance_water = 70 - ((Duty * 50) / 10);
			}
#endif
#if tube_length==90
			if (tube_length == 90) {
				Distance_water = 50 - ((Duty * 50) / 10);
			}
#endif
#if tube_length==70
			if (tube_length == 70) {
				Distance_water = 30 - ((Duty * 50) / 10);
			}
#endif
//			else
//			Distance_water=((Duty*50)/10); //don vi CM
			//Frequency = (TIMCLOCK / PRESCALER) / ICValue;
		}
	}
}

int connectSimcomA76xx() {
	//HAL_TIM_Base_Stop_IT(&htim6);
	previousTick = HAL_GetTick();

	while (isConnectSimcomA76xx == 0
			&& (previousTick + timeOutConnectA76XX) > HAL_GetTick()) {
		if (strstr((char*) rxBuffer, "PB DONE")) {
			isPBDONE = 1;
		}
//		if(strstr((char *)rxBuffer,"PDN ACT 1")){
//			A7677S_DONE = 1;
//			isPBDONE=1;
//			HAL_Delay(7000);
//		}
		if (isPBDONE == 1) {
			sendingToSimcomA76xx(ATE0);
			HAL_Delay(200);
			sendingToSimcomA76xx(AT_SIGNAL_SIM);
			HAL_Delay(200);
			SignalStrength = (rxBuffer[8] - 48) * 10 + (rxBuffer[9] - 48);
			if(SignalStrength>=31)
			{
				rssi=-51;
			}else rssi = (SignalStrength * 2 - 113);
			isConnectSimcomA76xx = 1;
			HAL_Delay(1000);
			return isConnectSimcomA76xx;
		}
	}
	if (isConnectSimcomA76xx == 0) {
		NVIC_SystemReset();
	}

	return isConnectSimcomA76xx;
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart -> Instance == USART1)
//	{
//		if((rxData!='\r')&&(rxData!='\n')){
//			simcomRxBuffer[rxIndex++]=rxData;
//			rxDataCouter++;
//		}
//		else{
//			rxDataCouter=0;
//			rxIndex=0;
//
//		}
//	}
//	HAL_UART_Receive_IT(&huart1, &rxData,1);
//}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (huart->Instance == USART1) {
		if (strstr((char*) rxBuffer, "ERROR")) {
			NVIC_SystemReset();
		}
		if (strstr((char*) rxBuffer, "unknown")) {
			NVIC_SystemReset();
		}
		HAL_UARTEx_ReceiveToIdle_IT(huart, (uint8_t*) rxBuffer, 50);
	}
	if (huart->Instance == USART2) {
#if vl53l1
		Distance_water = (rx_VL53L1[4] << 8 | rx_VL53L1[5]);
		HAL_UARTEx_ReceiveToIdle_IT(&huart2, (uint8_t*) rx_VL53L1, 8);
#endif
#if wt53r_ttl
		//Distance_water= (char)rx_wt53r_ttl[36];

		sprintf(data_w53r_ttl,"%c%c%c%c",rx_wt53r_ttl[34],rx_wt53r_ttl[35],rx_wt53r_ttl[36],rx_wt53r_ttl[37]);
		Distance_water=atoi(data_w53r_ttl);
		HAL_UARTEx_ReceiveToIdle_IT(&huart2, (uint8_t*) rx_wt53r_ttl, 80);
#endif
	}

}

void sendingToSimcomA76xx(char *cmd) {
	HAL_UART_Transmit(&huart1, (uint8_t*) cmd, strlen(cmd), 1000);
}
void read_vl53l1(uint8_t *data) {
	HAL_UART_Transmit(&huart2, data_vl53l1, 9, 1000);
}
void writte_mode_return_rate(uint8_t *data) {
	HAL_UART_Transmit(&huart2, save_mode_0_2hz, 9, 1000);
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
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_ADC_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	// HAL_TIM_Base_Start_IT(&htim6);
	HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);

	//sendingToSimcomA76xx(AT_RESET);

//  HAL_UART_Receive_IT(&huart1, &rxData,1);
	HAL_UARTEx_ReceiveToIdle_IT(&huart1, (uint8_t*) rxBuffer, 50);
#if vl53l1
	HAL_UARTEx_ReceiveToIdle_IT(&huart2, (uint8_t*) rx_VL53L1, 8);
#endif
#if wt53r_ttl
	HAL_UARTEx_ReceiveToIdle_IT(&huart2, (uint8_t*) rx_wt53r_ttl, 80);
#endif

	intDataWater = Flash_Read_Init(DATA_INT_WATER);
	floatDataWater = Flash_Read_Init(DATA_FLOAT_WATER);
	Negative = Flash_Read_Init(DATA_NEGATIVE_WATER);
	if (Negative == 2) {
		SaveDataWater = ((float) (intDataWater + (float) floatDataWater * 0.1))
				* -1;
	} else {
		SaveDataWater = (float) (intDataWater + (float) floatDataWater * 0.1);
	}
	turnOnA76XX();
	writte_mode_return_rate(save_mode_0_2hz);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//HAL_GPIO_WritePin(OPEN_SENSOR_GPIO_Port, OPEN_SENSOR_Pin, GPIO_PIN_SET);
		if (!isConnectSimcomA76xx) {
			isConnectSimcomA76xx = connectSimcomA76xx();
		}
		if (!isConnectMQTT) {
			isConnectMQTT = connectMQTT();
		}
		if (!isSleepMode) {
			isSleepMode = Sleep_Stm32_A7672S();
		}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_3;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_SYSCLK;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = ENABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 59999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = time_Period;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OPEN_SENSOR_GPIO_Port, OPEN_SENSOR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ENABLE_A7672C_GPIO_Port, ENABLE_A7672C_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : OPEN_SENSOR_Pin */
  GPIO_InitStruct.Pin = OPEN_SENSOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OPEN_SENSOR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_STATUS_Pin ENABLE_A7672C_Pin */
  GPIO_InitStruct.Pin = LED_STATUS_Pin|ENABLE_A7672C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int Filter_Baterry_Values() {
	for (int i = 0; i < times; i++) {
		HAL_ADC_Start(&hadc);
		HAL_ADC_PollForConversion(&hadc, 1000);
		HAL_Delay(100);
		arrayBattery[i] = HAL_ADC_GetValue(&hadc);
	}
	adcValue = MediumBattery();
	// vol max 2.1 vol min 1.25
	BatteryLevel = ((float) (adcValue / 4095.00) * 3.3);
	PercentageBattery = ((BatteryLevel / 2.1) * 100);
	if (PercentageBattery > 100) {
		PercentageBattery = 100;
	}
	HAL_ADC_Stop(&hadc);

	return PercentageBattery;
}
int MediumBattery() {
	for (int i = 0; i < times - 1; i++) {
		for (int j = i + 1; j < times; j++) {
			if (arrayBattery[i] > arrayBattery[j]) {

				temp = arrayBattery[i];

				arrayBattery[i] = arrayBattery[j];

				arrayBattery[j] = temp;

			}
		}
	}
	return arrayBattery[times / 2];
}

void turnOnA76XX() {
	HAL_GPIO_WritePin(ENABLE_A7672C_GPIO_Port, ENABLE_A7672C_Pin,
			GPIO_PIN_RESET);
	HAL_Delay(3000);
	HAL_GPIO_WritePin(ENABLE_A7672C_GPIO_Port, ENABLE_A7672C_Pin, GPIO_PIN_SET);
	HAL_Delay(4000);
	HAL_GPIO_WritePin(ENABLE_A7672C_GPIO_Port, ENABLE_A7672C_Pin,
			GPIO_PIN_RESET);
	HAL_Delay(300);
	HAL_GPIO_WritePin(ENABLE_A7672C_GPIO_Port, ENABLE_A7672C_Pin, GPIO_PIN_SET);
	HAL_Delay(3000);
	//Start sensor
	HAL_GPIO_WritePin(OPEN_SENSOR_GPIO_Port, OPEN_SENSOR_Pin, GPIO_PIN_SET);
}
int get_data_sensor(int times)
{
	for (int i = 0; i < times; i++) {
		//read_vl53l1(data_vl53l1);
		HAL_Delay(200);
		medium_distance[i] = Distance_water;
	}
	for (int i = 0; i < times - 1; i++) {
		for (int j = i + 1; j < times; j++) {
			if (medium_distance[i] > medium_distance[j]) {

				temp_distance = medium_distance[i];

				medium_distance[i] = medium_distance[j];

				medium_distance[j] = temp_distance;
			}
		}
	}
	return medium_distance[times/2];
}
int abnormal_sensor_data(int difference)
{
	absChange = fabs(water_level - SaveDataWater);
	if (absChange > difference)
		return 1;
	 else
		return 0;
	//The difference is in cm
}
int connectMQTT(void) {

#if wt53r_ttl
	//test123=get_data_sensor(30);
	water_level = (48.5-(get_data_sensor(30) * 0.1));
#endif
	big_change=abnormal_sensor_data(3);
	if(big_change)
	{
		get_data_sensor(30);
		water_level = (48.5-(get_data_sensor(30) * 0.1));
	}
	Filter_Value();
	HAL_GPIO_WritePin(OPEN_SENSOR_GPIO_Port, OPEN_SENSOR_Pin, GPIO_PIN_RESET);
	sendingToSimcomA76xx(ATE0);
	HAL_Delay(200);
	sendingToSimcomA76xx(AT_START_MQTT);
	HAL_Delay(200);
	sprintf(AT_COMMAND, AT_ACQUIRE_CLIENT, MQTT_CLIENT_ID);
	sendingToSimcomA76xx(AT_COMMAND);
	HAL_Delay(500);
	sprintf(AT_COMMAND, AT_CONNECT_MQTT, MQTT_HOST, MQTT_PORT, MQTT_USER,
	MQTT_PASS);
	sendingToSimcomA76xx(AT_COMMAND);
	HAL_Delay(1000);
	sprintf(BUFFER_TOPPIC_MQTT, "%s/sn/%s", FARM, MQTT_CLIENT_ID);
	sprintf(AT_COMMAND, AT_SET_PUBLISH_TOPIC, (int) strlen(BUFFER_TOPPIC_MQTT));
	sendingToSimcomA76xx(AT_COMMAND);
	HAL_Delay(500);
	sendingToSimcomA76xx(BUFFER_TOPPIC_MQTT);
	HAL_Delay(500);
	Filter_Baterry_Values();
	sprintf(BUFFER_DATA_PAYLOAD_MQTT, TOPPIC_PAYLOAD_MQTT, water_level,
			PercentageBattery, rssi);
//	sprintf(BUFFER_DATA_PAYLOAD_MQTT,TOPPIC_PAYLOAD_MQTT,Distance_water);
	HAL_Delay(500);
	sprintf(AT_COMMAND, AT_SET_PUBLISH_PAYLOAD,
			(int) strlen(BUFFER_DATA_PAYLOAD_MQTT));
	sendingToSimcomA76xx(AT_COMMAND);
	HAL_Delay(500);
	sendingToSimcomA76xx(BUFFER_DATA_PAYLOAD_MQTT);
	HAL_Delay(500);
	sendingToSimcomA76xx(AT_PUBLISH);
	HAL_Delay(500);
	isConnectMQTT = 1;
	return 1;
}
int Sleep_Stm32_A7672S() {
	//Sleep Simcom A7672S

	HAL_GPIO_WritePin(ENABLE_A7672C_GPIO_Port, ENABLE_A7672C_Pin,
			GPIO_PIN_RESET);
	HAL_Delay(4000);
	//sendingToSimcomA76xx("AT+CPOF");
	HAL_GPIO_WritePin(ENABLE_A7672C_GPIO_Port, ENABLE_A7672C_Pin, GPIO_PIN_SET);
	HAL_Delay(6000);
	HAL_GPIO_WritePin(GPIOB, LED_STATUS_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OPEN_SENSOR_GPIO_Port, OPEN_SENSOR_Pin, GPIO_PIN_RESET);
	HAL_TIM_Base_Start_IT(&htim6);
	//CLose sensor
	//HAL_GPIO_WritePin(GPIOB, LED_STATUS_Pin,GPIO_PIN_RESET);
	HAL_SuspendTick();
	HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFE);
	HAL_ResumeTick();
	NVIC_SystemReset();
	isSleepMode = 1;

	return isSleepMode;
}
void Filter_Value(void) {

	absChange = fabs(water_level - SaveDataWater);
	if (water_level > 0 && absChange > 0.5) {
		Negative = 1;
		isDistance = water_level;
		Flash_Erase(DATA_NEGATIVE_WATER);
		Flash_Write_Init(DATA_NEGATIVE_WATER, (uint32_t) Negative);

		intDataWater = (int) isDistance;
		floatDataWater = ((int) (isDistance * 10) % 10);

		Flash_Erase(DATA_INT_WATER);
		Flash_Write_Init(DATA_INT_WATER, (uint32_t) intDataWater);

		Flash_Erase(DATA_FLOAT_WATER);
		Flash_Write_Init(DATA_FLOAT_WATER, (uint32_t) floatDataWater);
	}
	if (water_level < 0 && absChange > 0.5) {

		Negative = 2;
		isDistance = water_level * -1;
		Flash_Erase(DATA_NEGATIVE_WATER);
		Flash_Write_Init(DATA_NEGATIVE_WATER, (uint32_t) Negative);

		intDataWater = (int) isDistance;
		floatDataWater = ((int) (isDistance * 10) % 10);

		Flash_Erase(DATA_INT_WATER);
		Flash_Write_Init(DATA_INT_WATER, (uint32_t) intDataWater);

		Flash_Erase(DATA_FLOAT_WATER);
		Flash_Write_Init(DATA_FLOAT_WATER, (uint32_t) floatDataWater);
	}
	if (absChange < 0.5) {
		water_level = SaveDataWater;
	}

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
	while (1) {
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
