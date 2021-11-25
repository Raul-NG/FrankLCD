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
  ***************************************-***************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <i2c-lcd.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define R1_PORT GPIOB
#define R1_PIN GPIO_PIN_11

#define R2_PORT GPIOB
#define R2_PIN GPIO_PIN_10

#define R3_PORT GPIOE
#define R3_PIN GPIO_PIN_15

#define R4_PORT GPIOE
#define R4_PIN GPIO_PIN_6

#define C1_PORT GPIOE
#define C1_PIN GPIO_PIN_12

#define C2_PORT GPIOE
#define C2_PIN GPIO_PIN_10

#define C3_PORT GPIOE
#define C3_PIN GPIO_PIN_7

#define C4_PORT GPIOE
#define C4_PIN GPIO_PIN_8

#define MPU6050 0XD0
#define WHO_AM_I 0x75
#define SMPLRT_DIV 0x19
#define PWR_MGMT_1 0x6B
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42

#define LEN 40

#define PWM_MIN -100
#define PWM_MAX 100
#define RPM_MIN -3300
#define RPM_MAX 3300
#define TEMP_MIN -40.0
#define TEMP_MAX 60.0

#define FREQ_M 100


#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
char i2c_buffer[16];
char uart_buf[100] = {0};
uint16_t uart_buf_len;

char buffer[9];
float temps[2];
int RPMS[2];
float err_TMP[2];
float err_RPM[2];
//float temperaturas[2];
int  temp_ref, RPM, PWM, error;
uint8_t key;

uint8_t lcdFlag, lcdImp_flag, lcdBlink_flag, lcdTmp_counter = 1;

char lcdTemp_buff[] = {' ',' ',' '};
char lcdTmp_temp_buff[] = {' ',' ',' '};
uint8_t cont =0;


int contT =0;
uint16_t tempADC;
float m1,m2,b1,b2;
uint8_t enter_flag = 1;
uint64_t counterRPM;

float K_pp = 0.1;
float K_pr = 0.1;
float K_d = 0.5;


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c4;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_I2C4_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM13_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t leerTeclado(void);
void procesarTeclado(int keyt);
float TMP_2_RPM(float error_tmp);
int RPM_2_PWM(int error_rpm);
void MPU6050_init(void);
volatile uint16_t adcData = 0;

void dispTempLCD(void);
void dispLcdText(void);
void clearTempBuff(void);
void lcdShiftTemp(void);
void editTempConfirmation(uint8_t PKey);

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
  int32_t timeout;
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_I2C4_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_TIM13_Init();
  /* USER CODE BEGIN 2 */
	lcd_init();
	MPU6050_init();
	TIM1->CCR2 = 100;
	m1 = (RPM_MAX - RPM_MIN)/(TEMP_MAX-TEMP_MIN);
	b1 = -m1*TEMP_MIN+RPM_MIN;
	m2 = (PWM_MAX - PWM_MIN)/(RPM_MAX - RPM_MIN);
	b2 = -m2*RPM_MIN+PWM_MIN;
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
    HAL_TIM_Base_Start_IT(&htim13);
    HAL_TIM_Base_Start_IT(&htim5);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adcData, 1);


//	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&temps[1], 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7)){
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
	  }else{
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		key = leerTeclado();
		if (key!= 0x16)
		{
		procesarTeclado(key);

		//lcd_send_string(temp_buff[0]);
		}

		uart_buf_len = sprintf(uart_buf,"prueba0\n");
		HAL_UART_Transmit(&huart3, (uint8_t*)uart_buf, uart_buf_len,100);

		err_RPM[0] = err_RPM[1];
		err_RPM[1] = TMP_2_RPM(temps[1] - temps[0]);
		RPMS[1] = RPMS[1] + K_pr*err_RPM[1] + (K_d/FREQ_M)*(err_RPM[1]-err_RPM[0]);

		error = RPM_2_PWM(RPMS[1] - RPMS[0]);
		PWM = PWM + K_pp*error;


	   if (PWM > PWM_MAX)
		PWM = PWM_MAX;
	   else if (PWM < PWM_MIN)
		PWM = PWM_MIN;

	   if (RPM > RPM_MAX)
		RPM = RPM_MAX;
	   else if (RPM < RPM_MIN)
		RPM = RPM_MIN;

	  TIM1->CCR2 = PWM;
	  HAL_Delay(100);
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 120;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV256;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x307075B1;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

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
  htim1.Init.Prescaler = 239;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
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
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 47999;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 1000;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

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
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 9;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE6 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE7 PE8 PE10 PE12 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
//	uart_buf_len = sprintf(uart_buf, "adcData: %i\n", adcData);
//	HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_buf_len, 100);

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adcData, 1);
}

void MPU6050_init(void){
	uint8_t check;
	uint8_t data = 0;
	HAL_I2C_Mem_Read(&hi2c4,MPU6050,WHO_AM_I,1,&check,1,1000);
	if(check == 0x68){
		HAL_I2C_Mem_Write(&hi2c4,MPU6050,PWR_MGMT_1,1,&data,1,1000);
		data = 0b00000111;
		HAL_I2C_Mem_Write(&hi2c4,MPU6050,SMPLRT_DIV,1,&data,1,1000);
	}
}

float MPU6050_READ_TEMP(void){
	uint8_t tempOut[2];
	HAL_I2C_Mem_Read(&hi2c4,MPU6050,TEMP_OUT_H,1,&tempOut[0],1,1000);
	HAL_I2C_Mem_Read(&hi2c4,MPU6050,TEMP_OUT_L,1,&tempOut[1],1,1000);
	int16_t temp = (int16_t)(tempOut[0] << 8 | tempOut[1]);
	return (temp/340.0) + 36.53;
}


void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef * htim){
	if (htim == &htim13){
		uart_buf_len = sprintf(uart_buf,"prueba  2\n");
		HAL_UART_Transmit(&huart3, (uint8_t*)uart_buf, uart_buf_len,100);
		temps[0] = temps[1];
		//temps[1] = MPU6050_READ_TEMP();
		temps[1] = (float)adcData*3.3*150.0/4095.0/1.5;

		dispTempLCD();
//		lcd_send_cmd(0xCA);
//		lcd_send_data(' ');
	}

	if (htim == &htim5){
		uart_buf_len = sprintf(uart_buf,"prueba   3\n");
		HAL_UART_Transmit(&huart3, (uint8_t*)uart_buf, uart_buf_len,100);
		counterRPM += 4294967295;
	}
}


void HAL_GPIO_EXIT_Callback(uint16_t GPIO_Pin){
	uart_buf_len = sprintf(uart_buf,"prueba 1\n");
	HAL_UART_Transmit(&huart3, (uint8_t*)uart_buf, uart_buf_len,100);

	if(GPIO_Pin == GPIO_PIN_7){

		counterRPM += __HAL_TIM_GET_COUNTER(&htim5);
		RPM = 60.0/((double)counterRPM/100000000.0);

		uart_buf_len = sprintf(uart_buf,"counterRPM: %llu \t RPM: %i\n", counterRPM, RPM);
		HAL_UART_Transmit(&huart3, (uint8_t*)uart_buf, uart_buf_len,100);

		counterRPM = 0;
		__HAL_TIM_SET_COUNTER(&htim5,0);
	}
}


uint8_t leerTeclado(void){
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_SET);

	HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_11);
	if(!(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_12))){
			return 1;
		}
	if(!(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_10))){
		return 2;
		}
	if(!(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_7))){
		return 3;
		}
	if(!(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_8))){
		return 10;
		}

	HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_11);
	HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_10);
	if(!(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_12))){
		return 4;
		}
	if(!(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_10))){
		return 5;
		}
	if(!(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_7))){
		return 6;
		}
	if(!(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_8))){
		return 11;
		}

	HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_10);
	HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_15);
	if(!(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_12))){
		return 7;
		}
	if(!(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_10))){
		return 8;
		}
	if(!(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_7))){
		return 9;
		}
	if(!(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_8))){
		return 12;
		}

	HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_15);
	HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_6);
	if(!(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_12))){
		return 14;
		}
	if(!(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_10))){
		return 0;
		}
	if(!(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_7))){
		return 15;
		}
	if(!(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_8))){
		return 13;
		}
	return 16;
}


float TMP_2_RPM(float error_tmp){
	return m1*(error_tmp) +b1;
}



int RPM_2_PWM(int error_rpm){
	return m2*(error_rpm) +b2;
}




void procesarTeclado(int keyt){
	 if(keyt<10){
		 lcdBlink_flag = 1;
		if(enter_flag){
			clearTempBuff();
			enter_flag = 0;
		}
		if(cont<3){
			lcdShiftTemp();
			cont++;
		}else{
			cont = 0;
		}

	 }else if(keyt == 10){
		 lcdBlink_flag = 0;
		 lcdImp_flag = 0;
		 editTempConfirmation(keyt);
		 lcdImp_flag = 1;
	 }else if(keyt == 13){
		temp_ref = (int)((lcdTemp_buff[2] == ' ' ? 0 : lcdTemp_buff[2] -48)) + (int)((lcdTemp_buff[1] == ' ' ? 0 : (lcdTemp_buff[1] -48)*10)) + (int)((lcdTemp_buff[0] == ' ' ? 0 : (lcdTemp_buff[0] -48)*100));
		cont=0;
		lcdImp_flag = 0;
		lcdBlink_flag = 0;
		enter_flag =1;
		editTempConfirmation(keyt);
	 }
	 lcdImp_flag = 1;
	 dispLcdText();
}

void dispLcdText(void){
	lcd_send_cmd (0x80);
	lcd_send_string("Temp Ref: ");
	lcd_send_cmd(0x8E);
	lcd_send_data(223);
	lcd_send_cmd(0x8F);
	lcd_send_data('C');
	lcd_send_cmd(0xC0);

	lcd_send_string("Temp Act: ");
	lcd_send_cmd(0xCE);
	lcd_send_data(223);
	lcd_send_cmd(0xCF);
	lcd_send_data('C');
}

void clearTempBuff(void){
	 for(int i =0;i<4;i++)
		 lcdTemp_buff[i] = ' ';
}

void lcdShiftTemp(void){
	lcdTemp_buff[0] = lcdTemp_buff[1];
	lcdTemp_buff[1] = lcdTemp_buff[2];
	lcdTemp_buff[2] = key+48;
}


void editTempConfirmation(uint8_t PKey){
	if(PKey == 10){
		clearTempBuff();
		 lcd_clear();
		 HAL_Delay(200);
		 lcd_send_cmd (0x80);
		 lcd_send_string("Temperatura");
		 lcd_send_cmd(0xC0);
		 lcd_send_string("reestablecida");
		 HAL_Delay(1000);
		 lcd_clear();
		 HAL_Delay(200);
	}else if(PKey ==13){
		if(temp_ref > 60 || temp_ref < 0){
			for(int i=0; i<3;i++)
				lcdTemp_buff[i] = lcdTmp_temp_buff[i];
			lcd_clear();
			HAL_Delay(500);
			lcd_send_cmd(0x80);
			lcd_send_string("La temperatura");
			lcd_send_cmd(0xC0);
			lcd_send_string("no es valida");
			HAL_Delay(1000);
			lcd_clear();
			HAL_Delay(200);
		}else{
			for(int i=0; i<3;i++)
				lcdTmp_temp_buff[i] = lcdTemp_buff[i];
			lcd_clear();
			HAL_Delay(500);
			lcd_send_cmd(0x80);
			lcd_send_string("La temperatura");
			lcd_send_cmd(0xC0);
			lcd_send_string("fue actualizada");
			HAL_Delay(1000);
			lcd_clear();
			HAL_Delay(200);
		}
	 }
}


void dispTempLCD(void){
	if(lcdImp_flag){
		lcd_send_cmd(0xCB);
		sprintf(buffer, "%i",(int)temps[1]);
		lcd_send_string(buffer);
		if(lcdBlink_flag){
			if(lcdTmp_counter){
				lcd_send_cmd(0x8A);
				lcd_send_data(lcdTemp_buff[0]);
				lcd_send_cmd(0x8B);
				lcd_send_data(lcdTemp_buff[1]);
				lcd_send_cmd(0x8C);
				lcd_send_data(lcdTemp_buff[2] == ' ' ? '0' : lcdTemp_buff[2]);
				lcdTmp_counter = 0;
			}else{
				lcd_send_cmd(0x8A);
				lcd_send_data(' ');
				lcd_send_cmd(0x8B);
				lcd_send_data(' ');
				lcd_send_cmd(0x8C);
				lcd_send_data(' ');
				lcdTmp_counter = 1;
			}
		}else{
			lcd_send_cmd(0x8A);
			lcd_send_data(lcdTemp_buff[0]);
			lcd_send_cmd(0x8B);
			lcd_send_data(lcdTemp_buff[1]);
			lcd_send_cmd(0x8C);
			lcd_send_data(lcdTemp_buff[2] == ' ' ? '0' : lcdTemp_buff[2]);
		}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
