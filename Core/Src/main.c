/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdio.h>
#include "stm32g4xx_ll_rtc.h"
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
ADC_HandleTypeDef hadc2;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static void GPIO_DeInit_To_Stop(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_LPTIM1_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */
uint8_t rx_data[2];
//UART_WakeUpTypeDef WakeUpSelection;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t go_stop = 0;

int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
  return len;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_13)
  {
    static int seq=0;
    int counter1 = 0;
    int counter2 = 0;
#if 1
    switch(seq)
    {
      case 0:
      {
  #if 0
        LL_TIM_EnableCounter(TIM1);
  #else
        RTC_TimeTypeDef sTime = {0};
        RTC_DateTypeDef sDate = {0};

        sTime.Hours = 0x1;
        sTime.Minutes = 0x0;
        sTime.Seconds = 0x0;
        sTime.SubSeconds = 0x0;
        sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
        sTime.StoreOperation = RTC_STOREOPERATION_RESET;
        if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
        {
          Error_Handler();
        }
        sDate.WeekDay = RTC_WEEKDAY_MONDAY;
        sDate.Month = RTC_MONTH_JANUARY;
        sDate.Date = 0x1;
        sDate.Year = 0x0;

        if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
        {
          Error_Handler();
        }

        //LPTIM
        LL_LPTIM_Enable(LPTIM1);
        LL_LPTIM_SetAutoReload(LPTIM1, 0xFFFF);
        LL_LPTIM_StartCounter(LPTIM1, LL_LPTIM_OPERATING_MODE_CONTINUOUS);

    #endif
      }
        break;

      default:
      {
  #if 0
        counter1 = LL_TIM_GetCounter(TIM1);
  #else
        RTC_TimeTypeDef sTime = {0};

        if (HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
        {
          Error_Handler();
        }

        sTime.SubSeconds = ((255 - sTime.SubSeconds) * 1000) / 255;
        //RTC
        counter1 = (sTime.Hours << 24) | (sTime.Minutes << 20) | (sTime.Seconds << 16) | (sTime.SubSeconds << 0);

        //LPTIM
        counter2 = LL_LPTIM_GetCounter(LPTIM1);

  #endif
        printf("1: %d\r\n", counter1);
        printf("2: %d\r\n", counter2);
      }
        break;
    }
#else
    switch(seq)
    {
      case 0:
        LL_LPTIM_Enable(LPTIM1);
        LL_LPTIM_SetAutoReload(LPTIM1, 0xFFFF);
        LL_LPTIM_StartCounter(LPTIM1, LL_LPTIM_OPERATING_MODE_CONTINUOUS);
        break;

      default:
        counter1 = LL_LPTIM_GetCounter(LPTIM1);
        printf("%d\r\n", counter1);
        break;
    }
#endif
    seq++;

#if 0
    go_stop = 1;

    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};
    RTC_AlarmTypeDef sAlarm = {0};

    if (HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
    {
      Error_Handler();
    }

    if (HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
    {
      Error_Handler();
    }

    sAlarm.AlarmTime.Hours = sTime.Hours;
    sAlarm.AlarmTime.Minutes = sTime.Minutes + 0x30;
    sAlarm.AlarmTime.Seconds = sTime.Seconds;
    sAlarm.AlarmTime.SubSeconds = sTime.SubSeconds;
    sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
    sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_NONE;
    sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
    sAlarm.AlarmDateWeekDay = sDate.Date;
    sAlarm.Alarm = RTC_ALARM_A;
    if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
    {
      Error_Handler();
    }

    HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);
#endif
  }
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
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
  MX_RTC_Init();
  MX_USART2_UART_Init();
  MX_LPTIM1_Init();
  MX_TIM1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  printf("Hello world!!\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int i=0;
  while (1)
  {
#if 1
    if(go_stop == 1)
    {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);

      printf("Stopping ... \r\n");
      HAL_Delay(10);

      HAL_SuspendTick();

      __WFI();

      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);

      HAL_ResumeTick();
      go_stop = 0;
    }
    else
    {
      __WFI();
    }
#else
    if(i++ > 5)
    {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
      printf("Stopping ... \r\n");
      HAL_Delay(10);

      HAL_SuspendTick();
      HAL_UARTEx_EnableStopMode(&huart2);
      HAL_UART_Receive_IT(&huart2, (uint8_t*)rx_data, 2);

      GPIO_DeInit_To_Stop();

      //Using HSI for uart to wake up from the stop mode
      HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
      SystemClock_Config();

      HAL_UARTEx_DisableStopMode(&huart2);
      HAL_ResumeTick();

      i = 0;
    }
    else
    {
      RTC_TimeTypeDef sTime = {0};
      RTC_DateTypeDef sDate = {0};

      HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
      HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

      printf("Running ... %d \r\n", sTime.Seconds);
      HAL_Delay(1000);
    }
#endif
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_LSE, RCC_MCODIV_1);
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

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_17;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief LPTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPTIM1_Init(void)
{

  /* USER CODE BEGIN LPTIM1_Init 0 */

  /* USER CODE END LPTIM1_Init 0 */

  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPTIM1;
  PeriphClkInit.Lptim1ClockSelection = RCC_LPTIM1CLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_LPTIM1);

  /* USER CODE BEGIN LPTIM1_Init 1 */

  /* USER CODE END LPTIM1_Init 1 */
  LL_LPTIM_SetClockSource(LPTIM1, LL_LPTIM_CLK_SOURCE_INTERNAL);
  LL_LPTIM_SetPrescaler(LPTIM1, LL_LPTIM_PRESCALER_DIV1);
  LL_LPTIM_SetPolarity(LPTIM1, LL_LPTIM_OUTPUT_POLARITY_REGULAR);
  LL_LPTIM_SetUpdateMode(LPTIM1, LL_LPTIM_UPDATE_MODE_IMMEDIATE);
  LL_LPTIM_SetCounterMode(LPTIM1, LL_LPTIM_COUNTER_MODE_INTERNAL);
  LL_LPTIM_TrigSw(LPTIM1);
  LL_LPTIM_SetInput1Src(LPTIM1, LL_LPTIM_INPUT1_SRC_GPIO);
  LL_LPTIM_SetInput2Src(LPTIM1, LL_LPTIM_INPUT2_SRC_GPIO);
  /* USER CODE BEGIN LPTIM1_Init 2 */

  /* USER CODE END LPTIM1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.SubSeconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
  __HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);
  LL_RTC_EnableShadowRegBypass(hrtc.Instance);
  __HAL_RTC_WRITEPROTECTION_ENABLE(&hrtc);
  /* USER CODE END RTC_Init 2 */

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

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  TIM_InitStruct.Prescaler = (24-1);
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 65535;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM1, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM1);
  LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
  LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM1);
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PG10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC2 PC3 PC4
                           PC5 PC6 PC7 PC8
                           PC9 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA7 PA9
                           PA10 PA11 PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_7|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB11 PB12 PB13 PB14
                           PB15 PB3 PB4 PB5
                           PB6 PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void GPIO_DeInit_To_Stop(void)
{
  GPIO_InitTypeDef GPIO_InitStructure = {0};

  //HAL_NVIC_DisableIRQ(EXTI0_IRQn);
  //HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);

  //HAL_UART_MspDeInit(&huart2);

   //__HAL_RCC_GPIOA_CLK_ENABLE();
   __HAL_RCC_GPIOB_CLK_ENABLE();
   //__HAL_RCC_GPIOC_CLK_ENABLE();
   __HAL_RCC_GPIOD_CLK_ENABLE();
   //__HAL_RCC_GPIOF_CLK_ENABLE();
   __HAL_RCC_GPIOG_CLK_ENABLE();

   GPIO_InitStructure.Pin = GPIO_PIN_All;
   GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
   GPIO_InitStructure.Pull = GPIO_NOPULL;

   //HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
   HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
   //HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
   HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
   //HAL_GPIO_Init(GPIOF, &GPIO_InitStructure);
   HAL_GPIO_Init(GPIOG, &GPIO_InitStructure);

   //__HAL_RCC_GPIOA_CLK_DISABLE();
   __HAL_RCC_GPIOB_CLK_DISABLE();
   //__HAL_RCC_GPIOC_CLK_DISABLE();
   __HAL_RCC_GPIOD_CLK_DISABLE();
   //__HAL_RCC_GPIOF_CLK_DISABLE();
   __HAL_RCC_GPIOG_CLK_DISABLE();
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
