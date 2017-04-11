/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l1xx_hal.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "gps.h"
#include <stdarg.h>
#include "menutal.h"
#include "stm32l1xx_nucleo.h"
#include "stm32_adafruit_sd.h"
#include "Gongshi.h"
#include "usb_device.h"
#include "usbd_core.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;
osThreadId Get_gps_info_Handle;
osThreadId SystemCallHandle;
osTimerId TimerUpdateHandle;
osMutexId gpsMutexHandle;
osMutexId SaveGpsMessHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

//FATFS SD_FatFs;  /* File system object for SD card logical drive */

nmea_msg *gpsx; 
FATFS *SD_FatFs;
system_flag *system_flag_table;

FRESULT fr;
FIL gps_fp ;

uint32_t gps_data_time = 0xffffffff;

uint8_t uart3_buffer[MAX_UART3_LEN];


/*GPS 数据接收标志位*/
uint16_t USART2_RX_STA_RP = 0; 
uint16_t USART2_RX_STA_WP = 0; 
uint8_t USART2_RX_STA = 0; 


uint8_t sound_working = 0 ;
uint8_t usb_init_flag = 0 ;





/* Private function prototypes 
-----------------------------------------------*/
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_RTC_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
void StartDefaultTask(void const * argument);
void Get_gps_info(void const * argument);
void MySystem(void const * argument);
void update_info(void const * argument);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
                                
                                
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

static void RTC_AlarmConfig(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint8_t str_cmpx(uint8_t* s1,uint8_t* s2,uint8_t len)
{
	uint8_t i;
	for(i=0;i<len;i++)if((*s1++)!=*s2++)return 0;
	return 1;
}

#if 1
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  while(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX){}
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}
#endif

static int inHandlerMode (void)
{
	 return __get_IPSR() != 0;
}

void print_usart1(char *format, ...)
{
    char buf[160];
    
    if(inHandlerMode() != 0)
    {
        taskDISABLE_INTERRUPTS();
    }
    else
    {
    	while(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX)
        {   
    	    osThreadYield();
        }
    }
    
    va_list ap;
    va_start(ap, format);
    if(vsprintf(buf, format, ap) > 0)
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)buf, strlen(buf), 160);
    }
    va_end(ap);
    
    if(inHandlerMode() != 0)
    {
        taskENABLE_INTERRUPTS();
    }
}


/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_ADC_Init();
  MX_RTC_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_TIM10_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();

  /* USER CODE BEGIN 2 */
  RTC_AlarmConfig();

  print_usart1("USER CODE BEGIN 2 \r\n");

  gpsx = malloc(sizeof(nmea_msg));    
  memset(gpsx,0,sizeof(nmea_msg));
  system_flag_table = malloc(sizeof(system_flag));    
  memset(system_flag_table,0,sizeof(system_flag));


  system_flag_table->guji_buffer = malloc(MAX_GUJI_BUFFER_MAX_LEN);    
  memset( system_flag_table->guji_buffer,0,MAX_GUJI_BUFFER_MAX_LEN);

  system_flag_table->time_zone                   = 29;
  system_flag_table->gujiFormats                 = GUJI_FORMATS_GPX;
  system_flag_table->guji_record.recoed_formats  = GUJI_FORMATS_GPX;
  system_flag_table->power_status                = POWER_STANBY;
  system_flag_table->guji_record.by_time_vaule   = 100; /*ms*/
  system_flag_table->guji_record.recoed_formats  = BY_TIMES;
  system_flag_table->power_mode                  = SENCSE_SURPORT_MODE;
  system_flag_table->guji_record.recoed_meth     = AUTO_STOP;


  //BSP_LED_Init(LED2);  

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of gpsMutex */
  osMutexDef(gpsMutex);
  gpsMutexHandle = osMutexCreate(osMutex(gpsMutex));

  /* definition and creation of SaveGpsMess */
  osMutexDef(SaveGpsMess);
  SaveGpsMessHandle = osMutexCreate(osMutex(SaveGpsMess));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of TimerUpdate */
  osTimerDef(TimerUpdate, update_info);
  TimerUpdateHandle = osTimerCreate(osTimer(TimerUpdate), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */

  /* Start Timer */
  osTimerStart(TimerUpdateHandle, 100);

  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Get_gps_info_ */
  osThreadDef(Get_gps_info_, Get_gps_info, osPriorityHigh, 0, 256);
  Get_gps_info_Handle = osThreadCreate(osThread(Get_gps_info_), NULL);

  /* definition and creation of SystemCall */
  osThreadDef(SystemCall, MySystem, osPriorityLow, 0, 128);
  SystemCallHandle = osThreadCreate(osThread(SystemCall), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadSuspend(Get_gps_info_Handle);
  //osThreadSuspend(defaultTaskHandle);

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

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

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_21;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* RTC init function */
static void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;
  RTC_AlarmTypeDef sAlarm;

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initialize RTC and set the Time and Date 
    */
  if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != 0x32F2){
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }

  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }

    HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR0,0x32F2);
  }
    /**Enable the Alarm A 
    */
  sAlarm.AlarmTime.Hours = 0;
  sAlarm.AlarmTime.Minutes = 0;
  sAlarm.AlarmTime.Seconds = 0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 11;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 500;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 250;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 11;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 500;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
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

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 11;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 500;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
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

  HAL_TIM_MspPostInit(&htim4);

}

/* TIM10 init function */
static void MX_TIM10_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;

  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 11;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 525;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 263;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim10);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPS_POWER_GPIO_Port, GPS_POWER_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : usb_hotplug_Pin */
  GPIO_InitStruct.Pin = usb_hotplug_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(usb_hotplug_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : surprot_line_Pin */
  GPIO_InitStruct.Pin = surprot_line_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(surprot_line_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPS_POWER_Pin */
  GPIO_InitStruct.Pin = GPS_POWER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPS_POWER_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */

void gps_power_mode(uint8_t mode)
{
    if(mode == 1)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); 
        //BSP_LED_Init(LED_GPS);
        //BSP_LED_On(LED_GPS);
    }            
    else
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);    
        //BSP_LED_Init(LED_GPS);
        //BSP_LED_Off(LED_GPS);
    }
}


/*呼一次2.5S  2500/100 = 25*/
static void Pwm_Breathing(uint8_t Led_pwm,uint8_t mode)
{

    TIM_OC_InitTypeDef sConfigOC;
    static uint16_t Pulse_vaule = 0 ;
    TIM_HandleTypeDef* htim;
    uint32_t Channel;
    static uint8_t led_flag = 0;

    switch(Led_pwm )

    {
        case STATUS_LED_GREEN:
            htim = &htim3;
            Channel = TIM_CHANNEL_3;
            break;
        case STATUS_LED_RED:
            htim = &htim3;
            Channel = TIM_CHANNEL_4;
            break;
        case STATUS_LED_BULE:
            htim = &htim3;
            Channel = TIM_CHANNEL_2;
            break;
        case SD_LED:
            htim = &htim2;
            Channel = TIM_CHANNEL_2;
            break;
        case GPS_LED:
            htim = &htim3;
            Channel = TIM_CHANNEL_1;
            break;
        case SPRORT_LED:
            htim = &htim4;
            Channel = TIM_CHANNEL_1;
            break;
            

    }
    if(mode == 1)
    {
        if(led_flag == 0)
        {
            Pulse_vaule += 6;
        }
        else         
        {
            Pulse_vaule -= 6;
        }

        sConfigOC.OCMode = TIM_OCMODE_PWM1;
        sConfigOC.Pulse = Pulse_vaule;
        sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
        

        if(Pulse_vaule == 450)
        {
            led_flag = 1;
        }

        if(Pulse_vaule == 0)
        {
            led_flag = 0;
        }


        if (HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, Channel) != HAL_OK)
        {
          Error_Handler();
        }
        HAL_TIM_PWM_Start(htim, Channel);


    }
    else
    {
        HAL_TIM_PWM_Stop(htim, Channel);
        Pulse_vaule = 0;
    
    }

}

/**
  * @brief  Configure the current time and date.
  * @param  None
  * @retval None
  */
static void RTC_AlarmConfig(void)
{
  RTC_DateTypeDef  sdatestructure;
  RTC_TimeTypeDef  stimestructure;
  uint8_t temp[3];
  uint8_t i,mon;
  uint8_t date;
  uint16_t year;
  uint8_t sec,min,hour;
  
  const uint8_t *COMPILED_TIME=__TIME__;//??μ?±àò?ê±??
  const uint8_t *COMPILED_DATE=__DATE__;//??μ?±àò?è??ú

  const uint8_t Month_Tab[12][3]={"Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"};
  //×??ˉéè??ê±???a±àò??÷ê±??
 
  for(i=0;i<3;i++)temp[i]=COMPILED_DATE[i];
  for(i=0;i<12;i++)if(str_cmpx((uint8_t*)Month_Tab[i],temp,3))break;
  mon=i+1;//μ?μ???·Y
  if(COMPILED_DATE[4]==' ')date=COMPILED_DATE[5]-'0';
  else date=10*(COMPILED_DATE[4]-'0')+COMPILED_DATE[5]-'0';
  year=/*1000*(COMPILED_DATE[7]-'0')+100*(COMPILED_DATE[8]-'0')*/+10*(COMPILED_DATE[9]-'0')+COMPILED_DATE[10]-'0';
  hour=10*(COMPILED_TIME[0]-'0')+COMPILED_TIME[1]-'0';
  min=10*(COMPILED_TIME[3]-'0')+COMPILED_TIME[4]-'0';
  sec=10*(COMPILED_TIME[6]-'0')+COMPILED_TIME[7]-'0';

 
  /*##-1- Configure the Date #################################################*/
  /* Set Date: Tuesday February 18th 2014 */
  sdatestructure.Year = year;
  sdatestructure.Month = mon;
  sdatestructure.Date = date; 
  sdatestructure.WeekDay = RTC_Get_Week(year,mon,date);   
  if(HAL_RTC_SetDate(&hrtc,&sdatestructure,RTC_FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  } 
  
  /*##-2- Configure the Time #################################################*/
  /* Set Time: 02:20:00 */
  stimestructure.Hours = hour;
  stimestructure.Minutes = min;
  stimestructure.Seconds = sec;
  stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;
  
  if(HAL_RTC_SetTime(&hrtc,&stimestructure,RTC_FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  }  


}


/**
  * @brief  Display the current time.
  * @param  showtime : pointer to buffer
  * @retval None
  */
void RTC_TimeShow(DWORD* fattime)
{
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructureget;

  //DWORD fattime = 0;
	
  /* Get the RTC current Time */
  HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BCD);
  /* Get the RTC current Date */
  HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BCD);
  /* Display time Format : hh:mm:ss */
  print_usart1("time: %02d:%02d:%02d \r\n",stimestructureget.Hours, stimestructureget.Minutes, stimestructureget.Seconds);
  print_usart1("date: %02d:%02d:%02d \r\n",sdatestructureget.Year, sdatestructureget.Month, sdatestructureget.Date);

  *fattime =  ((DWORD)((sdatestructureget.Year + 20) << 25) | (DWORD)(sdatestructureget.Month<< 21) | (DWORD)(sdatestructureget.Date<< 16));
  *fattime |= ((DWORD)(stimestructureget.Hours << 11) | (DWORD)(stimestructureget.Minutes<< 5)|((DWORD)(stimestructureget.Seconds)/2));  
 } 



/**
  * @brief  Rx Transfer completed callbacks.
  * @param  huart: Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

    if(huart->Instance == USART3)
    {       
       /* Start another reception: provide the buffer pointer with offset and the buffer size */
      
    	if(USART2_RX_STA_WP < (MAX_UART3_LEN - 1))		//还可以接收数据
    	{
            USART2_RX_STA_WP ++ ; 
    	}
		else 
    	{
    		USART2_RX_STA_WP = 0;
    	} 
		
		gps_data_time = HAL_GetTick(); 

        if((system_flag_table->guji_mode != RECORED_IDLE)&&(system_flag_table->guji_mode != RECORED_STOP))
        {
	        HAL_UART_Receive_IT(&huart3, (uint8_t *)(uart3_buffer + USART2_RX_STA_WP), 1); 		
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{    

    if ((GPIO_Pin == WAKEUP_BUTTON_PIN))
    {                
    }
    else if(GPIO_Pin == GPIO_PIN_1)
    {
        
    }
    else if(GPIO_Pin == GPIO_PIN_0)
    {
    }

    print_usart1("exit :%d \r\n",GPIO_Pin);
    
}/* USER CODE HAL_GPIO_EXTI_Callback*/


/**
  * @brief  Prepare the system to enter STOP mode.
  * @param  None
  * @retval None
  */
static void StopSequence_Config(void)
{
  GPIO_InitTypeDef      GPIO_InitStruct;
  
  /* PWR Peripheral clock enable */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* Enable GPIOs clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* Configure all GPIO port pins in Analog mode */
  GPIO_InitStruct.Pin = GPIO_PIN_All;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Request to enter STOP mode with regulator in low power */
  /* Disable all used wakeup sources: WKUP pin */
  HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
  HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2);  
  /* Clear all related wakeup flags */
  /* Clear PWR wake up Flag */
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
  
  /* Enable WKUP pin */
  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
  //HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2);  
  /* Request to enter STANDBY mode */
  HAL_PWR_EnterSTANDBYMode();
  //HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

}

void surport_mode_config(uint8_t mode,uint8_t *buf)
{
    
    float tp_distance = 0;
    uint8_t ret = 0 ; 
    uint16_t rtc_vaule = 0;

    switch(mode)
    {
        case POWER_RUN:
        case POWER_SURPORT_RUN:
             if((gpsx->gpssta >= 1)&&(gpsx->latitude >0)&&(gpsx->longitude>0))
             {                                    
                 if(system_flag_table->guji_mode == RECORED_START_DOING)
                 {  
                     tp_distance = getDistanceVer2( system_flag_table->tp_lati,gpsx->nshemi,system_flag_table->tp_long,
                                                   gpsx->ewhemi, gpsx->latitude, gpsx->nshemi, gpsx->longitude,gpsx->ewhemi);
                     if(system_flag_table->guji_record.recoed_formats == BY_DISTANCE)
                     {
                         if(tp_distance >= system_flag_table->guji_record.by_distance_vaule)
                         {
                             ret = 1;    
                         }
                     }
                     else if(system_flag_table->guji_record.recoed_formats == BY_TIMES)
                     {
                         if(system_flag_table->guji_record.by_time_vaule <= 100)
                         {
                             ret = 1;
                         }
                         else if(system_flag_table->guji_record.by_time_vaule <= (HAL_GetTick() - system_flag_table->grecord_timer_cnt))
                         {
                             system_flag_table->grecord_timer_cnt = HAL_GetTick();
                             ret = 1;
                         }
                     }
                     else if(system_flag_table->guji_record.recoed_formats == BY_SPEED)
                     {
                         if((gpsx->speed/1000) <= system_flag_table->guji_record.by_speed_vaule)
                         {
                             ret = 1;
                         }
                     }
                                                                                 
                 }
                   
            } 

            if(ret == 1)
            {
            #if 0
                if((system_flag_table->power_status == POWER_SURPORT_RUN)
                    &&(HAL_GPIO_ReadPin(SUPORT_DETECT_GPIO_PORT,SUPORT_DETECT_PIN) != GPIO_PIN_RESET))
                {
                    return;
                }
            #endif
                
                save_guiji_message(gpsx,system_flag_table,'T');
                system_flag_table->tp_long = gpsx->longitude;
                system_flag_table->tp_lati = gpsx->latitude;  
            }
            break;
        case POWER_LRUN:

            if(system_flag_table->guji_record.recoed_formats == BY_TIMES)
            {
                if(system_flag_table->guji_record.by_time_vaule <= (HAL_GetTick() - system_flag_table->grecord_timer_cnt))
                {
                    system_flag_table->grecord_timer_cnt = HAL_GetTick();
                    save_guiji_message(gpsx,system_flag_table,'T');
                }
            } 
            
            while(system_flag_table->guji_buffer_Index_rp != system_flag_table->guji_buffer_Index_wp) {;}
            
            {
                /* Disable Wakeup Counter */
                HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
                
                /* ## Setting the Wake up time ############################################*/
                /*  RTC Wakeup Interrupt Generation:
                        Wakeup Time Base = (RTC_WAKEUPCLOCK_RTCCLK_DIV /(LSE or LSI))
                        Wakeup Time = Wakeup Time Base * WakeUpCounter 
                        = (RTC_WAKEUPCLOCK_RTCCLK_DIV /(LSE or LSI)) * WakeUpCounter
                          ==> WakeUpCounter = Wakeup Time / Wakeup Time Base
                      
                        To configure the wake up timer to 4s the WakeUpCounter is set to 0x1FFF:
                        RTC_WAKEUPCLOCK_RTCCLK_DIV = RTCCLK_Div16 = 16 
                        Wakeup Time Base = 16 /(~39.000KHz) = ~0,410 ms
                        Wakeup Time = ~4s = 0,410ms  * WakeUpCounter
                        ==> WakeUpCounter = ~4s/0,410ms = 9750 = 0x2616 */
                rtc_vaule = (system_flag_table->guji_record.by_time_vaule*1000)/427;        
                HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, rtc_vaule, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
                print_usart1("****************************** \r\n");
                print_usart1("low run status go to stop mode \r\n");
                print_usart1("****************************** \r\n");

                /* Enter Stop Mode */
                HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);
                
                /* Configures system clock after wake-up from STOP: enable HSI, PLL and select
                        PLL as system clock source (HSI and PLL are disabled automatically in STOP mode) */                      
                SystemClock_Config();   
                HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
            }
            break;
        default :break;
    }
    

}


static uint8_t get_key(void)
{
    static uint8_t button_flag = 0; 
    static uint8_t button_press_cnt = 0xff; 
    uint8_t button_key = 0; 

    if((BSP_PB_GetState(BUTTON_USER) == 0)&&(BSP_PB_GetState(BUTTON_WAKEUP) == 0))
    {
        if(button_flag == USER_KEY_MARK|WAKEUP_KEY_MARK)
        {
            button_press_cnt++;
            if((button_press_cnt >= 50))
            {
                //button_key = POWER_USER_KEY_LONG;
                button_press_cnt = 51;  
                button_flag = 0xff;     
            }
        }
        else
        {
            button_press_cnt = 0;
        }
        
        if(button_flag == 0)
            button_flag = USER_KEY_MARK|WAKEUP_KEY_MARK;
    }
    else if(BSP_PB_GetState(BUTTON_WAKEUP) == 0)
    {
        if(button_flag == WAKEUP_KEY_MARK)
        {

            button_press_cnt++;
            if((button_press_cnt >= 50))
            {
                button_key = POWER_KEY_LONG;
                button_press_cnt = 51;
                button_flag = 0xff;     
                
            }

        }
        else
        {
            button_press_cnt = 0;
        }
        
        if(button_flag == 0)
            button_flag = WAKEUP_KEY_MARK;
    
    
    }
    else if(BSP_PB_GetState(BUTTON_USER) == 0)
    {

        if(button_flag == USER_KEY_MARK)
        {
#ifdef USER_KEY_NO_LONG        

            button_press_cnt++;
            if(button_press_cnt >= 50)
            {
                button_key = USER_KEY_LONG;
                
                button_press_cnt = 51;  
                button_flag = 0xff;
                
            }
#else
            button_key = USER_KEY;
            button_flag = 0xff;

#endif
        }
        else
        {
            button_press_cnt = 0;
        }
        
        if(button_flag == 0)        
           button_flag = USER_KEY_MARK;
    
    }
    else
    {
       if(button_key == 0)
        {
            //print_usart1("button_flag :%d \r\n",button_flag);
            switch(button_flag)
            {
                case WAKEUP_KEY_MARK:  button_key = POWER_KEY;
                    break;
                case USER_KEY_MARK:    button_key = USER_KEY;
                     break;
                default :break;
            }
        }
        else
            button_key = 0;
        
        button_flag = 0;
        button_press_cnt = 0;
    }
    

    return button_key;
}

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  /* init code for FATFS */
  MX_FATFS_Init();

  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 5 */
  usb_init_flag = 1;
  print_usart1("StartDefaultTask \r\n");

  SD_FatFs = malloc(sizeof(FATFS));   
  My_Fs_Init(SD_FatFs);
  /*保存文件*/
  /* Infinite loop */
  for(;;)
  {

    if (osMutexWait(gpsMutexHandle, osWaitForever) == osOK)
    {          
        Recording_guji(&gps_fp,system_flag_table,gpsx);
        //write_flash(&gps_fp,system_flag_table);
        //print_usart1("Recording_guji \r\n");
        if (osMutexRelease(gpsMutexHandle) != osOK)
        {
            Error_Handler();
        }

        //ThreadResume(Get_gps_info_Handle);
    }
    
    if(system_flag_table->power_status == POWER_STANBY)
    {
        if(system_flag_table->guji_mode  == RECORED_START_DOING)
        {
            system_flag_table->guji_mode = RECORED_STOP;
            Recording_guji(&gps_fp,system_flag_table,gpsx);
        }
        
        osThreadSuspend(NULL);
    }

    osDelay(1);

  }
  /* USER CODE END 5 */ 
}

/* Get_gps_info function */
void Get_gps_info(void const * argument)
{
  /* USER CODE BEGIN Get_gps_info */
  uint16_t rxlen = 0;
  uint8_t *gps_data = NULL;
  /* Infinite loop */
  print_usart1("Get_gps_info\r\n");
  /*##-4- Put UART peripheral in reception process ###########################*/  

  for(;;)
  {
   
      if(USART2_RX_STA == 1)
      {
		  
          if (osMutexWait(gpsMutexHandle, 0) == osOK)
          {
			  USART2_RX_STA = 0;		   //启动下一次接收

			  if(USART2_RX_STA_RP > USART2_RX_STA_WP)
			  {
                  gps_data = malloc(USART2_RX_STA_WP+MAX_UART3_LEN -USART2_RX_STA_RP+1);
			      memcpy(gps_data,uart3_buffer+USART2_RX_STA_RP,(MAX_UART3_LEN-USART2_RX_STA_RP));
			      memcpy(gps_data + (MAX_UART3_LEN-USART2_RX_STA_RP),uart3_buffer,USART2_RX_STA_WP);
                  rxlen = USART2_RX_STA_WP+MAX_UART3_LEN -USART2_RX_STA_RP;

			  }
			  else
			  {
			      gps_data = malloc((USART2_RX_STA_WP-USART2_RX_STA_RP)+1);
			      memcpy(gps_data,uart3_buffer+USART2_RX_STA_RP,(USART2_RX_STA_WP-USART2_RX_STA_RP));
                  rxlen = (USART2_RX_STA_WP-USART2_RX_STA_RP);

			  }
              gps_data[rxlen] = 0;
              USART2_RX_STA_RP = USART2_RX_STA_WP;	 //得到数据长度

              if((gpsx->gpssta <1)&&(rxlen < 160))
              {
                  print_usart1("%s",gps_data);
              } 
              else if(gpsx->gpssta >= 1)
              {

              }
              
              GPS_Analysis(gpsx,gps_data);
              free(gps_data);
/*recored mode config*/
	          surport_mode_config(system_flag_table->power_status,gps_data);
/*endi*/			  
              if (osMutexRelease(gpsMutexHandle) != osOK)
              {
                  Error_Handler();
              }  		  
          }
          

      }
     
      if(system_flag_table->power_status == POWER_STANBY)
        osThreadSuspend(NULL);
	  //print_usart1("Get_gps_info go\r\n");
      osDelay(1);

      /* Suspend ourselves to the medium priority thread can execute */
      //ThreadSuspend(NULL);


  }
  /* USER CODE END Get_gps_info */
}

/* MySystem function */
void MySystem(void const * argument)
{
  /* USER CODE BEGIN MySystem */
  uint8_t _user_key_ = 0;
  extern USBD_HandleTypeDef hUsbDeviceFS;
//  uint8_t _breath_flag_ = 0;
  
  BSP_PB_Init(BUTTON_USER,BUTTON_MODE_GPIO);  
  BSP_PB_Init(BUTTON_WAKEUP,BUTTON_MODE_GPIO);

  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_RED);
  BSP_LED_Init(LED_BULE);
  //BSP_LED_Init(LED_GPS);  
  BSP_LED_Init(LED_SD);  
  BSP_LED_Init(LED_SURPORT);  

  HAL_NVIC_DisableIRQ(EXTI1_IRQn);


  /* Infinite loop */
  for(;;)
  {
      _user_key_ = get_key();
      if(_user_key_  != 0x00)
      {
          print_usart1("_user_key_:%d \r\n",_user_key_);
         
          HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
          sound_working = 2;          
   
      }
      switch(_user_key_)
      {
          case USER_KEY:
                if(system_flag_table->power_status == POWER_RUN)
                {
                    if(system_flag_table->guji_mode == RECORED_START_DOING)
                    {
                        if((gpsx->gpssta >= 1)&&(gpsx->latitude >0)&&(gpsx->longitude>0))
                        {
                            save_guiji_message(gpsx,system_flag_table,'C');
                        }
                    }    
                }
              break;
          case POWER_KEY:

              break;
          case USER_KEY_LONG:
              if(system_flag_table->guji_mode >= RECORED_START)
              {
                  system_flag_table->guji_mode = RECORED_STOP;
              }
              else if(system_flag_table->guji_mode == RECORED_IDLE)
              {
                  system_flag_table->guji_mode = RECORED_START;
                  if(HAL_UART_Receive_IT(&huart3, (uint8_t *)uart3_buffer, 1) != HAL_OK)
                  {
                    Error_Handler();
                  }

              }
              print_usart1("guji_mode :%d \r\n",system_flag_table->guji_mode);
              break;
          case POWER_KEY_LONG:
              if(system_flag_table->power_status == POWER_STANBY)
              {
                  //print_usart1("SD_DETECT:%d \r\n",HAL_GPIO_ReadPin(SD_DETECT_GPIO_PORT, SD_DETECT_PIN));
                  if(HAL_GPIO_ReadPin(SD_DETECT_GPIO_PORT, SD_DETECT_PIN) != GPIO_PIN_RESET)
                  {                    
                      print_usart1("POWER ON \r\n");					 
                      system_flag_table->power_status = system_flag_table->power_mode;  
                      gps_power_mode(1);
                      /* init code for USB_DEVICE */
                      //USBD_Stop(&hUsbDeviceFS);
                      if(usb_init_flag == 1)
                      {
                          USBD_DeInit(&hUsbDeviceFS);
                          USBD_Stop(&hUsbDeviceFS);
                          usb_init_flag = 0;
                  	  }
                      osThreadResume(defaultTaskHandle);
                      osThreadResume(Get_gps_info_Handle);
                      
                  }
                  
              }
              else if(system_flag_table->power_status != POWER_STANBY)
              {
                  system_flag_table->power_status = POWER_STANBY;    
                  print_usart1("POWER OFF \r\n");
                  gps_power_mode(0);
                  //USBD_Start(&hUsbDeviceFS);
                  if(usb_init_flag == 0)
                  {
                      MX_USB_DEVICE_Init();
                      usb_init_flag = 1;
              	  }                  

				  while(osThreadGetState(Get_gps_info_Handle) != osThreadSuspended) { osDelay(1);}//|| (osThreadGetState(defaultTaskHandle) == osThreadSuspended))
				  print_usart1("************\r\n");
				  print_usart1("goto stanby.\r\n");
				  print_usart1("************\r\n");


				  StopSequence_Config();
              }            
              break;
          case POWER_USER_KEY_LONG:
              __set_FAULTMASK(1);      // 关闭所有中端
              HAL_NVIC_SystemReset();
              break;
          default:break;
      }

      _user_key_  = 0;


      
      if(gpsx->gpssta >= 1)
      {
          system_flag_table->Led_pwm_type = GPS_LED;
          Pwm_Breathing(system_flag_table->Led_pwm_type,1);    
      }
      else
      {
          //BSP_LED_Init(GPS_LED);
          //if()
          
      }

      
      osDelay(20);
  }
  /* USER CODE END MySystem */
}

/* update_info function */
void update_info(void const * argument)
{
  /* USER CODE BEGIN update_info */
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructureget;  
  static uint16_t timer_cnt = 0 ;

     /* Get the RTC current Time */
  HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BCD);
  /* Get the RTC current Date */
  HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BCD);   
  
  system_flag_table->RTC_DateStructure = sdatestructureget;
  system_flag_table->RTC_TimeStructure = stimestructureget;

  
  //print_usart1("date: %02d:%02d:%02d \r\n",sdatestructureget.Year, sdatestructureget.Month, sdatestructureget.Date);
  //print_usart1("time: %02d:%02d:%02d \r\n",stimestructureget.Hours, stimestructureget.Minutes, stimestructureget.Seconds);
  
#if 1  
  if(HAL_GPIO_ReadPin(SD_DETECT_GPIO_PORT, SD_DETECT_PIN) != GPIO_PIN_RESET)
  
  {
      BSP_LED_On(LED_SD);
	  //print_usart1("SD IN \r\n");
  } 
  else 
  {
      BSP_LED_Off(LED_SD);
	  //print_usart1("SD OFF \r\n");

  }
#endif

  if(sound_working)
  {
      sound_working --;
      if(sound_working == 0)
      {
          HAL_TIM_PWM_Stop(&htim10, TIM_CHANNEL_1);
      }
      
  }
  
  vddmv_adc_proess(system_flag_table); /*更新电池状态*/

  if((HAL_GPIO_ReadPin(USB_DETECT_GPIO_PORT, USB_DETECT_PIN) == GPIO_PIN_RESET)&&(system_flag_table->power_status == POWER_STANBY))
  {
      timer_cnt ++;
      if(timer_cnt == 10)
      {
          timer_cnt = 0;
          //StopSequence_Config();

          print_usart1("****************************** \r\n");
          print_usart1("when usb detect hotplug, goto stanby angin. \r\n");
          print_usart1("****************************** \r\n");

      }
  } 
  else if((system_flag_table->power_status == POWER_SURPORT_RUN)&&(gpsx->speed <= 10))
  {
      timer_cnt ++;
      if(timer_cnt == 3000)
      {
          timer_cnt = 0;
          //StopSequence_Config();
          
          print_usart1("****************************** \r\n");
          print_usart1("entry surport mode  go to stop \r\n");       
          print_usart1("****************************** \r\n");
		  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
          /* Enter Stop Mode */
          HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);
          SystemClock_Config();
                
	      HAL_NVIC_DisableIRQ(EXTI1_IRQn);

      }
	  else
	  {
          //print_usart1("timer_cnt:%d \r\n",timer_cnt);	     
	  }
  }
  else 
  {
      timer_cnt = 0;
  }

  /* USER CODE END update_info */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM7) {
  	if(HAL_GetTick() > (gps_data_time + 10))
  	{
  	    if(USART2_RX_STA_RP!= USART2_RX_STA_WP)
        { 
  	        USART2_RX_STA = 1;
        }
		gps_data_time = 0xffffffff;
    }
//    __HAL_TIM_DISABLE(&htim6);
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM6) {
    system_flag_table->Led_pwm_type = GPS_LED;
    Pwm_Breathing(system_flag_table->Led_pwm_type,1);
    }

/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
