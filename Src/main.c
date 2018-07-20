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
#include <string.h>

#include "menutal.h"
#include "stm32l1xx_nucleo.h"

#if defined(STM32L151xB)
#include "stm32_adafruit_sd.h"
#else
#include "v1000_sd.h"
#endif

#include "Gongshi.h"
#include "usb_device.h"
#include "usbd_core.h"
#include "stm_eeprom.h"
#include "RxP.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

RTC_HandleTypeDef hrtc;

//SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

WWDG_HandleTypeDef hwwdg;


osThreadId defaultTaskHandle;
osThreadId Get_gps_info_Handle;
osThreadId SystemCallHandle;
osTimerId TimerUpdateHandle;
osMutexId gpsMutexHandle;
osMutexId SaveGpsMessHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

//FATFS SD_FatFs;  /* File system object for SD card logical drive */

nmea_msg *gpsx = NULL; 
system_flag *system_flag_table = NULL;

nmea_msg gpsx_1; 
system_flag system_flag_table_1;

FRESULT fr;
FIL gps_fp ;

IWDG_HandleTypeDef Iwdg;


uint32_t gps_data_time = 0xffffffff;

uint8_t uart3_dma_buffer[100];
//uint8_t uart3_buffer[MAX_UART3_LEN];

uint8_t self_guiji_buffer[MAX_GUJI_BUFFER_MAX_LEN];
static uint16_t usb_timer_cnt = 0 ;


/*GPS 数据接收标志位*/
//uint16_t USART2_RX_STA_RP = 0; 
//uint16_t USART2_RX_STA_WP = 0; 
//uint8_t USART2_RX_STA = 0; 

//uint16_t save_usart2_wp = 0; 


uint16_t support_cnt = 0; 
uint8_t recored_flag = 0 ;
uint32_t save_file_cnt = 0 ;

#if 0
uint8_t sound_cnt= 0 ;
uint8_t sound_mode = 0 ;
#endif

uint8_t usb_init_flag = 0 ;
static uint8_t LED_SURPORT_FLAG = 0;
static uint8_t LED_SURPORT_F_FLAG = 0;

static uint8_t LED_Sd_FLAG = 0;
static uint8_t Wang_FLAG = 0;
//static uint8_t key_status = 0;
uint8_t sound_flag = 0;
static uint8_t start_hotplug  = 0;
static uint8_t is_locker  = 0;
static uint8_t is_power_from_auto  = 0;

#define NMEA_EX_LENGTH 		256  // In order to support YLS proprietary sentence


typedef enum{
  STN_GGA = 0,
  STN_GLL = 1,
  STN_GSA = 2,
  STN_GSV = 3,
  STN_RMC = 4,
  STN_VTG = 5,
  STN_QSA = 6,  
  STN_OTHER = 255,
} NMEA_STN_T;



typedef struct {        //NMEA Format structure
	GINT16 i2PacketType;  // 1: NMEA,  2: DEBUG, 3: HBD, 4: BIN
	GINT16 i2PacketSize;
	NMEA_STN_T eType;
	GCHAR Data[NMEA_EX_LENGTH];
} NMEA_STN_DATA_T;


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
static void MX_USART3_UART_Init(void);
static void MX_TIM10_Init(void);
static void MX_SPI1_Init(void);
//static void MX_TIM4_Init(void);
//static void MX_RTC_Init(void);
//static void MX_TIM2_Init(void);
void StartDefaultTask(void const * argument);
void Get_gps_info(void const * argument);
void MySystem(void const * argument);
void update_info(void const * argument);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
static void MX_USART3_UART_Init_9600(void);                                


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

//static void RTC_AlarmConfig(void);
uint8_t sound_toggle_simple(uint8_t cnt ,uint16_t sound_on_timer, uint16_t sound_off_timer);
uint8_t breathing_toggle(uint16_t breath_on_timer, uint16_t breath_off_timer);
void gps_power_mode(uint8_t mode);
void sd_power_mode(uint8_t mode);
void sleep_power_config(void);

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
  HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&ch, 1);

  return ch;
}
#endif

static int inHandlerMode (void)
{
	 return __get_IPSR() != 0;
}

void print_usart1(char *format, ...)
{


#if 1


    char buf[160];
    uint32_t timer_out = 0;
    va_list ap;
       
    if(inHandlerMode() != 0)
    {
        taskDISABLE_INTERRUPTS();
    }
    else
    {
    	while(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX)
        {   
    	    osThreadYield();
            timer_out++;
            if(timer_out > 1000)
                break;
        }
    }
    

    va_start(ap, format);
    if(vsprintf(buf, format, ap) > 0)
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)buf, strlen(buf),160);
    }
    va_end(ap);
    
    if(inHandlerMode() != 0)
    {
        taskENABLE_INTERRUPTS();
    }
#endif   
}


void reset_eeprom(void)
{
      //uint32_t eeprom_flag = 0;

      print_usart1("default info \r\n");
      system_flag_table->time_zone                   = 16;
      system_flag_table->gujiFormats                 = GUJI_FORMATS_CSV;
      system_flag_table->guji_record.by_time_vaule   = 1000; /*ms*/
      system_flag_table->guji_record.recoed_formats  = BY_TIMES;

      system_flag_table->lowpower_timer              = 15;
      system_flag_table->ODOR                        = 0;
      system_flag_table->buzzer                      = 1;
      system_flag_table->auto_power                  = 0;
      system_flag_table->guji_record.by_speed_vaule  = 0;
      system_flag_table->wanng_speed_vaule = 0;
      system_flag_table->unit = 0;
      stm_write_eerpom(0,system_flag_table->time_zone);
      /*Buzzer*/
      stm_write_eerpom(1,system_flag_table->buzzer);    
      /*SpeedWarning*/  
      stm_write_eerpom(2,system_flag_table->wanng_speed_vaule);      
      /*AutoPower*/   
      stm_write_eerpom(3,system_flag_table->auto_power);     
      stm_write_eerpom(4,system_flag_table->gujiFormats);      
      stm_write_eerpom(5,system_flag_table->guji_record.by_time_vaule);
      stm_write_eerpom(6,system_flag_table->guji_record.by_distance_vaule);
      stm_write_eerpom(7,system_flag_table->guji_record.recoed_formats);    
      stm_write_eerpom(8,system_flag_table->guji_record.by_speed_vaule);       
      stm_write_eerpom(9,system_flag_table->lowpower_timer);             
      stm_write_eerpom(10,system_flag_table->ODOR);
      stm_write_eerpom(11,0);
      stm_write_eerpom(12,system_flag_table->unit);

      /*一天一轨迹存储空间清零*/   
      stm_write_eerpom(20,0);
      stm_write_eerpom(21,0);
      stm_write_eerpom(22,0);
      stm_write_eerpom(23,0);
      stm_write_eerpom(24,0);
      stm_write_eerpom(25,0);
      stm_write_eerpom(30,0);

      /*标记EERPOM表示已初始化*/
      stm_write_eerpom(0xff,0x12345678);
      stm_write_eerpom(0xf0,0);   /*power mode save ! default is normal support mode*/
      system_flag_table->frist_power = 1;
      stm_write_eerpom(13,system_flag_table->frist_power);
      system_flag_table->lowpower_timer = system_flag_table->lowpower_timer*1000*60;

}


/* USER CODE END 0 */



int main(void)
{

  /* USER CODE BEGIN 1 */
  uint32_t eeprom_flag = 0;
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
  MX_USART3_UART_Init();
  MX_TIM10_Init();
  //MX_SPI1_Init();
  //MX_TIM4_Init();
  //MX_RTC_Init();
  //MX_TIM2_Init();
  //MX_WWDG_Init();


  //MX_Iwdg_Init();


  /* USER CODE BEGIN 2 */
  //sd_power_mode(1)

  print_usart1("P-1 running !!build:%s %s sb_flag :%x  wu_flag:%x\r\n",__DATE__,__TIME__,__HAL_PWR_GET_FLAG(PWR_FLAG_SB),__HAL_PWR_GET_FLAG(PWR_FLAG_WU));
  //RTC_AlarmConfig();

  gpsx = &gpsx_1;    
  memset(gpsx,0,sizeof(nmea_msg));
  system_flag_table = &system_flag_table_1;    

  memset(&system_flag_table_1,0,sizeof(system_flag));
  system_flag_table->guji_buffer = self_guiji_buffer;    
  memset( self_guiji_buffer,0,MAX_GUJI_BUFFER_MAX_LEN);

  BSP_PB_Init(BUTTON_USER,BUTTON_MODE_GPIO);  
  BSP_PB_Init(BUTTON_WAKEUP,BUTTON_MODE_GPIO);
  BSP_PB_Init(BUTTON_FUNCTION,BUTTON_MODE_GPIO);

  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_RED);
  BSP_LED_Init(LED_BULE);
  BSP_LED_Init(LED_GPS);  
  BSP_LED_Init(LED_SD);  
  BSP_LED_Init(LED_SURPORT);  

  LED_SURPORT_FLAG = 0;
  LED_Sd_FLAG = 0;
  //SystemClock_Config_msi();
  //sleep_power_config();
  HAL_NVIC_DisableIRQ(EXTI1_IRQn);
  //HAL_SuspendTick();
  //CLEAR_BIT(SysTick->CTRL,SysTick_CTRL_TICKINT_Msk);
  //while(1);
  //HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI); 
  stm_read_eerpom(0xff,&eeprom_flag);
  if(eeprom_flag == 0x12345677)
  {
      print_usart1("update eerpom info \r\n");
      stm_read_eerpom(0,&eeprom_flag);
      system_flag_table->time_zone = eeprom_flag;
      stm_read_eerpom(1,&eeprom_flag);
      system_flag_table->buzzer = eeprom_flag;
      stm_read_eerpom(2,&eeprom_flag);
      system_flag_table->wanng_speed_vaule = eeprom_flag;
      stm_read_eerpom(3,&eeprom_flag);
      system_flag_table->auto_power = eeprom_flag;
      stm_read_eerpom(4,&eeprom_flag);
      system_flag_table->gujiFormats = eeprom_flag;
      stm_read_eerpom(5,&eeprom_flag);
      system_flag_table->guji_record.by_time_vaule = eeprom_flag;
      stm_read_eerpom(6,&eeprom_flag);
      system_flag_table->guji_record.by_distance_vaule = eeprom_flag;
      stm_read_eerpom(7,&eeprom_flag);
      system_flag_table->guji_record.recoed_formats = eeprom_flag;
      stm_read_eerpom(8,&eeprom_flag);
      system_flag_table->guji_record.by_speed_vaule = eeprom_flag;
      stm_read_eerpom(9,&eeprom_flag);
      system_flag_table->lowpower_timer = eeprom_flag;
      if(system_flag_table->lowpower_timer == 0)
        system_flag_table->lowpower_timer = 15;
      
      system_flag_table->lowpower_timer = system_flag_table->lowpower_timer*1000*60;
      print_usart1("system_flag_table->lowpower_timer :%d \r\n",system_flag_table->lowpower_timer);
      print_usart1("system_flag_table->by_time_vaule :%d \r\n",system_flag_table->guji_record.by_time_vaule);

      stm_read_eerpom(10,&eeprom_flag);
      system_flag_table->ODOR = eeprom_flag;

      stm_read_eerpom(12,&eeprom_flag);
      system_flag_table->unit = eeprom_flag;
      stm_read_eerpom(13,&eeprom_flag);
      system_flag_table->frist_power = eeprom_flag;
      stm_read_eerpom(30,&eeprom_flag);
      system_flag_table->function_index = eeprom_flag;


  }
  else
  {
      reset_eeprom();
  }
  
  stm_read_eerpom(0xf0,&eeprom_flag);
  //system_flag_table->gujiFormats				 = GUJI_FORMATS_MEA;

  if(eeprom_flag == 0)
  {
      system_flag_table->power_mode                  = NORMAL_SURPORT_MODE;
  }
  else
  {
      system_flag_table->power_mode                  = SENCSE_SURPORT_MODE;
  }

  system_flag_table->batt_Status = BATT_HIGH;
  sd_power_mode(1);
  system_flag_table->power_status                = POWER_STANBY;
#if 0
  if(system_flag_table->auto_power == 0)
  {
      system_flag_table->power_status                = POWER_STANBY;

  }
  else
  {
     
      print_usart1("POWER ON \r\n");
      sound_toggle_simple(2,50,50);                                    
      system_flag_table->power_status                = system_flag_table->power_mode;  
      gps_power_mode(1);
      /* init code for USB_DEVICE */
	  system_flag_table->guji_mode                   = RECORED_START;
  	  HAL_UART_Receive_DMA(&huart3, (uint8_t *)uart3_buffer, 1); 
      if(usb_init_flag == 1)
      {
          USBD_DeInit(&hUsbDeviceFS);
          USBD_Stop(&hUsbDeviceFS);
          usb_init_flag = 0;
  	  }
      //osThreadResume(defaultTaskHandle);
      //osThreadResume(Get_gps_info_Handle);      
  }
#endif  
  print_usart1("system_flag_table->power_mode :%d \r\n",system_flag_table->power_mode);
  system_flag_table->guji_record.recoed_meth         = AUTO_STOP;

#if 0
  if(HAL_GPIO_ReadPin(USB_DETECT_GPIO_PORT, USB_DETECT_PIN) != GPIO_PIN_RESET)
  {
      system_flag_table->charger_connected = 1;
  }
#endif

  /* USER CODE END 2 */
  rxp_init_pcrx();

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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 768);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Get_gps_info_ */
  osThreadDef(Get_gps_info_, Get_gps_info, osPriorityAboveNormal, 0, 512);
  Get_gps_info_Handle = osThreadCreate(osThread(Get_gps_info_), NULL);

  /* definition and creation of SystemCall */
  osThreadDef(SystemCall, MySystem, osPriorityNormal, 0, 256);
  SystemCallHandle = osThreadCreate(osThread(SystemCall), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  if(system_flag_table->auto_power == 0)  
      osThreadSuspend(Get_gps_info_Handle);
  //osThreadSuspend(defaultTaskHandle);
  //osThreadSuspend(SystemCallHandle);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                                    |RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON; 
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.MSICalibrationValue = 0x00;  
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
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
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
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

static void MX_Iwdg_Init(void)
{

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  Iwdg.Instance = IWDG; 
  Iwdg.Init.Prescaler = IWDG_PRESCALER_32;
  Iwdg.Init.Reload = 2560;

  if (HAL_IWDG_Init(&Iwdg) != HAL_OK)
  {
    Error_Handler();
  }



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
#if 0
/* RTC init function */
static void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;

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
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
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

    HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR0,0x32F2);
  }

}
#endif

#if 0
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

#endif
#if 0
/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim2);

}
#endif
#if 0
/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 15;
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
#endif
/* TIM10 init function */
static void MX_TIM10_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;

  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 15;
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
  huart1.Init.BaudRate = 57600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX;
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


static void MX_USART3_UART_Init_9600(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  //HAL_GPIO_WritePin(GPS_POWER_GPIO_Port, GPS_POWER_Pin, GPIO_PIN_SET);
  //HAL_GPIO_WritePin(SD_POWER_GPIO_Port, SD_POWER_Pin, GPIO_PIN_SET);

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
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPS_POWER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPS_POWER_Pin */
  GPIO_InitStruct.Pin = SD_POWER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SD_POWER_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  //HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  //HAL_NVIC_EnableIRQ(EXTI1_IRQn);
  DDvm_ITConfig();

}

/* USER CODE BEGIN 4 */

void SystemClock_Config_msi(void)
{

    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0}; 
    GPIO_InitTypeDef GPIO_InitStruct;
    /* Select MSI as system clock source and configure the HCLK, PCLK1 and PCLK2 
       clocks dividers */
    RCC_ClkInitStruct.ClockType       = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource    = RCC_SYSCLKSOURCE_MSI;
    RCC_ClkInitStruct.AHBCLKDivider   = RCC_SYSCLK_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider  = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider  = RCC_HCLK_DIV1;
    if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
      /* Initialization Error */
      Error_Handler();
    }

    /**Initializes the CPU, AHB and APB busses clocks */
    
     /**Configure the Systick interrupt time 
     */

    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
    
     /**Configure the Systick 
     */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);


#if 1   
    if(system_flag_table->power_status == POWER_SURPORT_SLEEP)
    {
        /* EXTI interrupt init*/
        /*Configure GPIO pin : surprot_line_Pin */
  
        //HAL_InitTick(TICK_INT_PRIORITY);
        __HAL_RCC_GPIOA_CLK_ENABLE();
        GPIO_InitStruct.Pin = surprot_line_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(surprot_line_GPIO_Port, &GPIO_InitStruct);
        /* EXTI interrupt init*/
        HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(EXTI1_IRQn);        
    }
    MX_USART1_UART_Init();


    //print_usart1("msi clock = %d\r\n ",HAL_RCC_GetHCLKFreq());
#endif
}


void SystemClock_Config_resume(void)
{

  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  GPIO_InitTypeDef GPIO_InitStruct;

  
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
  
  /**Configure the Systick 
  */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);  
  //print_usart1("resume clock = %d\r\n ",HAL_RCC_GetHCLKFreq());
  if(system_flag_table->power_status == POWER_SURPORT_SLEEP)
  {
      /* EXTI interrupt init*/
      /*Configure GPIO pin : surprot_line_Pin */
      //__HAL_RCC_GPIOA_CLK_ENABLE();
      GPIO_InitStruct.Pin = surprot_line_Pin;
      GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
      GPIO_InitStruct.Pull = GPIO_PULLUP;
      HAL_GPIO_Init(surprot_line_GPIO_Port, &GPIO_InitStruct);
      /* EXTI interrupt init*/
      HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
      HAL_NVIC_DisableIRQ(EXTI1_IRQn);

  }
  MX_USART1_UART_Init();


}

const uint8_t BaudRate_config[]="$PMTK251,115200*1f\r\n";
const uint8_t filt_config[]="$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n";
const uint8_t A1hz_config[]="$PMTK220,1000*1f\r\n";
const uint8_t A5hz_config[]="$PMTK220,200*2c\r\n";
const uint8_t A10hz_config[]="$PMTK220,100*2f\r\n";

void gps_init(void)
{
	osDelay(1000);
	osDelay(1000);
    MX_USART3_UART_Init_9600();
	HAL_UART_Transmit(&huart3,(uint8_t*)BaudRate_config,sizeof(BaudRate_config),0xFFF);
	osDelay(1000);
	MX_USART3_UART_Init();
	osDelay(1000);
	HAL_UART_Transmit(&huart3,(uint8_t*)filt_config,sizeof(filt_config),0xFFF);  
	osDelay(1000);
	if(system_flag_table->guji_record.recoed_formats == BY_TIMES)
    {
    	if(system_flag_table->guji_record.by_time_vaule == 100)
        	HAL_UART_Transmit(&huart3,(uint8_t*)A10hz_config,sizeof(A10hz_config),0xfff);  
    	else if(system_flag_table->guji_record.by_time_vaule == 200)
    		HAL_UART_Transmit(&huart3,(uint8_t*)A5hz_config,sizeof(A5hz_config),0xfff);  
    	else if(system_flag_table->guji_record.by_time_vaule == 1000)
    		HAL_UART_Transmit(&huart3,(uint8_t*)A1hz_config,sizeof(A1hz_config),0xfff);  
    }
	else
    {
    	HAL_UART_Transmit(&huart3,(uint8_t*)A1hz_config,sizeof(A1hz_config),0xfff);          
    }
	
		

}

void gps_power_mode(uint8_t mode)
{

	GPIO_InitTypeDef GPIO_InitStruct;

    if(mode == 1)
    {
      /*Configure GPIO pin : GPS_POWER_Pin */    
		GPIO_InitStruct.Pin = GPS_POWER_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPS_POWER_GPIO_Port, &GPIO_InitStruct);	  
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); 
        memset(gpsx,0,sizeof(nmea_msg));
        //HAL_UART_Receive_DMA(&huart3, (uint8_t *)uart3_buffer, 1); 
		
    }            
    else
    {
		GPIO_InitStruct.Pin = GPS_POWER_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPS_POWER_GPIO_Port, &GPIO_InitStruct);
    
        //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);    
        memset(gpsx,0,sizeof(nmea_msg));
        //is_locker  = 0;
    }
}

void sd_power_mode(uint8_t mode)
{

	GPIO_InitTypeDef GPIO_InitStruct;

    if(mode == 1)
    {
		GPIO_InitStruct.Pin = SD_POWER_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(SD_POWER_GPIO_Port, &GPIO_InitStruct);	
        HAL_GPIO_WritePin(SD_POWER_GPIO_Port, SD_POWER_Pin, GPIO_PIN_RESET); 

    }            
    else
    {
        /*Configure GPIO pin : GPS_POWER_Pin */
        GPIO_InitStruct.Pin = SD_POWER_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(SD_POWER_GPIO_Port, &GPIO_InitStruct);    
        //HAL_GPIO_WritePin(SD_POWER_GPIO_Port, SD_POWER_Pin, GPIO_PIN_SET);    
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
            //htim = &htim3;
            //Channel = TIM_CHANNEL_3;
            break;
        case STATUS_LED_RED:
            //htim = &htim3;
            //Channel = TIM_CHANNEL_4;
            break;
        case STATUS_LED_BULE:
            //htim = &htim3;
           // Channel = TIM_CHANNEL_2;
            break;
        case SD_LED:
            htim = &htim2;
            Channel = TIM_CHANNEL_2;
            break;
        case GPS_LED:
            //htim = &htim3;
            //Channel = TIM_CHANNEL_1;
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
            Pulse_vaule += 25;
        }
        else if(led_flag == 1)        
        {
            Pulse_vaule -= 25;
        }
        else
        {
            HAL_TIM_PWM_Stop(htim, Channel);
            return ;
        }

        sConfigOC.OCMode = TIM_OCMODE_PWM1;
        sConfigOC.Pulse = Pulse_vaule;
        sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
        

        if(Pulse_vaule == 500)
        {
            led_flag = 1;
        }

        if(Pulse_vaule == 0)
        {
            led_flag = 0xff;
        }


        if (HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, Channel) != HAL_OK)
        {
          Error_Handler();
        }
        HAL_TIM_PWM_Start(htim, Channel);


    }
    else
    {
        led_flag = 0;
        Pulse_vaule = 0;
        HAL_TIM_PWM_Stop(htim, Channel);
    
    }

}

#if 0
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
#endif

/**
  * @brief  Display the current time.
  * @param  showtime : pointer to buffer
  * @retval None
  */
void RTC_TimeShow(DWORD* fattime)
{
    //RTC_DateTypeDef sdatestructureget;
    //RTC_TimeTypeDef stimestructureget;
  
    //DWORD fattime = 0;
    #if 0	
    /* Get the RTC current Time */
    HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BCD);
    /* Get the RTC current Date */
    HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BCD);
    #endif
    
    /* Display time Format : hh:mm:ss */
  #if 0
    *fattime =  ((DWORD)((sdatestructureget.Year + 20) << 25) | (DWORD)(sdatestructureget.Month<< 21) | (DWORD)(sdatestructureget.Date<< 16));
    *fattime |= ((DWORD)(stimestructureget.Hours << 11) | (DWORD)(stimestructureget.Minutes<< 5)|((DWORD)(stimestructureget.Seconds)/2));  
  #endif
    check_time(gpsx,system_flag_table);
    //print_usart1("time: %02d:%02d:%02d \r\n",system_flag_table->sys_tm.w_year, system_flag_table->sys_tm.w_month,system_flag_table->sys_tm.w_date);
    //print_usart1("date: %02d:%02d:%02d \r\n",system_flag_table->sys_tm.hour, system_flag_table->sys_tm.min, system_flag_table->sys_tm.sec);

    *fattime =  ((DWORD)((system_flag_table->sys_tm.w_year + 20) << 25) | (DWORD)(system_flag_table->sys_tm.w_month<< 21)\
               | (DWORD)(system_flag_table->sys_tm.w_date << 16));
    *fattime |= ((DWORD)(system_flag_table->sys_tm.hour << 11) | (DWORD)(system_flag_table->sys_tm.min<< 5)\
               |((DWORD)(system_flag_table->sys_tm.sec)/2));  
 
} 



/**
  * @brief  Rx Transfer completed callbacks.
  * @param  huart: Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    int i = 0;
    if(huart->Instance == USART3)
    {       
       /* Start another reception: provide the buffer pointer with offset and the buffer size */
      
        for(i = 0;i<100;i++)
        {
             rxp_pcrx_nmea(uart3_dma_buffer[i]);   
        }
#if 0
    	if(USART2_RX_STA_WP < (MAX_UART3_LEN - 100))		//还可以接收数据
    	{
            memcpy(uart3_buffer+USART2_RX_STA_WP,uart3_dma_buffer,100);
            USART2_RX_STA_WP += 100 ; 
    	}
		else 
    	{
   
            memcpy(uart3_buffer+USART2_RX_STA_WP,uart3_dma_buffer,(MAX_UART3_LEN - USART2_RX_STA_WP));
            memcpy(uart3_buffer,uart3_dma_buffer+(MAX_UART3_LEN - USART2_RX_STA_WP),100+USART2_RX_STA_WP -MAX_UART3_LEN);            
    		USART2_RX_STA_WP = 100+USART2_RX_STA_WP -MAX_UART3_LEN;
    	}        

        while(HAL_UART_Receive_DMA(&huart3, (uint8_t *)(uart3_buffer + USART2_RX_STA_WP), 1) != HAL_OK)
        {
	        print_usart1("err\r\n");
        }
#endif        
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
	    print_usart1("huar1t->ErrorCode :%x \r\n",huart->ErrorCode);    

    }
    else if(huart->Instance == USART3) 
    {
	    print_usart1("huart3->ErrorCode :%x \r\n",huart->ErrorCode);    
        //MX_USART3_UART_Init();
        //while(HAL_UART_Receive_DMA(&huart3, (uint8_t *)(uart3_buffer + USART2_RX_STA_WP), 1) != HAL_OK)
        {
	    //    print_usart1("HAL_UART_ErrorCallback err\r\n");
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
        support_cnt ++;

        if(support_cnt > 120)
        {
            HAL_NVIC_DisableIRQ(EXTI1_IRQn);
        }
    }
    else if(GPIO_Pin == GPIO_PIN_0)
    {
    }
	else if(GPIO_Pin == GPIO_PIN_13)
    {
         //HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);   
    }

    //print_usart1("exit :%d %d\r\n",GPIO_Pin,support_cnt);
    
}/* USER CODE HAL_GPIO_EXTI_Callback*/

void sleep_power_config(void)
{
    GPIO_InitTypeDef	  GPIO_InitStruct;
    
    /* Configure all GPIO port pins in Analog mode */
    GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

#if 0
    /* Configure all GPIO port pins in Analog mode */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
#endif

}

/**
  * @brief  Prepare the system to enter STOP mode.
  * @param  None
  * @retval None
  */
static void StopSequence_Config(void)
{
   GPIO_InitTypeDef      GPIO_InitStruct;

   if(HAL_GPIO_ReadPin(USB_DETECT_GPIO_PORT, USB_DETECT_PIN) == GPIO_PIN_RESET)
   {
       print_usart1("StopSequence_Config! \r\n");
       /* PWR Peripheral clock enable */
       __HAL_RCC_PWR_CLK_ENABLE();
  
       //SystemClock_Config_msi();
       /* Enable GPIOs clock */
       __HAL_RCC_GPIOA_CLK_ENABLE();
       __HAL_RCC_GPIOB_CLK_ENABLE();
       __HAL_RCC_GPIOC_CLK_ENABLE();
     
       /* Request to enter STOP mode with regulator in low power */
       /* Disable all used wakeup sources: WKUP pin */
       HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
       HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2);  
       /* Clear all related wakeup flags */
       /* Clear PWR wake up Flag */
       __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

     

#if 0
       /* Configure all GPIO port pins in Analog mode */
       GPIO_InitStruct.Pin = GPIO_PIN_All;
       GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
       GPIO_InitStruct.Pull = GPIO_NOPULL;
       HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
       HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
       HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
#endif

       BSP_PB_Init(BUTTON_WAKEUP,BUTTON_MODE_EXTI);	 
       //BSP_PB_Init(BUTTON_USER,BUTTON_MODE_GPIO);  

  
       BSP_LED_Init(LED_GREEN);
       BSP_LED_Init(LED_RED);
       BSP_LED_Init(LED_BULE);
       BSP_LED_Init(LED_GPS);  
       BSP_LED_Init(LED_SD);  
       BSP_LED_Init(LED_SURPORT); 

    
       /*Configure GPIO pin : usb_hotplug_Pin */
       GPIO_InitStruct.Pin = usb_hotplug_Pin;
       GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
       GPIO_InitStruct.Pull = GPIO_PULLDOWN;
       HAL_GPIO_Init(usb_hotplug_GPIO_Port, &GPIO_InitStruct);     
  
       /*Configure GPIO pin : GPS_POWER_Pin */
       GPIO_InitStruct.Pin = GPS_POWER_Pin;
       GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
       GPIO_InitStruct.Pull = GPIO_PULLUP;
       HAL_GPIO_Init(GPS_POWER_GPIO_Port, &GPIO_InitStruct);
       
       /*Configure GPIO pin : GPS_POWER_Pin */
       GPIO_InitStruct.Pin = SD_POWER_Pin;
       GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
       GPIO_InitStruct.Pull = GPIO_PULLUP;
       HAL_GPIO_Init(SD_POWER_GPIO_Port, &GPIO_InitStruct);


       /*Configure GPIO pin : surprot_line_Pin */
       GPIO_InitStruct.Pin = surprot_line_Pin;
       GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
       GPIO_InitStruct.Pull = GPIO_PULLUP;
       HAL_GPIO_Init(surprot_line_GPIO_Port, &GPIO_InitStruct);
         /* EXTI interrupt init*/
       //HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
       //HAL_NVIC_EnableIRQ(EXTI0_IRQn); 
      
       /* Enable WKUP pin */
       HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
       HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2);  
       memset(gpsx,0,sizeof(nmea_msg));
#if 1       
       if(Wang_FLAG == 1)
       {
          Wang_FLAG = 0;
          HAL_TIM_PWM_Stop(&htim10, TIM_CHANNEL_1);              
       }
#endif  
        //HAL_PWR_DisablePVD();
        //HAL_PWREx_EnableUltraLowPower();

        //osDelay(100);
        /* Request to enter STOP mode */
        HAL_PWR_EnterSTANDBYMode();
        //HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON,PWR_STOPENTRY_WFI);
        //while(1);
 #if 0       
        SystemClock_Config();
        MX_GPIO_Init();
        MX_USART1_UART_Init();
        MX_ADC_Init();
        MX_USART3_UART_Init();
        MX_TIM10_Init();
        MX_SPI1_Init();
        MX_TIM4_Init();
        MX_RTC_Init();
        MX_TIM2_Init();     
        BSP_PB_Init(BUTTON_USER,BUTTON_MODE_GPIO);  
        BSP_PB_Init(BUTTON_WAKEUP,BUTTON_MODE_GPIO);
      
        BSP_LED_Init(LED_GREEN);
        BSP_LED_Init(LED_RED);
        BSP_LED_Init(LED_BULE);
        BSP_LED_Init(LED_GPS);  
        BSP_LED_Init(LED_SD);  
        BSP_LED_Init(LED_SURPORT);   
   	    sd_power_mode(1);
        print_usart1("leve from stop! \r\n");
#endif        
   }
  //HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

}



void surport_mode_config(uint8_t mode,GCHAR *buf,uint16_t rxlen)
{
    
    float tp_distance = 0;
    uint8_t ret = 0 ; 
    static uint8_t lp_number = 0;
    //static uint16_t test_cnt = 0;
    float tp_lati = 0.0,tp_long = 0.0,latitude = 0.0,longitude = 0.0;
   

    switch(mode)
    {
        case POWER_RUN:
        case POWER_SURPORT_RUN:
            //print_usart1("gpsx->gpssta :%d \r\n",gpsx->gpssta); /*打印行驶距离*/
             if(gpsx->gpssta >= 1)
             {   
				if((system_flag_table->guji_mode == RECORED_START_DOING)||(system_flag_table->guji_mode == RECORED_SAVE)\
				||(system_flag_table->guji_mode == RECORED_T)||(system_flag_table->guji_mode == RECORED_D))
                 {  
                     tp_lati = system_flag_table->tp_lati;
                     tp_long = system_flag_table->tp_long;
                     latitude = gpsx->latitude;
                     longitude = gpsx->longitude;                     
                     tp_lati /=1000000;
                     tp_long /=1000000;
                     latitude /=1000000;
                     longitude /=1000000;

                     tp_distance = getDistanceVer2( tp_lati,gpsx->nshemi,tp_long,
                                                   gpsx->ewhemi, latitude, gpsx->nshemi, longitude,gpsx->ewhemi);
                     if(system_flag_table->guji_record.recoed_formats == BY_DISTANCE)
                     {

                         tp_distance = (tp_distance*1000);
                         print_usart1("tp_distance :%.3f \r\n",tp_distance); /*打印行驶距离*/

                         if((tp_distance) >= system_flag_table->guji_record.by_distance_vaule)
                         {
                             if((gpsx->speed) >= (system_flag_table->guji_record.by_speed_vaule))
                             {
                                 ret = 1;    
                             }
                         }
                     }
                     else if(system_flag_table->guji_record.recoed_formats == BY_TIMES)
                     {

#if 0					 
                         if(system_flag_table->guji_record.by_time_vaule <= 100)
                         {
                             if(gpsx->speed >= (system_flag_table->guji_record.by_speed_vaule*1000))
                             {
                                 ret = 1;
                             }
                         }
                         else if(system_flag_table->guji_record.by_time_vaule <= (HAL_GetTick() - system_flag_table->grecord_timer_cnt))
                         {
                             if(gpsx->speed >= (system_flag_table->guji_record.by_speed_vaule*1000))
                             {
                                 system_flag_table->grecord_timer_cnt = HAL_GetTick();
                                 ret = 1;
                             }
                         }
#endif					 
						 if(system_flag_table->guji_record.by_speed_vaule == 0)	
						 {
						 	ret = 1;
	
						 }
						 else  if(gpsx->speed >= (system_flag_table->guji_record.by_speed_vaule))
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
                
                if(system_flag_table->gujiFormats == GUJI_FORMATS_MEA)
                {
                    if(system_flag_table->Message_head_number == 0)
                    {
                        if(gpsx->hdop >= 500)
                            break;
                        else
                            system_flag_table->Message_head_number = 1; 
                    }

				    if(system_flag_table->guji_buffer_Index_wp + rxlen < MAX_GUJI_BUFFER_MAX_LEN)
				    {
                        memcpy(&system_flag_table->guji_buffer[system_flag_table->guji_buffer_Index_wp],buf,rxlen);
                        system_flag_table->guji_buffer_Index_wp  += rxlen;    
				    }
					else
                    {
                        memcpy(&system_flag_table->guji_buffer[system_flag_table->guji_buffer_Index_wp],buf,MAX_GUJI_BUFFER_MAX_LEN-system_flag_table->guji_buffer_Index_wp);
						memcpy(&system_flag_table->guji_buffer[0],(buf+MAX_GUJI_BUFFER_MAX_LEN-system_flag_table->guji_buffer_Index_wp),\
							system_flag_table->guji_buffer_Index_wp+rxlen - MAX_GUJI_BUFFER_MAX_LEN);
						
						system_flag_table->guji_buffer_Index_wp  += rxlen;	  
						system_flag_table->guji_buffer_Index_wp = (system_flag_table->guji_buffer_Index_wp - MAX_GUJI_BUFFER_MAX_LEN);
                    }              
                }
                else
                {
                     if(LED_SURPORT_F_FLAG == 1)
                     {
                         LED_SURPORT_F_FLAG = 0;
                         save_guiji_message(gpsx,system_flag_table,'G');

                     }
                     else
                         save_guiji_message(gpsx,system_flag_table,'T');
					 //test_cnt++;
					 //print_usart1("save :%d \r\n",test_cnt);

                    
                }
                
                system_flag_table->tp_long = gpsx->longitude;
                system_flag_table->tp_lati = gpsx->latitude;  
            }
            else
            {
                  #ifdef TEST_WRITE_SD
                      //memcpy(&system_flag_table->guji_buffer[system_flag_table->guji_buffer_Index_wp],buf,rxlen);
                  #endif
				  //print_usart1("save :%d \r\n",test_cnt);
            }
            break;
        case POWER_LRUN:

            if((system_flag_table->guji_mode == RECORED_START_DOING)||(system_flag_table->guji_mode == RECORED_RESTART_2))
            {
                if(gpsx->gpssta == 0 )
                {
                    if(180000 <= (HAL_GetTick() - system_flag_table->grecord_timer_cnt))
                    {
                        if(is_locker == 1)
                            lp_number = 8 ;
                    }
                }
                else
                {
                    if(gpsx->hdop < 500)
                    {
                        if(800 <= (HAL_GetTick() - system_flag_table->grecord_timer_cnt))
                         {
                            is_locker = 1;
                            save_guiji_message(gpsx,system_flag_table,'T');
                            lp_number++;
                            print_usart1("hdop :%d \r\n",gpsx->hdop );                                                    
                            system_flag_table->grecord_timer_cnt = HAL_GetTick();
                        }
                   }
                }
    
                
                if(lp_number >= 8)
                {

                    lp_number = 0;

                    if(is_locker == 0)
                        return ; 
                    
                    gps_power_mode(0);
                    //Recording_guji(&gps_fp,system_flag_table,gpsx);
                    print_usart1("go to lrun sleep \r\n");
                    system_flag_table->grecord_timer_cnt = HAL_GetTick();
                    system_flag_table->power_status = POWER_LRUN_SLEEP;
                    //write_flash(&gps_fp,system_flag_table);     
                    //system_flag_table->guji_mode = RECORED_SAVE;
                    //Recording_guji(&gps_fp,system_flag_table,gpsx);
                    while(osThreadGetState(defaultTaskHandle) != osThreadSuspended) { osDelay(10);}//|| (osThreadGetState(defaultTaskHandle) == osThreadSuspended))
                    //osThreadSuspend(Get_gps_info_Handle);
                    BSP_LED_Off(LED_SD);
                    BSP_LED_Off(LED_GPS);
                    BSP_LED_Off(LED_SURPORT);
                    
                    sd_power_mode(0);

                    SystemClock_Config_msi();
					//sleep_power_config();

                    
                }
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

    //key_status = 1;
#ifdef OLD  
    if((BSP_PB_GetState(BUTTON_USER) == 0)&&(BSP_PB_GetState(BUTTON_WAKEUP) == 0))

#else
    if((BSP_PB_GetState(BUTTON_USER) == 1)&&(BSP_PB_GetState(BUTTON_WAKEUP) == 1)&&(BSP_PB_GetState(BUTTON_FUNCTION) == 0))
#endif		
    {
        //print_usart1("button_flag :%d %d \r\n",button_flag,button_press_cnt);
        if(button_flag == (USER_KEY_MARK|WAKEUP_KEY_MARK))
        {
            button_press_cnt++;
            if((button_press_cnt >= 13))
            {
                button_key = POWER_USER_KEY_LONG;
                button_press_cnt = 6;  
                button_flag = 0xff;     
            }
        }
        else
        {
            button_press_cnt = 0;
        }
        
        if(button_flag != 0xff)
            button_flag = USER_KEY_MARK|WAKEUP_KEY_MARK;
        
    }
    else if((BSP_PB_GetState(BUTTON_FUNCTION) == 1)&&(BSP_PB_GetState(BUTTON_WAKEUP) == 1)&&(BSP_PB_GetState(BUTTON_USER) == 0))
    {
        if(button_flag == (FUNCTION_KEY_MARK|WAKEUP_KEY_MARK))
        {
            button_press_cnt++;
            if((button_press_cnt >= 13))
            {
                button_key = POWER_KEY_LONG_5S;
                button_press_cnt = 6;  
                button_flag = 0xff;     
            }
        }
        else
        {
            button_press_cnt = 0;
        }
        
        if(button_flag != 0xff)
            button_flag = FUNCTION_KEY_MARK|WAKEUP_KEY_MARK;   
    }
    else if((BSP_PB_GetState(BUTTON_FUNCTION) == 1)&&(BSP_PB_GetState(BUTTON_WAKEUP) == 1)&&(BSP_PB_GetState(BUTTON_USER) == 1))
    {
        if(button_flag == (FUNCTION_KEY_MARK|WAKEUP_KEY_MARK|USER_KEY_MARK))
        {
            button_press_cnt++;
            if((button_press_cnt >= 28))
            {
                button_key = RESTORE_KEY_LONG;
                button_press_cnt = 6;  
                button_flag = 0xff;     
            }
        }
        else
        {
            button_press_cnt = 0;
        }
        
        if(button_flag != 0xff)
            button_flag = USER_KEY_MARK|WAKEUP_KEY_MARK|FUNCTION_KEY_MARK;   
    }

#ifdef OLD  
	else if(BSP_PB_GetState(BUTTON_WAKEUP) == 0)
	
#else

    else if(BSP_PB_GetState(BUTTON_WAKEUP) == 1)
#endif		
    {
        if(button_flag == WAKEUP_KEY_MARK)
        {
            //print_usart1("button_flag :%d %d \r\n",button_flag,button_press_cnt);

            button_press_cnt++;
            if(button_press_cnt == 13)
            {
                button_key = POWER_KEY_LONG;
                button_press_cnt = 6;
                //if((system_flag_table->power_status == POWER_SURPORT_RUN || system_flag_table->power_status == POWER_RUN))
                button_flag = 0xff;     
                
            }

#ifdef OLD_POWER_LONG_5            
			else if(button_press_cnt == 50)
		    {
    		    button_key = POWER_KEY_LONG_5S;
				button_press_cnt = 13;
                button_flag = 0xff;     
		    }
#endif            

        }
        else
        {
            button_press_cnt = 0;
        }
        
        if(button_flag == 0)
            button_flag = WAKEUP_KEY_MARK;
    
    
    }
#ifdef OLD  
    else if(BSP_PB_GetState(BUTTON_USER) == 0)
		
#else

	else if(BSP_PB_GetState(BUTTON_USER) == 1)
#endif		
    {

        if(button_flag == USER_KEY_MARK)
        {
#ifdef USER_KEY_NO_LONG        

            button_press_cnt++;
            if(button_press_cnt >= 13)
            {
                button_key = USER_KEY_LONG;
                
                button_press_cnt = 6;  
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
    else if(BSP_PB_GetState(BUTTON_FUNCTION) == 1)
    {

        if(button_flag == FUNCTION_KEY_MARK)
        {

            button_press_cnt++;
            if(button_press_cnt >= 13)
            {
                button_key = FUNCTION_KEY_LONG;
                
                button_press_cnt = 6;  
                button_flag = 0xff;
                
            }


        }
        else
        {
            button_press_cnt = 0;
        }
        
        if(button_flag == 0)        
           button_flag = FUNCTION_KEY_MARK;
    
    }

    else
    {
       if(button_key == 0)
        {
            if(button_press_cnt < 13)
            {
                switch(button_flag)
                {
                    case WAKEUP_KEY_MARK:  button_key = POWER_KEY;
                        break;
                    case USER_KEY_MARK:    button_key = USER_KEY;
                        break;
                    case FUNCTION_KEY_MARK:    button_key = FUNCTION_KEY;
                        break;

                    default :break;
                }
            }
#if 0            
            else if(button_press_cnt < 12)
           /*POWER键起来后，先不切换到START，过滤掉POWER LP时出现的问题
                    start后如果马上认到GPS就会开启文件，会导致 后续连按会有问题。*/
            {
                if((button_flag == 0xff)&&(system_flag_table->guji_mode != RECORED_START))   					  
                {
                    system_flag_table->guji_mode = RECORED_START;
		            HAL_UART_Receive_DMA(&huart3, (uint8_t *)uart3_buffer, 1);            
                }
            }
#endif            
        }
        else
            button_key = 0;
        
        button_flag = 0;
        button_press_cnt = 0;
        //key_status = 0;
    }
    

    return button_key;
}


uint8_t sound_toggle_config(uint16_t sound_on_timer, uint16_t sound_off_timer)
{
    static uint8_t sound_flag = 0;
    static uint32_t sound_toggle_cnt = 0;
 
    if((sound_flag == 0)&&(HAL_GetTick() >= (sound_toggle_cnt + sound_off_timer)))
    {
        sound_toggle_cnt = HAL_GetTick();
        if(system_flag_table->buzzer == 1)
            HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
        sound_flag = 1;
    }
    
    if((sound_flag == 1)&&(HAL_GetTick() >= (sound_toggle_cnt + sound_on_timer)))
    {
        sound_toggle_cnt = HAL_GetTick();
        HAL_TIM_PWM_Stop(&htim10, TIM_CHANNEL_1);
        sound_flag = 0;
    } 


    return sound_flag;

}

uint8_t sound_toggle_simple(uint8_t cnt ,uint16_t sound_on_timer, uint16_t sound_off_timer)
{
   uint8_t i = 0;

   if(system_flag_table->buzzer == 1)
   {
       for(i = 0;i< cnt ; i++)
       {
           HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
           osDelay(sound_on_timer);
           HAL_TIM_PWM_Stop(&htim10, TIM_CHANNEL_1);
           osDelay(sound_off_timer);
       }
   }
	 
   return 0;
}

uint8_t sound_toggle_simple_Force(uint8_t cnt ,uint16_t sound_on_timer, uint16_t sound_off_timer)
{
   uint8_t i = 0;


   {
       for(i = 0;i< cnt ; i++)
       {
           HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
           HAL_Delay(sound_on_timer);
           HAL_TIM_PWM_Stop(&htim10, TIM_CHANNEL_1);
           HAL_Delay(sound_off_timer);
       }
   }
	 
   return 0;
}

#if 0
uint8_t breathing_toggle(uint16_t breath_on_timer, uint16_t breath_off_timer)
{

    static uint8_t breath_flag = 0;
    static uint32_t breath_toggle_cnt = 0;
    
    if((breath_flag == 0)&&(HAL_GetTick() >= (breath_toggle_cnt + breath_off_timer)))
    {
        breath_toggle_cnt = HAL_GetTick();

        //print_usart1("sp breath on \r\n");
        MX_TIM4_Init();
        LED_SURPORT_FLAG = 1;

        breath_flag = 1;
    }
    
    if((breath_flag == 1)&&(HAL_GetTick() >= (breath_toggle_cnt + breath_on_timer)))
    {
        breath_toggle_cnt = HAL_GetTick();
        Pwm_Breathing(SPRORT_LED,0);
        breath_flag = 0;
        //print_usart1("sp breath off \r\n");

        BSP_LED_Init(LED_SURPORT);
        LED_SURPORT_FLAG = 0;
        
        BSP_LED_Off(LED_SURPORT);        
    } 

    if(breath_flag == 1)
        Pwm_Breathing(SPRORT_LED,1);

    
    return 0 ;

}

uint8_t breathing_toggle_sd(uint16_t breath_on_timer, uint16_t breath_off_timer)
{

    static uint8_t breath_flag = 0;
    static uint32_t breath_toggle_cnt = 0;
    
    if((breath_flag == 0)&&(HAL_GetTick() >= (breath_toggle_cnt + breath_off_timer)))
    {
        breath_toggle_cnt = HAL_GetTick();

        //print_usart1("breath on \r\n");
#ifdef NEED_BREATH        
        Pwm_Breathing(SD_LED,0);
        MX_TIM2_Init();
#else
        BSP_LED_On(LED_SD);     
#endif		
        LED_Sd_FLAG = 1;
        breath_flag = 1;
    }
    
    if((breath_flag == 1)&&(HAL_GetTick() >= (breath_toggle_cnt + breath_on_timer)))
    {
        breath_toggle_cnt = HAL_GetTick();
#ifdef NEED_BREATH        
        Pwm_Breathing(SD_LED,0);
#endif
        breath_flag = 0;
        //print_usart1("breath off \r\n");
        BSP_LED_Init(LED_SD);
        LED_Sd_FLAG = 0;   
        BSP_LED_Off(LED_SD);        
    } 

#ifdef NEED_BREATH   
    if(breath_flag == 1)
        Pwm_Breathing(SD_LED,1);
#endif
    
    return 0 ;

}
#endif
/*get free size of flash !!!*/
uint8_t get_space(void)
{

    FATFS *fs;
    DWORD fre_clust, fre_sect, tot_sect;
    FRESULT res = FR_OK;
    float tp;
	int i = 0;
	
    /* Get volume information and free clusters of drive 1 */
    //__disable_irq();    
    res = f_getfree("", &fre_clust, &fs);
    //__enable_irq();
    if (res) 
    {
        print_usart1("f_getfree faild :%d\r\n",res);
		for(i= 0;i<100;i++)
	    {
		    osDelay(100);
		    res = f_getfree("", &fre_clust, &fs);
			if(res == 0)
				break;
	    }

		if(i == 100)
	    {
           system_flag_table->sd_stats = SD_STATS_ERROR_CARD;

           return 0;
	    }
    }

    {
        /* Get total sectors and free sectors */
        tot_sect = (fs->n_fatent - 2) * fs->csize;
        fre_sect = fre_clust * fs->csize ;
        
        /* Print the free space (assuming 512 bytes/sector) */
        //print_usart1("%10lu KiB total drive space.\r\n%10lu KiB available.\r\n",
        //tot_sect / 2, fre_sect / 2);
        tp = fre_sect;
        tp =((tp/tot_sect) *100);
        if(tp < 1)
        {
		    if(tp < 0.1)
                tp = 0.0;
			else
				tp = 1.0;
        }
        //print_usart1("get_space :%d\r\n",(uint8_t)tp);
        return (uint8_t)tp;

    }
    
}




void gps_led_config(void)
{

    static uint32_t gps_timer_cnt = 0 ;
    static uint8_t gps_led_flag = 0;
    
    if(gpsx->gpssta >= 1)
    {
        if((gps_led_flag == 0)&&(HAL_GetTick() >= (gps_timer_cnt + 700)))
        {
            gps_timer_cnt = HAL_GetTick();
            gps_led_flag = 1;
            BSP_LED_On(LED_GPS);
        }
        
        if((gps_led_flag == 1)&&(HAL_GetTick() >= (gps_timer_cnt + 250)))
        {
            gps_timer_cnt = HAL_GetTick();
            BSP_LED_Off(LED_GPS);
            gps_led_flag = 0;
        } 
    
    }
    else
    {
        BSP_LED_On(LED_GPS); 
    }

}


void sd_led_config(uint16_t breath_on_timer, uint16_t breath_off_timer)
{

    static uint32_t gps_timer_cnt = 0 ;
    static uint8_t gps_led_flag = 0;
    
    if(((gpsx->gpssta >= 1)&&(system_flag_table->Message_head_number > 0))||(system_flag_table->gujiFormats == GUJI_FORMATS_MEA))
    {
        if((gps_led_flag == 0)&&(HAL_GetTick() >= (gps_timer_cnt + breath_on_timer)))
        {
            gps_timer_cnt = HAL_GetTick();
            gps_led_flag = 1;
            BSP_LED_On(LED_SD);
        }
        
        if((gps_led_flag == 1)&&(HAL_GetTick() >= (gps_timer_cnt + breath_off_timer)))
        {
            gps_timer_cnt = HAL_GetTick();
            BSP_LED_Off(LED_SD);
            gps_led_flag = 0;
        } 
    
    }
    else
    {
        BSP_LED_On(LED_SD); 
    }

}


void surport_led_config(uint16_t breath_on_timer, uint16_t breath_off_timer)
{

    static uint32_t gps_timer_cnt = 0 ;
    static uint8_t gps_led_flag = 0;
    
    if(gpsx->gpssta >= 1)
    {
        if((gps_led_flag == 0)&&(HAL_GetTick() >= (gps_timer_cnt + breath_on_timer)))
        {
            gps_timer_cnt = HAL_GetTick();
            gps_led_flag = 1;
            BSP_LED_On(LED_SURPORT);
        }
        
        if((gps_led_flag == 1)&&(HAL_GetTick() >= (gps_timer_cnt + breath_off_timer)))
        {
            gps_timer_cnt = HAL_GetTick();
            BSP_LED_Off(LED_SURPORT);
            gps_led_flag = 0;
        } 
    
    }
    else
    {
        BSP_LED_On(LED_SD); 
    }

}
void auto_power_off(void)
{
    static uint8_t auto_power_timer = 0;  
//    uint32_t eeprom_flag = 0;  
 

    
    if((system_flag_table->charger_connected  == 1)||(is_power_from_auto  == 0))
    {
        auto_power_timer = 0;
        return ;
    }

    if((system_flag_table->auto_power_Status == 1)&&(system_flag_table->power_status != POWER_STANBY))
    {
        auto_power_timer++;
        if(auto_power_timer == 30)
        {
           system_flag_table->power_status = POWER_STANBY;   
           system_flag_table->auto_power_Status = 0;
           print_usart1("AUTO POWER OFF \r\n");
           is_power_from_auto = 0;
           start_hotplug = 0;
           BSP_LED_Off(LED_GREEN);
           //USBD_Start(&hUsbDeviceFS);
           if(usb_init_flag == 0)
           {
               MX_USB_DEVICE_Init();
               usb_init_flag = 1;
       	  }  

          SystemClock_Config_resume();
          MX_TIM10_Init();        
          sound_toggle_simple(1,500,150);  
 
 		  while(osThreadGetState(Get_gps_info_Handle) != osThreadSuspended) { osDelay(10);}//|| (osThreadGetState(defaultTaskHandle) == osThreadSuspended))
 		  while(osThreadGetState(defaultTaskHandle) != osThreadSuspended) { osDelay(10);}
 
          gps_power_mode(0);
          sd_power_mode(0);
 
 		  print_usart1("************\r\n");
 		  print_usart1("goto stanby.\r\n");
 		  print_usart1("************\r\n");
 		  StopSequence_Config();                  

           
       }
    }
      else
          auto_power_timer = 0;
      

}

void auto_power_on(void)
{
    static uint8_t auto_power_timer = 0;  
    uint32_t eeprom_flag = 0;  
     
    if((system_flag_table->charger_connected  == 0)||(start_hotplug == 0))
    {
        auto_power_timer = 0;
        return ;
    }

    if((system_flag_table->auto_power == 1)&&(system_flag_table->power_status == POWER_STANBY))
    {
        auto_power_timer++;
        if(auto_power_timer == 28)
        {
                    
            print_usart1("AUTO POWER ON \r\n");
            sound_flag = 1 ;
            start_hotplug = 0 ;
            is_power_from_auto = 1;
            stm_read_eerpom(11,&eeprom_flag);
            stm_write_eerpom(11,(eeprom_flag+1));                      
            sound_toggle_simple(1,50,50);                                    
            system_flag_table->power_status = system_flag_table->power_mode;  
            system_flag_table->auto_power_Status = 1;
            gps_power_mode(1);
            sd_power_mode(1);
            system_flag_table->guji_mode = RECORED_START;
  #if 1
            if(usb_init_flag == 1)
            {
                USBD_DeInit(&hUsbDeviceFS);
                USBD_Stop(&hUsbDeviceFS);
                usb_init_flag = 0;
            }
  #endif
            osThreadResume(defaultTaskHandle);
            osThreadResume(Get_gps_info_Handle);
            auto_power_timer = 0;
  
              
        }
    }
      else
          auto_power_timer = 0;
      

}
void status_led_config(void)
{
    static uint32_t read_timer_cnt = 0 ;
    static uint8_t read_led_flag = 0;
    static uint32_t green_timer_cnt = 0 ;
    static uint8_t green_led_flag = 0;    

    if(HAL_GPIO_ReadPin(USB_DETECT_GPIO_PORT, USB_DETECT_PIN) != GPIO_PIN_RESET) /*充电中*/
    {

        if(system_flag_table->batt_Status == BATT_CHARG_OK)
        {
            BSP_LED_On(LED_GREEN);
            BSP_LED_Off(LED_RED);
        }
        else
        {

            BSP_LED_On(LED_RED);

        }


        if(system_flag_table->charger_connected  == 0)
        {
           BSP_LED_Off(LED_GREEN);
           start_hotplug = 1 ;
        }
        

        system_flag_table->charger_connected = 1;
        if((system_flag_table->power_status != POWER_STANBY)&&(system_flag_table->power_status != POWER_LRUN_SLEEP)\
          &&(system_flag_table->power_status != POWER_SURPORT_SLEEP))  
        {
		
            gps_led_config();
        }
        else
            BSP_LED_Off(LED_GPS);

       if(system_flag_table->power_status == POWER_LRUN_SLEEP)
       {
  
           if((system_flag_table->lowpower_timer) > (HAL_GetTick() - system_flag_table->grecord_timer_cnt ))
           {
            ;
           }                   
           else
           {
                BSP_LED_On(LED_BULE);     
                system_flag_table->power_status = POWER_LRUN;

                SystemClock_Config_resume();
                BSP_SD_ITConfig();
                MX_TIM10_Init();                                        
                print_usart1("*********\r\n");
                print_usart1("levef lprun mode  resume \r\n");       
                print_usart1("******** \r\n");           
                gps_power_mode(1);
                sd_power_mode(1);
                //HAL_UART_Receive_DMA(&huart3, (uint8_t *)uart3_buffer, 1);
                osThreadResume(Get_gps_info_Handle);
                osThreadResume(defaultTaskHandle);    
                if(system_flag_table->guji_mode == RECORED_IDLE)
                    system_flag_table->guji_mode = RECORED_RESTART_2;
                else
                    system_flag_table->guji_mode = RECORED_START_DOING;
                system_flag_table->grecord_timer_cnt = HAL_GetTick();
                
           }
   
       }


    }
    else
    {


        system_flag_table->charger_connected = 0;
        start_hotplug = 0 ;

        if(system_flag_table->batt_Status <= BATT_LOW)
        {
            BSP_LED_Off(LED_GREEN);
            BSP_LED_Off(LED_BULE);				
            if((read_led_flag == 0)&&(HAL_GetTick() >= (read_timer_cnt + 1000)))
            {
                read_timer_cnt = HAL_GetTick();
                read_led_flag = 1;
                BSP_LED_On(LED_RED);
            }
            
            if((read_led_flag == 1)&&(HAL_GetTick() >= (read_timer_cnt + 1000)))
            {
                read_timer_cnt = HAL_GetTick();
                BSP_LED_Off(LED_RED);
                read_led_flag = 0;
            } 

			if((system_flag_table->batt_Status == BATT_EMPTY  || system_flag_table->batt_Status == 0xff))
		    {
		          system_flag_table->power_status = POWER_STANBY;   
                              
                  //print_usart1("POWER OFF \r\n");
                  BSP_LED_Off(LED_BULE);
                  BSP_LED_Off(LED_GREEN);
                  //USBD_Start(&hUsbDeviceFS);
                  if(usb_init_flag == 0)
                  {
                      MX_USB_DEVICE_Init();
                      usb_init_flag = 1;
              	  }  
                  
                  SystemClock_Config_resume();
                  MX_TIM10_Init();                          
                  sound_toggle_simple(1,500,150);  

				  while(osThreadGetState(Get_gps_info_Handle) != osThreadSuspended) { osDelay(10);}//|| (osThreadGetState(defaultTaskHandle) == osThreadSuspended))
				  while(osThreadGetState(defaultTaskHandle) != osThreadSuspended) { osDelay(10);}

                  gps_power_mode(0);
                  

				  print_usart1("************\r\n");
				  print_usart1("goto stanby.\r\n");
				  print_usart1("************\r\n");
                  if(HAL_GPIO_ReadPin(USB_DETECT_GPIO_PORT, USB_DETECT_PIN) == GPIO_PIN_RESET)
                  {
				      StopSequence_Config();                  
                  }
                  else
                  {
                      sd_power_mode(0);
                  }
                  return ;        
		    }
			
			
        }
		else
	    {
	        BSP_LED_Off(LED_RED);
	    }
		
        if((system_flag_table->power_status != POWER_STANBY)&&(system_flag_table->power_status != POWER_LRUN_SLEEP)\
          &&(system_flag_table->power_status != POWER_SURPORT_SLEEP))  
        {
            if(HAL_GPIO_ReadPin(USB_DETECT_GPIO_PORT, USB_DETECT_PIN) == GPIO_PIN_RESET)
            {
                if((system_flag_table->power_status  == POWER_RUN) ||(system_flag_table->power_status  == POWER_SURPORT_RUN))
                {
                    if(system_flag_table->batt_Status <= BATT_LOW)
                    {
                    }
                    else
                    {
                        BSP_LED_On(LED_GREEN);
                    }
                }
				else
					BSP_LED_Off(LED_GREEN);
            }
             
            if(system_flag_table->power_status == POWER_LRUN)
            {
                if(system_flag_table->batt_Status <= BATT_LOW)
                {
                    BSP_LED_Off(LED_BULE);  
                }
                else
                {
                    BSP_LED_On(LED_BULE);  
                }
            }        
    
            gps_led_config();
        }
        else
        {
    
            BSP_LED_Off(LED_GPS);
            if(system_flag_table->power_status == POWER_STANBY)
                BSP_LED_Off(LED_GREEN);

            if(system_flag_table->power_status == POWER_SURPORT_SLEEP)
            {
                //print_usart1("%d,%d,%d\r\n",green_led_flag,HAL_GetTick(),(green_timer_cnt + 2700));
                if(system_flag_table->batt_Status <= BATT_LOW)
                {
                    BSP_LED_Off(LED_GREEN);
                    return ; 
                }   
                if((green_led_flag == 0)&&(HAL_GetTick() >= (green_timer_cnt + 2700)))
                {
                    green_timer_cnt = HAL_GetTick();
                    green_led_flag = 1;
                    BSP_LED_On(LED_GREEN);
                    //print_usart1("LED_GREEN on \r\n");
                }
                
                if((green_led_flag == 1)&&(HAL_GetTick() >= (green_timer_cnt + 250)))
                {
                    green_timer_cnt = HAL_GetTick();
                    BSP_LED_Off(LED_GREEN);
                    green_led_flag = 0;
                    //print_usart1("LED_GREEN off :%d \r\n",SystemCoreClock);                
                } 
               
            }
            else if(system_flag_table->power_status == POWER_LRUN_SLEEP)
            {
       
              if((system_flag_table->lowpower_timer) > (HAL_GetTick() - system_flag_table->grecord_timer_cnt))
 //             if((5000) > (HAL_GetTick() - system_flag_table->grecord_timer_cnt))

             {
                    if((green_led_flag == 0)&&(HAL_GetTick() >= (green_timer_cnt + 2700)))
                    {
                        green_timer_cnt = HAL_GetTick();
                        green_led_flag = 1;
                        if(system_flag_table->batt_Status > BATT_LOW)
                            BSP_LED_On(LED_BULE);
                    }
                    
                    if((green_led_flag == 1)&&(HAL_GetTick() >= (green_timer_cnt + 300)))
                    {
                        green_timer_cnt = HAL_GetTick();
                        BSP_LED_Off(LED_BULE);
                        green_led_flag = 0;
                    } 
        
                }
                else
                {
                     BSP_LED_On(LED_BULE);     
                     system_flag_table->power_status = POWER_LRUN;
    
                     SystemClock_Config_resume();
                     BSP_SD_ITConfig();
                     //BSP_SD_Init();
                     MX_TIM10_Init();                                        
                     print_usart1("*********\r\n");
                     print_usart1("levef lprun mode  resume \r\n");       
                     print_usart1("******** \r\n");           
                     gps_power_mode(1);
                     sd_power_mode(1);
                     HAL_UART_Receive_DMA(&huart3, (uint8_t *)uart3_dma_buffer, 1);
                     osThreadResume(Get_gps_info_Handle);
                     osThreadResume(defaultTaskHandle);    
                     if(system_flag_table->guji_mode == RECORED_IDLE)
                         system_flag_table->guji_mode = RECORED_RESTART_2;
                     else
                         system_flag_table->guji_mode = RECORED_START_DOING;
                     system_flag_table->grecord_timer_cnt = HAL_GetTick();
                     
                }
        
            }
                       
        }
    
 
           
          
    }

}


/* USER CODE END 4 */
#ifdef TEST_WRITE_SD
  static   uint16_t cnt_record = 0;
#endif


/* WWDG init function */
static void MX_WWDG_Init(void)
{

  hwwdg.Instance = WWDG;
  hwwdg.Init.Prescaler = WWDG_PRESCALER_8;
  hwwdg.Init.Window = 100;
  hwwdg.Init.Counter = 100;
  hwwdg.Init.EWIMode = WWDG_EWI_DISABLE;
  if (HAL_WWDG_Init(&hwwdg) != HAL_OK)
  {
    Error_Handler();
  }

}


/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  FIL test_fp ;


  //uint8_t save_temp = 0;  
  /* init code for FATFS */
  MX_FATFS_Init();

  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 5 */
  usb_init_flag = 1;
  print_usart1("StartDefaultTask \r\n");

#if 0
  BSP_LED_On(LED_GREEN);
  BSP_LED_On(LED_RED);
  BSP_LED_On(LED_BULE);

  osDelay(2000);

  BSP_LED_Off(LED_GREEN);
  BSP_LED_Off(LED_RED);
  BSP_LED_Off(LED_BULE);
#endif
  //open_append_sp(&test_fp,"2018-01/10053644.CSV");

  /*保存文件*/
  /* Infinite loop */
  for(;;)
  {

    //if (osMutexWait(gpsMutexHandle, osWaitForever) == osOK)
    {    
        if(system_flag_table->guji_mode == RECORED_START_DOING)
        {
            if(HAL_GetTick() > (save_file_cnt + 1000))
            {
                system_flag_table->guji_mode = RECORED_SAVE; 
                //recored_flag = 1;
                save_file_cnt  = HAL_GetTick();
                //cnt_record = 10;
            }

        }
        else
        {
          save_file_cnt  = HAL_GetTick();
        }
        Recording_guji(&gps_fp,system_flag_table,gpsx);

#if 0   
        if (osMutexRelease(gpsMutexHandle) != osOK)
        {
            Error_Handler();
        }
#endif
        //ThreadResume(Get_gps_info_Handle);
    }
    
    if((system_flag_table->power_status == POWER_STANBY)
        ||(system_flag_table->power_status == POWER_LRUN_SLEEP)||(system_flag_table->power_status == POWER_SURPORT_SLEEP))
    {
        if(system_flag_table->guji_mode  == RECORED_START_DOING)
        {
            if(1)//system_flag_table->power_status == POWER_STANBY)
            {
                 system_flag_table->guji_mode = RECORED_STOP;
                 Recording_guji(&gps_fp,system_flag_table,gpsx);
            }
#if 0            
            else
            {
                save_temp = system_flag_table->power_status ; 
                system_flag_table->power_status = POWER_LRUN;
                system_flag_table->guji_mode = RECORED_SAVE;
                Recording_guji(&gps_fp,system_flag_table,gpsx);
                system_flag_table->power_status = save_temp;

             }
#endif
        }
        print_usart1("default suspend \r\n");
        osThreadSuspend(NULL);
    }
   
   //print_usart1("4\r\n");

    osDelay(1);

  }
  /* USER CODE END 5 */ 
}

/* Get_gps_info function */


//---------------------------------------------------------------------------
NMEA_STN_DATA_T rRawData;     //Output Sentence

short i2DataIdx = 0;
short m_i2PktDataSize = 0;
NMEA_STN_T m_eLastDecodedSTN;

//---------------------------------------------------------------------------
void DetermineStnType()
{
  if ( (strncmp(&rRawData.Data[0], "$GPGGA", 6) == 0) || 
  	   (strncmp(&rRawData.Data[0], "$GNGGA", 6) == 0) || 
  	   (strncmp(&rRawData.Data[0], "$BDGGA", 6) == 0) || 
  	   (strncmp(&rRawData.Data[0], "$GLGGA", 6) == 0) || 
  	   (strncmp(&rRawData.Data[0], "$GBGGA", 6) == 0) )
  {
    rRawData.eType = STN_GGA;
  }
  else if ( (strncmp(&rRawData.Data[0], "$GPGLL", 6) == 0) || 
  	 		(strncmp(&rRawData.Data[0], "$GNGLL", 6) == 0) || 
  	 		(strncmp(&rRawData.Data[0], "$BDGLL", 6) == 0) || 
  	 		(strncmp(&rRawData.Data[0], "$GLGLL", 6) == 0) || 
  	 		(strncmp(&rRawData.Data[0], "$GBGLL", 6) == 0) )
  {
    rRawData.eType = STN_GLL;
  }
  else if (	(strncmp(&rRawData.Data[0], "$GPGSA", 6) == 0) || 
  			(strncmp(&rRawData.Data[0], "$GNGSA", 6) == 0) || 
  			(strncmp(&rRawData.Data[0], "$BDGSA", 6) == 0) || 
  			(strncmp(&rRawData.Data[0], "$GLGSA", 6) == 0) || 
  			(strncmp(&rRawData.Data[0], "$GAGSA", 6) == 0) || 
  			(strncmp(&rRawData.Data[0], "$GBGSA", 6) == 0) )
  {
    rRawData.eType = STN_GSA;
  }
  else if (	(strncmp(&rRawData.Data[0], "$GPQSA", 6) == 0) || 
  			(strncmp(&rRawData.Data[0], "$QZQSA", 6) == 0) ||
  			(strncmp(&rRawData.Data[0], "$GBQSA", 6) == 0) )
  {
    rRawData.eType = STN_QSA;
  }
  else if ( (strncmp(&rRawData.Data[0], "$GPGSV", 6) == 0) || 
  			(strncmp(&rRawData.Data[0], "$QZGSV", 6) == 0) || 
  			(strncmp(&rRawData.Data[0], "$GLGSV", 6) == 0) || 
  			(strncmp(&rRawData.Data[0], "$GAGSV", 6) == 0) || 
  			(strncmp(&rRawData.Data[0], "$BDGSV", 6) == 0) || 
  			(strncmp(&rRawData.Data[0], "$GBGSV", 6) == 0) )
  {
    rRawData.eType = STN_GSV;
  }
  else if ( (strncmp(&rRawData.Data[0], "$GPRMC", 6) == 0) || 
  			(strncmp(&rRawData.Data[0], "$GNRMC", 6) == 0) || 
  			(strncmp(&rRawData.Data[0], "$BDRMC", 6) == 0) || 
  			(strncmp(&rRawData.Data[0], "$GLRMC", 6) == 0) ||
  			(strncmp(&rRawData.Data[0], "$GBRMC", 6) == 0) )
  {
    rRawData.eType = STN_RMC;
  }
  else if (	(strncmp(&rRawData.Data[0], "$GPVTG", 6) == 0) || 
  			(strncmp(&rRawData.Data[0], "$GNVTG", 6) == 0) || 
  			(strncmp(&rRawData.Data[0], "$BDVTG", 6) == 0) || 
  			(strncmp(&rRawData.Data[0], "$GLVTG", 6) == 0) ||
  			(strncmp(&rRawData.Data[0], "$GBVTG", 6) == 0) )
  {
    rRawData.eType = STN_VTG;
  }
  else
  {
    rRawData.eType = STN_OTHER;
  }
}

GBOOL fgNmeaCheckSum(GCHAR* pData, GINT32 i4Size)
{
    GINT32 i;
    int limit;
    GUCHAR chksum = 0, chksum2 = 0;

    if (i4Size < 6)
    {
        return false;
    }

    chksum = pData[1];
    limit = i4Size - 2;
    for (i = 2; i < (limit); i++)
    {
      if (pData[i] != '*')
      {
        chksum ^= pData[i];

        // Exclude invalid NMEA characters
        if (pData[i] & 0x80)
        {
          return false;
        }
      }
      else
      {
        if (pData[i + 1] >= 'A')
        {
          //chksum2 = (pData[i+1]-'A'+10)<<4;
          chksum2 = (pData[i+1]-55)<<4;
        }
        else
        {
          chksum2 = (pData[i+1]-'0')<<4;
        }
        if (pData[i + 2] >= 'A')
        {
          //chksum2 += pData[i+2]-'A'+10;
          chksum2 += pData[i+2]-55;
        }
        else
        {
          chksum2 += pData[i+2]-'0';
        }
        break;
      }
    }

    // if not found character '*'
    if (i >= (i4Size - 2))
    {
      return (false);
    }

    if (chksum == chksum2)
    {
      return (true);
    }
    else
    {
      return (false);
    }
}


//---------------------------------------------------------------------------
void ProcNmeaSentence(nmea_msg *Proc_gpsx)
{
  	GBOOL fgParserResult;
    GBOOL fgValidPkt;

    fgValidPkt = fgNmeaCheckSum(rRawData.Data, rRawData.i2PacketSize - 1);

	if (fgValidPkt)
    {
      // Determine sentence type
      DetermineStnType();

      // Decode NMEA sentence

      if (rRawData.eType == STN_GGA)
      {
          fgParserResult = NMEA_GPGGA_Analysis(Proc_gpsx,rRawData.Data);
          fgParserResult = NMEA_GNGGA_Analysis(Proc_gpsx,rRawData.Data);
      }

      else if (rRawData.eType == STN_GLL)
      {
      }

      else if (rRawData.eType == STN_GSA)
      {

      }

      else if (rRawData.eType == STN_QSA)
      {

      }

      else if (rRawData.eType == STN_GSV)
      {

      }

      else if (rRawData.eType == STN_RMC)
      {

          fgParserResult = NMEA_GPRMC_Analysis(Proc_gpsx,rRawData.Data);
          fgParserResult = NMEA_GNRMC_Analysis(Proc_gpsx,rRawData.Data);
          
      }

      else if (rRawData.eType == STN_VTG)
      {

      }

      else
      {
          fgParserResult = false;
          rRawData.eType = STN_OTHER;
      }

      m_eLastDecodedSTN = rRawData.eType;
   }

    
}



// 2，rxp_init_pcrx()初始化之后把rxp_pcrx_nmea()作为回调函数注册给串口接收程序

// 3，解析rxp_pcrx_nmea()存下来的NMEA语句



void Get_gps_info(void const * argument)
{
  /* USER CODE BEGIN Get_gps_info */
  uint16_t rxlen = 0;
 // uint8_t *gps_data = NULL;
  uint8_t recode_cnt = 0 ;



  /* Infinite loop */
  print_usart1("Get_gps_info\r\n");
  /*##-4- Put UART peripheral in reception process ###########################*/  
  gps_init();  
  //osDelay(1000);
  HAL_UART_Receive_DMA(&huart3, (uint8_t *)uart3_dma_buffer, 100); 
  //print_usart1("Get_gps_info start !\r\n");
  
  for(;;)
  {  
      // 判断rxp_init_pcrx()是否解析到可用的NMEA语句
      if(rxp_inst_avail(&rRawData.i2PacketType, &i2DataIdx, &m_i2PktDataSize))
      {
          // 把整条NMEA语句拷贝到rRawData.Data[]
          rxp_get_inst(i2DataIdx, m_i2PktDataSize, &rRawData.Data[0]);
                      
          /* we don't need <CR>, replace it with string ending symbol */
          rRawData.Data[m_i2PktDataSize] = '\n';  
          rRawData.Data[m_i2PktDataSize + 1] = 0x00;  
          rRawData.i2PacketSize = m_i2PktDataSize;
          if(recode_cnt == 20)
          {
              print_usart1("\r\n%d - %d\r\n",i2DataIdx,rRawData.i2PacketSize);
              print_usart1("%s",rRawData.Data);
              recode_cnt = 0;
          }
          else
          {
              recode_cnt++;
          }
          // 解析NMEA语句
          ProcNmeaSentence(gpsx);


         
#ifdef TEST_WRITE_SD
          gpsx->gpssta = 2; /*for test*/
          gpsx->posslnum = 5 ;
          gpsx->utc.year = 2018;
          gpsx->utc.month= 1;
          gpsx->utc.date = 13;
          gpsx->latitude = 101; 
          gpsx->longitude = 29;
          gpsx->nshemi = 'N';
          gpsx->ewhemi= 'E';
          gpsx->speed = 0;


          gpsx->hdop = 20;
          system_flag_table->gujiFormats = GUJI_FORMATS_MEA;

#endif          

          surport_mode_config(system_flag_table->power_status,rRawData.Data,m_i2PktDataSize+1);            

      }

      if((system_flag_table->power_status == POWER_STANBY)
        ||(system_flag_table->power_status == POWER_LRUN_SLEEP)||(system_flag_table->power_status == POWER_SURPORT_SLEEP))
      {
          print_usart1("gps_info suspend \r\n");
          osThreadSuspend(NULL);
      }
      //print_usart1("3\r\n");
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
  uint32_t eeprom_flag = 0;

  extern USBD_HandleTypeDef hUsbDeviceFS;
//  uint8_t _breath_flag_ = 0;
  //HAL_NVIC_DisableIRQ(EXTI1_IRQn);
  BSP_PB_Init(BUTTON_USER,BUTTON_MODE_GPIO);  
  BSP_PB_Init(BUTTON_WAKEUP,BUTTON_MODE_GPIO);

  /* Infinite loop */
  for(;;)
  {  

      _user_key_ = get_key();
      if(_user_key_  != 0x00)
      {
          print_usart1("_user_key_:%d \r\n",_user_key_);
		  usb_timer_cnt = 0;
   
      }
      switch(_user_key_)
      {
          case USER_KEY:

		  	    if(system_flag_table->power_status == POWER_SURPORT_SLEEP)
		  	    {			
                    system_flag_table->power_status = POWER_SURPORT_RUN;
                    //BSP_SD_Init();
                    gps_power_mode(1);
                    sd_power_mode(1) ;
                    SystemClock_Config_resume();

                    HAL_NVIC_DisableIRQ(EXTI1_IRQn);
 				    MX_TIM10_Init();
                    osThreadResume(Get_gps_info_Handle);
                    osThreadResume(defaultTaskHandle);
                    print_usart1("****************************** \r\n");
                    print_usart1("levef surport mode  resume \r\n");       
                    print_usart1("****************************** \r\n");                    
                    sound_toggle_simple(1,50,50);                    
                    system_flag_table->guji_mode = RECORED_RESTART_2;
                    break; 
		  	    }
				else if(system_flag_table->power_status == POWER_LRUN_SLEEP)
				{
                    BSP_LED_On(LED_BULE);     
                    system_flag_table->power_status = POWER_LRUN;
    
                    SystemClock_Config_resume();
// 				       BSP_SD_Init();
 				    MX_TIM10_Init();                    
                    print_usart1("*********\r\n");
                    print_usart1("levef lprun mode  resume \r\n");       
                    print_usart1("******** \r\n");           
                    gps_power_mode(1);
                    sd_power_mode(1);
                    //HAL_UART_Receive_DMA(&huart3, (uint8_t *)uart3_buffer, 1);
                    osThreadResume(Get_gps_info_Handle);
                    osThreadResume(defaultTaskHandle);                 
                    sound_toggle_simple(1,50,50);
                    system_flag_table->guji_mode = RECORED_RESTART_2;
                    system_flag_table->grecord_timer_cnt = HAL_GetTick();				
                    break; 				
				}

 
                if(system_flag_table->guji_mode == RECORED_START_DOING)
                {
                    if((gpsx->gpssta >= 1)&&(gpsx->latitude >0)&&(gpsx->longitude>0))
                    {
                        system_flag_table->guji_mode = RECORED_T;
                        sound_toggle_simple(1,50,50);

                    }
                }    
                         
              break;
          case POWER_KEY:  /*切换记录模式，运动和普通*/
		  	
    			if(system_flag_table->power_status == POWER_SURPORT_SLEEP)
    			{			
    				system_flag_table->power_status = POWER_SURPORT_RUN;
//                    BSP_SD_Init();
    				gps_power_mode(1);
    				sd_power_mode(1) ;
    				SystemClock_Config_resume();

    				HAL_NVIC_DisableIRQ(EXTI1_IRQn);
    				MX_TIM10_Init();
    				osThreadResume(Get_gps_info_Handle);
    				osThreadResume(defaultTaskHandle);
    				print_usart1("****************************** \r\n");
    				print_usart1("levef surport mode  resume \r\n");	   
    				print_usart1("****************************** \r\n");					
    			    sound_toggle_simple(1,50,50);
                    system_flag_table->guji_mode = RECORED_RESTART_2;
    				break; 
    			}
    			else if(system_flag_table->power_status == POWER_LRUN_SLEEP)
    			{
    			   BSP_LED_On(LED_BULE);	 
    			   system_flag_table->power_status = POWER_LRUN;
    			   SystemClock_Config_resume();
// 				   BSP_SD_Init();
 				   MX_TIM10_Init();                       
    			   print_usart1("*********\r\n");
    			   print_usart1("levef lprun mode  resume \r\n");		
    			   print_usart1("******** \r\n");			
    			   gps_power_mode(1);
    			   sd_power_mode(1);
//    			   HAL_UART_Receive_DMA(&huart3, (uint8_t *)uart3_buffer, 1);
    			   osThreadResume(Get_gps_info_Handle);
    			   osThreadResume(defaultTaskHandle);				  
    			   sound_toggle_simple(1,50,50);
    			   system_flag_table->grecord_timer_cnt = HAL_GetTick();	
                   system_flag_table->guji_mode = RECORED_RESTART_2;
    			   break;				
    			}

				if(system_flag_table->power_status == POWER_RUN)
                {       
                    system_flag_table->power_status = POWER_SURPORT_RUN;
                    system_flag_table->power_mode = POWER_SURPORT_RUN;
                    if(LED_SURPORT_FLAG == 1)
                    {
                        BSP_LED_Init(LED_SURPORT);
                        LED_SURPORT_FLAG = 0;
                    }
                    BSP_LED_On(LED_SURPORT);   
                    stm_write_eerpom(0xf0,1);
					sound_toggle_simple(1,50,50);

                }
				else if(system_flag_table->power_status == POWER_SURPORT_RUN) 
                {             
					system_flag_table->power_status = POWER_RUN;
                    system_flag_table->power_mode = POWER_RUN;	
                    if(LED_SURPORT_FLAG == 1)
                    {
                        BSP_LED_Init(LED_SURPORT);
                        LED_SURPORT_FLAG = 0;
                    }
                    BSP_LED_Off(LED_SURPORT);
                    stm_write_eerpom(0xf0,0);
                    sound_toggle_simple(1,50,50);					

                }

           
                print_usart1("POWER MODE :%d \r\n",system_flag_table->power_status);


              break;
          case USER_KEY_LONG:  /*重新开启一条轨迹*/
              if(system_flag_table->guji_mode >= RECORED_START)
              {
                  system_flag_table->guji_mode = RECORED_STOP;		                      

				  osThreadSuspend(Get_gps_info_Handle);
				  sound_toggle_simple(3,50,50);
                  Recording_guji(&gps_fp,system_flag_table,gpsx);
                  system_flag_table->guji_mode = RECORED_RESTART;
                  is_locker = 0;
//                  HAL_UART_Receive_DMA(&huart3, (uint8_t *)uart3_buffer, 1);
				  osThreadResume(Get_gps_info_Handle);
 

				  
              }
			  
              break;
          case POWER_KEY_LONG_5S:
              if(system_flag_table->power_status == POWER_STANBY)
              {
                  system_flag_table->power_status = POWER_LRUN ;                    
                  system_flag_table->auto_power_Status = 0;
                  //sound_flag = 1 ;
                  is_power_from_auto = 0;
                  stm_read_eerpom(11,&eeprom_flag);
                  stm_write_eerpom(11,(eeprom_flag+1));     
                  sound_toggle_simple(2,500,150); 
                  gps_power_mode(1);
                  sd_power_mode(1);
      
      
                  print_usart1("L - POWER ON \r\n");
                  BSP_LED_Off(LED_SURPORT);
                
                  system_flag_table->grecord_timer_cnt = HAL_GetTick();
                  if(system_flag_table->guji_mode != RECORED_START)
                  {
                   
                       system_flag_table->guji_mode = RECORED_START;
                       //HAL_UART_Receive_DMA(&huart3, (uint8_t *)uart3_buffer, 1);
                  }
#if 1
                  if(usb_init_flag == 1)
                  {
                      USBD_DeInit(&hUsbDeviceFS);
                      USBD_Stop(&hUsbDeviceFS);
                      usb_init_flag = 0;
                  }
   #endif
                  osThreadResume(defaultTaskHandle);
                  osThreadResume(Get_gps_info_Handle);

              }
              
              break;
          case POWER_KEY_LONG:

              if(system_flag_table->power_status == POWER_STANBY)
              {
                  //print_usart1("SD_DETECT:%d \r\n",HAL_GPIO_ReadPin(SD_DETECT_GPIO_PORT, SD_DETECT_PIN));
                  //if(HAL_GPIO_ReadPin(SD_DETECT_GPIO_PORT, SD_DETECT_PIN) != GPIO_PIN_RESET)
                  {                    
                      print_usart1("POWER ON \r\n");
                      system_flag_table->auto_power_Status = 0;
                      sound_flag = 1 ;
                      is_power_from_auto = 0;
                      stm_read_eerpom(11,&eeprom_flag);
                      stm_write_eerpom(11,(eeprom_flag+1));                      
                      sound_toggle_simple(2,50,50);                                    
                      system_flag_table->power_status = system_flag_table->power_mode;  
                      gps_power_mode(1);
					  sd_power_mode(1);
                      system_flag_table->guji_mode = RECORED_START;
#if 1
                      if(usb_init_flag == 1)
                      {
                          USBD_DeInit(&hUsbDeviceFS);
                          USBD_Stop(&hUsbDeviceFS);
                          usb_init_flag = 0;
                  	  }
#endif
                      osThreadResume(defaultTaskHandle);
                      osThreadResume(Get_gps_info_Handle);

                  }

              }
              else if(system_flag_table->power_status != POWER_STANBY)
              {
                  system_flag_table->power_status = POWER_STANBY;   
                              
                  //print_usart1("POWER OFF \r\n");
                  SystemClock_Config_resume();
                  
                  HAL_NVIC_DisableIRQ(EXTI1_IRQn);
                  MX_TIM10_Init();

                  BSP_LED_Off(LED_GREEN);
                  //USBD_Start(&hUsbDeviceFS);
                  if(usb_init_flag == 0)
                  {
                      MX_USB_DEVICE_Init();
                      usb_init_flag = 1;
              	  }  
                  is_power_from_auto = 0;
                  start_hotplug = 0; 
                  sound_toggle_simple(1,500,150);  

				  while(osThreadGetState(Get_gps_info_Handle) != osThreadSuspended) { osDelay(10);}//|| (osThreadGetState(defaultTaskHandle) == osThreadSuspended))
				  while(osThreadGetState(defaultTaskHandle) != osThreadSuspended) { osDelay(10);}

                  gps_power_mode(0);
                  sd_power_mode(0);

				  print_usart1("************\r\n");
				  print_usart1("goto stanby.\r\n");
				  print_usart1("************\r\n");
                  if(HAL_GPIO_ReadPin(USB_DETECT_GPIO_PORT, USB_DETECT_PIN) == GPIO_PIN_RESET)
                  {
				      StopSequence_Config();                  
                  }
                  
              }            
              break;
          case POWER_USER_KEY_LONG:
              if(usb_init_flag == 1)
              {
                  USBD_DeInit(&hUsbDeviceFS);
                  USBD_Stop(&hUsbDeviceFS);
                  usb_init_flag = 0;
          	  }            
              if(entry_config_mode(system_flag_table) == 0)
                  sound_toggle_simple(1,500,150); 

              if(usb_init_flag == 0)
              {
                  MX_USB_DEVICE_Init();
                  usb_init_flag = 1;
          	  }   
			  print_usart1("************\r\n");
			  print_usart1("goto stanby.\r\n");
			  print_usart1("************\r\n");
              if(HAL_GPIO_ReadPin(USB_DETECT_GPIO_PORT, USB_DETECT_PIN) == GPIO_PIN_RESET)
              {
			      StopSequence_Config();                  
              };
          case FUNCTION_KEY:

 		  	    if(system_flag_table->power_status == POWER_SURPORT_SLEEP)
		  	    {			
                    system_flag_table->power_status = POWER_SURPORT_RUN;
                    //BSP_SD_Init();
                    gps_power_mode(1);
                    sd_power_mode(1) ;
                    SystemClock_Config_resume();

                    HAL_NVIC_DisableIRQ(EXTI1_IRQn);
 				    MX_TIM10_Init();
                    osThreadResume(Get_gps_info_Handle);
                    osThreadResume(defaultTaskHandle);
                    print_usart1("****************************** \r\n");
                    print_usart1("levef surport mode  resume \r\n");       
                    print_usart1("****************************** \r\n");                    
                    sound_toggle_simple(1,50,50);
                    system_flag_table->guji_mode = RECORED_RESTART_2;
                    break; 
		  	    }
				else if(system_flag_table->power_status == POWER_LRUN_SLEEP)
				{
                    BSP_LED_On(LED_BULE);     
                    system_flag_table->power_status = POWER_LRUN;
    
                    SystemClock_Config_resume();
// 				       BSP_SD_Init();
 				    MX_TIM10_Init();                    
                    print_usart1("*********\r\n");
                    print_usart1("levef lprun mode  resume \r\n");       
                    print_usart1("******** \r\n");           
                    gps_power_mode(1);
                    sd_power_mode(1);
//                    HAL_UART_Receive_DMA(&huart3, (uint8_t *)uart3_buffer, 1);
                    osThreadResume(Get_gps_info_Handle);
                    osThreadResume(defaultTaskHandle);                 
                    sound_toggle_simple(1,50,50);
                    system_flag_table->guji_mode = RECORED_RESTART_2;
                    system_flag_table->grecord_timer_cnt = HAL_GetTick();				
                    break; 				
				}
                
                if(system_flag_table->guji_mode == RECORED_START_DOING)
                {
                    if((gpsx->gpssta >= 1)&&(gpsx->latitude >0)&&(gpsx->longitude>0))
                    {
                        if(system_flag_table->function_index == 0)/*pause*/
                        {
                            system_flag_table->guji_mode = RECORED_PAUSE; 
                            print_usart1("pause\r\n");
                        }
                        else if(system_flag_table->function_index == 1)
                        {
                            system_flag_table->guji_mode = RECORED_D; 
                            print_usart1("RECORED_D\r\n");    
                        }
                        sound_toggle_simple(1,50,50);
    
                    }
                }
                else if(system_flag_table->guji_mode == RECORED_PAUSE)
                {

                    if((gpsx->gpssta >= 1)&&(gpsx->latitude >0)&&(gpsx->longitude>0))
                    {
                        if(system_flag_table->function_index == 0)/*pause*/
                        {
                            system_flag_table->guji_mode = RECORED_START_DOING; 
                        }
                        sound_toggle_simple(1,50,50);
                    }
                }
                           
              break;

          case RESTORE_KEY_LONG:
              if(system_flag_table->power_status == POWER_STANBY)
              {
                  system_flag_table->power_status = POWER_STANBY;   
                              
                  reset_eeprom();
                  SystemClock_Config_resume();
                  
                  HAL_NVIC_DisableIRQ(EXTI1_IRQn);
                  MX_TIM10_Init();

                  BSP_LED_Off(LED_GREEN);
                  //USBD_Start(&hUsbDeviceFS);
                  if(usb_init_flag == 0)
                  {
                      MX_USB_DEVICE_Init();
                      usb_init_flag = 1;
              	  }  
                  is_power_from_auto = 0;
                  start_hotplug = 0; 
                  sound_toggle_simple(1,500,150);  

				  while(osThreadGetState(Get_gps_info_Handle) != osThreadSuspended) { osDelay(10);}//|| (osThreadGetState(defaultTaskHandle) == osThreadSuspended))
				  while(osThreadGetState(defaultTaskHandle) != osThreadSuspended) { osDelay(10);}

                  gps_power_mode(0);
                  sd_power_mode(0);

				  print_usart1("************\r\n");
				  print_usart1("reset eeprom goto stanby.\r\n");
				  print_usart1("************\r\n");
                  if(HAL_GPIO_ReadPin(USB_DETECT_GPIO_PORT, USB_DETECT_PIN) == GPIO_PIN_RESET)
                  {
				      StopSequence_Config();                  
                  } 
              }
              break;
          default:break;
      }

      _user_key_  = 0;
#if 0
       if(system_flag_table->power_status != POWER_STANBY)
       {
           if(_commit_ > 0)
           {
               _commit_--;
               if(_commit_ == 0)
               {
                   system_flag_table->guji_mode = RECORED_START;
                   HAL_UART_Receive_DMA(&huart3, (uint8_t *)uart3_buffer, 1);
                   print_usart1("to  start !");
               }
           }
            

       }
#endif       
      osDelay(100);
  }
  /* USER CODE END MySystem */
}


/* update_info function */
void update_info(void const * argument)
{
  /* USER CODE BEGIN update_info */
//  RTC_DateTypeDef sdatestructureget;
//  RTC_TimeTypeDef stimestructureget;  

  static uint16_t sd_timer_cnt = 0 ;
  static uint16_t support_timer_cnt = 0 ;
  static uint16_t adc_cnt = 0 ;

#if 0
     /* Get the RTC current Time */
  HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BCD);
  /* Get the RTC current Date */
  HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BCD);   
  
  system_flag_table->RTC_DateStructure = sdatestructureget;
  system_flag_table->RTC_TimeStructure = stimestructureget;
#endif
  //HAL_WWDG_Refresh(&hwwdg);

  auto_power_on();
  auto_power_off();
  status_led_config();
  HAL_IWDG_Refresh(&Iwdg);

  if(adc_cnt > 600)
  {
      vddmv_adc_proess(system_flag_table); /*更新电池状态*/   
	  adc_cnt = 0; 
  }
  else
  	   adc_cnt ++;

  if(sound_flag == 1)
  {
      sound_toggle_simple(1,50,50);
      sound_flag = 0;
  }
  
  if(system_flag_table->power_status == POWER_STANBY)
  {
     if(HAL_GPIO_ReadPin(USB_DETECT_GPIO_PORT, USB_DETECT_PIN) == GPIO_PIN_RESET)
     {
         usb_timer_cnt ++;
         if(usb_timer_cnt == 100)
         {
             usb_timer_cnt = 0;

   
             print_usart1("******************************************* \r\n");
             print_usart1("when usb detect hotplug, goto stanby angin. \r\n");
             print_usart1("******************************************* \r\n");
             StopSequence_Config();   
         }
     }
    else 
      usb_timer_cnt = 0;   
     
    if(LED_Sd_FLAG == 1)
    {
        BSP_LED_Init(LED_SD);
        //Pwm_Breathing(SD_LED,0);
        LED_Sd_FLAG = 0;
    }  
    BSP_LED_Off(LED_SD);      

    if(LED_SURPORT_FLAG == 1)
    {
        BSP_LED_Init(LED_SURPORT);
        LED_SURPORT_FLAG = 0;
    }
    BSP_LED_Off(LED_SURPORT);
                    
  }
  else   
  {
       usb_timer_cnt = 0;
       if(system_flag_table->power_status == POWER_SURPORT_RUN)
       {
           if((gpsx->speed < 3000)&&(system_flag_table->Message_head_number > 0)&&(system_flag_table->guji_mode != RECORED_IDLE))
           {
               support_timer_cnt ++;
               if(support_timer_cnt == 3000)
               {
                   support_timer_cnt = 0;
                   //StopSequence_Config();
     
                   //HAL_NVIC_EnableIRQ(EXTI1_IRQn);              

                   if(LED_SURPORT_FLAG == 1)
                   {
                       //Pwm_Breathing(SPRORT_LED,0);
                       BSP_LED_Init(LED_SURPORT);
                       LED_SURPORT_FLAG = 0;
                   }                    
                   BSP_LED_Off(LED_SURPORT);
                   BSP_LED_Off(LED_SD);

     
                   system_flag_table->power_status = POWER_SURPORT_SLEEP;

                   //osThreadSuspend(Get_gps_info_Handle);
                   //osThreadSuspend(defaultTaskHandle);
          		   while(osThreadGetState(Get_gps_info_Handle) != osThreadSuspended) { osDelay(1);}//|| (osThreadGetState(defaultTaskHandle) == osThreadSuspended))
          		   while(osThreadGetState(defaultTaskHandle) != osThreadSuspended) { osDelay(1);}//|| (osThreadGetState(defaultTaskHandle) == osThreadSuspended))
                   gps_power_mode(0);
                   sd_power_mode(0) ;

                   SystemClock_Config_msi();
				   //sleep_power_config();				   
                   //osThreadSuspend(SystemCallHandle);                                           
                   HAL_NVIC_EnableIRQ(EXTI1_IRQn);
                   print_usart1("****************************** \r\n");
                   print_usart1("entry surport mode  go to stop \r\n");       
                   print_usart1("****************************** \r\n");   
             
               }
               else
               {
                   if(LED_SURPORT_FLAG == 1)
                   {
                       BSP_LED_Init(LED_SURPORT);
                       LED_SURPORT_FLAG = 0;
                   }
                   BSP_LED_On(LED_SURPORT);
               }
           }
           else
           {
               support_timer_cnt = 0; 
               if(system_flag_table->Message_head_number > 0)
                   surport_led_config(300,700);    
               else
                    BSP_LED_On(LED_SURPORT);
           }
       } 
       else if(system_flag_table->power_status == POWER_SURPORT_SLEEP)
       {
           //lowpower_record_config(1000);
           //print_usart1("--%d \r\n",(HAL_GetTick() - system_flag_table->grecord_timer_cnt));
           if( 2000 <= (HAL_GetTick() - system_flag_table->grecord_timer_cnt))
           {
               if(support_cnt > 120)
               {
                   
                   support_timer_cnt = 0;

                   //system_flag_table->guji_mode  = RECORED_RESTART_2;
                   system_flag_table->power_status = POWER_SURPORT_RUN; 

                   HAL_NVIC_DisableIRQ(EXTI1_IRQn);
                   SystemClock_Config_resume();                 
                   BSP_SD_ITConfig();
//                           SD_IO_Init();     
                   gps_power_mode(1);
                   sd_power_mode(1) ;                   
				   MX_TIM10_Init();
                                     
                   osThreadResume(Get_gps_info_Handle);
                   osThreadResume(defaultTaskHandle);
                   print_usart1("****************************** \r\n");
                   print_usart1("levef surport mode  resume \r\n");       
                   print_usart1("****************************** \r\n");     
                   system_flag_table->guji_mode = RECORED_RESTART_2;
                   LED_SURPORT_F_FLAG = 1;
                   //osThreadResume(SystemCallHandle); 
               }
               system_flag_table->grecord_timer_cnt = HAL_GetTick();
               support_cnt = 0;
          }
   
       }
      if ((system_flag_table->power_status != POWER_STANBY)&&(system_flag_table->power_status != POWER_LRUN_SLEEP)\
      &&(system_flag_table->power_status != POWER_SURPORT_SLEEP))  
      	{
       	   if((HAL_GPIO_ReadPin(SD_DETECT_GPIO_PORT, SD_DETECT_PIN) != GPIO_PIN_RESET)&&(system_flag_table->sd_stats == SD_STATS_OK))
	       {
	     
	           if(system_flag_table->guji_mode != RECORED_START_DOING)
	           {
	              if(LED_Sd_FLAG == 1)
	              {
	                  BSP_LED_Init(LED_SD);
	                  BSP_LED_On(LED_SD);
	                  LED_Sd_FLAG = 0;
	              }
	           }
	           
	           if(system_flag_table->power_status == POWER_SURPORT_RUN ||system_flag_table->power_status == POWER_RUN ||system_flag_table->power_status == POWER_LRUN)
	           {
	               if((gpsx->gpssta >= 1)&&(system_flag_table->wirte_storge_flag == 1))
	               //if((system_flag_table->guji_mode == RECORED_START_DOING))
	               {
	                   if((system_flag_table->guji_record.recoed_formats == BY_TIMES) && (system_flag_table->guji_record.by_time_vaule < 1000))     
	                       sd_led_config(300,700);     
	                   else
	                       sd_led_config(300,2700);     
                       system_flag_table->wirte_storge_flag = 0;
	               }
	               else
	               {
	                   if(LED_Sd_FLAG == 1)
	                   {
	                       BSP_LED_Init(LED_SD);
	                       //Pwm_Breathing(SD_LED,0);
	                       LED_Sd_FLAG = 0;
	                   }  
	                   BSP_LED_On(LED_SD);                    
	               }
	           }
	           else
	           {
	               if(LED_Sd_FLAG == 1)
	               {
	                   BSP_LED_Init(LED_SD);
	                   //Pwm_Breathing(SD_LED,0);
	                   LED_Sd_FLAG = 0;
	               }  
	               BSP_LED_Off(LED_SD);                    
	               
	           }
	           
	   
	           sd_timer_cnt = 0;
	       } 
           else 
	       {
	   
	            if(LED_Sd_FLAG == 1)
	            {
	                BSP_LED_Init(LED_SD);
	                BSP_LED_On(LED_SD);
	                //Pwm_Breathing(SD_LED,0);
	                LED_Sd_FLAG = 0;
	            }
	   
	            if(system_flag_table->sd_stats == SD_STATS_ERROR_CARD)
	            {
	                sound_toggle_config(50,50);
	            }
	            
	            sd_timer_cnt ++;
	   
	            BSP_LED_Toggle(LED_SD);                
	   
	            if(sd_timer_cnt == 100)
	            {
	                sd_timer_cnt = 0;    
                    sound_toggle_simple(1,500,150);  
	                HAL_TIM_PWM_Stop(&htim10, TIM_CHANNEL_1);
	                system_flag_table->power_status  = POWER_STANBY;
	                print_usart1("****************************** \r\n");
	                print_usart1("sd error, goto stanby angin. \r\n");
	                print_usart1("****************************** \r\n");
	          
	                StopSequence_Config();
	                
	          
	            }
	       }
      	}
   
         
    }






    if(system_flag_table->wanng_speed_vaule > 0)
    {
        if((gpsx->speed) > (system_flag_table->wanng_speed_vaule))
        {
           HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
           Wang_FLAG = 1;
        }
        else
        {
           if(Wang_FLAG == 1)
           {
              Wang_FLAG = 0;
              HAL_TIM_PWM_Stop(&htim10, TIM_CHANNEL_1);              
           }
        }
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

#if 0
  if (htim->Instance == TIM7) {
  	if(HAL_GetTick() > (gps_data_time + 10))
  	{
  	    if((USART2_RX_STA_RP != USART2_RX_STA_WP)&&(USART2_RX_STA == 0))
        { 
            if(USART2_RX_STA_WP >= 1)
            {
                if(uart3_buffer[USART2_RX_STA_WP-1] == 0x0a)
                {
                    USART2_RX_STA = 1;
                    save_usart2_wp = USART2_RX_STA_WP;
                   // print_usart1("-%d- \r\n",HAL_GetTick());
                }
            }
            else
            {
                if(uart3_buffer[MAX_UART3_LEN-1] == 0x0a)
                {
                    USART2_RX_STA = 1;
                    save_usart2_wp = USART2_RX_STA_WP;
                    //print_usart1("-%d-\r\n",HAL_GetTick());

                }   
            }
            
        }
		gps_data_time = 0xffffffff;
        recored_flag = 1;
    }
  
//    __HAL_TIM_DISABLE(&htim6);
  }
#endif    
  /* USER CODE BEGIN Callback 1 */

 


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
