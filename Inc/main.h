/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define usb_hotplug_Pin GPIO_PIN_0
#define usb_hotplug_GPIO_Port GPIOA
#define surprot_line_Pin GPIO_PIN_1
#define surprot_line_GPIO_Port GPIOA
#define GPS_POWER_Pin GPIO_PIN_8
#define GPS_POWER_GPIO_Port GPIOA
#define SD_POWER_Pin GPIO_PIN_2
#define SD_POWER_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
#define  MAX_BMP_FILES  25
#define  MAX_BMP_FILE_NAME 11

#define  MAX_UART3_LEN 0x400



#define BUTTON_PRESS 1
#define BUTTON_REALSE 0


#define TIM2_FREQUENCY 12000


#define STATUS_LED_GREEN 0   /*LOW*/
#define STATUS_LED_RED   1   /*LOW*/
#define STATUS_LED_BULE  2   /*LOW*/
#define SD_LED           3   /*HIGH*/
#define SPRORT_LED       4   /*HIGH*/
#define GPS_LED          5   /*HIGH*/


#define POWER_RUN                0
#define POWER_SURPORT_RUN        1  
#define POWER_SURPORT_SLEEP         2  
#define POWER_LRUN               3  
#define POWER_LRUN_SLEEP         4  
#define POWER_STOP               5
#define POWER_STANBY             6  
#define POWER_SD_ERROR             7  


/*button release flag*/
#define USER_KEY_MARK     0x01
#define WAKEUP_KEY_MARK   0x02
#define FUNCTION_KEY_MARK   0x04


#define USER_KEY          0x01
#define POWER_KEY         0x02

#define USER_KEY_LONG     0x03
#define POWER_KEY_LONG    0x04
#define POWER_USER_KEY_LONG    0x05
#define POWER_KEY_LONG_5S    0x06
#define FUNCTION_KEY    0x07
#define FUNCTION_KEY_LONG 0x08
#define RESTORE_KEY_LONG 0x09

//#define OLD 


#define USER_KEY_NO_LONG


#define NORMAL_SURPORT_MODE 0
#define SENCSE_SURPORT_MODE 1
#define LOW_POWER_SURPORT_MODE 3


#define SD_STATS_OK         0
#define SD_STATS_ERROR_CARD 1
#define SD_STATS_NO_CARD    2
#define SD_STATS_BE_WRITE   3


//#define HDOP_RECODE_VAULE 150
//#define HDOP_RECODE_VAULE 120
#define HDOP_RECODE_VAULE 100

extern void print_usart1(char *format, ...);
extern void SystemClock_Config_msi(void);
extern void SystemClock_Config_resume(void);


//#define TEST_WRITE_SD

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
