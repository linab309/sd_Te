/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"

/* USER CODE BEGIN Includes */     
#include "stm32l1xx_hal.h"
#include "FreeRTOSConfig.h"
#include "menutal.h"

#define MY_SYSTICK_CTRL_REG			( * ( ( volatile uint32_t * ) 0xe000e010 ) )
#define MY_SYSTICK_LOAD_REG			( * ( ( volatile uint32_t * ) 0xe000e014 ) )
#define MY_SYSTICK_CURRENT_VALUE_REG	( * ( ( volatile uint32_t * ) 0xe000e018 ) )
#define MY_SYSPRI2_REG				( * ( ( volatile uint32_t * ) 0xe000ed20 ) )

/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/

/* USER CODE BEGIN Variables */
#if configUSE_TICKLESS_IDLE == 1     
    extern uint32_t ulTimerCountsForOneTick ;
#endif

extern system_flag *system_flag_table;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim10;


/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Pre/Post sleep processing prototypes */
void PreSleepProcessing(uint32_t *ulExpectedIdleTime);
void PostSleepProcessing(uint32_t *ulExpectedIdleTime);

/* Hook prototypes */
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 5 */
__weak void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
}

void vApplicationIdleHook(void)
{
 ;
}

/* USER CODE END 5 */

/* USER CODE BEGIN PREPOSTSLEEP */
__weak void PreSleepProcessing(uint32_t *ulExpectedIdleTime)
{
/* place for user code */ 
    /* Called by the kernel before it places the MCU into a sleep mode because
    configPRE_SLEEP_PROCESSING() is #defined to PreSleepProcessing().
    
    NOTE:  Additional actions can be taken here to get the power consumption
    even lower.  For example, peripherals can be turned off here, and then back
    on again in the post sleep processing function.  For maximum power saving
    ensure all unused pins are in their lowest power state. */
 
    /* 
      (*ulExpectedIdleTime) is set to 0 to indicate that PreSleepProcessing contains
      its own wait for interrupt or wait for event instruction and so the kernel vPortSuppressTicksAndSleep 
      function does not need to execute the wfi instruction  
    */  


    /*Enter to sleep Mode using the HAL function HAL_PWR_EnterSLEEPMode with WFI instruction*/
    if((system_flag_table->power_status == POWER_SURPORT_SLEEP)||(system_flag_table->power_status == POWER_LRUN_SLEEP))
        HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI); 
    
    *ulExpectedIdleTime = 0;


}

__weak void PostSleepProcessing(uint32_t *ulExpectedIdleTime)
{
/* place for user code */
    /* Called by the kernel when the MCU exits a sleep mode because
    configPOST_SLEEP_PROCESSING is #defined to PostSleepProcessing(). */
    
    /* Avoid compiler warnings about the unused parameter. */
    uint32_t i;

    (void) ulExpectedIdleTime;
#if 1
    if((system_flag_table->power_status == POWER_SURPORT_SLEEP)||(system_flag_table->power_status == POWER_LRUN_SLEEP))
    {
       for(i = 0;i<*ulExpectedIdleTime ;i++)
       {
           HAL_IncTick();
       }
       
    }
	if((system_flag_table->power_status == POWER_SURPORT_SLEEP)||(system_flag_table->power_status == POWER_LRUN_SLEEP))	
    HAL_ResumeTick();    
#endif
    


}
/* USER CODE END PREPOSTSLEEP */

/* USER CODE BEGIN Application */
void Begin_low_power(void)
{
    //if((__HAL_TIM_GET_ENABLE(&htim10) == 0) && (__HAL_TIM_GET_ENABLE(&htim2) == 0) &&(__HAL_TIM_GET_ENABLE(&htim4) == 0))
    {
        //if((system_flag_table->power_status != POWER_SURPORT_SLEEP)&&(system_flag_table->power_status != POWER_LRUN_SLEEP))
           // SystemClock_Config_msi();
    }
#if configUSE_TICKLESS_IDLE == 1     
    ulTimerCountsForOneTick = ( configCPU_CLOCK_HZ / configTICK_RATE_HZ );
#endif
	if((system_flag_table->power_status == POWER_SURPORT_SLEEP)||(system_flag_table->power_status == POWER_LRUN_SLEEP))
       HAL_SuspendTick();
    //print_usart1("start !\r\n");

}

void End_low_power(void)
{
    //if((__HAL_TIM_GET_ENABLE(&htim10) == 0) && (__HAL_TIM_GET_ENABLE(&htim2) == 0) &&(__HAL_TIM_GET_ENABLE(&htim4) == 0))
    {
        //if((system_flag_table->power_status != POWER_SURPORT_SLEEP)&&(system_flag_table->power_status != POWER_LRUN_SLEEP))
           // SystemClock_Config_resume();
    }
#if configUSE_TICKLESS_IDLE == 1     
    ulTimerCountsForOneTick = ( configCPU_CLOCK_HZ / configTICK_RATE_HZ );
#endif
//print_usart1("end !\r\n");

}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
