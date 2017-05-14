/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications
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

#include "fatfs.h"

uint8_t retUSER;    /* Return value for USER */
char USER_Path[4];  /* USER logical drive path */

/* USER CODE BEGIN Variables */
#include "string.h"
#include "stm_eeprom.h"

FATFS SD_FatFs;
extern system_flag *system_flag_table;
extern void RTC_TimeShow(DWORD* fattime);
extern char *GetIniKeyString(char *title,char *key,char *filename);
extern int GetIniKeyInt(char *title,char *key,char *filename);

//char* pDirectoryFiles[MAX_BMP_FILES];

/* USER CODE END Variables */    

void MX_FATFS_Init(void) 
{
  /*## FatFS: Link the USER driver ###########################*/
  retUSER = FATFS_LinkDriver(&USER_Driver, USER_Path);

  /* USER CODE BEGIN Init */
  //SD_FatFs = malloc(sizeof(FATFS));   
  if(My_Fs_Init(&SD_FatFs) == 1)
  {
      system_flag_table->sd_stats = SD_STATS_ERROR_CARD;
  }

  /* USER CODE END Init */
}

/**
  * @brief  Gets Time from RTC 
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
  /* USER CODE BEGIN get_fattime */

	DWORD fattime = 0;

	RTC_TimeShow(&fattime);

	return fattime;


  /* USER CODE END get_fattime */  
}

/* USER CODE BEGIN Application */
/*------------------------------------------------------------/
/ Open or create a file in append mode
/------------------------------------------------------------*/

FRESULT open_append (
    FIL* fp,            /* [OUT] File object to create */
    const char* path    /* [IN]  File name to be opened */
)
{
    FRESULT fr;

    /* Opens an existing file. If not exist, creates a new file. */
    fr = f_open(fp, path, FA_WRITE | FA_OPEN_ALWAYS);
    if (fr == FR_OK) {
        /* Seek to end of the file to append data */
        fr = f_lseek(fp, f_size(fp));
        if (fr != FR_OK)
            f_close(fp);
    }
    return fr;
}

/* USER CODE END 4 */

uint8_t  My_Fs_Init(FATFS *SD_FatFs)
{

	uint8_t ret = 0;
	//FRESULT fr;
//	FIL fil;	

	/* Check the mounted device */
    if(retUSER == 0)
    {
    	if(f_mount(SD_FatFs, (TCHAR const*)USER_Path, 0) != FR_OK)
    	{
    	   print_usart1("BSP_SD_INIT_FAILED \r\n");
           ret  = 1;
    	}  
    	else
    	{
    	   print_usart1("\r\n f_mount ok \r\n");
    #if 0	
    		fr = f_mkdir("POI");
    		print_usart1("\r\n mkdir_init %d \r\n",fr);
    		
    		
    		if(fr == FR_EXIST)
    			return ret;
    		else if(fr != FR_OK)
    		{
    			fr = f_mkdir("POI");
    			if((fr != FR_OK)&&(fr != FR_EXIST))
    			{
    				print_usart1("\r\n f_mkdir faild :%d \r\n",fr);
                    fr = f_mkfs((TCHAR const*)USER_Path, 0, 0); 
                    fr = f_mkdir("POI");
                    print_usart1("\r\n mkf :%d \r\n",fr);
                    if((fr != FR_OK)&&(fr != FR_EXIST))
                    {
                        print_usart1("\r\n BAD SD CARD \r\n");
                        ret  = 1;
                    }
    			 }
    		 }
#endif        
    	}
    }
    else
    {
        FATFS_UnLinkDriver(USER_Path);
    }

    return ret;
  
}

void configfs_set(FIL *update_config_fp)
{


    char *string = NULL;
    uint8_t flash_cnt = 0;
    uint8_t i = 0;

    BSP_LED_Init(LED_GPS);  
    BSP_LED_Init(LED_SD);  
    BSP_LED_Init(LED_SURPORT); 
    print_usart1("%s\r\n", GetIniKeyString("SETTINGS", "TimeZone", "config.ini"));
    print_usart1("%s\r\n", GetIniKeyString("SETTINGS", "SpeedAlert", "config.ini")); 
    print_usart1("%s\r\n", GetIniKeyString("SETTINGS", "AutoPowerOn", "config.ini"));
    print_usart1("%s\r\n", GetIniKeyString("SETTINGS", "Beeper", "config.ini")); 
    print_usart1("%s\r\n", GetIniKeyString("SETTINGS", "FunctionKey", "config.ini")); 
    print_usart1("%s\r\n", GetIniKeyString("RECORD", "Format", "config.ini"));
    print_usart1("%s\r\n", GetIniKeyString("RECORD", "LogMode", "config.ini")); 
    print_usart1("%s\r\n", GetIniKeyString("RECORD", "SpeedMask", "config.ini"));
    print_usart1("%d\r\n", GetIniKeyInt("RECORD", "SpyModeTimer", "config.ini")); 
    print_usart1("%s\r\n", GetIniKeyString("RECORD", "OneTrackPerDay", "config.ini")); 

    string = GetIniKeyString("SETTINGS", "TimeZone", "config.ini");
    /*TimeZone*/
    i = 0;
    while(timer_zone_Aarry[i] != NULL)
    { 

        if(strcmp(timer_zone_Aarry[i],string) == 0)
            break;
        
        i++;
    }
    system_flag_table->time_zone = i;
    stm_write_eerpom(0,system_flag_table->time_zone);
    /*Buzzer*/

    string = GetIniKeyString("SETTINGS", "Beeper", "config.ini");

    if(strcmp("ON",string) == 0)
    {
       system_flag_table->buzzer = 1; 
    }
    else if(strcmp("OFF",string) == 0)    
    {
       system_flag_table->buzzer = 0;             
    }
    stm_write_eerpom(1,system_flag_table->buzzer);

    /*SpeedWarning*/

    string = GetIniKeyString("SETTINGS", "SpeedAlert", "config.ini");

    if(strcmp("OFF",string) == 0)    
    {
       system_flag_table->wanng_speed_vaule = 0;             
    }
    else
    {
       system_flag_table->wanng_speed_vaule = GetIniKeyInt("SETTINGS", "SpeedAlert", "config.ini"); 
    }

    stm_write_eerpom(2,system_flag_table->wanng_speed_vaule);

    /*AutoPower*/

    string = GetIniKeyString("SETTINGS", "AutoPowerOn", "config.ini");

    if(strcmp("ON",string) == 0)
    {
       system_flag_table->auto_power = 1; 
    }
    else if(strcmp("OFF",string) == 0)    
    {
       system_flag_table->auto_power = 0;             
    } 
    stm_write_eerpom(3,system_flag_table->auto_power);

    /*Format*/
    /*
#define GUJI_FORMATS_CSV 0
#define GUJI_FORMATS_GPX 1
#define GUJI_FORMATS_GPS 2
#define GUJI_FORMATS_MEA 3
#define GUJI_FORMATS_KML 4    
        */
    string = GetIniKeyString("RECORD", "Format", "config.ini");
    //CSV, GPX, NMEA, KML
    if(strcmp("CSV",string) == 0)
    {
       system_flag_table->gujiFormats = GUJI_FORMATS_CSV; 
    }
    else if(strcmp("GPX",string) == 0)    
    {
       system_flag_table->gujiFormats = GUJI_FORMATS_GPX;             
    } 
    else if(strcmp("NMEA",string) == 0)    
    {
       system_flag_table->gujiFormats = GUJI_FORMATS_MEA;             
    } 
    else if(strcmp("KML",string) == 0)    
    {
       system_flag_table->gujiFormats = GUJI_FORMATS_KML;             
    } 

    stm_write_eerpom(4,system_flag_table->gujiFormats);

    /*Mode*/
    /*
    1Hz, 5Hz, 10Hz, 5m, 10m, 20m, 50m, 100m

        */
    string = GetIniKeyString("RECORD", "LogMode", "config.ini");
    //CSV, GPX, NMEA, KML
    if(strcmp("1Hz",string) == 0)
    {
        system_flag_table->guji_record.by_time_vaule   = 1000; /*ms*/
        system_flag_table->guji_record.recoed_formats  = BY_TIMES;
    }
    else if(strcmp("5Hz",string) == 0)    
    {
        system_flag_table->guji_record.by_time_vaule   = 200; /*ms*/
        system_flag_table->guji_record.recoed_formats  = BY_TIMES;

    } 
    else if(strcmp("10Hz",string) == 0)    
    {
        system_flag_table->guji_record.by_time_vaule   = 100; /*ms*/
        system_flag_table->guji_record.recoed_formats  = BY_TIMES;

    } 
    else if(strcmp("5m",string) == 0)    
    {
        system_flag_table->guji_record.by_distance_vaule   = 5; /*ms*/
        system_flag_table->guji_record.recoed_formats  = BY_DISTANCE;

    } 
    else if(strcmp("10m",string) == 0)    
    {
        system_flag_table->guji_record.by_distance_vaule   = 10; /*ms*/
        system_flag_table->guji_record.recoed_formats  = BY_DISTANCE;

    } 
    else if(strcmp("20m",string) == 0)    
    {
        system_flag_table->guji_record.by_distance_vaule   = 20; /*ms*/
        system_flag_table->guji_record.recoed_formats  = BY_DISTANCE;

    } 
    else if(strcmp("50m",string) == 0)    
    {
        system_flag_table->guji_record.by_distance_vaule   = 50; /*ms*/
        system_flag_table->guji_record.recoed_formats  = BY_DISTANCE;

    } 
    else if(strcmp("100m",string) == 0)    
    {
        system_flag_table->guji_record.by_distance_vaule   = 100; /*ms*/
        system_flag_table->guji_record.recoed_formats  = BY_DISTANCE;

    } 
    stm_write_eerpom(5,system_flag_table->guji_record.by_time_vaule);
    stm_write_eerpom(6,system_flag_table->guji_record.by_distance_vaule);
    stm_write_eerpom(7,system_flag_table->guji_record.recoed_formats);    


    string = GetIniKeyString("RECORD", "SpeedMask", "config.ini");
    //CSV, GPX, NMEA, KML

    if(strcmp("OFF",string) == 0)    
    {
       system_flag_table->guji_record.by_speed_vaule = 0;             
    }
    else
    {
       system_flag_table->guji_record.by_speed_vaule = GetIniKeyInt("RECORD", "SpeedMask", "config.ini");
       if(system_flag_table->guji_record.by_speed_vaule < 1 || system_flag_table->guji_record.by_speed_vaule>200)
        system_flag_table->guji_record.by_speed_vaule = 0;
    }
    stm_write_eerpom(8,system_flag_table->guji_record.by_speed_vaule);  

    system_flag_table->lowpower_timer = GetIniKeyInt("RECORD", "SpyModeTimer", "config.ini");
    if(system_flag_table->lowpower_timer < 5 || system_flag_table->lowpower_timer>60)
     system_flag_table->lowpower_timer = 15;

    stm_write_eerpom(9,system_flag_table->lowpower_timer);  


    /*AutoPower*/

    string = GetIniKeyString("RECORD", "OneTrackPerDay", "config.ini");

    if(strcmp("ON",string) == 0)
    {
       system_flag_table->ODOR = 1; 
    }
    else if(strcmp("OFF",string) == 0)    
    {
       system_flag_table->ODOR = 0;             
    } 
    stm_write_eerpom(10,system_flag_table->ODOR);

    stm_write_eerpom(0xff,0x12345677);
    f_close(update_config_fp);

    while(1)
    {
        if(flash_cnt != 3 )
        {           
            BSP_LED_Toggle(LED_GPS);
            BSP_LED_Toggle(LED_SD);
            BSP_LED_Toggle(LED_SURPORT); 
            HAL_Delay(300);
            flash_cnt++;
        }
        else
        {  
           flash_cnt = 0;
           break;
        }
            
    }


    BSP_LED_Off(LED_GPS);  
    BSP_LED_Off(LED_SD);  
    BSP_LED_Off(LED_SURPORT); 

}

void entry_config_mode(system_flag *system_flag_table)
{
    FIL update_config_fp;
    FRESULT fr;
    uint32_t eeprom_flag = 0;

    if(f_open(&update_config_fp,(TCHAR const*)"P-1.BIN",FA_READ) == FR_OK)
    {
        f_close(&update_config_fp);
        __set_FAULTMASK(1);      // 关闭所有中端
        HAL_NVIC_SystemReset();
    }
    else if(f_open(&update_config_fp,(TCHAR const*)"config.ini",FA_READ) == FR_OK)
    {
    
        configfs_set(&update_config_fp);
        print_usart1("\r\n read config.ini \r\n");
    }
    else
    {
        fr = f_open(&update_config_fp, "INFO.TXT",FA_WRITE | FA_OPEN_ALWAYS);
        f_printf(&update_config_fp,"%s\r\n",timer_zone_Aarry[system_flag_table->time_zone]);
        f_printf(&update_config_fp,"%s\r\n",format_Aarry[system_flag_table->gujiFormats]);
        f_printf(&update_config_fp,"%dHz\r\n",system_flag_table->guji_record.by_time_vaule);
        f_printf(&update_config_fp,"%d\r\n",system_flag_table->guji_record.by_speed_vaule);
        f_printf(&update_config_fp,"%d\r\n",system_flag_table->wanng_speed_vaule);
        f_printf(&update_config_fp,"%d\r\n",system_flag_table->lowpower_timer);
        f_printf(&update_config_fp,"%s\r\n", system_flag_table->auto_new_guji ? "ON" : "OFF");
        f_printf(&update_config_fp,"%s\r\n",system_flag_table->auto_power ? "ON" : "OFF");
        f_printf(&update_config_fp,"%s\r\n",system_flag_table->buzzer? "ON" : "OFF");
        f_printf(&update_config_fp,"Firmware: V0.01 \r\n");
        stm_read_eerpom(11,&eeprom_flag);
        f_printf(&update_config_fp,"PowerOn: %d\r\n",eeprom_flag);
        stm_read_eerpom(12,&eeprom_flag);
        if(eeprom_flag == 0xaaaaaaaa)
        {
            stm_read_eerpom(12,&eeprom_flag);
            f_printf(&update_config_fp,"First Use: %02d-%02d-%02d\r\n",(eeprom_flag>>16)&0xff,(eeprom_flag>>8)&0xff,(eeprom_flag)&0xff);
        }

        else
            f_printf(&update_config_fp,"First Use: ---\r\n");

        f_close(&update_config_fp);
        print_usart1("\r\n write info.txt \r\n");

    }

    


}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
