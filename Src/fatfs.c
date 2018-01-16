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
static uint8_t open_cnt = 0;
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
    //print_usart1("fattime :%x \r\n",fattime);

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

//    __disable_irq();

    /* Opens an existing file. If not exist, creates a new file. */
    fr = f_open(fp, path, FA_WRITE | FA_OPEN_ALWAYS);
    if (fr == FR_OK) {
        /* Seek to end of the file to append data */
        fr = f_lseek(fp, f_size(fp));
        if (fr != FR_OK)
            f_close(fp);
    }
	open_cnt ++;
	if((fr != FR_OK) &&(FR_EXIST != fr) && (open_cnt <10))
		open_append(fp,path);

    else
		open_cnt = 0;
//	__enable_irq();
    return fr;
}

FRESULT open_append_sp (
    FIL* fp,            /* [OUT] File object to create */
    const char* path    /* [IN]  File name to be opened */
)
{
    FRESULT fr;

    char sLine[20] ={0};

    /* Opens an existing file. If not exist, creates a new file. */
    fr = f_open(fp, path, FA_WRITE|FA_READ| FA_OPEN_ALWAYS);
    if (fr == FR_OK) {
        /* Seek to end of the file to append data */
        if( f_size(fp) > 27)
        {
            fr = f_lseek(fp, f_size(fp)-27);        
            f_gets(sLine, 20, fp);
            if (0 == strncmp("</trkseg>", sLine, 9)) 
            { // 长度依文件读取的为准
                 //print_usart1("%s sLine 1 \r\n",sLine);
#if 1                 
                 if(NULL != f_gets(sLine, 20, fp))
                 {
                     //print_usart1("%s sLine 2 \r\n",sLine);
                     if (0 == strncmp("</trk>", sLine, 6)) 
                     {
                        
                         if(NULL != f_gets(sLine, 20, fp))
                         {
                             //print_usart1("%s sLine 3 \r\n",sLine);
                             if (0 == strncmp("</gpx>", sLine, 6)) 
                             {
                               
                                    fr = f_lseek(fp, 0);
                                    fr = f_lseek(fp, f_size(fp)-27); 
                             }

                         }
                     }
                 }  
                 
#endif
            }
            else
            {
                fr = f_lseek(fp, 0);
                fr = f_lseek(fp, f_size(fp));
                //print_usart1("%s sLine 1 else \r\n",sLine);
            }
        }
        else
        {
             fr = f_lseek(fp, 0);
             fr = f_lseek(fp, f_size(fp)); 
             //print_usart1("default \r\n");
        }        
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

uint8_t configfs_set(FIL *update_config_fp)
{
    char *string = NULL;
    uint8_t flash_cnt = 0;
    uint8_t i = 0;
    uint8_t no_support_char = 0;

    BSP_LED_Init(LED_GPS);  
    BSP_LED_Init(LED_SD);  
    BSP_LED_Init(LED_SURPORT); 
    BSP_LED_Init(LED_RED); 
    
    print_usart1("%s\r\n", GetIniKeyString("SETTINGS", "TimeZone", "config.txt"));
    print_usart1("%s\r\n", GetIniKeyString("SETTINGS", "SpeedAlert", "config.txt")); 
    print_usart1("%s\r\n", GetIniKeyString("SETTINGS", "AutoPowerOn", "config.txt"));
    print_usart1("%s\r\n", GetIniKeyString("SETTINGS", "Beeper", "config.txt")); 
    print_usart1("%s\r\n", GetIniKeyString("SETTINGS", "FunctionKey", "config.txt")); 
    print_usart1("%s\r\n", GetIniKeyString("RECORD", "Format", "config.txt"));
    print_usart1("%s\r\n", GetIniKeyString("RECORD", "LogMode", "config.txt")); 
    print_usart1("%s\r\n", GetIniKeyString("RECORD", "SpeedMask", "config.txt"));
    print_usart1("%d\r\n", GetIniKeyInt("RECORD", "SpyModeTimer", "config.txt")); 
    print_usart1("%s\r\n", GetIniKeyString("RECORD", "OneTrackPerDay", "config.txt")); 
    print_usart1("%s\r\n", GetIniKeyString("Unit", "Speed", "config.txt")); 

    string = GetIniKeyString("Unit", "Speed", "config.txt");

    if(strcmp("km/h",string) == 0)
    {
       system_flag_table->unit = 0; 
    }
    else if(strcmp("mi/h",string) == 0)    
    {
       system_flag_table->unit = 1;             
    }
    else
    {
        system_flag_table->unit = 0; 
        no_support_char = 1;

    }
    
    stm_write_eerpom(12,system_flag_table->unit);

    string = GetIniKeyString("SETTINGS", "TimeZone", "config.txt");
    /*TimeZone*/
    i = 0;
    while((timer_zone_Aarry[i] != NULL) && (i < 42))
    { 

        if(strcmp(timer_zone_Aarry[i],string) == 0)
            break;
        
        i++;
    }

    if(i >= 42)
    {
        i = 16;
        no_support_char = 1;
    }

    system_flag_table->time_zone = i;
    stm_write_eerpom(0,system_flag_table->time_zone);
    /*Buzzer*/

    string = GetIniKeyString("SETTINGS", "Beeper", "config.txt");

    if(strcmp("ON",string) == 0)
    {
       system_flag_table->buzzer = 1; 
    }
    else if(strcmp("OFF",string) == 0)    
    {
       system_flag_table->buzzer = 0;             
    }
    else
    {
       system_flag_table->buzzer = 1;             
       no_support_char = 1;
    }
    
    stm_write_eerpom(1,system_flag_table->buzzer);

    /*function*/

    string = GetIniKeyString("SETTINGS", "FunctionKey", "config.txt");

    if(strcmp("Pause",string) == 0)
    {
       system_flag_table->function_index = 0; 
    }
    else if(strcmp("POI",string) == 0)    
    {
       system_flag_table->function_index = 1;             
    }
    else
    {
        system_flag_table->function_index = 0;             
        no_support_char = 1;

    }
    
    stm_write_eerpom(30,system_flag_table->function_index);

    /*SpeedWarning*/

    string = GetIniKeyString("SETTINGS", "SpeedAlert", "config.txt");

    if(strcmp("OFF",string) == 0)    
    {
       system_flag_table->wanng_speed_vaule = 0;             
    }
    else
    {
       system_flag_table->wanng_speed_vaule = GetIniKeyInt("SETTINGS", "SpeedAlert", "config.txt"); 
       if(system_flag_table->wanng_speed_vaule < 1 || system_flag_table->wanng_speed_vaule>200)
       {
          system_flag_table->wanng_speed_vaule = 0;
          no_support_char = 1;

       }
       else
       {
           if(system_flag_table->unit == 1)
           {
               system_flag_table->wanng_speed_vaule = (system_flag_table->wanng_speed_vaule*1609);   
           }
           else
           {
               system_flag_table->wanng_speed_vaule = (system_flag_table->wanng_speed_vaule*1000);                  
           }
       }

    }


    stm_write_eerpom(2,system_flag_table->wanng_speed_vaule);

    /*AutoPower*/

    string = GetIniKeyString("SETTINGS", "AutoPowerOn", "config.txt");

    if(strcmp("ON",string) == 0)
    {
       system_flag_table->auto_power = 1; 
    }
    else if(strcmp("OFF",string) == 0)    
    {
       system_flag_table->auto_power = 0;             
    } 
    else
    {
        system_flag_table->auto_power = 0;             
        no_support_char = 1;

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
    string = GetIniKeyString("RECORD", "Format", "config.txt");
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
    else
    {
       system_flag_table->gujiFormats = GUJI_FORMATS_CSV; 
       no_support_char = 1;
    }

    stm_write_eerpom(4,system_flag_table->gujiFormats);

    /*Mode*/
    /*
    1Hz, 5Hz, 10Hz, 5m, 10m, 20m, 50m, 100m

        */
    string = GetIniKeyString("RECORD", "LogMode", "config.txt");
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
    else if(strcmp("20ft",string) == 0)    
    {
        system_flag_table->guji_record.by_distance_vaule   = (20*0.3048); /*ms*/
        system_flag_table->guji_record.recoed_formats  = BY_DISTANCE;

    } 
    else if(strcmp("50ft",string) == 0)    
    {
        system_flag_table->guji_record.by_distance_vaule   = (50*0.3048); /*ms*/
        system_flag_table->guji_record.recoed_formats  = BY_DISTANCE;

    } 
    else if(strcmp("100ft",string) == 0)    
    {
        system_flag_table->guji_record.by_distance_vaule   = (100*0.3048); /*ms*/
        system_flag_table->guji_record.recoed_formats  = BY_DISTANCE;

    } 
    else if(strcmp("200ft",string) == 0)    
    {
        system_flag_table->guji_record.by_distance_vaule   = (200*0.3048); /*ms*/
        system_flag_table->guji_record.recoed_formats  = BY_DISTANCE;

    } 
    else if(strcmp("500ft",string) == 0)    
    {
        system_flag_table->guji_record.by_distance_vaule   = (500*0.3048); /*ms*/
        system_flag_table->guji_record.recoed_formats  = BY_DISTANCE;

    } 
    else
    {
        no_support_char = 1;
        system_flag_table->guji_record.by_time_vaule   = 1000; /*ms*/
        system_flag_table->guji_record.recoed_formats  = BY_TIMES;
    }

    
    stm_write_eerpom(5,system_flag_table->guji_record.by_time_vaule);
    stm_write_eerpom(6,system_flag_table->guji_record.by_distance_vaule);
    stm_write_eerpom(7,system_flag_table->guji_record.recoed_formats);    


    string = GetIniKeyString("RECORD", "SpeedMask", "config.txt");
    //CSV, GPX, NMEA, KML

    if(strcmp("OFF",string) == 0)    
    {
       system_flag_table->guji_record.by_speed_vaule = 0;             
    }
    else
    {
       system_flag_table->guji_record.by_speed_vaule = GetIniKeyInt("RECORD", "SpeedMask", "config.txt");
       if(system_flag_table->guji_record.by_speed_vaule < 1 || system_flag_table->guji_record.by_speed_vaule>200)
       {
            system_flag_table->guji_record.by_speed_vaule = 0;
            no_support_char = 1;

       }
       else
       {
           if(system_flag_table->unit == 1)
           {
               system_flag_table->guji_record.by_speed_vaule = (system_flag_table->guji_record.by_speed_vaule*1609);   
           }
           else
           {
               system_flag_table->guji_record.by_speed_vaule = (system_flag_table->guji_record.by_speed_vaule*1000);   
           }
       }
       
    }
    stm_write_eerpom(8,system_flag_table->guji_record.by_speed_vaule);  

    system_flag_table->lowpower_timer = GetIniKeyInt("RECORD", "SpyModeTimer", "config.txt");
    if(system_flag_table->lowpower_timer < 5 || system_flag_table->lowpower_timer>60)
    {
         system_flag_table->lowpower_timer = 15;
         no_support_char = 1;

    }

    stm_write_eerpom(9,system_flag_table->lowpower_timer);  


    /*AutoPower*/

    string = GetIniKeyString("RECORD", "OneTrackPerDay", "config.txt");

    if(strcmp("ON",string) == 0)
    {
       system_flag_table->ODOR = 1; 
    }
    else if(strcmp("OFF",string) == 0)    
    {
       system_flag_table->ODOR = 0;             
    } 
    else
    {
        system_flag_table->ODOR = 0;             
        no_support_char = 1;

    }
    
    stm_write_eerpom(10,system_flag_table->ODOR);

    stm_write_eerpom(0xff,0x12345677);
    f_close(update_config_fp);


          

    BSP_LED_On(LED_GPS);  
    BSP_LED_On(LED_SD);	
    BSP_LED_On(LED_SURPORT);


    HAL_Delay(300);

    BSP_LED_Off(LED_GPS);  
    BSP_LED_Off(LED_SD);  
    BSP_LED_Off(LED_SURPORT); 
 
    HAL_Delay(300);
    BSP_LED_On(LED_GPS);  
    BSP_LED_On(LED_SD);	
    BSP_LED_On(LED_SURPORT);

    HAL_Delay(300);
    BSP_LED_Off(LED_GPS);  
    BSP_LED_Off(LED_SD);  
    BSP_LED_Off(LED_SURPORT); 

    HAL_Delay(300);
    BSP_LED_On(LED_GPS);  
    BSP_LED_On(LED_SD); 
    BSP_LED_On(LED_SURPORT);
    
    HAL_Delay(300);
    BSP_LED_Off(LED_GPS);  
    BSP_LED_Off(LED_SD);  
    BSP_LED_Off(LED_SURPORT); 
    //print_usart1("\r\n no_support_char :%d \r\n",no_support_char); 
    if(no_support_char == 1)
    {
        for(i=0;i<10;i++)
        {
            if(i%2 == 0)
                sound_toggle_simple_Force(1,50,50);
            HAL_Delay(50);
            BSP_LED_On(LED_RED); 
            HAL_Delay(150);
            BSP_LED_Off(LED_RED); 
        }
    }
    return no_support_char;
}

uint8_t entry_config_mode(system_flag *system_flag_table)
{
    FIL update_config_fp;
    FRESULT fr;
    uint32_t eeprom_flag = 0;
    uint8_t ret  = 0; 

    if(f_open(&update_config_fp,(TCHAR const*)"P-1.BIN",FA_READ) == FR_OK)
    {
        f_close(&update_config_fp);
        __set_FAULTMASK(1);      // 关闭所有中端
        HAL_NVIC_SystemReset();
    }
    else if(f_open(&update_config_fp,(TCHAR const*)"config.txt",FA_READ) == FR_OK)
    {
        sound_toggle_simple(3,50,50);
        ret = configfs_set(&update_config_fp);
        print_usart1("\r\n read config.txt \r\n");
    }
    else
    {
        sound_toggle_simple(3,50,50);
        fr = f_open(&update_config_fp, "INFO.TXT",FA_WRITE | FA_OPEN_ALWAYS);
        f_printf(&update_config_fp,"[SETTINGS]\r\n");
        f_printf(&update_config_fp,"TimeZone=%s\r\n",timer_zone_Aarry[system_flag_table->time_zone]);
        if(system_flag_table->wanng_speed_vaule == 0)
        {
            f_printf(&update_config_fp,"SpeedAlert=OFF\r\n");
        }
        else
        {
            if(system_flag_table->unit == 1)
                f_printf(&update_config_fp,"SpeedAlert=%d\r\n",system_flag_table->wanng_speed_vaule/1609);
            else
                f_printf(&update_config_fp,"SpeedAlert=%d\r\n",system_flag_table->wanng_speed_vaule/1000);
        }
        f_printf(&update_config_fp,"AutoPowerOn=%s\r\n",system_flag_table->auto_power ? "ON" : "OFF");
        f_printf(&update_config_fp,"Beeper=%s\r\n",system_flag_table->buzzer? "ON" : "OFF");
        f_printf(&update_config_fp,"FunctionKey=%s\r\n",functionkey_Aarry[system_flag_table->function_index]);
        f_printf(&update_config_fp,"[RECORD]\r\n");
        f_printf(&update_config_fp,"Format=%s\r\n",format_Aarry[system_flag_table->gujiFormats]);
        if( system_flag_table->guji_record.recoed_formats  == BY_TIMES)
            f_printf(&update_config_fp,"LogMode=%dHz\r\n",1000/(system_flag_table->guji_record.by_time_vaule));
        else
        {
            if(system_flag_table->unit == 1)
                f_printf(&update_config_fp,"LogMode=%dft\r\n",(uint32_t)(system_flag_table->guji_record.by_distance_vaule/0.3048)); 
            else
                f_printf(&update_config_fp,"LogMode=%dm\r\n",(uint32_t)(system_flag_table->guji_record.by_distance_vaule)); 
        }
            
        if(system_flag_table->guji_record.by_speed_vaule == 0)
        {
            f_printf(&update_config_fp,"SpeedMask=OFF\r\n");    
        }
        else
        {
            if(system_flag_table->unit == 1)
                f_printf(&update_config_fp,"SpeedAlert=%d\r\n",system_flag_table->guji_record.by_speed_vaule/1609);
            else        
                f_printf(&update_config_fp,"SpeedMask=%d\r\n",system_flag_table->guji_record.by_speed_vaule/1000);
        }
        f_printf(&update_config_fp,"SpyModeTimer=%d\r\n",(system_flag_table->lowpower_timer/60000));
        f_printf(&update_config_fp,"OneTrackPerDay=%s\r\n", system_flag_table->auto_new_guji ? "ON" : "OFF");       
        f_printf(&update_config_fp,"[Unit]\r\n");

        if(system_flag_table->unit == 1)
        {
            f_printf(&update_config_fp,"Speed=mi/h\r\n");  
        }
        else
        {
            f_printf(&update_config_fp,"Speed=km/h\r\n");  
        }


        

        f_printf(&update_config_fp,"Firmware: V 0.9 \r\n");
        stm_read_eerpom(11,&eeprom_flag);
        f_printf(&update_config_fp,"PowerOn: %d\r\n",eeprom_flag);
        stm_read_eerpom(12,&eeprom_flag);

        stm_read_eerpom(13,&eeprom_flag);
        if(system_flag_table->frist_power == 0)
        {
            stm_read_eerpom(14,&eeprom_flag);
            f_printf(&update_config_fp,"First Use: %02d-%02d-%02d\r\n",(eeprom_flag>>16)&0xff,(eeprom_flag>>8)&0xff,(eeprom_flag)&0xff);
        }

        else
            f_printf(&update_config_fp,"First Use: ---\r\n");

        f_close(&update_config_fp);
        print_usart1("\r\n write info.txt \r\n");

    }


    return ret ;


}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
