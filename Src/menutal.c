#include "menutal.h"


#define PRESSURE 0
#define TEMPERATURE 0
#define ALTITUDE 0




typedef union { /* PMU_IRQ_CLR_RTC */
    uint8_t all;
    struct {
        uint8_t tag                      : 2;
        uint8_t ns                       : 1;
        uint8_t ew                       : 1;
        uint8_t                          : 4;
    } bitc;
} GUJI_TAG;

#if 0
typedef union { /* FPC_Reg6964_Dat6 */
    uint32_t all;
    struct {             
        uint32_t year                       : 6;
        uint32_t month                       : 4;
        uint32_t date                      : 5;
        uint32_t hour                      : 5;
        uint32_t min                     : 6;
        uint32_t sec                      : 6;
    } bitc;
} GUJI_DATE;
#else
typedef union { /* FPC_Reg6964_Dat6 */
    uint32_t all;
    struct {             
        uint32_t sec                      : 6;
        uint32_t min                      : 6;
        uint32_t hour                     : 5;
        uint32_t date                     : 5;      
        uint32_t month                    : 4;
        uint32_t year                     : 6;
    } bitc;
} GUJI_DATE;
#endif

typedef struct 
{	//公历日月年周
	u16 w_year;
	u8  w_month;
	u8  w_date;
	u8  week;		 
	u8  sec;		 
}tm;	



const u8 timer_zone_Aarry[][7]=
{
	"-12:00",   // 1
	"-11:00",   // 2
//	"-10:30",   // 3
	"-10:00",   // 4
	"-09:30",   //5
	"-09:00",
//	"-08:30",
	"-08:00",
	"-07:00",
	"-06:00",
	"-05:00",
	"-04:30",
	"-04:00",
	"-03:30",
	"-03:00",
	"-02:30",
	"-02:00",
	"-01:00",
//	"-00:44",
//	"-00:25",
	"+00:00",
//	"+00:20",
//	"+00:30",
	"+01:00",
	"+02:00",
	"+03:00",
	"+03:30",
	"+04:00",
	"+04:30",
//	"+04:51",
	"+05:00",
	"+05:30",
//	"+05:40",
	"+05:45",
	"+06:00",
	"+06:30",
	"+07:00",
//	"+07:20",
//	"+07:30",
	"+08:00",
//	"+08:30",
	"+08:45",
	"+09:00",
	"+09:30",
//	"+09:45",
	"+10:00",
	"+10:30",
	"+11:00",
	"+11:30",
	"+12:00",
	"+12:45",
	"+13:00",
	"+13:45",
	"+14:00",
};

//Monday Tuesday Wednesday Thursday Friday Saturday SunUay
const u8 week_word[7][3]={"SUN","MON","TUE","WED","THU","FRI","SAT"};



const uint8_t  table_week[12]={0,3,3,6,1,4,6,2,5,0,3,5}; //月修正数据表
//平年的月份日期表
const uint8_t mon_table[12]={31,28,31,30,31,30,31,31,30,31,30,31};

//获得现在是星期几
//功能描述:输入公历日期得到星期(只允许1901-2099年)
//输入参数：公历年月日
//返回值：星期号
uint8_t RTC_Get_Week(uint16_t year,uint8_t month,uint8_t day)
{
	uint16_t temp2;
	uint8_t yearH,yearL;

	yearH = year/100;	
    yearL = year%100;
	// 如果为21世纪,年份数加100
	if (yearH>19)yearL += 100;
	// 所过闰年数只算1900年之后的
	temp2 = yearL+yearL/4;
	temp2 = temp2%7;
	temp2 = temp2 + day + table_week[month-1];
	if ((yearL%4 == 0)&&(month < 3))temp2--;
	return(temp2%7);
}


//判断是否是闰年函数
//月份   1  2  3  4  5  6  7  8  9  10 11 12
//闰年   31 29 31 30 31 30 31 31 30 31 30 31
//非闰年 31 28 31 30 31 30 31 31 30 31 30 31
//输入:年份
//输出:该年份是不是闰年.1,是.0,不是
u8 Is_Leap_Year(u16 year)
{
	if(year%4==0) //必须能被4整除
	{
		if(year%100==0)
		{
			if(year%400==0)return 1;//如果以00结尾,还要能被400整除
			else return 0;
		}else return 1;
	}else return 0;
}




//得到当前的时间
//返回值:0,成功;其他:错误代码.
u8 RTC_Get(u8 flag,tm *my_timer)
{
//	extern tm my_timer;
	u16 temp1=0;
	if(flag)
	{
		my_timer->w_date +=1;  //得到日期
		if(Is_Leap_Year(my_timer->w_year)&&(my_timer->w_month == 1))
		{
			temp1 = 29;
		}
		else
		{
			temp1 = mon_table[my_timer->w_month - 1];
	    }

		if(my_timer->w_date>temp1 )
	    {
			my_timer->w_date=1;//平年
			my_timer->w_month++;//得到月份
			if(my_timer->w_month>12)
				{
				my_timer->w_month = 1;
				my_timer->w_year++;
				}
	    }

	}
    else
	{
	    my_timer->w_date -=1;  //得到日期
	    if(my_timer->w_date == 0)
		{
	    	my_timer->w_month--;//得到月份

		    if(Is_Leap_Year(my_timer->w_year)&&my_timer->w_month==1)
			{
			    temp1 = 29;
			}
		    else
			    temp1 = mon_table[my_timer->w_month - 1];


		    my_timer->w_date= temp1;//平年

		    if(my_timer->w_month == 0)
			{
			    my_timer->w_month = 12;
			    my_timer->w_year--;
			}
		}

    }

	my_timer->week = RTC_Get_Week(my_timer->w_year,my_timer->w_month,my_timer->w_date);//获取星期
	return 0;
}


void  rtc_set(system_flag *system_flag_table,u16 syear,u8 smon,u8 sday,u8 hour,u8 min,u8 sec,u8 week)
{

    RTC_DateTypeDef RTC_DateStructure;
    RTC_TimeTypeDef RTC_TimeStructure;
    extern RTC_HandleTypeDef hrtc;

    RTC_DateStructure = system_flag_table->RTC_DateStructure;
    RTC_DateStructure = system_flag_table->RTC_TimeStructure;
    

	print_usart1("rtc_set %d-%d-%d  %d:%d:%d %d\n",syear ,smon,sday,hour,min,sec,week);
	RTC_TimeStructure.RTC_Hours =hour;
	RTC_TimeStructure.RTC_Minutes = min;
	RTC_TimeStructure.RTC_Seconds = sec;



	RTC_DateStructure.RTC_WeekDay =  RTC_Get_Week(syear+2000, smon, sday);
	RTC_DateStructure.RTC_Date = sday;
	RTC_DateStructure.RTC_Month = smon;
	RTC_DateStructure.RTC_Year = syear;
    
    if(HAL_RTC_SetDate(&hrtc,&RTC_DateStructure,RTC_FORMAT_BCD) != HAL_OK)
    {
        /* Initialization Error */
        print_usart1("Error_Handler()\r\n"); 
    } 
    
    if(HAL_RTC_SetTime(&hrtc,&RTC_TimeStructure,RTC_FORMAT_BCD) != HAL_OK)
    {
        /* Initialization Error */
        print_usart1("Error_Handler()\r\n"); 
    }  

}



void check_time(nmea_msg *gpsx ,system_flag *system_flag_table,float time_zone)
{
    uint16_t gpsx_utc;
    uint16_t hour_timer,min_timer;
    tm *my_timer = NULL;
    int16_t diff_tone = 0.0;
    uint32_t setting_tp = 0;

    RTC_DateTypeDef RTC_DateStructure;
    RTC_TimeTypeDef RTC_TimeStructure;

    RTC_DateStructure = system_flag_table->RTC_DateStructure;
    RTC_DateStructure = system_flag_table->RTC_TimeStructure;

	print_usart1("check_time\r\n");
    my_timer = system_flag_table->sys_tm;
    if((system_flag_table->run_mode == 1)&&(gpsx->svnum >= 2))
    {
        if((gpsx->utc.year < 2014)||(gpsx->utc.year == 2080))
        {
            return;
        }
        my_timer->w_year = gpsx->utc.year -2000;
        my_timer->w_month =gpsx->utc.month;
        my_timer->w_date = gpsx->utc.date;
        my_timer->week = RTC_Get_Week(gpsx->utc.year ,my_timer->w_month,my_timer->w_date);
        gpsx_utc = (gpsx->utc.hour*60+gpsx->utc.min);
        my_timer->sec = gpsx->utc.sec;

	    if(timer_zone_Aarry[system_flag_table->time_zone][0] == '+')
		{
    		hour_timer = ((timer_zone_Aarry[system_flag_table->time_zone][1]-'0')*10)+(timer_zone_Aarry[system_flag_table->time_zone][2]-'0');
    		min_timer = ((timer_zone_Aarry[system_flag_table->time_zone][4]-'0')*10)+(timer_zone_Aarry[system_flag_table->time_zone][5]-'0');
    		print_usart1("\n\r hour_timer : %d   min_timer :%d  ",hour_timer,min_timer);
    		if((gpsx_utc +(hour_timer*60+min_timer))>=(24*60))
    	    {
                RTC_Get(1,&my_timer);
                gpsx_utc  = (gpsx_utc + (hour_timer*60 + min_timer) - (24*60));
    
                hour_timer = gpsx_utc/60;
                min_timer  = gpsx_utc - hour_timer*60;
    		}
    		else
    		{
                gpsx_utc  = (gpsx_utc + (hour_timer*60 + min_timer));
    
                hour_timer = gpsx_utc /60;
                min_timer  = gpsx_utc - hour_timer*60;
    
    		}

		}
	    else if(timer_zone_Aarry[system_flag_table->time_zone][0] == '-')
		{
    		hour_timer = ((timer_zone_Aarry[system_flag_table->time_zone][1]-'0')*10)+(timer_zone_Aarry[system_flag_table->time_zone][2]-'0');
    		min_timer = ((timer_zone_Aarry[system_flag_table->time_zone][4]-'0')*10)+(timer_zone_Aarry[system_flag_table->time_zone][5]-'0');
			print_usart1("\n\r hour_timer : %d   min_timer :%d  ",hour_timer,min_timer);
    		if(gpsx_utc > (hour_timer*60+min_timer))
    		{
                gpsx_utc  = (gpsx_utc - (hour_timer*60+min_timer));
            
                hour_timer = gpsx_utc/60;
                min_timer  = gpsx_utc - hour_timer*60;
    		}
    		else
    		{
                gpsx_utc  = (gpsx_utc+(24*60) - (hour_timer*60+min_timer));
    
                hour_timer = gpsx_utc/60;
                min_timer  = gpsx_utc - hour_timer*60;
                RTC_Get(0,&my_timer);
    		}
		 }

	 }
	 else
     {
         my_timer->w_year = RTC_DateStructure.Year;
         my_timer->w_month =RTC_DateStructure.Month;
         my_timer->w_date = RTC_DateStructure.Date;
         my_timer->week = RTC_DateStructure.WeekDay;
         gpsx_utc = (RTC_TimeStructure.Hours *60+RTC_TimeStructure.Minutes);
         my_timer->sec = RTC_TimeStructure.Seconds ;
         diff_tone = gpsx_utc;
		 print_usart1("time_zone:%.1f \r\n",time_zone);
		 print_usart1("index_timerzone:%.1f \r\n",system_flag_table->index_timerzone);
         diff_tone +=(int16_t)((time_zone - system_flag_table->index_timerzone)*60);
 
         if(diff_tone >= (24*60))
         {
             RTC_Get(1,&my_timer);
             diff_tone = diff_tone - (24*60);
         }
         if(diff_tone < 0)
         {
             RTC_Get(0,&my_timer);
             diff_tone = diff_tone + (24*60);
         }
         hour_timer = diff_tone/60;
         min_timer  = diff_tone - hour_timer*60;
     }

	rtc_set(my_timer->w_year,my_timer->w_month,my_timer->w_date,hour_timer,min_timer,my_timer->sec,my_timer->week);

	system_flag_table->index_timerzone = time_zone;
}



/*get free size of flash !!!*/
uint8_t get_space(void)
{

    FATFS *fs;
    DWORD fre_clust, fre_sect, tot_sect;
    FRESULT res = FR_OK;
    float tp;
	
    /* Get volume information and free clusters of drive 1 */
    res = f_getfree("", &fre_clust, &fs);
    if (res) 
    {
        print_usart1("f_getfree faild :%d\r\n",res);
        return 0;
    }
    else
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
        return (uint8_t)tp;

    }

    return (uint8_t)tp;      
}



uint8_t save_guiji_message(nmea_msg *gpsx ,system_flag *system_flag_table,uint8_t guji_record_type,
                              RTC_DateTypeDef RTC_DateStructure,RTC_TimeTypeDef RTC_TimeStructure)
{
    uint8_t one_shot_buffer[28];
    uint8_t index = 0;
    GUJI_TAG flag ;
    GUJI_DATE guji_data ;
    uint8_t error = 0;

//	one_shot_buffer[index++] = (uint8_t)(Message_head_number>>24)&0xff;  // 1mb
	one_shot_buffer[index++] = (uint8_t)(( system_flag_table->Message_head_number +1)>>16)&0xff;  // 1mb
	one_shot_buffer[index++] = (uint8_t)(( system_flag_table->Message_head_number +1)>>8)&0xff;  // 2mb
	one_shot_buffer[index++] = (uint8_t)(( system_flag_table->Message_head_number +1))&0xff;  // 3 mb

    flag.all = 0;
    guji_data.all = 0;

    if(guji_record_type == 'T')
    {
       flag.bitc.tag = 0;
    }
    else if(guji_record_type == 'C')
    {
       flag.bitc.tag = 1;  
    }

    if(gpsx->nshemi == 'N')
    {
       flag.bitc.ns = 0;    
    }
    else if(gpsx->nshemi == 'S')
    {
       flag.bitc.ns = 1;     
    }
    else
    {
       error = 1;
    }


    if(gpsx->ewhemi== 'E')
    {
       flag.bitc.ew = 0;    
    }
    else if(gpsx->ewhemi== 'W')
    {
       flag.bitc.ew = 1;    
    }
    else
    {
       error = 1;
    }


    if(error != 0)
        return error;
    
    one_shot_buffer[index++]  = flag.all;  // 4mb

	if(system_flag_table->gujiFormats == GUJI_FORMATS_GPX)
	{
		guji_data.bitc.year  = gpsx->utc.year-2016;
		guji_data.bitc.month = gpsx->utc.month;
		guji_data.bitc.date  = gpsx->utc.date;
		guji_data.bitc.hour  = gpsx->utc.hour;
		guji_data.bitc.min   =  gpsx->utc.min;
		guji_data.bitc.sec   =  gpsx->utc.sec;
	}
	else
    {
        if(gpsx->utc.sec != RTC_TimeStructure.Seconds)
        {
            check_time();
        }
        
		guji_data.bitc.year  = (RTC_DateStructure.Year-16);
		guji_data.bitc.month = RTC_DateStructure.Month;
		guji_data.bitc.date  = RTC_DateStructure.Date;
		guji_data.bitc.hour  = RTC_TimeStructure.Hours;
		guji_data.bitc.min =   RTC_TimeStructure.Minutes;
		guji_data.bitc.sec =   RTC_TimeStructure.Seconds;
	}
   // print_usart1("date :%x \r\n",guji_data.all);

  //  memcpy(one_shot_buffer+index,&guji_data.all,4);
 //   index = index+4;
	one_shot_buffer[index++]  = (uint8_t)((guji_data.all>>24)&0xff);  // 11mb
	one_shot_buffer[index++]  = (uint8_t)((guji_data.all>>16)&0xff);  // 12mb
	one_shot_buffer[index++]  = (uint8_t)((guji_data.all>>8)&0xff);  // 13mb
	one_shot_buffer[index++]  = (uint8_t)((guji_data.all)&0xff);  // 14mb
	
	one_shot_buffer[index++]  = (uint8_t)((gpsx->latitude>>24)&0xff);  // 11mb
	one_shot_buffer[index++]  = (uint8_t)((gpsx->latitude>>16)&0xff);  // 12mb
	one_shot_buffer[index++]  = (uint8_t)((gpsx->latitude>>8)&0xff);  // 13mb
	one_shot_buffer[index++]  = (uint8_t)((gpsx->latitude)&0xff);  // 14mb

	one_shot_buffer[index++]  = (uint8_t)((gpsx->longitude>>24)&0xff);  // 15mb
	one_shot_buffer[index++]  = (uint8_t)((gpsx->longitude>>16)&0xff);  // 16mb
	one_shot_buffer[index++]  = (uint8_t)((gpsx->longitude>>8)&0xff);  // 17mb
	one_shot_buffer[index++]  = (uint8_t)((gpsx->longitude)&0xff);  // 18mb

	one_shot_buffer[index++]  = (uint8_t)((ALTITUDE>>24)&0xff);  // 15mb
	one_shot_buffer[index++]  = (uint8_t)((ALTITUDE>>16)&0xff);  // 16mb
	one_shot_buffer[index++]  = (uint8_t)((ALTITUDE>>8)&0xff);  // 21mb
	one_shot_buffer[index++]  = (uint8_t)((ALTITUDE)&0xff);  // 22mb

	one_shot_buffer[index++]  = (uint8_t)(((gpsx->speed/100)>>8)&0xff);  // 23mb
	one_shot_buffer[index++]  = (uint8_t)(((gpsx->speed/100))&0xff);  // 24mb

	one_shot_buffer[index++]  = (uint8_t)(((gpsx->angle/1000)>>8)&0xff);  // 25mb
	one_shot_buffer[index++]  = (uint8_t)((gpsx->angle/1000)&0xff);  // 26mb


	one_shot_buffer[index++]  = (uint8_t)(PRESSURE>>8)&0xff;  // 29mb
	one_shot_buffer[index++]  = (uint8_t)(PRESSURE)&0xff;  // 30mb


	one_shot_buffer[index++]  = (uint8_t)(TEMPERATURE>>8)&0xff;  // 27mb
	one_shot_buffer[index++]  = (uint8_t)(TEMPERATURE)&0xff;  // 28mb

	

	memcpy(system_flag_table->guji_buffer[system_flag_table->guji_buffer_Index],one_shot_buffer,28);

    system_flag_table->guji_buffer_Index  += 28;
    system_flag_table->Message_head_number++;

    return error;

}


void buffer_Analysis(FIL *sys_fp ,system_flag *system_flag_table,uint16_t munber)
{
    uint16_t index = 0,angle = 0,i = 0;
    uint8_t lat_flag,lon_flag,record_type;
    uint32_t message_number_index;
    int32_t attiautl;
    int16_t tsemp;
    float tp_lat =0.0,tp_lon =0.0,per=0.0,speed =0.0;
    GUJI_TAG flag ;
    GUJI_DATE guji_data ;
    FRESULT sys_fr ;
    uint8_t *buffer;
    __align(4) uint8_t dtbuf[50];                                //打印缓存器

    buffer = system_flag_table->guji_buffer;

    do
    {
        message_number_index = (buffer[2 + index]|(buffer[1 + index]<<8)|(buffer[0 + index]<<16));
        flag.all = buffer[3 + index];
       // memcpy(&guji_data.all,buffer+index+4,4);
        guji_data.all = (buffer[7 + index]|(buffer[6 + index]<<8)|(buffer[5 + index]<<16)|(buffer[4 + index]<<24));
        tp_lat= (buffer[11 + index]|(buffer[10 + index]<<8)|(buffer[9 + index]<<16)|(buffer[8 + index]<<24));	    	

        tp_lon = (buffer[15 + index]|(buffer[14 + index]<<8)|(buffer[13 + index]<<16)|(buffer[12 + index]<<24));	    	

        attiautl = (buffer[19 + index]|(buffer[18 + index]<<8)|(buffer[17+index]<<16)|(buffer[16+index]<<24));

        speed = (buffer[21 + index]|(buffer[20 + index]<<8));

        angle=  (buffer[23 + index]|(buffer[22 + index]<<8));

        per =  (buffer[25 + index]|(buffer[24 + index]<<8));		

        temp =   (buffer[27 + index]|(buffer[26+ index]<<8));

        index = index+28;


        record_type = flag.bitc.tag ? 'C':'T';
        lat_flag = flag.bitc.ns ? 'S':'N';
        lon_flag = flag.bitc.ew ? 'W':'E';

        if(angle >360)
             angle = 0;
        
        if(system_flag_table->gujiFormats == GUJI_FORMATS_GPX)
        {

            /*
            <trkpt lat="26.098903" lon="119.269088"><ele>149</ele>
            <time>2014-07-22T01:49:28Z</time></trkpt>
            */  
			if(lat_flag == 'S')
		    {
				sprintf((char *)dtbuf,"<trkpt lat=\"-%.6f\"",tp_lat/1000000);	

		    
		    }
			else 
		    {
				sprintf((char *)dtbuf,"<trkpt lat=\"%.6f\"",tp_lat/1000000);	

		    }


            f_printf(sys_fp,"%s",(char *)dtbuf);
			if(lon_flag == 'W')
		    {
				sprintf((char *)dtbuf," lon=\"-%.6f\">",tp_lon/1000000);	
		    
		    }
			else 
		    {
		        sprintf((char *)dtbuf," lon=\"%.6f\">",tp_lon/1000000);	
		    }
		    f_printf(sys_fp,"%s",(char *)dtbuf);
            sprintf((char *)dtbuf,"<ele>%d</ele>",attiautl/10);	
            f_printf(sys_fp,"%s\n",(char *)dtbuf);
            sprintf((char *)dtbuf,"<time>%04d-%02d-%02dT%02d:%02d:%02dZ</time></trkpt>",
               guji_data.bitc.year+2016,guji_data.bitc.month,guji_data.bitc.date,
               guji_data.bitc.hour,guji_data.bitc.min,guji_data.bitc.sec);   
            f_printf(sys_fp,"%s\n",(char *)dtbuf);
        }
        else
        {
            if(message_number_index == 1)
            {
                if(FR_OK  == sys_fr)
                {
                    f_printf(sys_fp,"INDEX,TAG,DATE,TIME,LATITUDE N/S,LONGITUDE E/W,HEIGHT,SPEED,HEADING,PRES,TEMP\n");
                }
            }
            
            if(FR_OK  == sys_fr)
            {
                sprintf((char *)dtbuf,"%d,%c,%02d%02d%02d,",message_number_index,record_type,
                    guji_data.bitc.year+16,guji_data.bitc.month,guji_data.bitc.date);
                
                f_printf(sys_fp,"%s",(char *)dtbuf);
                sprintf((char *)dtbuf,"%02d%02d%02d,%.6f%c,",guji_data.bitc.
                guji_data.bitc.min,guji_data.bitc.sec,tp_lat/1000000,lat_flag);
        
                f_printf(sys_fp,"%s",(char *)dtbuf);
                sprintf((char *)dtbuf,"%.6f%c,%d,%.1f,%d,%.1f,%d",tp_lon/1000000,lon_flag,attiautl/10,(speed/10),angle,per/10,(int)(temp/10));	
                f_printf(sys_fp,"%s \n",(char *)dtbuf);
    
            }
        }
        //print_usart1("\r\n index :%d  munber= %d\r\n ",index,munber);
    }while (index <munber);	
}



void gpx_filehead_write(FIL* fp,char* filename)
{
 /*
    <?xml version="1.0" encoding="UTF-8"?>
    <gpx
    version="1.1"
    creator="Columbus GPS - http://cbgps.com/"
    xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
    xmlns="http://www.topografix.com/GPX/1/1"
    xsi:schemaLocation="http://www.topografix.com/GPX/1/1 http://www.
topografix.com/GPX/1/1/gpx.xsd">
    <trk>
    <name>21174928.gpx</name>
    <trkseg>
 
 */
    f_printf(fp,"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    f_printf(fp,"<gpx\n");
    f_printf(fp,"version=\"1.1\"\n");
    f_printf(fp,"creator=\"Columbus GPS - http://cbgps.com/\"\n");
    f_printf(fp,"xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"\n");
    f_printf(fp,"xmlns=\"http://www.topografix.com/GPX/1/1\"\n");
    f_printf(fp,"xsi:schemaLocation=\"http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd\">\n");
    f_printf(fp,"<trk>\n");
    f_printf(fp,"<name>%s</name>\n",filename);
    f_printf(fp,"<trkseg>\n");

}

void write_flash(FIL *sys_fp,system_flag *system_flag_table)  /*write to  the file by true*/
{
    int wb = 0;
    
    if(system_flag_table->guji_buffer_Index)
    {
        if(get_space() < 1)
        {
            if(system_flag_table->guji_record.recoed_meth == 0)
            {
      
                Del_oldfile_nospace();
                
            }
            else
            {
                system_flag_table->guji_mode = RECORED_STOP;
				system_flag_table->guji_buffer_Index = 0;
            }

            return;
        }

        if(system_flag_table->gujiFormats == GUJI_FORMATS_CSV)
        {
            buffer_Analysis(sys_fp,system_flag_table);
        }
        else if(system_flag_table->gujiFormats == GUJI_FORMATS_GPS)
        {

            f_write(sys_fp,system_flag_table->guji_buffer,system_flag_table->guji_buffer_Index,&wb);

        }
        else if(system_flag_table->gujiFormats == GUJI_FORMATS_GPX)
        {
            //todo:add gpx file conote
            buffer_Analysis(system_flag_table->guji_buffer,system_flag_table);
        }
    
        system_flag_table->guji_buffer_Index = 0;
    }
}


FRESULT scan_directory_oldfile (
    char* path,      /* Working buffer filled with start directory */
    char* old_path,      /* Working buffer filled with start directory */
    unsigned long* date_time,
    char mode,
    int *filenumer

)
{
    UINT i, j;
    FRESULT fr;
    DIR dir;
    FILINFO fno;
#if _USE_LFN
    fno.lfname = 0; /* Disable LFN output */
#endif
    fr = f_opendir(&dir, path);
    if (fr == FR_OK) 
    {
        for (i = 0; path[i]; i++) ;
        path[i++] = '/';
        for (;;) 
        {
            fr = f_readdir(&dir, &fno);
            if (fr != FR_OK || !fno.fname[0]) break;
            if (_FS_RPATH && fno.fname[0] == '.') continue;
            j = 0;
            do
                path[i+j] = fno.fname[j];
            while (fno.fname[j++]);
            if (fno.fattrib & AM_DIR) 
            {   
                print_usart1("fno.name :%s\r\n",fno.fname);
                
                if((strcmp(fno.fname,"POI") != 0)&&(strcmp(fno.fname,"SYSTEM~1") != 0))
                {            
                    fr = scan_directory_oldfile(path,old_path,date_time,mode,filenumer);
                    if (fr != FR_OK) break;
                
                
                    if(mode == 1)
                    {
                        j = 0;
                        do
                           old_path[j] = path[j];
                        while (path[j++]);
                        fr = f_unlink(path);
                        if (fr != FR_OK) break;                      
                    }
                               
                }

            }
            else
            {
                *filenumer +=1;
                if((*date_time > ((fno.fdate <<16 )|fno.ftime))&&(mode == 0))
                {
                    *date_time = ((fno.fdate <<16 )|fno.ftime);
                    j = 0;
                    do
                        old_path[j] = path[j];
                    while (path[j++]);
                   
                    print_usart1("name :%s, data :%x, timer:%x  j:%d\r\n",path,fno.fdate,fno.ftime,*filenumer);
                }
                
                if(mode == 1)
                {
                    j = 0;
                    do
                        old_path[j] = path[j];
                     while (path[j++]);
                    fr = f_unlink(path);
                    if (fr != FR_OK) break;     
                     
                }                
                
            }

        }
        path[--i] = '\0';
        f_closedir(&dir);
    }
    

    return fr;
}


void Del_oldfile_nospace(system_flag *system_flag_table)
{
    FRESULT fr = FR_OK;
    unsigned long data_time = 0xffffffff;
    char buff[64] = {0};    /* Working buffer */
    char out_buff[64] = {0};    /* Working buffer */
    int filenumber = 0;
	extern uint16_t  Flash_write_buffer_Index ;
    
    strcpy(buff, "/");  /* Directory to be emptied */

    get_space();

    fr = scan_directory_oldfile(buff,out_buff,&data_time,0,&filenumber);
     
    if (fr) 
    {
        print_usart1("Function failed. (%u) \r\n", fr );
        return ;
    } 
    else 
    {
        print_usart1("found oldest file :%s.%d\r\n", out_buff,filenumber);
        if((data_time != 0xffffffff)&&(filenumber > 1))
        {
            fr = f_unlink(out_buff);
            return ;
        }
        else if (filenumber <= 1)
        {
             
			 system_flag_table->guji_mode = RECORED_STOP;
			 system_flag_table->guji_buffer_Index = 0;
             
        }
    }
}


void Del_allgujifile(void)
{
    FRESULT fr = FR_OK;
    unsigned long data_time = 0xffffffff;
    char buff[64] = {0};    /* Working buffer */
    char out_buff[64] = {0};    /* Working buffer */
    int filenumber = 0;
    
    strcpy(buff, "/");  /* Directory to be emptied */
    
    fr = scan_directory_oldfile(buff,out_buff,&data_time,1,&filenumber);

    if (fr) 
    {
        print_usart1("Function failed. (%u)\r\n", fr);
        return ;
    } 
    else 
    {
        print_usart1("found oldest file :%s. %d\r\n", out_buff,filenumber);

    }
}


void Recording_guji(FIL *sys_fp,system_flag *system_flag_table,nmea_msg *gpsx)
{
	uint32_t interst_pos_number;
	DIR* dp = NULL;

    char   track_file[26] ={0};
    FRESULT fr;
    FRESULT sys_fr;

    UINT wb;
    RTC_DateTypeDef RTC_DateStructure;
    RTC_TimeTypeDef RTC_TimeStructure;

    RTC_DateStructure = system_flag_table->RTC_DateStructure;
    RTC_DateStructure = system_flag_table->RTC_TimeStructure;

	switch(system_flag_table->guji_mode )
    {
		case RECORED_IDLE:

			break;
		case RECORED_START:
			system_flag_table->Message_head_number = 0;
			if((gpsx->fixmode >= 2)&&(gpsx->latitude >0)&&(gpsx->longitude>0))
			{
				system_flag_table->guji_buffer_Index = 0;
                check_time(gpsx,system_flag_table,8.0);  
                //print_usart1("w_year :%d \r\n",my_timer->w_year);
                if(get_space() == 0)
                {

				    if(system_flag_table->guji_record.recoed_meth == 1)
                    {
					    system_flag_table->guji_mode = RECORED_IDLE;
					    return ;
				    }
					else
				    {
				        Del_oldfile_nospace();			
						if(system_flag_table->guji_mode == RECORED_STOP)
					    {
						    system_flag_table->guji_mode = RECORED_IDLE;
						    return ;
					    }
				    }					 
                }
	
				
                if(RTC_DateStructure.Year  >= 15)
                {
                    sprintf(track_file,"%04d-%02d",RTC_DateStructure.Year + 2000,RTC_DateStructure.Month); 
                    print_usart1("track_file :%s \r\n",track_file);
                    fr= f_opendir(dp,track_file);
                    if((FR_OK  == fr) && (FR_EXIST == fr)) 
                    {
                        print_usart1("track file dir exist %d\r\n",fr);
                        //return;
                    }
					else  
				    {
				        fr = f_mkdir(track_file);
				    }
                    
                    print_usart1("track file dir fr %d\r\n",fr);
                    
                    if(system_flag_table->guji_record.recoed_formats == GUJI_FORMATS_CSV)
                    {
                        system_flag_table->gujiFormats = GUJI_FORMATS_CSV;
                        sprintf(track_file,"%04d-%02d/%02d%02d%02d%02d.CSV",RTC_DateStructure.Year+2000,RTC_DateStructure.Month,RTC_DateStructure.Date,
                        RTC_TimeStructure.Hours,RTC_TimeStructure.Minutes,RTC_TimeStructure.Seconds);
                        
                    }
                    else if(system_flag_table->guji_record.recoed_formats == GUJI_FORMATS_GPS)  
                    {
                        system_flag_table->gujiFormats = GUJI_FORMATS_GPS;
                        sprintf(track_file,"%04d-%02d/%02d%02d%02d%02d.GPS",RTC_DateStructure.Year+2000,
                            RTC_DateStructure.Month,RTC_DateStructure.Date,
                            RTC_TimeStructure.Hours,RTC_TimeStructure.Minutes,RTC_TimeStructure.Seconds);
                    }
                    else if(system_flag_table->guji_record.recoed_formats == GUJI_FORMATS_GPX)  
                    {
                        system_flag_table->gujiFormats = GUJI_FORMATS_GPX;
                        sprintf(track_file,"%04d-%02d/%02d%02d%02d%02d.GPX",RTC_DateStructure.Year+2000,RTC_DateStructure.Month,
                            RTC_DateStructure.Date,
                            RTC_TimeStructure.Hours,RTC_TimeStructure.Minutes,RTC_TimeStructure.Seconds);
                    }                    
                    print_usart1("\r\n track_file :%s \r\n ",track_file);
                    sys_fr = open_append(sys_fp, track_file);
                    
                    if(FR_OK  != sys_fr)
                    {
                        print_usart1("open append faild \n");
                        return; 
                    }
                    if(system_flag_table->guji_record.recoed_formats == GUJI_FORMATS_GPS) 
                    {
                        track_file[0] = 0x07;
                        track_file[1] = 0x07;
                        f_write(sys_fp,track_file,2,&wb);
                    }
                    else if(system_flag_table->guji_record.recoed_formats == GUJI_FORMATS_GPX)
                    {
                        //todo: add gpx file head.
                        gpx_filehead_write(track_file);
                    }
                    
        			save_guiji_message('T');
        		
        			system_flag_table->guji_mode = RECORED_START_DOING;
        			interst_pos_number = 0;
        			//	sprintf((char *)dtbuf,"%d",interst_pos_number);		    		
                    //得到速度字符串
        			//	OLED_ShowString(17,32,dtbuf);
        			//stm_write_eerpom(GUJI_INTERST_POS,interst_pos_number);
               }
               else
                   system_flag_table->recored_run_flag = 1;
			}
			else
			{
			    system_flag_table->recored_run_flag = 1;
			}

			break;
		case RECORED_START_DOING:
			if((gpsx->fixmode >= 2)&&(gpsx->latitude >0)&&(gpsx->longitude>0))
			{
				save_guiji_message('T');
			}
			else
			{
			}
			break;
		case RECORED_T:
			save_guiji_message('C');
			//stm_read_eerpom(GUJI_INTERST_POS ,&interst_pos_number);
			//if(interst_pos_number>100)
			//	interst_pos_number = 0;
			interst_pos_number++;
    		//sprintf((char *)dtbuf,"%02d",interst_pos_number);		    		//得到速度字符串
    		//OLED_ShowString(18,32,dtbuf);
			//stm_write_eerpom(GUJI_INTERST_POS,interst_pos_number);
			system_flag_table->guji_mode = RECORED_START_DOING;
			break;
		case RECORED_STOP:

			if(system_flag_table->baifenbi > 0)
		    {
              	if((gpsx->fixmode >= 2)&&(gpsx->latitude >0)&&(gpsx->longitude>0))
              	    save_guiji_message('T');              
              	write_flash();              
              	//stm_write_eerpom(CUURENT_FLASH_ADDRER ,Flash_Index);
                print_usart1("\r\n close file \r\n ");
			}
			
			system_flag_table->guji_mode = RECORED_IDLE;
            if(FR_OK == sys_fr)
            {
                if(system_flag_table->gujiFormats == GUJI_FORMATS_GPX)
                {
                    f_printf(sys_fp,"</trkseg>\n");
                    f_printf(sys_fp,"</trk>\n");
                    f_printf(sys_fp,"</gpx>\n");

                }
                f_close(&sys_fp);
            }

			break;

	}
}

