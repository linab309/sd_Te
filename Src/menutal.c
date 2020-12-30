 /* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l1xx_hal.h"
#include "cmsis_os.h"
#include "fatfs.h"

/* USER CODE BEGIN Includes */
#include "gps.h"
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

#include "menutal.h"
#include "stm32l1xx_nucleo.h"

#include "stm_eeprom.h"

#define PRESSURE 0
#define TEMPERATURE 0
#define ALTITUDE 0

extern osMutexId SaveGpsMessHandle;
extern uint8_t get_space(void);
static char   track_file[26] ={0};
extern tm tm_odor;

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

#if 1
/*1: nema->nmea*/
const char format_Aarry[][7]=
{
    "CSV",
    "GPX",
    "NMEA",
    "KML",

};
const char functionkey_Aarry[][7]=
{
    "Pause",
    "POI",

};

#endif

const char timer_zone_Aarry[][7]=
{
	"-12:00",   // 0
	"-11:00",   // 1
//	"-10:30",   // 3
	"-10:00",   // 2
	"-09:30",   // 3
	"-09:00", // 4
//	"-08:30",
	"-08:00", // 5
	"-07:00",// 6
	"-06:00",// 7
	"-05:00",// 8
	"-04:30",// 9
	"-04:00",// 10
	"-03:30",// 11
	"-03:00",// 12
	"-02:30",// 13
	"-02:00",// 14
	"-01:00",// 15
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
const uint8_t week_word[7][3]={"SUN","MON","TUE","WED","THU","FRI","SAT"};



const uint8_t  table_week[12]={0,3,3,6,1,4,6,2,5,0,3,5}; //���������ݱ�
//ƽ����·����ڱ�
const uint8_t mon_table[12]={31,28,31,30,31,30,31,31,30,31,30,31};


//void Del_oldfile_nospace(system_flag *system_flag_table);



//������������ڼ�
//��������:���빫�����ڵõ�����(ֻ����1901-2099��)
//�������������������
//����ֵ�����ں�
uint8_t RTC_Get_Week(uint16_t year,uint8_t month,uint8_t day)
{
	uint16_t temp2;
	uint8_t yearH,yearL;

	yearH = year/100;
    yearL = year%100;
	// ���Ϊ21����,�������100
	if (yearH>19)yearL += 100;
	// ����������ֻ��1900��֮���
	temp2 = yearL+yearL/4;
	temp2 = temp2%7;
	temp2 = temp2 + day + table_week[month-1];
	if ((yearL%4 == 0)&&(month < 3))temp2--;
	return(temp2%7);
}


//�ж��Ƿ������꺯��
//�·�   1  2  3  4  5  6  7  8  9  10 11 12
//����   31 29 31 30 31 30 31 31 30 31 30 31
//������ 31 28 31 30 31 30 31 31 30 31 30 31
//����:���
//���:������ǲ�������.1,��.0,����
uint8_t Is_Leap_Year(uint16_t year)
{
	if(year%4==0) //�����ܱ�4����
	{
		if(year%100==0)
		{
			if(year%400==0)return 1;//�����00��β,��Ҫ�ܱ�400����
			else return 0;
		}else return 1;
	}else return 0;
}




//�õ���ǰ��ʱ��
//����ֵ:0,�ɹ�;����:�������.
uint8_t RTC_Get(uint8_t flag,tm *my_timer)
{
//	extern tm my_timer;
	uint16_t temp1=0;
	if(flag)
	{
		my_timer->w_date +=1;  //�õ�����
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
			my_timer->w_date=1;//ƽ��
			my_timer->w_month++;//�õ��·�
			if(my_timer->w_month>12)
				{
				my_timer->w_month = 1;
				my_timer->w_year++;
				}
	    }

	}
    else
	{
	    my_timer->w_date -=1;  //�õ�����
	    if(my_timer->w_date == 0)
		{
	    	my_timer->w_month--;//�õ��·�

		    if(Is_Leap_Year(my_timer->w_year)&&my_timer->w_month==1)
			{
			    temp1 = 29;
			}
		    else
			    temp1 = mon_table[my_timer->w_month - 1];


		    my_timer->w_date= temp1;//ƽ��

		    if(my_timer->w_month == 0)
			{
			    my_timer->w_month = 12;
			    my_timer->w_year--;
			}
		}

    }

	my_timer->week = RTC_Get_Week(my_timer->w_year,my_timer->w_month,my_timer->w_date);//��ȡ����
	return 0;
}

#ifdef NEED_RTC
void  rtc_set(uint16_t syear,uint8_t smon,uint8_t sday,uint8_t hour,uint8_t min,uint8_t sec,uint8_t week)
{

    RTC_DateTypeDef RTC_DateStructure;
    RTC_TimeTypeDef RTC_TimeStructure;
    extern RTC_HandleTypeDef hrtc;



	print_usart1("rtc_set %d-%d-%d  %d:%d:%d %d \r\n",syear ,smon,sday,hour,min,sec,week);
	RTC_TimeStructure.Hours =hour;
	RTC_TimeStructure.Minutes = min;
	RTC_TimeStructure.Seconds = sec;



	RTC_DateStructure.WeekDay =  RTC_Get_Week(syear+2000, smon, sday);
	RTC_DateStructure.Date = sday;
	RTC_DateStructure.Month = smon;
	RTC_DateStructure.Year = syear;

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

#endif

void check_time(nmea_msg *gpsx ,system_flag *system_flag_table)
{
    uint16_t gpsx_utc;
    uint16_t hour_timer,min_timer;
    tm *my_timer = NULL;


    my_timer = &system_flag_table->sys_tm;

    if(gpsx->posslnum >= 2)
    {
        if((gpsx->utc.year < 2014)||(gpsx->utc.year == 2080))
        {
            return;
        }
        my_timer->w_year = gpsx->utc.year -2000;
        my_timer->w_month = gpsx->utc.month;
        my_timer->w_date = gpsx->utc.date;
        my_timer->week = RTC_Get_Week(gpsx->utc.year ,my_timer->w_month,my_timer->w_date);
        gpsx_utc = (gpsx->utc.hour*60+gpsx->utc.min);
        my_timer->sec = gpsx->utc.sec;


	    if(timer_zone_Aarry[system_flag_table->time_zone][0] == '+')
		{
    		hour_timer = ((timer_zone_Aarry[system_flag_table->time_zone][1]-'0')*10)+(timer_zone_Aarry[system_flag_table->time_zone][2]-'0');
    		min_timer = ((timer_zone_Aarry[system_flag_table->time_zone][4]-'0')*10)+(timer_zone_Aarry[system_flag_table->time_zone][5]-'0');
    		//print_usart1("\n\r hour_timer : %d   min_timer :%d  ",hour_timer,min_timer);
    		if((gpsx_utc +(hour_timer*60+min_timer))>=(24*60))
    	    {
                RTC_Get(1,my_timer);
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
			//print_usart1("\n\r hour_timer : %d   min_timer :%d  ",hour_timer,min_timer);
    		if(gpsx_utc >= (hour_timer*60+min_timer))
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
                RTC_Get(0,my_timer);
    		}
		 }

         if(hour_timer == 24)
         {
            hour_timer =0;
         }
         my_timer->hour = hour_timer;
         my_timer->min = min_timer;
#ifdef NEED_RTC
         rtc_set(my_timer->w_year,my_timer->w_month,my_timer->w_date,hour_timer,min_timer,my_timer->sec,my_timer->week);
#endif
         if(system_flag_table->frist_power == 1)
         {
             stm_write_eerpom(13,0);
             stm_write_eerpom(14,(my_timer->w_year<<16)|(my_timer->w_month<<8)|my_timer->w_date);
             system_flag_table->frist_power = 0;
         }
    }

}

uint8_t save_guiji_message(nmea_msg *gpsx ,system_flag *system_flag_table,uint8_t guji_record_type)
{
    uint8_t one_shot_buffer[MESSAGE_LEN] = {0};
    uint8_t index = 0;
    GUJI_TAG flag ;
    GUJI_DATE guji_data ;
    uint8_t error = 0;
    static uint8_t svae_nshemi = 0;
    static uint8_t save_ewhemi = 0;

    //RTC_DateTypeDef RTC_DateStructure;
    //RTC_TimeTypeDef RTC_TimeStructure;


    //if (osMutexWait(SaveGpsMessHandle, osWaitForever) == osOK)
    {
        //RTC_DateStructure = system_flag_table->RTC_DateStructure;
        //RTC_TimeStructure = system_flag_table->RTC_TimeStructure;

        //print_usart1("Message_head_number :%d\r\n",system_flag_table->Message_head_number );

        if(system_flag_table->Message_head_number == 0)
        {
            if(gpsx->hdop >= HDOP_RECODE_VAULE)
                return 1;
        }

        one_shot_buffer[index++] = (uint8_t)(( system_flag_table->Message_head_number +1)>>24)&0xff;  // 1mb
        one_shot_buffer[index++] = (uint8_t)(( system_flag_table->Message_head_number +1)>>16)&0xff;  // 2mb
        one_shot_buffer[index++] = (uint8_t)(( system_flag_table->Message_head_number +1)>>8)&0xff;  // 3mb
        one_shot_buffer[index++] = (uint8_t)(( system_flag_table->Message_head_number +1))&0xff;  // 4 mb

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
        else if(guji_record_type == 'D')
        {
            flag.bitc.tag = 2;
        }
        else if(guji_record_type == 'G')
        {
            flag.bitc.tag = 3;
        }
        else
        {
            flag.bitc.tag = 0;
        }

        if(gpsx->nshemi == 'N')
        {
            flag.bitc.ns = 0;
            svae_nshemi = gpsx->nshemi;
        }
        else if(gpsx->nshemi == 'S')
        {
            flag.bitc.ns = 1;
            svae_nshemi = gpsx->nshemi;

        }
        else
        {
            if(svae_nshemi == 'N')
            {
                flag.bitc.ns = 0;
            }
            else if(svae_nshemi == 'S')
            {
                flag.bitc.ns = 1;

            }

            print_usart1("error :%x \r\n",gpsx->nshemi);
        }


        if(gpsx->ewhemi== 'E')
        {
            flag.bitc.ew = 0;
            save_ewhemi = gpsx->ewhemi;
        }
        else if(gpsx->ewhemi== 'W')
        {
            flag.bitc.ew = 1;
            save_ewhemi = gpsx->ewhemi;
        }
        else
        {
            if(save_ewhemi== 'E')
            {
                flag.bitc.ew = 0;
            }
            else if(save_ewhemi== 'W')
            {
                flag.bitc.ew = 1;
            }

            print_usart1("error :%x \r\n",gpsx->ewhemi);
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

            check_time(gpsx,system_flag_table);
            guji_data.bitc.year  = (system_flag_table->sys_tm.w_year -16);
            guji_data.bitc.month = system_flag_table->sys_tm.w_month;
            guji_data.bitc.date  = system_flag_table->sys_tm.w_date;
            guji_data.bitc.hour  = system_flag_table->sys_tm.hour;
            guji_data.bitc.min =   system_flag_table->sys_tm.min;
            guji_data.bitc.sec =   system_flag_table->sys_tm.sec;
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

        one_shot_buffer[index++]  = (uint8_t)((gpsx->altitude>>24)&0xff);  // 15mb
        one_shot_buffer[index++]  = (uint8_t)((gpsx->altitude>>16)&0xff);  // 16mb
        one_shot_buffer[index++]  = (uint8_t)((gpsx->altitude>>8)&0xff);  // 21mb
        one_shot_buffer[index++]  = (uint8_t)((gpsx->altitude)&0xff);  // 22mb

        one_shot_buffer[index++]  = (uint8_t)(((gpsx->speed/100)>>8)&0xff);  // 23mb
        one_shot_buffer[index++]  = (uint8_t)(((gpsx->speed/100))&0xff);  // 24mb

        one_shot_buffer[index++]  = (uint8_t)(((gpsx->angle/1000)>>8)&0xff);  // 25mb
        one_shot_buffer[index++]  = (uint8_t)((gpsx->angle/1000)&0xff);  // 26mb

#ifdef P1_USAD
        one_shot_buffer[index++]  = (uint8_t)(gpsx->gpssta & 0xff);  // 27mb
        one_shot_buffer[index++]  = (uint8_t)(((gpsx->hdop)>>8)&0xff);  // 29mb
        one_shot_buffer[index++]  = (uint8_t)((gpsx->hdop)&0xff);  // 30mb

#endif


        memcpy(&system_flag_table->guji_buffer[system_flag_table->guji_buffer_Index_wp],one_shot_buffer,index);

        system_flag_table->guji_buffer_Index_wp  += index;

        if(system_flag_table->guji_buffer_Index_wp >= MAX_GUJI_BUFFER_MAX_LEN)
        {
            system_flag_table->guji_buffer_Index_wp = 0;//(system_flag_table->guji_buffer_Index_wp - MAX_GUJI_BUFFER_MAX_LEN);
           // print_usart1("over flow 2 \r\n");
        }


        system_flag_table->Message_head_number++;
        //system_flag_table->feq++;

        //print_usart1("save :%d\r\n",system_flag_table->guji_buffer_Index_wp );
        //if (osMutexRelease(SaveGpsMessHandle) != osOK)
        {
            //Error_Handler();
        }
    }


    return error;

}


void buffer_Analysis(FIL *sys_fp ,system_flag *system_flag_table, uint8_t *buffer,uint16_t munber)
{
    uint16_t index = 0,angle = 0;
    uint8_t lat_flag,lon_flag,record_type;
    uint32_t message_number_index;
    int32_t attiautl = 0;
#ifdef P1_USAD
    uint8_t fixmode = 0;
    float hodp = 0.0;
#endif
    //int16_t temp;
    float tp_lat =0.0,tp_lon =0.0,speed =0.0;
    GUJI_TAG flag ;
    GUJI_DATE guji_data ;
    //FRESULT sys_fr ;

    __align(4) uint8_t dtbuf[80];                                //��ӡ������


    do
    {
        message_number_index = ((buffer[3 + index])|(buffer[2 + index]<<8)|(buffer[1 + index]<<16)|(buffer[0 + index]<<24));
        flag.all             =  buffer[FLAG_OFFSET + index];
       // memcpy(&guji_data.all,buffer+index+4,4);
        guji_data.all        = (buffer[GUJI_DATA_OFFSET+3 + index]|(buffer[GUJI_DATA_OFFSET+2 + index]<<8)\
                                |(buffer[GUJI_DATA_OFFSET+1 + index]<<16)|(buffer[GUJI_DATA_OFFSET + index]<<24));
        tp_lat	             = (buffer[TP_LAT_OFFSET+3 + index]|(buffer[TP_LAT_OFFSET+2 + index]<<8)\
			                    |(buffer[TP_LAT_OFFSET+1 + index]<<16)|(buffer[TP_LAT_OFFSET + index]<<24));

        tp_lon               = (buffer[TP_LON_OFFSET+3 + index]|(buffer[TP_LON_OFFSET+2 + index]<<8)\
			                    |(buffer[TP_LON_OFFSET+1 + index]<<16)|(buffer[TP_LON_OFFSET + index]<<24));

        attiautl             = (buffer[ATTIAUTL_OFFSET+3 + index]|(buffer[ATTIAUTL_OFFSET+2 + index]<<8)\
			                    |(buffer[ATTIAUTL_OFFSET+1+index]<<16)|(buffer[ATTIAUTL_OFFSET+index]<<24));

        speed                = (buffer[SPEED_OFFSET+1 + index]|(buffer[SPEED_OFFSET + index]<<8));

        angle                = (buffer[ANGLE_OFFSET+1 + index]|(buffer[ANGLE_OFFSET + index]<<8));
#ifdef P1_USAD
        fixmode              =  buffer[VALID_OFFSET + index];
        hodp                 = (buffer[HDOP_OFFSET+1 + index]|(buffer[HDOP_OFFSET+ index]<<8));
#endif

        index                =  index + MESSAGE_LEN;


        record_type = flag.bitc.tag ? 'C':'T';
        if(flag.bitc.tag == 0)
        {
            record_type = 'T';

        }
        else if(flag.bitc.tag == 1)
        {
            record_type = 'C';

        }
        else if(flag.bitc.tag == 2)
        {
            record_type = 'D';

        }
        else if(flag.bitc.tag == 3)
        {
            record_type = 'G';

        }

        lat_flag    = flag.bitc.ns ? 'S':'N';
        lon_flag    = flag.bitc.ew ? 'W':'E';

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
				sprintf((char *)dtbuf,"<trkpt lat=\"-%.7f\"",tp_lat/1000000);


		    }
			else
		    {
				sprintf((char *)dtbuf,"<trkpt lat=\"%.7f\"",tp_lat/1000000);

		    }


            f_printf(sys_fp,"%s",(char *)dtbuf);
			if(lon_flag == 'W')
		    {
				sprintf((char *)dtbuf," lon=\"-%.7f\">",tp_lon/1000000);

		    }
			else
		    {
		        sprintf((char *)dtbuf," lon=\"%.7f\">",tp_lon/1000000);
		    }
		    f_printf(sys_fp,"%s",(char *)dtbuf);
            sprintf((char *)dtbuf,"<ele>%d</ele>",attiautl/10);
            f_printf(sys_fp,"%s\n",(char *)dtbuf);
            sprintf((char *)dtbuf,"<time>%04d-%02d-%02dT%02d:%02d:%02dZ</time></trkpt>",
               guji_data.bitc.year+2016,guji_data.bitc.month,guji_data.bitc.date,
               guji_data.bitc.hour,guji_data.bitc.min,guji_data.bitc.sec);
            f_printf(sys_fp,"%s\r\n",(char *)dtbuf);
        }
        else
        {
            if(message_number_index == 1)
            {
#ifdef P1_USAD
                f_printf(sys_fp,"INDEX,TAG,DATE,TIME,LATITUDE N/S,LONGITUDE E/W,HEIGHT,SPEED,HEADING,VALID,HDOP\n");
#else
                f_printf(sys_fp,"INDEX,TAG,DATE,TIME,LATITUDE N/S,LONGITUDE E/W,HEIGHT,SPEED,HEADING\n");
#endif
            }


            sprintf((char *)dtbuf,"%d,%c,%02d%02d%02d,",message_number_index,record_type,
                guji_data.bitc.year+16,guji_data.bitc.month,guji_data.bitc.date);
            f_printf(sys_fp,"%s",(char *)dtbuf);
            sprintf((char *)dtbuf,"%02d%02d%02d,%.7f%c,",guji_data.bitc.hour,guji_data.bitc.min,guji_data.bitc.sec,tp_lat/1000000,lat_flag);
            f_printf(sys_fp,"%s",(char *)dtbuf);

#ifdef P1_USAD
            sprintf((char *)dtbuf,"%.7f%c,%d,%.1f,%d",tp_lon/1000000,lon_flag,attiautl/10,(speed/10),angle);
            f_printf(sys_fp,"%s",(char *)dtbuf);

            if(fixmode  == 2)
                sprintf((char *)dtbuf,",DGPS,%.2f",(hodp/100));
            else if(fixmode  == 3)
                sprintf((char *)dtbuf,",PPS,%.2f",(hodp/100));
            else
                sprintf((char *)dtbuf,",SPS,%.2f",(hodp/100));

            f_printf(sys_fp,"%s\n",(char *)dtbuf);
#else
            sprintf((char *)dtbuf,"%.7f%c,%d,%.1f,%d",tp_lon/1000000,lon_flag,attiautl/10,(speed/10),angle);
            f_printf(sys_fp,"%s\n",(char *)dtbuf);

#endif
            //print_usart1("index :%d \r\n",message_number_index);

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

//uint32_t loading_leng = 0;

void write_flash(FIL *sys_fp,system_flag *system_flag_table)  /*write to  the file by true*/
{
    UINT wb = 0;
    uint8_t *guji_buffer_ = NULL;
    uint16_t rxlen = 0;
//    FRESULT fr ;
    //static uint8_t debug_cnt = 0;
    uint16_t save_wp = 0;

    save_wp = system_flag_table->guji_buffer_Index_wp;

    if(save_wp!= system_flag_table->guji_buffer_Index_rp)
    {
        if(system_flag_table->guji_buffer_Index_rp > save_wp)
        {
            rxlen = save_wp + MAX_GUJI_BUFFER_MAX_LEN -system_flag_table->guji_buffer_Index_rp;

            guji_buffer_ = malloc(rxlen+1);
            memcpy(guji_buffer_,system_flag_table->guji_buffer+system_flag_table->guji_buffer_Index_rp,(MAX_GUJI_BUFFER_MAX_LEN-system_flag_table->guji_buffer_Index_rp));
            memcpy(guji_buffer_ + (MAX_GUJI_BUFFER_MAX_LEN-system_flag_table->guji_buffer_Index_rp),system_flag_table->guji_buffer,save_wp);


        }
        else
        {
            rxlen = (save_wp - system_flag_table->guji_buffer_Index_rp);
            guji_buffer_ = malloc(rxlen+1);
            memcpy(guji_buffer_,system_flag_table->guji_buffer+system_flag_table->guji_buffer_Index_rp,rxlen);


        }
        guji_buffer_[rxlen] = '\0';

        //loading_leng += rxlen;

        if(guji_buffer_ == NULL)
        {
            print_usart1("Error: guji_buffer_ malloc failed  rxlen = %d\r\n",rxlen);
            return;
        }
        else
        {
            //print_usart1("%d-%d-%d\r\n",rxlen,system_flag_table->guji_buffer_Index_wp ,system_flag_table->guji_buffer_Index_rp );

        }
#if 0
        system_flag_table->guji_buffer_Index_rp += rxlen;
        if(system_flag_table->guji_buffer_Index_rp >= MAX_GUJI_BUFFER_MAX_LEN)
        {
            system_flag_table->guji_buffer_Index_rp = system_flag_table->guji_buffer_Index_rp  - MAX_GUJI_BUFFER_MAX_LEN;
        }
#endif
        //system_flag_table->guji_buffer_Index_wp = 0;
        //system_flag_table->guji_buffer_Index_rp = 0;
        system_flag_table->guji_buffer_Index_rp = save_wp;
        if(get_space() < 1)
        {
            if(system_flag_table->guji_record.recoed_meth == 0)
            {

                //Del_oldfile_nospace(system_flag_table);

            }
            else
            {
                system_flag_table->guji_mode = RECORED_STOP;
    			system_flag_table->guji_buffer_Index_rp = 0;
    			system_flag_table->guji_buffer_Index_wp = 0;

            }
            free(guji_buffer_);
            return;
        }

       if((system_flag_table->gujiFormats == GUJI_FORMATS_CSV) || (system_flag_table->power_status == POWER_LRUN)|| (system_flag_table->power_status == POWER_LRUN_SLEEP))
       //if(system_flag_table->gujiFormats == GUJI_FORMATS_CSV)

        {
            buffer_Analysis(sys_fp,system_flag_table,guji_buffer_,rxlen);
        }
        else if((system_flag_table->gujiFormats == GUJI_FORMATS_GPS)||(system_flag_table->gujiFormats == GUJI_FORMATS_MEA))
        {

            f_write(sys_fp,guji_buffer_,rxlen,&wb);
#if 0
            if(debug_cnt >= 200)
            {
                //print_usart1("rxlen = %d\r\n",loading_leng);
                print_usart1("%s",guji_buffer_);
                debug_cnt++;
                if(debug_cnt>= 220)
                {
                    print_usart1("\r\n*********\r\n");
                    debug_cnt = 0;
                }
            }
            else
            {
                debug_cnt++;
            }
#endif



        }
        else if(system_flag_table->gujiFormats == GUJI_FORMATS_GPX)
        {
            //todo:add gpx file conote
            buffer_Analysis(sys_fp,system_flag_table,guji_buffer_,rxlen);

        }

        free(guji_buffer_);
        system_flag_table->wirte_storge_flag = 1;
        system_flag_table->File_status = 1;





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

#if 0

void Del_oldfile_nospace(system_flag *system_flag_table)
{
    FRESULT fr = FR_OK;
    unsigned long data_time = 0xffffffff;
    char buff[64] = {0};    /* Working buffer */
    char out_buff[64] = {0};    /* Working buffer */
    int filenumber = 0;

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
			 system_flag_table->guji_buffer_Index_rp = 0;
			 system_flag_table->guji_buffer_Index_wp = 0;
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

#endif
extern uint32_t save_file_cnt ;
void Recording_guji(FIL *sys_fp,system_flag *system_flag_table,nmea_msg *gpsx)
{

	DIR* dp = NULL;
    FRESULT fr;
    FRESULT sys_fr;
    uint32_t eeprom_vaule = 0;
    UINT wb;
    tm       eeprom_tm;
//    int cnt_read = 0;
    uint8_t ret = 0 ;


    uint8_t mode = system_flag_table->guji_mode;


	switch(mode )
    {
		case RECORED_IDLE:

			break;
		case RECORED_START:
        case RECORED_RESTART:

        case RECORED_RESTART_2:

            if(mode != RECORED_RESTART_2)
            {
                if((system_flag_table->ODOR == 0)||(mode == RECORED_RESTART ))
    			     system_flag_table->Message_head_number = 0;
            }

			if((gpsx->gpssta >= 1)&&(gpsx->latitude >0)&&(gpsx->longitude>0))
            //if(gpsx->gpssta >= 1)
			{
				system_flag_table->guji_buffer_Index_rp = 0;
				system_flag_table->guji_buffer_Index_wp = 0;


                check_time(gpsx,system_flag_table);
                //RTC_DateStructure = system_flag_table->RTC_DateStructure;
                //RTC_TimeStructure = system_flag_table->RTC_TimeStructure;
                //print_usart1("w_year :%d \r\n",RTC_DateStructure.Year);
                if(get_space() == 0)
                {

				    if(system_flag_table->guji_record.recoed_meth == 1)
                    {
					    system_flag_table->guji_mode = RECORED_IDLE;
					    return ;
				    }
#if 0
					else
				    {
				        Del_oldfile_nospace(system_flag_table);
						if(system_flag_table->guji_mode == RECORED_STOP)
					    {
						    system_flag_table->guji_mode = RECORED_IDLE;
						    return ;
					    }
				    }
#endif
                }

                //print_usart1("gpsx->utc.year :%d \r\n",system_flag_table->sys_tm.w_year);

                if(gpsx->utc.year >= 15)
                {
                    sprintf(track_file,"%04d-%02d",system_flag_table->sys_tm.w_year + 2000,system_flag_table->sys_tm.w_month);
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

                    stm_read_eerpom(20,&eeprom_vaule);
                    eeprom_tm.w_year  = eeprom_vaule;
                    stm_read_eerpom(21,&eeprom_vaule);
                    eeprom_tm.w_month = eeprom_vaule;
                    stm_read_eerpom(22,&eeprom_vaule);
                    eeprom_tm.w_date  = eeprom_vaule;
                    stm_read_eerpom(23,&eeprom_vaule);
                    eeprom_tm.hour    = eeprom_vaule;
                    stm_read_eerpom(24,&eeprom_vaule);
                    eeprom_tm.min     = eeprom_vaule;
                    stm_read_eerpom(25,&eeprom_vaule);
                    eeprom_tm.sec     = eeprom_vaule;

                    if(system_flag_table->ODOR == 1)
                    {
                        /*eeprom 20 -25*/


                        //print_usart1("%04d-%02d/%02d%02d%02d%02d.CSV \r\n",eeprom_tm.w_year,eeprom_tm.w_month,
                        //    eeprom_tm.w_date, eeprom_tm.hour,eeprom_tm.min,eeprom_tm.sec);

                        if((eeprom_tm.w_year != system_flag_table->sys_tm.w_year+2000)||
                          (eeprom_tm.w_month!= system_flag_table->sys_tm.w_month)||
                          (eeprom_tm.w_date!= system_flag_table->sys_tm.w_date))
                        {
                            ret = 1;
                            stm_write_eerpom(26,0);

                        }

                        stm_read_eerpom(26,&eeprom_vaule);
                        if(eeprom_vaule & (1<<system_flag_table->gujiFormats) != (1<<system_flag_table->gujiFormats))
                        {
                            ret = 1;
                        }

                        if((eeprom_tm.w_year <2017 )||(eeprom_tm.w_year >2210 ))
                        {
                            ret = 1;
                        }

                        if(RECORED_RESTART == mode)
                            ret = 1;


                    }
                    else
                    {
                        if((mode == RECORED_START )||(RECORED_RESTART == mode))
                            ret = 1;

                    }



                    if(ret == 0)
                    {

                        if((system_flag_table->gujiFormats == GUJI_FORMATS_CSV) || (system_flag_table->power_status == POWER_LRUN))
                        //if(system_flag_table->gujiFormats == GUJI_FORMATS_CSV)
                        {
                            system_flag_table->gujiFormats = GUJI_FORMATS_CSV;
                            sprintf(track_file,"%04d-%02d/%02d%02d%02d%02d.CSV",eeprom_tm.w_year,eeprom_tm.w_month,
                            eeprom_tm.w_date, eeprom_tm.hour,eeprom_tm.min,eeprom_tm.sec);

                        }
                        else if(system_flag_table->gujiFormats == GUJI_FORMATS_GPS)
                        {
                            sprintf(track_file,"%04d-%02d/%02d%02d%02d%02d.GPS",eeprom_tm.w_year,eeprom_tm.w_month,
                            eeprom_tm.w_date, eeprom_tm.hour,eeprom_tm.min,eeprom_tm.sec);
                        }
                        else if(system_flag_table->gujiFormats == GUJI_FORMATS_GPX)
                        {
                            sprintf(track_file,"%04d-%02d/%02d%02d%02d%02d.GPX",eeprom_tm.w_year,eeprom_tm.w_month,
                            eeprom_tm.w_date, eeprom_tm.hour,eeprom_tm.min,eeprom_tm.sec);
                        }
                        else if(system_flag_table->gujiFormats == GUJI_FORMATS_MEA)
                        {
                            sprintf(track_file,"%04d-%02d/%02d%02d%02d%02d.TXT",eeprom_tm.w_year,eeprom_tm.w_month,
                            eeprom_tm.w_date, eeprom_tm.hour,eeprom_tm.min,eeprom_tm.sec);
                        }
                        print_usart1("\r\n open old file :%s \r\n ",track_file);

                        stm_read_eerpom(100,&eeprom_vaule);
                        system_flag_table->Message_head_number = eeprom_vaule ;
                        memcpy(&tm_odor,&(eeprom_tm),sizeof(tm));
                        if(tm_odor.w_year> 2000)
                            tm_odor.w_year = tm_odor.w_year -2000;

                        print_usart1("eeprom %04d-%02d/%02d%02d%02d%02d\r\n",tm_odor.w_year,tm_odor.w_month,tm_odor.w_date,tm_odor.hour,tm_odor.min,tm_odor.sec);

                        sys_fr = f_open(sys_fp,track_file,FA_READ);
                        print_usart1("--open--  :%d \n",sys_fr);
                        if(FR_OK  != sys_fr)
                        {
                            system_flag_table->Message_head_number = 0;
                            ret = 1;
                        }
                        f_close(sys_fp);

                    }

                    if(ret == 1)
                    {

                        if((system_flag_table->gujiFormats == GUJI_FORMATS_CSV) || (system_flag_table->power_status == POWER_LRUN))
                        //if(system_flag_table->gujiFormats == GUJI_FORMATS_CSV)
                        {
                            system_flag_table->gujiFormats = GUJI_FORMATS_CSV;
                            sprintf(track_file,"%04d-%02d/%02d%02d%02d%02d.CSV",system_flag_table->sys_tm.w_year+2000,system_flag_table->sys_tm.w_month,
                            system_flag_table->sys_tm.w_date, system_flag_table->sys_tm.hour,system_flag_table->sys_tm.min,system_flag_table->sys_tm.sec);

                        }
                        else if(system_flag_table->gujiFormats == GUJI_FORMATS_GPS)
                        {
                            sprintf(track_file,"%04d-%02d/%02d%02d%02d%02d.GPS",system_flag_table->sys_tm.w_year+2000,system_flag_table->sys_tm.w_month,
                            system_flag_table->sys_tm.w_date, system_flag_table->sys_tm.hour,system_flag_table->sys_tm.min,system_flag_table->sys_tm.sec);
                        }
                        else if(system_flag_table->gujiFormats == GUJI_FORMATS_GPX)
                        {
                            sprintf(track_file,"%04d-%02d/%02d%02d%02d%02d.GPX",system_flag_table->sys_tm.w_year+2000,system_flag_table->sys_tm.w_month,
                            system_flag_table->sys_tm.w_date, system_flag_table->sys_tm.hour,system_flag_table->sys_tm.min,system_flag_table->sys_tm.sec);
                        }
                        else if(system_flag_table->gujiFormats == GUJI_FORMATS_MEA)
                        {
                            sprintf(track_file,"%04d-%02d/%02d%02d%02d%02d.TXT",system_flag_table->sys_tm.w_year+2000,system_flag_table->sys_tm.w_month,
                            system_flag_table->sys_tm.w_date, system_flag_table->sys_tm.hour,system_flag_table->sys_tm.min,system_flag_table->sys_tm.sec);
                        }
                        print_usart1("\r\n create new file :%s \r\n ",track_file);

                        stm_write_eerpom(20,system_flag_table->sys_tm.w_year+2000);
                        stm_write_eerpom(21,system_flag_table->sys_tm.w_month);
                        stm_write_eerpom(22,system_flag_table->sys_tm.w_date);
                        stm_write_eerpom(23,system_flag_table->sys_tm.hour);
                        stm_write_eerpom(24,system_flag_table->sys_tm.min);
                        stm_write_eerpom(25,system_flag_table->sys_tm.sec);
                        memcpy(&tm_odor,&(system_flag_table->sys_tm),sizeof(tm));
                        print_usart1("new %04d-%02d/%02d%02d%02d%02d\r\n",tm_odor.w_year+2000,tm_odor.w_month,tm_odor.w_date,tm_odor.hour,tm_odor.min,tm_odor.sec);

                        stm_read_eerpom(26,&eeprom_vaule);
                        stm_write_eerpom(26,(eeprom_vaule|(1<<system_flag_table->gujiFormats)));

                    }



                    if(system_flag_table->gujiFormats == GUJI_FORMATS_GPX)
                    {
                        sys_fr = open_append_sp(sys_fp, track_file);
                    }
                    else
                        sys_fr = open_append(sys_fp, track_file);

                    if((FR_OK  != sys_fr)&&(FR_EXIST  != sys_fr))
                    {
                        print_usart1("open append faild :%d \n",sys_fr);
                        return;
                    }
                    else
                    {

                        system_flag_table->File_status = 1;
                    }

                    save_file_cnt  = HAL_GetTick();
                    if((ret == 1) && (system_flag_table->power_status != POWER_LRUN))
                    {
                        if(system_flag_table->gujiFormats == GUJI_FORMATS_GPS)
                        {
                            track_file[0] = 0x07;
                            track_file[1] = 0x07;
                            f_write(sys_fp,track_file,2,&wb);
                        }
                        else if(system_flag_table->gujiFormats == GUJI_FORMATS_GPX)
                        {
                            //todo: add gpx file head.

                            gpx_filehead_write(sys_fp,track_file);
                        }

                    }
                    print_usart1("delay \r\n");
                    system_flag_table->grecord_timer_cnt = HAL_GetTick();
        			//save_guiji_message(gpsx,system_flag_table,'T');

        			//interst_pos_number = 0;
        			while(system_flag_table->grecord_timer_cnt +2000 > HAL_GetTick())
                    {
                        osDelay(100);
                    }

                    system_flag_table->grecord_timer_cnt = HAL_GetTick();
                    if(system_flag_table->puase_flag == 1)
                    {
                          system_flag_table->guji_mode = RECORED_PAUSE;
                    }
                    else
                          system_flag_table->guji_mode = 2;
                    print_usart1("start -> doing \r\n");

               }

			}
            else
            {
                #if 0//def TEST_WRITE_SD
                    //RTC_DateStructure = system_flag_table->RTC_DateStructure;
                   // RTC_TimeStructure = system_flag_table->RTC_TimeStructure;

                    sprintf(track_file,"%04d-%02d",17+ 2000,6);
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


                    sprintf(track_file,"%04d-%02d/%02d%02d%02d%02d.NMA",2017,
                        6,11,11,11,10);
                    print_usart1("\r\n track_file :%s \r\n ",track_file);
                    sys_fr = open_append(sys_fp, track_file);

                    if(FR_OK  != sys_fr)
                    {
                        print_usart1("open append faild :%d \r\n",sys_fr);
                        system_flag_table->guji_mode = RECORED_IDLE;
                        return;
                    }

                    system_flag_table->guji_mode = 2;

                #endif
            }


			break;
		case RECORED_START_DOING:
            write_flash(sys_fp,system_flag_table);
			break;
		case RECORED_T:
			save_guiji_message(gpsx,system_flag_table,'C');
			system_flag_table->guji_mode = RECORED_START_DOING;
			break;
        case RECORED_D:
            save_guiji_message(gpsx,system_flag_table,'D');
            //interst_pos_number++;
            system_flag_table->guji_mode = RECORED_START_DOING;
            break;

        case RECORED_SAVE:
            if((system_flag_table->power_status != POWER_STANBY)&&(system_flag_table->power_status != POWER_LRUN_SLEEP)\
            &&(system_flag_table->power_status != POWER_SURPORT_SLEEP))
            {

                if(system_flag_table->File_status == 1)
                {
#if 1

                   sys_fr = f_sync(sys_fp);					/* Flush cached data */
                   //print_usart1("=%d=\r\n",HAL_GetTick());

#else
                   sys_fr = f_close(sys_fp);
                   print_usart1("\r\n close :%d\r\n ",sys_fr);
#endif
                   if(FR_OK  == sys_fr)
                   {
                        print_usart1("\r\n sync \r\n ");
                        system_flag_table->File_status = 0;
                   }
                   else
                   {

                        print_usart1("\r\n sync err :%d \r\n ",sys_fr);

                        #if 0
                        sys_fr = open_append(sys_fp, track_file);
                        if((FR_OK  != sys_fr)&&(FR_EXIST  != sys_fr))
                        {

                            print_usart1("open append faild(save) :%d :%s\n",sys_fr,track_file);
                            system_flag_table->sd_stats = SD_STATS_ERROR_CARD;
                            system_flag_table->guji_mode = RECORED_STOP;
                            return;

                        }
                        else
                        {
                            system_flag_table->File_status = 1;
                        }
                        #endif

                        system_flag_table->File_status = 0;

                   }

                }
            }
            system_flag_table->guji_mode = 2;


            break;
		case RECORED_STOP:

            if(sys_fp != NULL)
            {
    			if(get_space() > 0)
    		    {
                  	if((gpsx->gpssta >= 1)&&(gpsx->latitude >0)&&(gpsx->longitude>0))
                    {
                        //save_guiji_message(gpsx,system_flag_table,'T');
                        write_flash(sys_fp,system_flag_table);
                        print_usart1("\r\n close file \r\n ");
                     }
    			}


                if(system_flag_table->gujiFormats == GUJI_FORMATS_GPX)
                {
                    //if(!((system_flag_table->power_status == POWER_LRUN_SLEEP)||(system_flag_table->power_status == POWER_SURPORT_SLEEP)))
                    {
                        f_printf(sys_fp,"</trkseg>\n");
                        f_printf(sys_fp,"</trk>\n");
                        f_printf(sys_fp,"</gpx>\n");
                    }
                }
                fr = f_close(sys_fp);
                print_usart1("\r\n close file :%d\r\n ",fr);

                stm_write_eerpom(100,system_flag_table->Message_head_number);

            }

			 print_usart1("\r\n 11-close file :%d--%x\r\n ",fr,sys_fp);
            system_flag_table->guji_mode = RECORED_IDLE;

			break;
       case RECORED_PAUSE:
            if(0)//(system_flag_table->power_status == POWER_LRUN_SLEEP)||(system_flag_table->power_status == POWER_SURPORT_SLEEP))
            {
                if(sys_fp != NULL)
                {
        			if(get_space() > 0)
        		    {
                      	if((gpsx->gpssta >= 1)&&(gpsx->latitude >0)&&(gpsx->longitude>0))
                        {
                            //save_guiji_message(gpsx,system_flag_table,'T');
                            write_flash(sys_fp,system_flag_table);
                            print_usart1("\r\n RECORED_PAUSE close file :%x \r\n ",sys_fp);
                         }
        			}


                    if(system_flag_table->gujiFormats == GUJI_FORMATS_GPX)
                    {
                        //if(!((system_flag_table->power_status == POWER_LRUN_SLEEP)||(system_flag_table->power_status == POWER_SURPORT_SLEEP)))
                        {
                            f_printf(sys_fp,"</trkseg>\n");
                            f_printf(sys_fp,"</trk>\n");
                            f_printf(sys_fp,"</gpx>\n");
                        }
                    }
                    fr = f_close(sys_fp);
                    print_usart1("\r\n close file :%d--%x\r\n ",fr,sys_fp);

                    stm_write_eerpom(100,system_flag_table->Message_head_number);

                }

    			print_usart1("\r\n close file-- :%d\r\n ",fr);
            }

            if(system_flag_table->power_status == POWER_LRUN_SLEEP)
            {
                //system_flag_table->guji_mode = RECORED_IDLE;
                system_flag_table->puase_flag = 0;
            }
            else
            {
                system_flag_table->puase_flag = 1;
            }

            break;
	}
}

