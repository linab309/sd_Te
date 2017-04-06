#ifndef __MENUTAL_H
#define __MENUTAL_H	 

#include "stm32l1xx_hal.h"
#include "gps.h"
#include "ff.h"


#define SYSTEM_POWER_STANBY 0
#define SYSTEM_POWER_ON 1
#define SYSTEM_POWER_LOW  2
#define SYSTEM_POWER_LOW_2  3
#define SYSTEM_POWER_ON_USB 4



#define RECORED_IDLE 0
#define RECORED_START 1
#define RECORED_START_DOING 2
#define RECORED_T  3
#define RECORED_STOP 4


#define KEY_UP 0
#define KEY_DOWN 1
#define KEY_ENTER 2
#define KEY_LONG_ENTER 3


/****设置菜单*****/
#define MENU_SETTING_MAIN_PAGE 0
#define MENU_SETTING_SEC_PAGE 1
#define MENU_SETTING_THR_PAGE 2
#define MENU_SETTING_FOR_PAGE 3

#define PAGE_RUN_MODEL 0
#define PAGE_GUJI_RECORD 1
#define PAGE_GESHI_DANGWEI 2
#define PAGE_GEREN_ZHILIAO 3
#define PAGE_SYSTEM_SET 4
#define PAGE_QUIT 5


#define GUJI_FORMATS_CSV 0
#define GUJI_FORMATS_GPX 1
#define GUJI_FORMATS_GPS 2



typedef enum
{
  SENSOR_MODEL = 0,
  GPS_MODEL,
}RUN_MODEL;

typedef enum
{
  CUSTOM_RECORED,
  RECORED_MODEL,
  AUTO_MODEL,
  RECORED_FORMAT,
  DELECT_ALL,
  CUTOME_RECORED_RETURN,
}CUTOME_RECORED;


typedef enum
{
  UNIT_TIME = 0,
  UNIT_DATE,
  UNIT_COORD,
  UNIT_DISCANCE,
  UNIT_TEMP,
  UNIT_PRES,
  UNIT_REUTNR,

}UNIT_MODEL;

typedef enum
{
    PERSONAL_FALEM = 0,
    PERSONAL_HEIGHT ,
    PERSONAL_WEIGHT,
    PERSONAL_REUTRN,
}PERSONAL_data;

typedef enum
{
    SYSTEM_ADR_ADJUST = 0,
    SCREEN_STANBY,
    POWERDONW_CONTRL,
    TIMER_ZONE,
    WANNER_SOUND,
   // STORM_ALARM,
    LANGUAGE,
    SYSTEM_INFO,
    FACTROY,
    FIRMWARE_UPDATE,
    SYSTEM_RETURN,
}SYSTEM_SETS;



typedef enum
{
    BY_DISTANCE = 0,
    BY_TIMES,
    BY_SPEED,
    RECORED_MODEL_OPTIONS_RETURN,
}RECORED_CUSTOM_OPTIONS;


typedef enum
{
    FORMATS_CSV = 0,
    FORMATS_GPX,
    FORMATS_GPS,
}RECORED_FORMATS_OPTIONS;


typedef enum
{
    AUTO_RECOVRY = 0,
    AUTO_STOP,
    AUTO_MODEL_OPTIONS_RETURN,
}RECORED_MODEL_OPTIONS;

typedef enum
{
    AUTO_ON = 0,
    AUTO_OFF,
}AUTO_MODEL_OPTIONS;

typedef struct 
{	//公历日月年周
	uint16_t w_year;
	uint8_t  w_month;
	uint8_t  w_date;
	uint8_t  week;		 
	uint8_t  sec;		 
}tm;	


typedef struct  
{		
	uint8_t    by_distance_time_flag;
	uint16_t   by_distance_vaule;
	uint16_t    by_time_vaule;  /*ms*/
	uint16_t   by_speed_vaule;
    uint8_t    recoed_formats;
	uint8_t    recoed_meth;  /*自动覆盖或是记满停止*/
	uint8_t    auto_recoed_flag;
}GUJI_RECOCE_TABLE; 


typedef struct  
{
    GUJI_RECOCE_TABLE guji_record;
    uint8_t  guji_mode;
    uint8_t  power_status ;
    uint8_t  power_mode ;
    uint8_t  *guji_buffer;
    uint16_t guji_buffer_Index_rp;
    uint16_t guji_buffer_Index_wp;
    uint32_t Message_head_number;
    uint8_t  gujiFormats;
    uint8_t  baifenbi;
    uint8_t  time_zone;
    float    index_timerzone;
    tm       sys_tm;    
    uint8_t batt_Status;
	uint8_t charger_connected;
    uint32_t batt_change_ok_cnt;
    RTC_DateTypeDef RTC_DateStructure;
    RTC_TimeTypeDef RTC_TimeStructure;
    uint8_t Led_pwm_type;
    uint32_t tp_long;
    uint32_t tp_lati;
    uint32_t grecord_timer_cnt;
}system_flag;


extern uint8_t save_guiji_message(nmea_msg *gpsx ,system_flag *system_flag_table,uint8_t guji_record_type);
extern void Recording_guji(FIL *sys_fp,system_flag *system_flag_table,nmea_msg *gpsx);
extern uint8_t RTC_Get_Week(uint16_t year,uint8_t month,uint8_t day);
extern void write_flash(FIL *sys_fp,system_flag *system_flag_table);  /*write to  the file by true*/

#endif

