#include <stdio.h>
#include <string.h>
#include "stm32l1xx_hal.h"
#include "ff.h"


/*
 * 函数名：			GetIniKeyString
 * 入口参数：		title
 *						配置文件中一组数据的标识
 *					key
 *						这组数据中要读出的值的标识
 *					filename
 *						要读取的文件路径
 * 返回值：			找到需要查的值则返回正确结果
 *					否则返回NULL
 */
char *GetIniKeyString(char *title,char *key,char *filename)
{
    FIL  fp;
    int  flag = 0;
    //FRESULT fr = FR_OK ;
    char sTitle[32], *wTmp;
    char sLine[256] ={0};
    
    sprintf(sTitle, "[%s]", title);
    if(FR_OK != f_open(&fp, filename, FA_READ)) {
    	print_usart1("f_open \r\n");
    	return NULL;
    }

    while (NULL != f_gets(sLine, 256, &fp)) 
    {
    	// 这是注释行
    	if (0 == strncmp("//", sLine, 2)) continue;
    	if ('#' == sLine[0])              continue;
    
    	wTmp = strchr(sLine, '=');
    	if ((NULL != wTmp) && (1 == flag)) {
    		if (0 == strncmp(key, sLine, wTmp-sLine)) { // 长度依文件读取的为准
    			sLine[strlen(sLine) - 1] = '\0';
    			f_close(&fp);
    			return wTmp + 1;
    		}
    	} else {
    		if (0 == strncmp(sTitle, sLine, strlen(sLine) - 1)) { // 长度依文件读取的为准
    			flag = 1; // 找到标题位置
    		}
    	}
    }
    f_close(&fp);
    return NULL;
}

/*
 * 函数名：			GetIniKeyInt
 * 入口参数：		title
 *						配置文件中一组数据的标识
 *					key
 *						这组数据中要读出的值的标识
 *					filename
 *						要读取的文件路径
 * 返回值：			找到需要查的值则返回正确结果
 *					否则返回NULL
 */
int GetIniKeyInt(char *title,char *key,char *filename)
{
    return atoi(GetIniKeyString(title, key, filename));
}

/*
 * 函数名：			PutIniKeyString
 * 入口参数：		title
 *						配置文件中一组数据的标识
 *					key
 *						这组数据中要读出的值的标识
 *					val
 *						更改后的值
 *					filename
 *						要读取的文件路径
 * 返回值：			成功返回  0
 *					否则返回 -1
 */
int PutIniKeyString(char *title,char *key,char *val,char *filename)
{
    FIL fpr, fpw;
    int  flag = 0;
    char sLine[256], sTitle[32], *wTmp;
    
    sprintf(sTitle, "[%s]", title);
    if (FR_OK != f_open(&fpr,filename, FA_READ))
    {   
    	//PRN_ERRMSG_RETURN("f_open");// 读取原文件
    	print_usart1("f_open %s error R \r\n",filename);
    	return -1;
    }
    sprintf(sLine, "%s.tmp", filename);
    if (FR_OK != f_open(&fpw,sLine,FA_WRITE))
    {   
        //PRN_ERRMSG_RETURN("f_open");// 读取原文件
        print_usart1("f_open %s error W \r\n",filename);
        return -1;
    }

    while (NULL != f_gets(sLine, 256, &fpr)) 
    {
    	if (2 != flag) { // 如果找到要修改的那一行，则不会执行内部的操作
    		wTmp = strchr(sLine, '=');
    		if ((NULL != wTmp) && (1 == flag)) {
    			if (0 == strncmp(key, sLine, wTmp-sLine)) { // 长度依文件读取的为准
    				flag = 2;// 更改值，方便写入文件
    				sprintf(wTmp + 1, "%s\n", val);
    			}
    		} else {
    			if (0 == strncmp(sTitle, sLine, strlen(sLine) - 1)) { // 长度依文件读取的为准
    				flag = 1; // 找到标题位置
    			}
    		}
    	}
    
    	f_puts(sLine, &fpw); // 写入临时文件
    }
    f_close(&fpr);
    f_close(&fpw);
    
    sprintf(sLine, "%s.tmp", filename);
    return f_rename(sLine, filename);// 将临时文件更新到原文件
}

/*
 * 函数名：			PutIniKeyString
 * 入口参数：		title
 *						配置文件中一组数据的标识
 *					key
 *						这组数据中要读出的值的标识
 *					val
 *						更改后的值
 *					filename
 *						要读取的文件路径
 * 返回值：			成功返回  0
 *					否则返回 -1
 */
int PutIniKeyInt(char *title,char *key,int val,char *filename)
{
	char sVal[32];
	sprintf(sVal, "%d", val);
	return PutIniKeyString(title, key, sVal, filename);
}

/*
===============================================================================
                    ##### DATA EEPROM Programming functions ##### 
===============================================================================  

   [..] Any operation of erase or program should follow these steps:
   (#) Call the @ref HAL_FLASHEx_DATAEEPROM_Unlock() function to enable the data EEPROM access
       and Flash program erase control register access.
   (#) Call the desired function to erase or program data.
   (#) Call the @ref HAL_FLASHEx_DATAEEPROM_Lock() to disable the data EEPROM access
       and Flash program erase control register access(recommended
       to protect the DATA_EEPROM against possible unwanted operation).
*/



#if 0
int main(int argc,char *argv[])
{
	printf("%s\n", GetIniKeyString("DOG", "name", "config.ini"));
	printf("%d\n", GetIniKeyInt("DOG", "age", "config.ini"));
	PutIniKeyString("CAT", "name", "ccc", "config.ini");
	PutIniKeyInt("DOG", "age", 56, "config.ini");
	return 0;
}
#endif
