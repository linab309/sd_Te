#include <stdio.h>
#include <string.h>
#include "stm32l1xx_hal.h"
#include "ff.h"


/*
 * ��������			GetIniKeyString
 * ��ڲ�����		title
 *						�����ļ���һ�����ݵı�ʶ
 *					key
 *						����������Ҫ������ֵ�ı�ʶ
 *					filename
 *						Ҫ��ȡ���ļ�·��
 * ����ֵ��			�ҵ���Ҫ���ֵ�򷵻���ȷ���
 *					���򷵻�NULL
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
    	// ����ע����
    	if (0 == strncmp("//", sLine, 2)) continue;
    	if ('#' == sLine[0])              continue;
    
    	wTmp = strchr(sLine, '=');
    	if ((NULL != wTmp) && (1 == flag)) {
    		if (0 == strncmp(key, sLine, wTmp-sLine)) { // �������ļ���ȡ��Ϊ׼
    			sLine[strlen(sLine) - 1] = '\0';
    			f_close(&fp);
    			return wTmp + 1;
    		}
    	} else {
    		if (0 == strncmp(sTitle, sLine, strlen(sLine) - 1)) { // �������ļ���ȡ��Ϊ׼
    			flag = 1; // �ҵ�����λ��
    		}
    	}
    }
    f_close(&fp);
    return NULL;
}

/*
 * ��������			GetIniKeyInt
 * ��ڲ�����		title
 *						�����ļ���һ�����ݵı�ʶ
 *					key
 *						����������Ҫ������ֵ�ı�ʶ
 *					filename
 *						Ҫ��ȡ���ļ�·��
 * ����ֵ��			�ҵ���Ҫ���ֵ�򷵻���ȷ���
 *					���򷵻�NULL
 */
int GetIniKeyInt(char *title,char *key,char *filename)
{
    return atoi(GetIniKeyString(title, key, filename));
}

/*
 * ��������			PutIniKeyString
 * ��ڲ�����		title
 *						�����ļ���һ�����ݵı�ʶ
 *					key
 *						����������Ҫ������ֵ�ı�ʶ
 *					val
 *						���ĺ��ֵ
 *					filename
 *						Ҫ��ȡ���ļ�·��
 * ����ֵ��			�ɹ�����  0
 *					���򷵻� -1
 */
int PutIniKeyString(char *title,char *key,char *val,char *filename)
{
    FIL fpr, fpw;
    int  flag = 0;
    char sLine[256], sTitle[32], *wTmp;
    
    sprintf(sTitle, "[%s]", title);
    if (FR_OK != f_open(&fpr,filename, FA_READ))
    {   
    	//PRN_ERRMSG_RETURN("f_open");// ��ȡԭ�ļ�
    	print_usart1("f_open %s error R \r\n",filename);
    	return -1;
    }
    sprintf(sLine, "%s.tmp", filename);
    if (FR_OK != f_open(&fpw,sLine,FA_WRITE))
    {   
        //PRN_ERRMSG_RETURN("f_open");// ��ȡԭ�ļ�
        print_usart1("f_open %s error W \r\n",filename);
        return -1;
    }

    while (NULL != f_gets(sLine, 256, &fpr)) 
    {
    	if (2 != flag) { // ����ҵ�Ҫ�޸ĵ���һ�У��򲻻�ִ���ڲ��Ĳ���
    		wTmp = strchr(sLine, '=');
    		if ((NULL != wTmp) && (1 == flag)) {
    			if (0 == strncmp(key, sLine, wTmp-sLine)) { // �������ļ���ȡ��Ϊ׼
    				flag = 2;// ����ֵ������д���ļ�
    				sprintf(wTmp + 1, "%s\n", val);
    			}
    		} else {
    			if (0 == strncmp(sTitle, sLine, strlen(sLine) - 1)) { // �������ļ���ȡ��Ϊ׼
    				flag = 1; // �ҵ�����λ��
    			}
    		}
    	}
    
    	f_puts(sLine, &fpw); // д����ʱ�ļ�
    }
    f_close(&fpr);
    f_close(&fpw);
    
    sprintf(sLine, "%s.tmp", filename);
    return f_rename(sLine, filename);// ����ʱ�ļ����µ�ԭ�ļ�
}

/*
 * ��������			PutIniKeyString
 * ��ڲ�����		title
 *						�����ļ���һ�����ݵı�ʶ
 *					key
 *						����������Ҫ������ֵ�ı�ʶ
 *					val
 *						���ĺ��ֵ
 *					filename
 *						Ҫ��ȡ���ļ�·��
 * ����ֵ��			�ɹ�����  0
 *					���򷵻� -1
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
