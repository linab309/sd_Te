#include "gps.h"
#include "stdio.h"
#include "stdarg.h"
#include "string.h"
#include "math.h"
/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"

//////////////////////////////////////////////////////////////////////////////////
//
//////////////////////////////////////////////////////////////////////////////////

//��buf����õ���cx���������ڵ�λ��
//����ֵ:0~0XFE,����������λ�õ�ƫ��.
//       0XFF,�������ڵ�cx������


//static uint8_t NS_LG = 0 ;

uint8_t NMEA_Comma_Pos(uint8_t *buf,uint8_t cx)
{
	uint8_t *p=buf;
	while(cx)
	{
		if(*buf=='*'||*buf<' '||*buf>'z')return 0XFF;//����'*'���߷Ƿ��ַ�,�򲻴��ڵ�cx������
		if(*buf==',')cx--;
		buf++;
	}
	return buf-p;
}
//m^n����
//����ֵ:m^n�η�.
uint32_t NMEA_Pow(uint8_t m,uint8_t n)
{
	uint32_t result=1;
	while(n--)result*=m;
	return result;
}
//strת��Ϊ����,��','����'*'����
//buf:���ִ洢��
//dx:С����λ��,���ظ����ú���
//����ֵ:ת�������ֵ
int NMEA_Str2num(uint8_t *buf,uint8_t*dx)
{
	uint8_t *p=buf;
	uint32_t ires=0,fres=0;
	uint8_t ilen=0,flen=0,i;
	uint8_t mask=0;
	int res;
	while(1) //�õ�������С���ĳ���
	{
		if(*p=='-'){mask|=0X02;p++;}//�Ǹ���
		if(*p==','||(*p=='*')||(*p=='\0'))break;//����������
		if(*p=='.'){mask|=0X01;p++;}//����С������
		else if(*p>'9'||(*p<'0'))	//�зǷ��ַ�
		{
			ilen=0;
			flen=0;
			break;
		}
		if(mask&0X01)flen++;
		else ilen++;
		p++;
	}
	if(mask&0X02)buf++;	//ȥ������
	for(i=0;i<ilen;i++)	//�õ�������������
	{
		ires+=NMEA_Pow(10,ilen-1-i)*(buf[i]-'0');
	}
	if(flen>5) flen=5;	//���ȡ5λС��
	*dx=flen;	 		//С����λ��
	for(i=0;i<flen;i++)	//�õ�С����������
	{
		fres+=NMEA_Pow(10,flen-1-i)*(buf[ilen+1+i]-'0');
	}
	res=ires*NMEA_Pow(10,flen)+fres;
	if(mask&0X02)res=-res;
	return res;
}

//strת��Ϊ����,��','����'*'����
//buf:���ִ洢��
//dx:С����λ��,���ظ����ú���
//����ֵ:ת�������ֵ
uint64_t NMEA_Str2num_64(uint8_t *buf,uint8_t*dx)
{
	uint8_t *p=buf;
	uint64_t ires=0,fres=0;
	uint8_t ilen=0,flen=0,i;
	uint8_t mask=0;
	uint64_t res;
	while(1) //�õ�������С���ĳ���
	{
		if(*p=='-'){mask|=0X02;p++;}//�Ǹ���
		if(*p==','||(*p=='*')||(*p=='\0'))break;//����������
		if(*p=='.'){mask|=0X01;p++;}//����С������
		else if(*p>'9'||(*p<'0'))	//�зǷ��ַ�
		{
			ilen=0;
			flen=0;
			break;
		}
		if(mask&0X01)flen++;
		else ilen++;
		p++;
	}
	if(mask&0X02)buf++;	//ȥ������
	for(i=0;i<ilen;i++)	//�õ�������������
	{
		ires+=NMEA_Pow(10,ilen-1-i)*(buf[i]-'0');
	}
	if(flen>7) flen=7;	//���ȡ5λС��
	*dx=flen;	 		//С����λ��
	for(i=0;i<flen;i++)	//�õ�С����������
	{
		fres+=NMEA_Pow(10,flen-1-i)*(buf[ilen+1+i]-'0');
	}
	res=ires*NMEA_Pow(10,flen)+fres;
	if(mask&0X02)res=-res;
	return res;
}

//����GPGSV��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
void NMEA_GPGSV_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p,*p1,dx;
	uint8_t len,i,j,slx=0;
	uint8_t posx;

	p = buf;
	p1 = (uint8_t*)strstr((const char *)p,"$GPGSV");
	len = p1[7]-'0';								//�õ�GPGSV������
	posx = NMEA_Comma_Pos(p1,3); 					//�õ��ɼ���������
	if(posx != 0XFF)
        gpsx->svnum = NMEA_Str2num(p1+posx,&dx);

	for(i=0;i<len;i++)
	{
		p1=(uint8_t*)strstr((const char *)p,"$GPGSV");
		for(j=0;j<4;j++)
		{
			posx = NMEA_Comma_Pos(p1,4+j*4);

            if(posx != 0XFF)
                gpsx->slmsg[slx].num=NMEA_Str2num(p1+posx,&dx); //�õ����Ǳ��
			else
                break;

			posx = NMEA_Comma_Pos(p1,5+j*4);

			if(posx != 0XFF)
                gpsx->slmsg[slx].eledeg=NMEA_Str2num(p1+posx,&dx);//�õ���������
			else
                break;

			posx = NMEA_Comma_Pos(p1,6+j*4);

			if(posx != 0XFF)
                gpsx->slmsg[slx].azideg=NMEA_Str2num(p1+posx,&dx);//�õ����Ƿ�λ��
			else
                break;

			posx = NMEA_Comma_Pos(p1,7+j*4);

			if(posx != 0XFF)
                gpsx->slmsg[slx].sn=NMEA_Str2num(p1+posx,&dx);  //�õ����������
			else
                break;

			slx++;
		}
 		p = p1+1;//�л�����һ��GPGSV��Ϣ
	}

//	menu_3d_seq();

}
//����GPGGA��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
uint8_t NMEA_GPGGA_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p,*p1,dx;
	uint8_t posx;
  //uint8_t ret = 0;

    p    = buf;
	p1   = (uint8_t*)strstr((const char *)buf,"$GPGGA");
	posx = NMEA_Comma_Pos(p1,6);								//�õ�GPS״̬
	if(posx != 0XFF) gpsx->gpssta = NMEA_Str2num(p1+posx,&dx);
	posx=NMEA_Comma_Pos(p1,7);								//�õ����ڶ�λ��������
	if(posx != 0XFF) gpsx->posslnum = NMEA_Str2num(p1+posx,&dx);
	posx = NMEA_Comma_Pos(p1,8);								//�õ����ڶ�λ��������
	if(posx != 0XFF) gpsx->hdop = NMEA_Str2num(p1+posx,&dx);

	posx = NMEA_Comma_Pos(p1,9);
    //�õ����θ߶�
#ifdef NG_LG_ENABLE
    if(NS_LG > 0)
    {
    	if(posx != 0XFF) gpsx->altitude= 0;//NMEA_Str2num(p1+posx,&dx);
    }
    else
#endif
    {
    	if(posx != 0XFF) gpsx->altitude= NMEA_Str2num(p1+posx,&dx);
    }

#if 1
    p  = p1+1;//�л�����һ��GPGSV��Ϣ
    p1 = (uint8_t*)strstr((const char *)p,"$GPGGA");
    if(p1 != NULL)
    {
        print_usart1("$GPGGA more \r\n");

        return 1;
    }
#endif
    return 0;


}

//����GPGGA��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
uint8_t NMEA_GNGGA_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p,*p1,dx;
	uint8_t posx;

    p = buf;
	p1=(uint8_t*)strstr((const char *)buf,"$GNGGA");
	posx=NMEA_Comma_Pos(p1,6);								//�õ�GPS״̬
	if(posx!=0XFF)gpsx->gpssta=NMEA_Str2num(p1+posx,&dx);
	posx=NMEA_Comma_Pos(p1,7);								//�õ����ڶ�λ��������
	if(posx!=0XFF)gpsx->posslnum=NMEA_Str2num(p1+posx,&dx);
	posx=NMEA_Comma_Pos(p1,8);								//�õ����ڶ�λ��������
	if(posx!=0XFF)gpsx->hdop=NMEA_Str2num(p1+posx,&dx);

	posx=NMEA_Comma_Pos(p1,9);
    //�õ����θ߶�
#ifdef NG_LG_ENABLE
    if(NS_LG > 0)
    {
    	if(posx!=0XFF)gpsx->altitude= 0;//NMEA_Str2num(p1+posx,&dx);
    }
    else
#endif
    {
    	if(posx!=0XFF)gpsx->altitude= NMEA_Str2num(p1+posx,&dx);
    }

    p  = p1+1;//�л�����һ��GPGSV��Ϣ
    p1 = (uint8_t*)strstr((const char *)p,"$GNGGA");
    if(p1 != NULL)
    {
        print_usart1("$GNGGA more \r\n");
        return 1;
    }

    return 0;

}



// void test_float_double_u32_u64(uint8_t *buf)
// {
    		
//     uint32_t longitude = 0;     
//     double   rs_64 = 0;
//     uint64_t temp = 0;
//     uint8_t dx;
//     uint8_t one_shot_buffer[4] = {0};

//     double tp_lon =0.0;
//     float  tp_lon_float = 0.0;

//     temp=NMEA_Str2num_64(buf,&dx);
// 	print_usart1("temp = %d \r\n",temp);
//     longitude = (uint32_t)(temp/NMEA_Pow(10,dx+2));	//�õ���
//     rs_64 = temp%NMEA_Pow(10,dx+2);				//�õ�'
//     print_usart1("rs_64 = %f \r\n",rs_64);
  
//     print_usart1("longitude-1 = %d \r\n",longitude);
//     longitude = longitude*NMEA_Pow(10,7)+(rs_64*NMEA_Pow(10,7-dx))/60;//ת��Ϊ��
//     print_usart1("(rs_64*NMEA_Pow(10,6-dx)) = %f \r\n",(rs_64*NMEA_Pow(10,6-dx)));	
// 	print_usart1("/60 = %f \r\n",(rs_64*NMEA_Pow(10,6-dx))/60);		
// 	print_usart1("/60 = %d \r\n",(rs_64*NMEA_Pow(10,6-dx))/60);	
// 	print_usart1("longitude-2 = %d \r\n",longitude);	


// 	one_shot_buffer[0] = (uint8_t)(longitude>>24)&0xff;  // 1mb
// 	one_shot_buffer[1] = (uint8_t)(longitude>>16)&0xff;  // 2mb
// 	one_shot_buffer[2] = (uint8_t)(longitude>>8)&0xff;  // 3mb
// 	one_shot_buffer[3] = (uint8_t)(longitude)&0xff;  // 4 mb	


//     tp_lon = (one_shot_buffer[3]|(one_shot_buffer[2]<<8)\
// 			 |(one_shot_buffer[1]<<16)|(one_shot_buffer[0]<<24));

//     tp_lon_float = (one_shot_buffer[3]|(one_shot_buffer[2]<<8)\
// 			 |(one_shot_buffer[1]<<16)|(one_shot_buffer[0]<<24));

//     longitude = (one_shot_buffer[3]|(one_shot_buffer[2]<<8)\
// 			 |(one_shot_buffer[1]<<16)|(one_shot_buffer[0]<<24));
//     print_usart1("tp_lon = %f \r\n",tp_lon);
//     print_usart1("tp_lon = %.7f \r\n",tp_lon/10000000);	
// 	print_usart1("tp_lon_float = %f \r\n",tp_lon_float);	
// 	print_usart1("tp_lon_float = %.7f \r\n",tp_lon_float/10000000);				 
// 	print_usart1("longitude = %d \r\n",longitude);				 
// }


//����GPGSA��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
void NMEA_GPGSA_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p1,dx;
	uint8_t posx;
	uint8_t i;
	p1=(uint8_t*)strstr((const char *)buf,"$GPGSA");
	posx=NMEA_Comma_Pos(p1,2);
	//�õ���λ����
#ifdef NG_LG_ENABLE
	if(NS_LG > 0 )
	{
	    if(posx!=0XFF)gpsx->fixmode=3;
	}
	else
#endif
	{
        if(posx!=0XFF)gpsx->fixmode=NMEA_Str2num(p1+posx,&dx);
	}
	for(i=0;i<12;i++)										//�õ���λ���Ǳ��
	{
		posx=NMEA_Comma_Pos(p1,3+i);
		if(posx!=0XFF)gpsx->possl[i]=NMEA_Str2num(p1+posx,&dx);
		else break;
	}
	posx=NMEA_Comma_Pos(p1,15);								//�õ�PDOPλ�þ�������
	if(posx!=0XFF)gpsx->pdop=NMEA_Str2num(p1+posx,&dx);
	posx=NMEA_Comma_Pos(p1,16);								//�õ�HDOPλ�þ�������
	if(posx!=0XFF)gpsx->hdop=NMEA_Str2num(p1+posx,&dx);
	posx=NMEA_Comma_Pos(p1,17);								//�õ�VDOPλ�þ�������
	if(posx!=0XFF)gpsx->vdop=NMEA_Str2num(p1+posx,&dx);
}
//����GPRMC��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
uint8_t NMEA_GPRMC_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p,*p1,dx;
	uint8_t posx;
	uint32_t temp;
	float rs;

    p = buf;
	p1=(uint8_t*)strstr((const char *)buf,"$GPRMC");
	posx=NMEA_Comma_Pos(p1,1);								//�õ�UTCʱ��
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx)/NMEA_Pow(10,dx);	 	//�õ�UTCʱ��,ȥ��ms
		gpsx->utc.hour=temp/10000;
		gpsx->utc.min=(temp/100)%100;
		gpsx->utc.sec=temp%100;
	}
	posx=NMEA_Comma_Pos(p1,3);								//�õ�γ��
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);
#ifdef NG_LG_ENABLE

        if(NS_LG > 0)
        {
           gpsx->latitude= 45;//temp/NMEA_Pow(10,dx+2);	//�õ���
           rs= 0 ;//temp%NMEA_Pow(10,dx+2);
        }

		else
#endif

		{
		    gpsx->latitude=temp/NMEA_Pow(10,dx+2);	//�õ���
		    rs= temp%NMEA_Pow(10,dx+2);
		}//�õ�'
		gpsx->latitude=gpsx->latitude*NMEA_Pow(10,6)+(rs*NMEA_Pow(10,6-dx))/60;//ת��Ϊ��

//		gpsx->latitude = temp;
	}
	posx=NMEA_Comma_Pos(p1,4);
	//��γ���Ǳ�γ
	if(posx!=0XFF)
    {
#ifdef NG_LG_ENABLE
        if((NS_LG == 1)||(NS_LG == 3))
            gpsx->nshemi='N';
        else if((NS_LG == 2)||(NS_LG == 4))
            gpsx->nshemi='S';//*(p1+posx);
        else
#endif
        gpsx->nshemi=*(p1+posx);
	}
 	posx=NMEA_Comma_Pos(p1,5);								//�õ�����
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);
#ifdef NG_LG_ENABLE


        if(NS_LG > 0)
        {
    		gpsx->longitude = 90;//temp/NMEA_Pow(10,dx+2);	//�õ���
    		rs= 0 ;//temp%NMEA_Pow(10,dx+2);
		}

		else
#endif

		{
    		gpsx->longitude=temp/NMEA_Pow(10,dx+2);	//�õ���
    		rs=temp%NMEA_Pow(10,dx+2);				//�õ�'
		}
		gpsx->longitude=gpsx->longitude*NMEA_Pow(10,6)+(rs*NMEA_Pow(10,6-dx))/60;//ת��Ϊ��

//		gpsx->longitude = temp;
	}
	posx=NMEA_Comma_Pos(p1,6);								//������������
	if(posx!=0XFF)
	{

#ifdef NG_LG_ENABLE
	     if((NS_LG == 1)||(NS_LG == 2))
	        gpsx->ewhemi='E';
	     else if((NS_LG == 3)||(NS_LG == 4))
  	         gpsx->ewhemi='W';//*(p1+posx);
  	     else
#endif
  	         gpsx->ewhemi=*(p1+posx);

	 }

	posx = NMEA_Comma_Pos(p1,7);								//�õ���������

	if(posx!=0XFF)
	{
        gpsx->speed = NMEA_Str2num(p1+posx,&dx);

        if(dx<3)
        {
            gpsx->speed*=NMEA_Pow(10,3-dx);             //ȷ������1000��
            gpsx->speed = gpsx->speed*1852/1000;             //ȷ������1000��

        }
    }
    posx = NMEA_Comma_Pos(p1,8);
    if(posx!=0XFF)
    {

#ifdef NG_LG_ENABLE
        if(NS_LG > 0)
        {
            gpsx->angle= 0;//NMEA_Str2num(p1+posx,&dx);
        }
        else
#endif
        {
            gpsx->angle= NMEA_Str2num(p1+posx,&dx);

        }
        if(dx<3)gpsx->angle*=NMEA_Pow(10,3-dx);             //ȷ������1000��

    }

#if 1
	posx=NMEA_Comma_Pos(p1,9);								//�õ�UTC����
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);		 				//�õ�UTC����
		gpsx->utc.date=temp/10000;
		gpsx->utc.month=(temp/100)%100;
		gpsx->utc.year=2000+temp%100;

	    //v1000_debug("\r\n%d-%d-%d    %d-%d-%d\r\n",gpsx->utc.year,gpsx->utc.month,gpsx->utc.date,gpsx->utc.hour,gpsx->utc.min,gpsx->utc.sec);



	}
#endif


#if 1
    p  = p1+1;//�л�����һ��GPGSV��Ϣ
    p1 = (uint8_t*)strstr((const char *)p,"$GPRMC");
    if(p1 != NULL)
    {
        print_usart1("$GPRMC more \r\n");
        return 1;
    }
#endif

    return 0;
}

//����GPRMC��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
uint8_t NMEA_GNRMC_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p,*p1,dx;
	uint8_t posx;
	uint64_t temp;
	float rs;
	double rs_64;

    p = buf;
	p1=(uint8_t*)strstr((const char *)buf,"$GNRMC");
	posx=NMEA_Comma_Pos(p1,1);								//�õ�UTCʱ��
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx)/NMEA_Pow(10,dx);	 	//�õ�UTCʱ��,ȥ��ms
		gpsx->utc.hour=temp/10000;
		gpsx->utc.min=(temp/100)%100;
		gpsx->utc.sec=temp%100;
	}
	posx=NMEA_Comma_Pos(p1,3);								//�õ�γ��
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num_64(p1+posx,&dx);
		// print_usart1("temp = %d \r\n",temp);
		// print_usart1("dx = %d \r\n",dx);
#ifdef NG_LG_ENABLE

        if(NS_LG > 0)
        {
           gpsx->latitude= 45;//temp/NMEA_Pow(10,dx+2);	//�õ���
           rs= 0 ;//temp%NMEA_Pow(10,dx+2);
        }

		else
#endif

		{
		    gpsx->latitude=(uint32_t)(temp/NMEA_Pow(10,dx+2));	//�õ���
		    rs= temp%NMEA_Pow(10,dx+2);
		}//�õ�'
		gpsx->latitude=gpsx->latitude*NMEA_Pow(10,7)+(rs*NMEA_Pow(10,7-dx))/60;//ת��Ϊ��

//		gpsx->latitude = temp;
	}
	posx=NMEA_Comma_Pos(p1,4);
	//��γ���Ǳ�γ
	if(posx!=0XFF)
    {
#ifdef NG_LG_ENABLE
        if((NS_LG == 1)||(NS_LG == 3))
            gpsx->nshemi='N';
        else if((NS_LG == 2)||(NS_LG == 4))
            gpsx->nshemi='S';//*(p1+posx);
        else
#endif
            gpsx->nshemi=*(p1+posx);
	}
 	posx=NMEA_Comma_Pos(p1,5);								//�õ�����
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num_64(p1+posx,&dx);
#ifdef NG_LG_ENABLE


        if(NS_LG > 0)
        {
    		gpsx->longitude = 90;//temp/NMEA_Pow(10,dx+2);	//�õ���
    		rs= 0 ;//temp%NMEA_Pow(10,dx+2);
		}

		else
#endif

		{
    		gpsx->longitude=(uint32_t)(temp/NMEA_Pow(10,dx+2));	//�õ���
    		rs_64 = temp%NMEA_Pow(10,dx+2);				//�õ�'
			//print_usart1("rs_64 = %f \r\n",rs_64);
			//rs = (float)rs_64;
			//print_usart1("rs = %f \r\n",rs);
		}
		gpsx->longitude=gpsx->longitude*NMEA_Pow(10,7)+(rs_64*NMEA_Pow(10,7-dx))/60;//ת��Ϊ��

//		gpsx->longitude = temp;
	}
	// print_usart1("gpsx->latitude = %d \r\n",gpsx->latitude);
	// print_usart1("gpsx->longitude = %d \r\n",gpsx->longitude);
	posx=NMEA_Comma_Pos(p1,6);								//������������
	if(posx!=0XFF)
	{

#ifdef NG_LG_ENABLE
	     if((NS_LG == 1)||(NS_LG == 2))
	        gpsx->ewhemi='E';
	     else if((NS_LG == 3)||(NS_LG == 4))
  	         gpsx->ewhemi='W';//*(p1+posx);
  	     else
#endif
  	         gpsx->ewhemi=*(p1+posx);

	 }

	posx = NMEA_Comma_Pos(p1,7);								//�õ���������

	if(posx!=0XFF)
	{
        gpsx->speed = NMEA_Str2num(p1+posx,&dx);

        if(dx<3)
        {
            gpsx->speed*=NMEA_Pow(10,3-dx);             //ȷ������1000��
            gpsx->speed = gpsx->speed*1852/1000;             //ȷ������1000��

        }
    }
    posx = NMEA_Comma_Pos(p1,8);
    if(posx!=0XFF)
    {

#ifdef NG_LG_ENABLE
        if(NS_LG > 0)
        {
            gpsx->angle= 0;//NMEA_Str2num(p1+posx,&dx);
        }
        else
#endif
        {
            gpsx->angle= NMEA_Str2num(p1+posx,&dx);

        }
        if(dx<3)gpsx->angle*=NMEA_Pow(10,3-dx);             //ȷ������1000��

    }

#if 1
	posx=NMEA_Comma_Pos(p1,9);								//�õ�UTC����
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);		 				//�õ�UTC����
		gpsx->utc.date=temp/10000;
		gpsx->utc.month=(temp/100)%100;
		gpsx->utc.year=2000+temp%100;
	}
#endif

    p  = p1+1;//�л�����һ��GPGSV��Ϣ
    p1 = (uint8_t*)strstr((const char *)p,"$GNRMC");
    if(p1 != NULL)
    {
        print_usart1("GNRMC more \r\n");
        return 1;
    }

    return 0;

}


//����GPVTG��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
void NMEA_GPVTG_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p1,dx;
	uint8_t posx;
	p1=(uint8_t*)strstr((const char *)buf,"$GPVTG");
	posx=NMEA_Comma_Pos(p1,7);								//�õ���������
	if(posx!=0XFF)
	{
#ifdef NG_LG_ENABLE
         if(NS_LG > 0)
         {
	       	gpsx->speed= 100;//NMEA_Str2num(p1+posx,&dx);
         }
         else
#endif
          	gpsx->speed= NMEA_Str2num(p1+posx,&dx);
		if(dx<3)gpsx->speed*=NMEA_Pow(10,3-dx);	 	 		//ȷ������1000��

	}

	posx=NMEA_Comma_Pos(p1,1);								//�õ���������
	if(posx!=0XFF)
	{

#ifdef NG_LG_ENABLE
        if(NS_LG > 0)
        {
            gpsx->angle= 0;//NMEA_Str2num(p1+posx,&dx);
		}
		else
#endif
		{
		    gpsx->angle= NMEA_Str2num(p1+posx,&dx);

		}
		if(dx<3)gpsx->angle*=NMEA_Pow(10,3-dx);	 	 		//ȷ������1000��

	}

}
//��ȡNMEA-0183��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
uint8_t GPS_Analysis(nmea_msg *gpsx,uint8_t *buf)
{

    uint8_t ret = 0 ;

	//NMEA_GPGSV_Analysis(gpsx,buf);	//GPGSV����
	ret = NMEA_GPGGA_Analysis(gpsx,buf);	//GPGGA����
	//NMEA_GPGSA_Analysis(gpsx,buf);	//GPGSA����
	ret = NMEA_GPRMC_Analysis(gpsx,buf);	//GPRMC����
	//NMEA_GPVTG_Analysis(gpsx,buf);	//GPVTG����
	ret = NMEA_GNGGA_Analysis(gpsx,buf);	//GnGGA����
	ret = NMEA_GNRMC_Analysis(gpsx,buf);	//GnRMC����

    return ret  ;
}
#if 0

//GPSУ��ͼ���
//buf:���ݻ������׵�ַ
//len:���ݳ���
//cka,ckb:����У����.
void Ublox_CheckSum(uint8_t *buf,uint16_t len,uint8_t* cka,uint8_t*ckb)
{
	uint16_t i;
	*cka=0;*ckb=0;
	for(i=0;i<len;i++)
	{
		*cka=*cka+buf[i];
		*ckb=*ckb+*cka;
	}
}


//����UBLOX NEO-6��ʱ���������
//interval:������
//length:������
//status:��������:1,�ߵ�ƽ��Ч;0,�ر�;-1,�͵�ƽ��Ч.
void Ublox_Cfg_Tp(uint32_t interval,uint32_t length,signed char status)
{
	_ublox_cfg_tp *cfg_tp=(_ublox_cfg_tp *)USART2_TX_BUF;
	cfg_tp->header=0X62B5;		//cfg header
	cfg_tp->id=0X0706;			//cfg tp id
	cfg_tp->dlength=20;			//����������Ϊ20���ֽ�.
	cfg_tp->interval=interval;	//������,us
	cfg_tp->length=length;		//������,us
	cfg_tp->status=status;	   	//ʱ����������
	cfg_tp->timeref=0;			//�ο�UTC ʱ��
	cfg_tp->flags=0;			//flagsΪ0
	cfg_tp->reserved=0;		 	//����λΪ0
	cfg_tp->antdelay=820;    	//������ʱΪ820ns
	cfg_tp->rfdelay=0;    		//RF��ʱΪ0ns
	cfg_tp->userdelay=0;    	//�û���ʱΪ0ns
	Ublox_CheckSum((uint8_t*)(&cfg_tp->id),sizeof(_ublox_cfg_tp)-4,&cfg_tp->cka,&cfg_tp->ckb);
	while(DMA1_Channel7->CNDTR!=0);	//�ȴ�ͨ��7�������
	UART_DMA_Enable(DMA1_Channel7,sizeof(_ublox_cfg_tp));	//ͨ��dma���ͳ�ȥ
}
//����UBLOX NEO-6�ĸ�������
//measrate:����ʱ��������λΪms�����ٲ���С��200ms��5Hz��
//reftime:�ο�ʱ�䣬0=UTC Time��1=GPS Time��һ������Ϊ1��
void Ublox_Cfg_Rate(uint16_t measrate,uint8_t reftime)
{
	_ublox_cfg_rate *cfg_rate=(_ublox_cfg_rate *)USART2_TX_BUF;
 	if(measrate<200)return;		//С��200ms��ֱ���˳�
 	cfg_rate->header=0X62B5;	//cfg header
	cfg_rate->id=0X0806;	 	//cfg rate id
	cfg_rate->dlength=6;	 	//����������Ϊ6���ֽ�.
	cfg_rate->measrate=measrate;//������,us
	cfg_rate->navrate=1;		//�������ʣ����ڣ����̶�Ϊ1
	cfg_rate->timeref=reftime; 	//�ο�ʱ��ΪGPSʱ��
	Ublox_CheckSum((uint8_t*)(&cfg_rate->id),sizeof(_ublox_cfg_rate)-4,&cfg_rate->cka,&cfg_rate->ckb);
	while(DMA1_Channel7->CNDTR!=0);	//�ȴ�ͨ��7�������
	UART_DMA_Enable(DMA1_Channel7,sizeof(_ublox_cfg_rate));//ͨ��dma���ͳ�ȥ
}

#endif































