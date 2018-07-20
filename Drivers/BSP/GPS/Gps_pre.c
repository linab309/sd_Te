
//---------------------------------------------------------------------------
NMEA_STN_DATA_T rRawData;     //Output Sentence


NMEA_STN_T m_eLastDecodedSTN;

//---------------------------------------------------------------------------
void DetermineStnType()
{
  if ( (strncmp(&rRawData.Data[0], "$GPGGA", 6) == 0) || 
  	   (strncmp(&rRawData.Data[0], "$GNGGA", 6) == 0) || 
  	   (strncmp(&rRawData.Data[0], "$BDGGA", 6) == 0) || 
  	   (strncmp(&rRawData.Data[0], "$GLGGA", 6) == 0) || 
  	   (strncmp(&rRawData.Data[0], "$GBGGA", 6) == 0) )
  {
    rRawData.eType = STN_GGA;
  }
  else if ( (strncmp(&rRawData.Data[0], "$GPGLL", 6) == 0) || 
  	 		(strncmp(&rRawData.Data[0], "$GNGLL", 6) == 0) || 
  	 		(strncmp(&rRawData.Data[0], "$BDGLL", 6) == 0) || 
  	 		(strncmp(&rRawData.Data[0], "$GLGLL", 6) == 0) || 
  	 		(strncmp(&rRawData.Data[0], "$GBGLL", 6) == 0) )
  {
    rRawData.eType = STN_GLL;
  }
  else if (	(strncmp(&rRawData.Data[0], "$GPGSA", 6) == 0) || 
  			(strncmp(&rRawData.Data[0], "$GNGSA", 6) == 0) || 
  			(strncmp(&rRawData.Data[0], "$BDGSA", 6) == 0) || 
  			(strncmp(&rRawData.Data[0], "$GLGSA", 6) == 0) || 
  			(strncmp(&rRawData.Data[0], "$GAGSA", 6) == 0) || 
  			(strncmp(&rRawData.Data[0], "$GBGSA", 6) == 0) )
  {
    rRawData.eType = STN_GSA;
  }
  else if (	(strncmp(&rRawData.Data[0], "$GPQSA", 6) == 0) || 
  			(strncmp(&rRawData.Data[0], "$QZQSA", 6) == 0) ||
  			(strncmp(&rRawData.Data[0], "$GBQSA", 6) == 0) )
  {
    rRawData.eType = STN_QSA;
  }
  else if ( (strncmp(&rRawData.Data[0], "$GPGSV", 6) == 0) || 
  			(strncmp(&rRawData.Data[0], "$QZGSV", 6) == 0) || 
  			(strncmp(&rRawData.Data[0], "$GLGSV", 6) == 0) || 
  			(strncmp(&rRawData.Data[0], "$GAGSV", 6) == 0) || 
  			(strncmp(&rRawData.Data[0], "$BDGSV", 6) == 0) || 
  			(strncmp(&rRawData.Data[0], "$GBGSV", 6) == 0) )
  {
    rRawData.eType = STN_GSV;
  }
  else if ( (strncmp(&rRawData.Data[0], "$GPRMC", 6) == 0) || 
  			(strncmp(&rRawData.Data[0], "$GNRMC", 6) == 0) || 
  			(strncmp(&rRawData.Data[0], "$BDRMC", 6) == 0) || 
  			(strncmp(&rRawData.Data[0], "$GLRMC", 6) == 0) ||
  			(strncmp(&rRawData.Data[0], "$GBRMC", 6) == 0) )
  {
    rRawData.eType = STN_RMC;
  }
  else if (	(strncmp(&rRawData.Data[0], "$GPVTG", 6) == 0) || 
  			(strncmp(&rRawData.Data[0], "$GNVTG", 6) == 0) || 
  			(strncmp(&rRawData.Data[0], "$BDVTG", 6) == 0) || 
  			(strncmp(&rRawData.Data[0], "$GLVTG", 6) == 0) ||
  			(strncmp(&rRawData.Data[0], "$GBVTG", 6) == 0) )
  {
    rRawData.eType = STN_VTG;
  }
  else
  {
    rRawData.eType = STN_OTHER;
  }
}

GBOOL fgNmeaCheckSum(GCHAR* pData, GINT32 i4Size)
{
    GINT32 i;
    int limit;
    GUCHAR chksum = 0, chksum2 = 0;

    if (i4Size < 6)
    {
        return false;
    }

    chksum = pData[1];
    limit = i4Size - 2;
    for (i = 2; i < (limit); i++)
    {
      if (pData[i] != '*')
      {
        chksum ^= pData[i];

        // Exclude invalid NMEA characters
        if (pData[i] & 0x80)
        {
          return false;
        }
      }
      else
      {
        if (pData[i + 1] >= 'A')
        {
          //chksum2 = (pData[i+1]-'A'+10)<<4;
          chksum2 = (pData[i+1]-55)<<4;
        }
        else
        {
          chksum2 = (pData[i+1]-'0')<<4;
        }
        if (pData[i + 2] >= 'A')
        {
          //chksum2 += pData[i+2]-'A'+10;
          chksum2 += pData[i+2]-55;
        }
        else
        {
          chksum2 += pData[i+2]-'0';
        }
        break;
      }
    }

    // if not found character '*'
    if (i >= (i4Size - 2))
    {
      return (false);
    }

    if (chksum == chksum2)
    {
      return (true);
    }
    else
    {
      return (false);
    }
}



/*
            memset(&rSGPGGA, 0, sizeof(NMEA_GGA_T));
            fgParserResult = fgNmeaGPGGAParser(rRawData.Data, &rSGPGGA);
            if (fgParserResult)
            {
              ;//fgHasValidGGA = true;
            }
	ret = NMEA_GPGGA_Analysis(gpsx,buf);	//GPGGA解析 	
	//NMEA_GPGSA_Analysis(gpsx,buf);	//GPGSA解析
	ret = NMEA_GPRMC_Analysis(gpsx,buf);	//GPRMC解析
	//NMEA_GPVTG_Analysis(gpsx,buf);	//GPVTG解析
	ret = NMEA_GNGGA_Analysis(gpsx,buf);	//GnGGA解析 	
	ret = NMEA_GNRMC_Analysis(gpsx,buf);	//GnRMC解析            
*/
//---------------------------------------------------------------------------
void ProcNmeaSentence(nmea_msg *Proc_gpsx)
{
	BOOL fgParserResult;
    BOOL fgValidPkt;

    fgValidPkt = fgNmeaCheckSum(rRawData.Data, rRawData.i2PacketSize - 1);

	if (fgValidPkt)
    {
      // Determine sentence type
      DetermineStnType();

      // Decode NMEA sentence

      if (rRawData.eType == STN_GGA)
      {
          fgParserResult = NMEA_GPGGA_Analysis(Proc_gpsx,rRawData.Data);
          fgParserResult = NMEA_GNGGA_Analysis(Proc_gpsx,rRawData.Data);
      }

      else if (rRawData.eType == STN_GLL)
      {
      }

      else if (rRawData.eType == STN_GSA)
      {

      }

      else if (rRawData.eType == STN_QSA)
      {

      }

      else if (rRawData.eType == STN_GSV)
      {

      }

      else if (rRawData.eType == STN_RMC)
      {

          fgParserResult = NMEA_GPRMC_Analysis(Proc_gpsx,rRawData.Data);
          fgParserResult = NMEA_GNRMC_Analysis(Proc_gpsx,rRawData.Data);
      }

      else if (rRawData.eType == STN_VTG)
      {

      }

      else
      {
          fgParserResult = false;
          rRawData.eType = STN_OTHER;
      }

      m_eLastDecodedSTN = rRawData.eType;
   }

    
}


// 1，初始化串口接收
void gps_pre_init(void )
{
	rxp_init_pcrx();

}


// 2，rxp_init_pcrx()初始化之后把rxp_pcrx_nmea()作为回调函数注册给串口接收程序

// 3，解析rxp_pcrx_nmea()存下来的NMEA语句
void gps_pre_proc(nmea_msg *Proc_gpsx)
{

	// 判断rxp_init_pcrx()是否解析到可用的NMEA语句
	while (rxp_inst_avail(&rRawData.i2PacketType, &i2DataIdx, &m_i2PktDataSize))
 	{
 		// 把整条NMEA语句拷贝到rRawData.Data[]
		rxp_get_inst(i2DataIdx, m_i2PktDataSize, &rRawData.Data[0]);
                    
		/* we don't need <CR>, replace it with string ending symbol */
		rRawData.Data[m_i2PktDataSize - 1] = 0x00;  
		rRawData.i2PacketSize = m_i2PktDataSize;
        print_usart1("m_i2PktDataSize:%d\r\n",rRawData.i2PacketSize);
        print_usart1("%s",rRawData.Data);
		// 解析NMEA语句
		//ProcNmeaSentence(Proc_gpsx);
	}
}
