/**
  ******************************************************************************
  * @file    ADC/ADC1_IDDmeas/main.c
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    13-April-2012
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define EVAL_RESISTOR_RATIO    100    /* R36 is multiplied by 100 */
#define EVAL_MAX9938_GAIN      50     /* Ampli-op gain = 50 */
#define ADC_CONVERT_RATIO      732    /* (3300mV / 4095)* 1000 */



extern ADC_HandleTypeDef hadc;


//	ncl = (res == FR_DISK_ERR) ? 0xFFFFFFFF : 1;
static bool IsBatteryPoweroff(uint16_t mv)
{
    return (mv<=3200)? TRUE:FALSE;
}

static bool IsBatteryLow(uint16_t mv) {return mv>3400 ? TRUE:FALSE;}
/* Check INTERNAL value for shut off battery level  - around 3.00V */
static bool IsBatteryDead(uint16_t mv) {return mv>=3200 ? TRUE:FALSE;}
/* battery full is indicated in firmware for this chip*/
static bool IsBatteryMid(uint16_t mv) {return mv>3550 ? TRUE:FALSE;}
static bool IsBatteryHIGH(uint16_t mv) {return mv>3850 ? TRUE:FALSE;}
static bool IsBatteryFull(uint16_t mv) {return mv>=4100 ? TRUE:FALSE;}

/**
  * @brief  Display the IDD measured Value On the LCD Glass.
  * @param  IDD measure
  * @retval None
  */

void DisplayIDDrunmV(uint32_t IDDmeas)
{
    static int ddrunmv = 0;
    /* x  current value*/
    static u8 ddrunmv_cnt = 0;

    ddrunmv += IDDmeas;
    if(ddrunmv_cnt<10)
    {
        ddrunmv_cnt++;
        return;
    }
    else
    {
        ddrunmv = ddrunmv/11;
        ddrunmv_cnt = 0;

    }
    //v1000_debug("IDDRUNMV: %d  \r\n",ddrunmv);
    if(IsBatteryPoweroff(ddrunmv))
    {
        //  headsetPowerOff(getApp());system_flag_table->batt_Status
        system_flag_table->batt_Status = 0xFF;
    }

    else
    {

        if (IsBatteryFull(ddrunmv))
        {
            if(system_flag_table->charger_connected == 1)
            {
                if(system_flag_table->batt_change_ok_cnt == 0)
                {
                    system_flag_table->batt_change_ok_cnt = 30*60*100;//ms
                    //system_flag_table->batt_Status  =  BATT_CHARG_OK;
                }
            }
            else
            {
                system_flag_table->batt_Status  = BATT_HIGH;
            }

        }

        else if (IsBatteryHIGH(ddrunmv))
        {
            if((system_flag_table->charger_connected == 1)&&(system_flag_table->batt_Status  ==  BATT_CHARG_OK))
                ;
            else
                system_flag_table->batt_Status  =  BATT_HIGH;

        }
        else if (IsBatteryMid(ddrunmv))
        {
            system_flag_table->batt_Status  =  BATT_MID;
        }
        else  if (IsBatteryLow(ddrunmv))
        {
            system_flag_table->batt_Status  =  BATT_LOW;
        }
        else if (IsBatteryDead(ddrunmv))
        {
            system_flag_table->batt_Status  = BATT_EMPTY;
        }

    }

    ddrunmv = 0;

}



/**
  * @brief   Main program
  * @param  None
  * @retval None
  */
void vddmv_adc_proess(void)
{
  /*!< At this stage the microcontroller clock setting is already configured,
       this is done through SystemInit() function which is called from startup
       file (startup_stm32l1xx_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32l1xx.c file
     */
    /* Read ADC conversion result */
    __IO uint16_t ADCdata = 0;
    __IO uint32_t VDDmV = 0;

    /* Start the conversion process */
     HAL_ADC_Start(&hadc);

     /* Wait for the end of conversion */
     if (HAL_ADC_PollForConversion(&hadc, 10) != HAL_TIMEOUT)
     {
         /* Get the converted value of regular channel */
         ADCdata = HAL_ADC_GetValue(&hadc);
     }

    /* Calculate voltage value*/
    VDDmV = (uint32_t)((uint32_t)ADCdata *6000/4095);


    /* Display the IDD measured Value On the LCD Glass (mA) */
    DisplayIDDrunmV(VDDmV);

}

