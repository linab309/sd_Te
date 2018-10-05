 #ifndef _MMC_SD_H_
#define _MMC_SD_H_		 
	 

 						    	 
// SD�����Ͷ���  
#define SD_TYPE_ERR     0X00
#define SD_TYPE_MMC     0X01
#define SD_TYPE_V1      0X02
#define SD_TYPE_V2      0X04
#define SD_TYPE_V2HC    0X06	   
// SD��ָ���  	   
#define CMD0    0       //����λ
#define CMD1    1
#define CMD8    8       //����8 ��SEND_IF_COND
#define CMD9    9       //����9 ����CSD����
#define CMD10   10      //����10����CID����
#define CMD12   12      //����12��ֹͣ���ݴ���
#define CMD16   16      //����16������SectorSize Ӧ����0x00
#define CMD17   17      //����17����sector
#define CMD18   18      //����18����Multi sector
#define CMD23   23      //����23�����ö�sectorд��ǰԤ�Ȳ���N��block
#define CMD24   24      //����24��дsector
#define CMD25   25      //����25��дMulti sector
#define CMD41   41      //����41��Ӧ����0x00
#define CMD55   55      //����55��Ӧ����0x01
#define CMD58   58      //����58����OCR��Ϣ
#define CMD59   59      //����59��ʹ��/��ֹCRC��Ӧ����0x00
//����д���Ӧ������
#define MSD_DATA_OK                0x05
#define MSD_DATA_CRC_ERROR         0x0B
#define MSD_DATA_WRITE_ERROR       0x0D
#define MSD_DATA_OTHER_ERROR       0xFF
//SD����Ӧ�����
#define MSD_RESPONSE_NO_ERROR      0x00
#define MSD_IN_IDLE_STATE          0x01
#define MSD_ERASE_RESET            0x02
#define MSD_ILLEGAL_COMMAND        0x04
#define MSD_COM_CRC_ERROR          0x08
#define MSD_ERASE_SEQUENCE_ERROR   0x10
#define MSD_ADDRESS_ERROR          0x20
#define MSD_PARAMETER_ERROR        0x40
#define MSD_RESPONSE_FAILURE       0xFF


/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the I2C communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */   
#define EVAL_I2Cx_TIMEOUT_MAX                   3000

/*##################### SPI2 ###################################*/
#define EVAL_SPIx                               SPI1
#define EVAL_SPIx_CLK_ENABLE()                  __HAL_RCC_SPI1_CLK_ENABLE()


#define EVAL_SPIx_CS_GPIO_PORT                 GPIOA             /* PA.02*/
#define EVAL_SPIx_CS_PIN                       GPIO_PIN_3
#define EVAL_SPIx_CS_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOA_CLK_ENABLE()
#define EVAL_SPIx_CS_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOA_CLK_DISABLE()


#define EVAL_SPIx_SCK_AF                        GPIO_AF5_SPI1
#define EVAL_SPIx_SCK_GPIO_PORT                 GPIOA             /* PA.05*/
#define EVAL_SPIx_SCK_PIN                       GPIO_PIN_5
#define EVAL_SPIx_SCK_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOA_CLK_ENABLE()
#define EVAL_SPIx_SCK_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOA_CLK_DISABLE()

#define EVAL_SPIx_MISO_MOSI_AF                  GPIO_AF5_SPI1
#define EVAL_SPIx_MISO_MOSI_GPIO_PORT           GPIOA
#define EVAL_SPIx_MISO_MOSI_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOA_CLK_ENABLE()
#define EVAL_SPIx_MISO_MOSI_GPIO_CLK_DISABLE()  __HAL_RCC_GPIOA_CLK_DISABLE()
#define EVAL_SPIx_MISO_PIN                      GPIO_PIN_6       /* PE.14*/
#define EVAL_SPIx_MOSI_PIN                      GPIO_PIN_7       /* PE.15*/
/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the SPI communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */   
#define EVAL_SPIx_TIMEOUT_MAX                   1000




/** 
  * @brief  SD status structure definition  
  */     
#define MSD_OK         0x00
#define MSD_ERROR      0x01

/**
  * @}
  */


/* Exported constants --------------------------------------------------------*/  
 
/** @defgroup STM324x9I_EVAL_SD_Exported_Constants Exported Constants
  * @{
  */ 
#define SD_DETECT_PIN                    GPIO_PIN_7
#define SD_DETECT_GPIO_PORT              GPIOC
#define __SD_DETECT_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOC_CLK_ENABLE()
#define SD_DETECT_IRQn                   EXTI9_5_IRQn
   
#define SD_DATATIMEOUT           ((uint32_t)0x10000000)
    
#define SD_PRESENT               ((uint8_t)0x01)
#define SD_NOT_PRESENT           ((uint8_t)0x00)
   
/* DMA definitions for SD DMA transfer */
#define __DMAx_TxRx_CLK_ENABLE            __HAL_RCC_DMA2_CLK_ENABLE
#define SD_DMAx_Tx_STREAM                 DMA2_Channel4  
#define SD_DMAx_Rx_STREAM                 DMA2_Channel4  
#define SD_DMAx_Tx_IRQn                   DMA2_Channel4_IRQn
#define SD_DMAx_Rx_IRQn                   DMA2_Channel4_IRQn
#define SD_DMAx_Tx_IRQHandler             DMA2_Channel4_IRQHandler
#define SD_DMAx_Rx_IRQHandler             DMA2_Channel4_IRQHandler
#define SD_DetectIRQHandler()             HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7)

/**
  * @}
  */
  


//�ⲿ��Ӧ���ݾ�����������޸�!
//Mini STM32ʹ�õ���PA3��ΪSD����CS��.
/**
  * @brief  SD Control Lines management
  */  
#define SD_CS_LOW()       HAL_GPIO_WritePin(EVAL_SPIx_CS_GPIO_PORT, EVAL_SPIx_CS_PIN, GPIO_PIN_RESET)
#define SD_CS_HIGH()      HAL_GPIO_WritePin(EVAL_SPIx_CS_GPIO_PORT, EVAL_SPIx_CS_PIN, GPIO_PIN_SET)				    	  

extern uint8_t  SD_Type;//SD��������
//���������� 
uint8_t SD_WaitReady(void);							//�ȴ�SD��׼��
uint8_t SD_GetResponse(uint8_t Response);					//�����Ӧ
uint8_t SD_Initialize(void);							//��ʼ��
uint8_t SD_ReadDisk(uint8_t*buf,uint32_t sector,uint8_t cnt);		//����
uint8_t SD_WriteDisk(uint8_t*buf,uint32_t sector,uint8_t cnt);		//д��
uint32_t SD_GetSectorCount(void);   					//��������
uint8_t SD_GetCID(uint8_t *cid_data);                     //��SD��CID
uint8_t SD_GetCSD(uint8_t *csd_data);                     //��SD��CSD
 
#endif





