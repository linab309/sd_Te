/* USER CODE BEGIN USART1_MspInit 0 */

/* USER CODE END USART1_MspInit 0 */
  /* Peripheral clock enable */
  __HAL_RCC_USART1_CLK_ENABLE();

  /**USART1 GPIO Configuration	  
  PA9	  ------> USART1_TX
  PA10	   ------> USART1_RX 
  */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN USART1_MspInit 1 */

/* USER CODE END USART1_MspInit 1 */

