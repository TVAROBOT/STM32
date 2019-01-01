
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "string.h"
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */
/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/


CAN_FilterConfTypeDef sFilterConfig;
CanTxMsgTypeDef TxMessage;
CanRxMsgTypeDef RxMessage;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CAN_Init(void);
void Serialprintln(char _out[]);
void newline(void);
HAL_StatusTypeDef T_HAL_CAN_Transmit(CAN_HandleTypeDef* hcan, uint32_t Timeout);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
	Serialprintln("Welcome");
	Serialprintln("UART and GPIO is initiated");
	Serialprintln("Clock configured ");
	Serialprintln("HAL initiated");
	Serialprintln("Going to try to initiate MX_CAN");
	Serialprintln("INITIALISING CAN BUS NOW");
  MX_CAN_Init();
	HAL_Delay(500);
	Serialprintln("Setting the Messages and perameters");
	Serialprintln("Starting with sFilterConfig");	
	sFilterConfig.FilterNumber = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.BankNumber = 14;
	
	Serialprintln("Filter config was done now to link to hcan");
	
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
	
	Serialprintln("All linked up to hcan");
	
	Serialprintln("Now for the Tx side of it");
	
	Serialprintln("Setting up the TxMessage");
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.StdId = 0xF1;
	TxMessage.RTR = CAN_RTR_DATA;
	
	TxMessage.DLC = 8;
	TxMessage.Data[0] = 0x00;
	TxMessage.Data[1] = 0x01;
	TxMessage.Data[2] = 0x02;
	TxMessage.Data[3] = 0x03;
	TxMessage.Data[4] = 0x04;
	TxMessage.Data[5] = 0x05;
	TxMessage.Data[6] = 0x06;
	TxMessage.Data[7] = 0x07;
	
	Serialprintln("Message data configured");
	Serialprintln("Linking it to the hcan ");
	/* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_Delay(500);
  while (1)
  {
		Serialprintln("In the loop now");
		Serialprintln("Trying to send a message");
		T_HAL_CAN_Transmit(&hcan1, 10);
		Serialprintln("Message sent");

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* CAN init function */
static void MX_CAN_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 9;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SJW = CAN_SJW_1TQ;
  hcan1.Init.BS1 = CAN_BS1_13TQ;
  hcan1.Init.BS2 = CAN_BS2_2TQ;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.ABOM = DISABLE;
  hcan1.Init.AWUM = DISABLE;
  hcan1.Init.NART = DISABLE;
  hcan1.Init.RFLM = DISABLE;
  hcan1.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
		newline();
		Serialprintln("/////////////////////////////////////////");
		Serialprintln("/////////////////////////////////////////");
		Serialprintln("ERROR INITIATING CAN BUS");

    _Error_Handler(__FILE__, __LINE__);
  }
	else{
		Serialprintln("CAN BUS INITIATED");
	}

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Serialprintln(char _out[]){	
	HAL_UART_Transmit(&huart1, (uint8_t *) _out, strlen(_out), 10);
	char newline[2] = "\r\n";
	HAL_UART_Transmit(&huart1, (uint8_t *) newline, 2, 10);
}

void newline(void){
	char newline[2] = "\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t *) newline, 2, 10);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
Copied and renamed the transmit function for debugging 
*/
HAL_StatusTypeDef T_HAL_CAN_Transmit(CAN_HandleTypeDef* hcan, uint32_t Timeout)
{
	Serialprintln("Starting the transmit function now");
  uint32_t transmitmailbox = CAN_TXSTATUS_NOMAILBOX;
	Serialprintln("transmitmailbox int created");
  uint32_t tickstart = 0U;
	Serialprintln("tickstart created");

  /* Check the parameters */
  assert_param(IS_CAN_IDTYPE(hcan->pTxMsg->IDE));
  assert_param(IS_CAN_RTR(hcan->pTxMsg->RTR));
  assert_param(IS_CAN_DLC(hcan->pTxMsg->DLC));
	Serialprintln("Parameters checked");

  if(((hcan->Instance->TSR&CAN_TSR_TME0) == CAN_TSR_TME0) || \
     ((hcan->Instance->TSR&CAN_TSR_TME1) == CAN_TSR_TME1) || \
     ((hcan->Instance->TSR&CAN_TSR_TME2) == CAN_TSR_TME2))
  {
    /* Process locked */
    __HAL_LOCK(hcan);
		Serialprintln("process locked");
		Serialprintln("checking hcan status now");

    /* Change CAN state */
    switch(hcan->State)
    {
      case(HAL_CAN_STATE_BUSY_RX0):
				Serialprintln("HAL_CAN_STATE_BUSY_TX_RX0");
          hcan->State = HAL_CAN_STATE_BUSY_TX_RX0;
          break;
      case(HAL_CAN_STATE_BUSY_RX1):
				Serialprintln("HAL_CAN_STATE_BUSY_TX_RX1");
          hcan->State = HAL_CAN_STATE_BUSY_TX_RX1;
          break;
      case(HAL_CAN_STATE_BUSY_RX0_RX1):
				Serialprintln("HAL_CAN_STATE_BUSY_TX_RX0_RX1");
          hcan->State = HAL_CAN_STATE_BUSY_TX_RX0_RX1;
          break;
      default: /* HAL_CAN_STATE_READY */
				Serialprintln("HAL_CAN_STATE_BUSY_TX");
          hcan->State = HAL_CAN_STATE_BUSY_TX;
          break;
    }
		Serialprintln("Select one empty transmit mailbox");

    /* Select one empty transmit mailbox */
    if (HAL_IS_BIT_SET(hcan->Instance->TSR, CAN_TSR_TME0))
    {
			Serialprintln("transmitmailbox = CAN_TXMAILBOX_0;");
      transmitmailbox = CAN_TXMAILBOX_0;
    }
    else if (HAL_IS_BIT_SET(hcan->Instance->TSR, CAN_TSR_TME1))
    {
			Serialprintln("transmitmailbox = CAN_TXMAILBOX_1;");
      transmitmailbox = CAN_TXMAILBOX_1;
    }
    else
    {
			Serialprintln("transmitmailbox = CAN_TXMAILBOX_2;");
      transmitmailbox = CAN_TXMAILBOX_2;
    }
		
		Serialprintln("Set up the Id");
    /* Set up the Id */
    hcan->Instance->sTxMailBox[transmitmailbox].TIR &= CAN_TI0R_TXRQ;
		
    if (TxMessage.IDE == CAN_ID_STD)
    {
      assert_param(IS_CAN_STDID(TxMessage.StdId));  
			
      hcan->Instance->sTxMailBox[transmitmailbox].TIR |= ((TxMessage.StdId << CAN_TI0R_STID_Pos) |
                                                           TxMessage.RTR);
			Serialprintln("Setting up a STD ID");
    }
    else
    {
      assert_param(IS_CAN_EXTID(hcan->pTxMsg->ExtId));
      hcan->Instance->sTxMailBox[transmitmailbox].TIR |= ((hcan->pTxMsg->ExtId << CAN_TI0R_EXID_Pos) |
                                                           hcan->pTxMsg->IDE |
                                                           hcan->pTxMsg->RTR);
			Serialprintln("Setting up an extended ID");
    }
Serialprintln("Done setting the ID");
    /* Set up the DLC */
		Serialprintln("Set up the DLC");
		
		TxMessage.DLC &= (uint8_t)0x0000000F;
		Serialprintln("hcan->pTxMsg->DLC &= (uint8_t)0x0000000F; DONE");
    //hcan->sTxMailBox[transmit_mailbox].TDTR &= (uint32_t)0xFFFFFFF0;
		//hcan->sTxMailBox[transmit_mailbox].TDTR |= TxMessage->DLC;
		
    //hcan->pTxMsg->DLC &= (uint8_t)0x0000000F;
		
		
    hcan->Instance->sTxMailBox[transmitmailbox].TDTR &= (uint32_t)0xFFFFFFF0;		
		Serialprintln("hcan->Instance->sTxMailBox[transmitmailbox].TDTR &= 0xFFFFFFF0; DONE");
		
		
    hcan->Instance->sTxMailBox[transmitmailbox].TDTR |= TxMessage.DLC;
		Serialprintln("hcan->Instance->sTxMailBox[transmitmailbox].TDTR |= hcan->pTxMsg->DLC; DONE");
		
		
		Serialprintln("Set up the DLC DONE");

    /* Set up the data field */
		Serialprintln("Set up the data field");
    WRITE_REG(hcan->Instance->sTxMailBox[transmitmailbox].TDLR, ((uint32_t)TxMessage.Data[3] << CAN_TDL0R_DATA3_Pos) | 
                                                                ((uint32_t)TxMessage.Data[2] << CAN_TDL0R_DATA2_Pos) |
                                                                ((uint32_t)TxMessage.Data[1] << CAN_TDL0R_DATA1_Pos) | 
                                                                ((uint32_t)TxMessage.Data[0] << CAN_TDL0R_DATA0_Pos));
    WRITE_REG(hcan->Instance->sTxMailBox[transmitmailbox].TDHR, ((uint32_t)TxMessage.Data[7] << CAN_TDL0R_DATA3_Pos) | 
                                                                ((uint32_t)TxMessage.Data[6] << CAN_TDL0R_DATA2_Pos) |
                                                                ((uint32_t)TxMessage.Data[5] << CAN_TDL0R_DATA1_Pos) |
                                                                ((uint32_t)TxMessage.Data[4] << CAN_TDL0R_DATA0_Pos));
		Serialprintln("Set up the data field DONE");
    /* Request transmission */
    SET_BIT(hcan->Instance->sTxMailBox[transmitmailbox].TIR, CAN_TI0R_TXRQ);

    /* Get tick */
    tickstart = HAL_GetTick();

    /* Check End of transmission flag */
    while(!(__HAL_CAN_TRANSMIT_STATUS(hcan, transmitmailbox)))
    {
      /* Check for the Timeout */
      if(Timeout != HAL_MAX_DELAY)
      {
        if((Timeout == 0U) || ((HAL_GetTick()-tickstart) > Timeout))
        {
          hcan->State = HAL_CAN_STATE_TIMEOUT;

          /* Cancel transmission */
          __HAL_CAN_CANCEL_TRANSMIT(hcan, transmitmailbox);

          /* Process unlocked */
          __HAL_UNLOCK(hcan);
          return HAL_TIMEOUT;
        }
      }
    }
    /* Change CAN state */
    switch(hcan->State)
    {
      case(HAL_CAN_STATE_BUSY_TX_RX0):
          hcan->State = HAL_CAN_STATE_BUSY_RX0;
          break;
      case(HAL_CAN_STATE_BUSY_TX_RX1):
          hcan->State = HAL_CAN_STATE_BUSY_RX1;
          break;
      case(HAL_CAN_STATE_BUSY_TX_RX0_RX1):
          hcan->State = HAL_CAN_STATE_BUSY_RX0_RX1;
          break;
      default: /* HAL_CAN_STATE_BUSY_TX */
          hcan->State = HAL_CAN_STATE_READY;
          break;
    }

    /* Process unlocked */
    __HAL_UNLOCK(hcan);

    /* Return function status */
    return HAL_OK;
  }
  else
  {
    /* Change CAN state */
    hcan->State = HAL_CAN_STATE_ERROR;

    /* Return function status */
    return HAL_ERROR;
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
