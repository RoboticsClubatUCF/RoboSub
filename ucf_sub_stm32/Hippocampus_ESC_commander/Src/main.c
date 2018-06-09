
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
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32l4xx_hal.h"
#include "crc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
int16_t motorSettings[8];

uint16_t dshotData[136];
uint16_t currentDshotOutput;

uint32_t pwmData[8];
uint16_t currentPWMOutput;

uint8_t pcRxData;
uint8_t pcRxDataPosition;
uint8_t pcRxBuffer[50];

uint8_t escRxBuffer[10];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void setDshotData(int16_t cmd, uint8_t tlm, uint8_t id);
void setPwmData(int16_t cmd, uint8_t tlm, uint8_t id);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
	if(htim->Instance == TIM1)
	{
		currentDshotOutput++;
		if(currentDshotOutput < 8)
		{
			//Set multiplexer state
			if((currentDshotOutput&0x01)!= 0)
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

			if((currentDshotOutput&0x02)!= 0)
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

			if((currentDshotOutput&0x04)!= 0)
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

			HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, dshotData+currentDshotOutput*17, 17);
		}
		else
		{
			currentDshotOutput = 0;
		}
	}
	if(htim->Instance == TIM2)
	{
		currentPWMOutput++;
		if(currentPWMOutput < 8)
		{
			//Set multiplexer state
			if((currentPWMOutput&0x01)!= 0)
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);

			if((currentPWMOutput&0x02)!= 0)
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);

			if((currentPWMOutput&0x04)!= 0)
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
			htim2.Instance->CCR1 = pwmData[currentPWMOutput];
		}
		else
		{
			currentPWMOutput = 0;
			htim2.Instance->CCR1 = 0;
			HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_1);
		}
	}
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM6)
	{
		//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
		HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, dshotData, 17);
		//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
	}
	if(htim->Instance == TIM7)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
		htim2.Instance->CCR1 = pwmData[currentPWMOutput];
		HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
	if(huart->Instance == USART1)
	{

	}
	if(huart->Instance == USART2)
	{
		if(pcRxData == 0){
			//Process packet
			uint8_t nextSubstitution = pcRxBuffer[0];
			if(nextSubstitution <= pcRxDataPosition) //Make sure the substitution is inside the buffer
			{
				for(int i = 0; i < pcRxDataPosition; i++) //Iterate through the buffer
				{
					if(i == nextSubstitution){
						nextSubstitution += pcRxBuffer[i];
						if(nextSubstitution >= pcRxDataPosition)
						{
							//nextSubstitution is outside the data buffer so do something
						}
						pcRxBuffer[i] = 0;
					}
				}
				uint8_t checksum = 0;
				for(int i = 1; i < pcRxDataPosition-1; i++)
				{
					checksum += pcRxBuffer[i];
				}
				if(checksum == pcRxBuffer[pcRxDataPosition-1])
				{
					HAL_UART_Transmit_IT(&huart2, &checksum, 1);
					if(pcRxDataPosition == 18)
					{
						for(int i = 0; i < 8; i++)
						{
							setDshotData((pcRxBuffer[i*2+2] << 8) | pcRxBuffer[i*2+1], 0, i);
							setPwmData((pcRxBuffer[i*2+2] << 8) | pcRxBuffer[i*2+1], 0, i);
						}
					}
				}
			}
			pcRxDataPosition = 0; //Reset buffer
		}
		else
		{
			pcRxBuffer[pcRxDataPosition] = pcRxData; //put byte in the buffer
			pcRxDataPosition++; //Next byte goes in next buffer position
		}
		if(pcRxDataPosition < 50) //We can get another byte
		{
			HAL_UART_Receive_IT(&huart2, &pcRxData, 1); //Set UART to get another byte
		}
		else
		{
			//Buffer is full. Reset
			pcRxDataPosition = 0;
			HAL_UART_Receive_IT(&huart2, &pcRxData, 1);
		}
	}
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
	if((huart->Instance == USART2) && ((huart->ErrorCode & HAL_UART_ERROR_ORE) != RESET))
	{
		HAL_UART_Receive_IT(&huart2, &pcRxData, 1);
	}
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	for(int i = 0; i<=7; i++)
	{
		motorSettings[i] = 0;
	}

	for(int i = 0; i <= 7; i++)
	{
		setDshotData(0,0,i);
		setPwmData(0, 0, i);
	}
	currentDshotOutput = 0;
	currentPWMOutput = 0;
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_CRC_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
	//Start PWM and DSHOT generation
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);

	//Activate Multiplexers
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

	//HAL_UART_Receive_DMA();
	pcRxDataPosition = 0;
	HAL_UART_Receive_IT(&huart2, &pcRxData, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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

/* USER CODE BEGIN 4 */
void setDshotData(int16_t cmd, uint8_t tlm, uint8_t id){
	if(id<8){
		//All stop is 0
		//Forward is 1048-2047
		//Backward is 48-1047
		if(cmd > 0){
			cmd = (cmd>>5) + 1048;
			if(cmd > 2047) cmd = 2047;
		} else if(cmd < 0) {
			cmd = ((-cmd)>>5) + 48;
			if(cmd > 1047) cmd = 1047;
		}

		cmd = cmd << 5;
		cmd = cmd | (tlm<<4);
		cmd = cmd | (( ((cmd&0xF000)>>12) ^ ((cmd&0x0F00)>>8) ^ ((cmd&0x00F0)>>4) )&0xF);
		for(int i = 0; i < 16; i++){
			if((cmd&(0x1<<(15-i)))>0){
				dshotData[i+id*17] = 100;
			}
			else {
				dshotData[i+id*17] = 50;
			}
		}
	}
}
void setPwmData(int16_t cmd, uint8_t tlm, uint8_t id){
	//All stop is 1.5ms -> 120,000 counts
	//Full reverse is 1ms -> 80,000 counts
	//Flank is 2ms -> 160,000 counts
	if(id < 8){
		int32_t counts = (cmd * 5/4) + 120000;
		if(tlm == 0)
		{
			pwmData[id] = counts;
		}
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
