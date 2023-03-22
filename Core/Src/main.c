/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

extern uint8_t UART_Recieve;  // UART echo flag

uint8_t buttonpresdout[4];
uint16_t buttonctr = 0;


extern uint8_t middleButton;  // External variable flag for button debounce.
extern uint8_t topButton;
extern uint8_t right_button;
extern uint8_t left_button;

uint32_t current_time = 0;
uint32_t previous_time = 0;

uint8_t mbp = 0;
uint8_t mbpf = 0;

uint8_t rxdata[20];
uint8_t msg[20];
uint8_t send_request_msg[19];
uint8_t decode = 0;
int j = 0;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

void echo(void);

void buttondebounce(void);

void IsButtonPressed(void);
void convertfunc(void);

void decode_func(void);
void set_mode_func(void);
void request_mode_func(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	msg[j] = rxdata[0];
	if(msg[j] == 10)  // Checking for the new line character
	{
		decode = 1;
	}
	HAL_UART_Receive_IT(&huart2, rxdata, 1);
	j = j+1;
 }

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Transmit(&huart2, (uint8_t*) "#:22569081:$\n", 13, 50);

  HAL_UART_Receive_IT(&huart2, rxdata, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_UART_Receive_IT(&huart2, rxdata, 1);

	  if(decode == 1)
	  {
		  decode_func();
	  }

	  buttondebounce();







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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 57600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_2;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pins : B1_Pin PC0 PC1 */
  GPIO_InitStruct.Pin = B1_Pin|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

void buttondebounce(void)
{
	if(middleButton == 1)
	{
		current_time = HAL_GetTick();
		if(current_time - previous_time > 50)
		{
			mbp = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1);

			if(mbp == 1)
			{
				HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_11);

				buttonctr++;

				convertfunc();

				HAL_UART_Transmit(&huart2,(uint8_t*) buttonpresdout, sizeof(buttonpresdout), 100);
			}
			previous_time = current_time;
		}
	}
	middleButton = 0;
}

// This code is currently not used.

void convertfunc()
{
	buttonpresdout[0] = (buttonctr / 100) + '0';  // Hundreds
	buttonpresdout[1] = ((buttonctr / 10) % 10)+ '0';  // Tens
	buttonpresdout[2] = (buttonctr % 10)+ '0';  // Ones
	buttonpresdout[3] = '\n';
}

void decode_func()
{
	if(msg[0] == '#'){
		if(msg[1] == ':'){
			if(msg[4] == ':'){
				if(msg[5] == '$')
				{
					request_mode_func();
				}

				if(msg[8] == ':'){
					if(msg[12] == ':'){
						if(msg[16] == ':'){
							if(msg[17] == '$'){

								set_mode_func();
							}
						}
					}
				}
			}
		}
	}
}

void set_mode_func()
{
	if((msg[2] == 'M') && (msg[17] == '$')){
		if(msg[3] == 'F'){

			//Flashlight mode
		}
		else if(msg[3] == 'E'){

			// Emergency mode
		}
		else if(msg[3] == 'M'){

			//mood mode
		}
	}
}

void request_mode_func()
{
	if((msg[2] == 'M') && (msg[5] == '$')){
		if(msg[3] == 'F'){

			send_request_msg[0] = '#';
			send_request_msg[1] = ':';
			send_request_msg[2] = 'M';
			send_request_msg[3] = 'F';
			send_request_msg[4] = ':';
			// State
			send_request_msg[5] = '0'; // This will actually be a variable one from the adc.
			send_request_msg[6] = '0'; // This will actually be a variable one from the adc.
			send_request_msg[7] = '0'; // This will actually be a variable one from the adc.
			send_request_msg[8] = ':';
			// Param1
			send_request_msg[9] = '0';
			send_request_msg[10] = '0';
			send_request_msg[11] = '0';
			send_request_msg[12] = ':';
			// Param2
			send_request_msg[13] = '0';
			send_request_msg[14] = '0';
			send_request_msg[15] = '0';
			send_request_msg[16] = ':';

			send_request_msg[17] = '$';
			send_request_msg[18] = 10;

			HAL_UART_Transmit(&huart2, (uint8_t*) send_request_msg, 19, 100);

		}
		if(msg[3] == 'E'){

			send_request_msg[0] = '#';
			send_request_msg[1] = ':';
			send_request_msg[2] = 'M';
			send_request_msg[3] = 'E';
			send_request_msg[4] = ':';
			// State
			send_request_msg[5] = '0'; // This will actually be a variable one from the adc.
			send_request_msg[6] = '0'; // This will actually be a variable one from the adc.
			send_request_msg[7] = '0'; // This will actually be a variable one from the adc.
			send_request_msg[8] = ':';
			// Param1
			send_request_msg[9] = '0'; // This will actually be a variable "on" time.
			send_request_msg[10] = '0'; // This will actually be a variable "on" time.
			send_request_msg[11] = '0'; // This will actually be a variable "on" time.
			send_request_msg[12] = ':';
			// Param2
			send_request_msg[13] = '0'; // This will be SOS from the morse code
			send_request_msg[14] = '0'; // This will be SOS from the morse code
			send_request_msg[15] = '0'; // This will be SOS from the morse code
			send_request_msg[16] = ':';

			send_request_msg[17] = '$';
			send_request_msg[18] = 10;

		}
		if(msg[3] == 'M'){

			send_request_msg[0] = '#';
			send_request_msg[1] = ':';
			send_request_msg[2] = 'M';
			send_request_msg[3] = 'M';
			send_request_msg[4] = ':';
			// State
			send_request_msg[5] = '0';  // This will be the intensity of the R channel of the RGB
			send_request_msg[6] = '0';  // This will be the intensity of the R channel of the RGB
			send_request_msg[7] = '0';  // This will be the intensity of the R channel of the RGB
			send_request_msg[8] = ':';
			// Param1
			send_request_msg[9] = '0';  // This will be the intensity of the G channel of the RGB
			send_request_msg[10] = '0'; // This will be the intensity of the G channel of the RGB
			send_request_msg[11] = '0'; // This will be the intensity of the G channel of the RGB
			send_request_msg[12] = ':';
			// Param2
			send_request_msg[13] = '0'; // This will be the intensity of the B channel of the RGB
			send_request_msg[14] = '0'; // This will be the intensity of the B channel of the RGB
			send_request_msg[15] = '0'; // This will be the intensity of the B channel of the RGB
			send_request_msg[16] = ':';

			send_request_msg[17] = '$';
			send_request_msg[18] = 10;
		}
	}
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
