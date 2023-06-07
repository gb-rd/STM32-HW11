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
#include <stdio.h>
#include <string.h>
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
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint8_t flag_uart_received = 0;
typedef enum {LINE_OK=1, LINE_ERR, LINE_PROGRESS} line_status_t;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void activate_bootloader();
line_status_t validate_line(uint8_t*);
uint8_t read_2_bytes(uint8_t * buffer);
void parse_buffer(uint8_t *text_buffer, uint8_t *d_length, size_t *address, uint8_t *data, uint8_t *cmd);
uint32_t erase_flash_sector(uint8_t sector_to_erase);
uint32_t write_flash_page(uint8_t *page_buffer, uint32_t address);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  if (!HAL_GPIO_ReadPin (B1_GPIO_Port, B1_Pin))
  {
  	  activate_bootloader();
  }
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8400-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 5000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint8_t validate_line(uint8_t *text)
{
	uint8_t sum = 0;

	if(*text != ':')
	{
		return 0;
	}
	text++;

	int size = read_2_bytes(text);

	for (int i=0; i<size+5 && *(text+i); i++)
	{
		sum += read_2_bytes(text + i*2);
	}

	return (sum==0)? 1:0;
}

uint32_t erase_flash_sector(uint8_t sector_to_erase)
{
	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t SECTORError;

	HAL_FLASH_Unlock();

	  /* Erase the user Flash area */
	EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector        = sector_to_erase;
	EraseInitStruct.NbSectors     = 1; //NumOfSectors;

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK)
	{
		HAL_FLASH_Lock();
		return HAL_FLASH_GetError ();
	}
	else
	{
		HAL_FLASH_Lock();
		return 0;
	}
}

uint32_t write_flash_data(uint8_t *data_buffer, uint32_t address)
{
	  /* Program the user Flash area word by word */
	int ctr=0;

	HAL_FLASH_Unlock();

	uint8_t number_of_words = 4;

	while (ctr<number_of_words)
	{
	    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, *(((uint32_t*)data_buffer) + ctr)) == HAL_OK)
	    {
	    	address += 4;  // Word (use StartPageAddress += 2 for half word and 8 for double word)
	    	ctr++;
	    }
	    else
	    {
	    	/* Error occurred while writing data in Flash memory*/
	    	HAL_FLASH_Lock();
	    	return HAL_FLASH_GetError ();
	    }
	}

	HAL_FLASH_Lock();
	return 0;
}


uint8_t read_2_bytes(uint8_t *buffer)
{
	unsigned int res;
	sscanf((char*)buffer, "%2x", &res);
	return (uint8_t)res;
}

void parse_buffer(uint8_t* text_buffer, uint8_t *d_length, size_t *address, uint8_t *data, uint8_t *cmd)
{
	static size_t base_address = 0;
	size_t ret_address = 0;

	*d_length = read_2_bytes(text_buffer+1);
	*cmd = read_2_bytes(text_buffer+7);

	switch (*cmd)
	{
	case 4:  //base address
		base_address = (uint32_t)read_2_bytes(text_buffer+9)<<8 | read_2_bytes(text_buffer+11);
		base_address <<= 16;
		break;
	case 0:
		ret_address = base_address | (uint32_t)read_2_bytes(text_buffer+3)<<8 | read_2_bytes(text_buffer+5);
		break;
	}

	*address = ret_address;

	for (int i=0; i<*d_length; i++)
	{
		data[i] = read_2_bytes(text_buffer+2*i+9);
	}
}

void activate_bootloader()
{
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  TIM2->CCR1 = 500-1;

  uint8_t flag_done = 0;
  uint8_t line_received = 0;

  const int FLASH_BUF_SIZE = 256;
  uint8_t flash_buffer[FLASH_BUF_SIZE];
  uint8_t byte_buffer = 0;

  //const int TEXT_LINES_NUM = 5;
  const int TEXT_BUF_SIZE = 50;
  uint8_t text_buffer[TEXT_BUF_SIZE];
  size_t text_index = 0;

  uint8_t d_length;
  size_t address;
  uint8_t data[20];
  uint8_t cmd;

  const char *msg_enter_flashing = "\nReady to receive data\n";
  const char *msg_got_line = "Got line: ";
  const char *msg_got_ext_addr = "  Got extended address\n";
  const char *msg_line_err = "Line error\n";
  const char *msg_data_received = "\nData received\n";

  HAL_UART_Transmit(&huart2, (uint8_t*)msg_enter_flashing, strlen(msg_enter_flashing), 100);
  HAL_UART_Receive_IT(&huart2, &byte_buffer, sizeof(byte_buffer));

  while(1)
  {
	  if (flag_uart_received)
	  {
       	  if (byte_buffer == ':')
       	  {
       		  text_index = 0;
       	  }

       	  text_buffer[text_index] = byte_buffer;
       	  text_index++;

       	  if (text_index == TEXT_BUF_SIZE-1)
       	  {
       		  line_received = LINE_ERR;
       		  text_index = 0;
       	  }
       	  else if (byte_buffer == '\n')
       	  {
       		  if (validate_line(text_buffer))
       		  {
       			  line_received = LINE_OK;
       		  }
       	  }

       	  flag_uart_received = 0;
       	  HAL_UART_Receive_IT(&huart2, &byte_buffer, sizeof(byte_buffer));
	  }


	  if (line_received == LINE_OK)
	  {
		  line_received = LINE_PROGRESS;
		  HAL_UART_Transmit(&huart2, (uint8_t*)msg_got_line, strlen(msg_got_line), 100);
		  HAL_UART_Transmit(&huart2, (uint8_t*)text_buffer, text_index, 100);

		  parse_buffer(text_buffer, &d_length, &address, data, &cmd);

		  switch (cmd)
		  {
		  case 4:  //extended address
		  {
			  HAL_UART_Transmit(&huart2, (uint8_t*)msg_got_ext_addr, strlen(msg_got_ext_addr), 100);
			  erase_flash_sector(2);  //TODO: calc sector num
			  break;
		  }
		  case 0:  //data
	      {
			  for (int i=0; i<d_length; i++)
			  {
				  flash_buffer[i] = data[i];
			  }

			  write_flash_data(flash_buffer, address);
			  break;
	      }
		  case 1:  //EOF
			  flag_done = 1;
			  HAL_UART_Transmit(&huart2, (uint8_t*)msg_data_received, strlen(msg_data_received), 100);
			  break;
		  default:
			  HAL_UART_Transmit(&huart2, (uint8_t*)msg_line_err, strlen(msg_line_err), 100);
			  break;
		  }
		  if (flag_done)
		  {
			  break;
		  }
	  }
	  else if (line_received == LINE_ERR)
	  {
		  HAL_UART_Transmit(&huart2, (uint8_t*)msg_line_err, strlen(msg_line_err), 100);
		  break;
	  }

  }
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
	if (uart == &huart2)
	{
		flag_uart_received = 1;
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
