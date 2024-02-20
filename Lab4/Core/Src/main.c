/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

void configureUSART(void) {
    // 1. Enable the system clock to the USART peripheral
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;  

    // 2. Set the Baud Rate
    USART3->BRR = HAL_RCC_GetHCLKFreq()/9600; // This gets the 115200 baud rate

    // 3. Enable the Transmitter and Receiver
    //USART3->CR1 |= (USART_CR1_TE | USART_CR1_RE);  // Enable transmitter and receiver
		USART3->CR1 |= (1<<2);
		USART3->CR1 |= (1<<3);
	// 4. Enable the USART Peripheral
    //USART3->CR1 |= USART_CR1_UE;  // Enable USART
		USART1->CR1 |= (1<<0);

    // Note: Other configurations like word length, stop bits, parity, etc., should also be configured here as needed.
}



/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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

  HAL_Init();

  configureUSART();  // Configure USART with the desired settings

	// Enable peripheral clock to Port B
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	
	 // Enable the system clock to the USART peripheral
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;  
  SystemClock_Config();
	
	// Set pins 11 & 10 to alternate function mode
	GPIOB -> MODER |= (1<<11);
	GPIOB -> MODER &= ~(1<<10);
	
	GPIOB ->AFR[1] |= (1<<10);
	GPIOB ->AFR[1] |= (1<<14);
	
	// Set the Baud Rate
  USART3->BRR = HAL_RCC_GetHCLKFreq()/9600; // This gets the 115200 baud rate
		
	// Enable the Transmitter and Receiver
  //USART3->CR1 |= (USART_CR1_TE | USART_CR1_RE);  // Enable transmitter and receiver
	USART3->CR1 |= (1<<2);
	USART3->CR1 |= (1<<3);
	
	// Enable the USART Peripheral
  //USART3->CR1 |= USART_CR1_UE;  // Enable USART
	USART1->CR1 |= (1<<0);
	

  while (1)
  {
	
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
