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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

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

  SystemClock_Config();

	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;

	//setup leds 67
	//GPIOC->MODER |= (1<<12);
	//GPIOC->MODER &= ~(1<<13);
	//GPIOC->MODER |= (1<<14);
	//GPIOC->MODER &= ~(1<<15);

	//GPIOC->OTYPER &= ~(1<<6);
	//GPIOC->OTYPER &= ~(1<<7);
	
	//GPIOC->OSPEEDR &= ~(1<<12);
	//GPIOC->OSPEEDR &= ~(1<<14);

	//GPIOC->PUPDR &= ~(1<<12);
	//GPIOC->PUPDR &= ~(1<<13);
	//GPIOC->PUPDR &= ~(1<<14);
	//GPIOC->PUPDR &= ~(1<<15);
	
	//PB11
	GPIOB->MODER |= (1<<23);
	GPIOB->MODER &= ~(1<<22);
	GPIOB->OTYPER |= (1<<11);
	GPIOB->AFR[1] |= (1<<12);
	GPIOB->AFR[1] &= ~(1<<13);
	GPIOB->AFR[1] &= ~(1<<14);
	GPIOB->AFR[1] &= ~(1<<15);
	
	//PB13
	GPIOB->MODER |= (1<<27);
	GPIOB->MODER &= ~(1<<26);
	GPIOB->OTYPER |= (1<<13);
	GPIOB->AFR[1] |= (1<<20);
	GPIOB->AFR[1] &= ~(1<<21);
	GPIOB->AFR[1] |= (1<<22);
	GPIOB->AFR[1] &= ~(1<<23);
	
	//PB14
	GPIOB->MODER &= ~(1<<29);
	GPIOB->MODER |= (1<<28);
	GPIOB->OTYPER &= ~(1<<14);
	GPIOB->ODR |= (1<<14);
	
	//PC0
	GPIOC->MODER &= ~(1<<1);
	GPIOC->MODER |= (1<<0);
	GPIOC->OTYPER &= ~(1<<0);
	GPIOC->ODR |= (1<<0);
		
	
	// Setting I2C2 to 1kHz
  I2C2 -> TIMINGR |= 0x13;
  I2C2 -> TIMINGR |= (0xF << 8);
  I2C2 -> TIMINGR |= (0x2 << 16);
  I2C2 -> TIMINGR |= (0x4 << 20);
  I2C2 -> TIMINGR |= (1 << 28); 
	
		//enable peripheral
	I2C2->CR1 |= (1<<0);
	
	
		//set slave address to 0x6b
		I2C2->CR2 |= (0x69<<1);
		//number of bytes to transmit = 1
		I2C2->CR2 |= (1<<16);
		//SET RD_WRN to write
		I2C2->CR2 &= ~(1<<10);
		//set start bit
		I2C2->CR2 |= (1<<13);
		
		
		//  Checks the TXIS (Transmit Register Empty/Ready)
		while (1){
			if(I2C2->ISR & (1<<1)){
			break;
			}
			// Checks the NACKF 
			if(I2C2->ISR & (1<<4)){
			}
		}
		// WHO_AM_I Register
		I2C2->TXDR |= 0x0F;
		
		// Checks TC resistor
		while(1){
			if(I2C2->ISR & (1<<6)){
			break;
			}
			
		}
		//set slave address to 0x6b
		I2C2->CR2 |= (0x69<<1);
		//number of bytes to transmit = 1
		I2C2->CR2 |= (1<<16);
		//SET RD_WRN to Read
		I2C2->CR2 |= I2C_CR2_RD_WRN;
		//set start bit
		I2C2->CR2 |= I2C_CR2_START;
		
		// Checks the RXNE (Receive Register Not Empty) 
		while (1)
		{
			if(I2C2->ISR & (1<<2))
			{
			 break;
			}
			// Checks the NACKF
			if(I2C2->ISR & (1<<4))
			{
			}
		}
		
		// Checks the TC Register
		while(1){
			if(I2C2->ISR & (1<<6)){
			 break;
			}
		}
		
		// Check the contents of the RXDR register to see if it matches 0xD3. 
		if(I2C2->RXDR == 0xD3){
			// Sets the stop bit
			I2C2->CR2 |= (1<<14);
		}
		
		
		
		while (1)
  {
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