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

void SystemClock_Config(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  HAL_Init();

  // Enable peripheral clock to Port C
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
	// Enable the ADC1 in the RCC
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

  SystemClock_Config();


	  
	// Set pins 6-9 to output mode
    GPIOC->MODER &= ~(1 << 19);
    GPIOC->MODER |= (1 << 18);
    GPIOC->MODER &= ~(1 << 17);
    GPIOC->MODER |= (1 << 16);
    GPIOC->MODER &= ~(1 << 15);
    GPIOC->MODER |= (1 << 14);
    GPIOC->MODER &= ~(1 << 13);
    GPIOC->MODER |= (1 << 12);

    // Set output type to push-pull for pins 6-9
    GPIOC->OTYPER &= ~(1 << 6);
    GPIOC->OTYPER &= ~(1 << 7);
    GPIOC->OTYPER &= ~(1 << 8);
    GPIOC->OTYPER &= ~(1 << 9);

    // Set low speed for pins 6-9
    GPIOC->OSPEEDR &= ~(1 << 18);
    GPIOC->OSPEEDR &= ~(1 << 16);
    GPIOC->OSPEEDR &= ~(1 << 14);
    GPIOC->OSPEEDR &= ~(1 << 12);

    // Disable pull-up/pull-down for pins 6-9
    GPIOC->PUPDR &= ~(1 << 19);
    GPIOC->PUPDR &= ~(1 << 18);
    GPIOC->PUPDR &= ~(1 << 17);
    GPIOC->PUPDR &= ~(1 << 16);
    GPIOC->PUPDR &= ~(1 << 15);
    GPIOC->PUPDR &= ~(1 << 14);
    GPIOC->PUPDR &= ~(1 << 13);
    GPIOC->PUPDR &= ~(1 << 12);

    // Set pins 6-9 high (turn LEDs on)
    GPIOC->ODR &= ~(1 << 9);
    GPIOC->ODR &= ~(1 << 8);
    GPIOC->ODR &= ~(1 << 7);
    GPIOC->ODR &= ~(1 << 6);
		
		// PC0
	  GPIOC->MODER |= (1 << 0);
		GPIOC->MODER |= (1 << 1);
		GPIOC->PUPDR	&= ~(1 << 0);
		GPIOC->PUPDR &= ~(1 << 1);
		
		// 8-bit resolution
		ADC1->CFGR1 |= (1 << 4);
		ADC1->CFGR1 &= ~(1 << 3);

		// continuous conversion mode
		ADC1->CFGR1 |= (1 << 13);

		//hardware triggers disabled (software trigger only)
		ADC1->CFGR1 &= ~(1 << 10);
		ADC1->CFGR1 &= ~(1 << 11);
		
		// Select/enable the input pin’s channel 10 for ADC conversion
		ADC1->CHSELR |= (1 << 10);



// Perform a self-calibration, enable, and start the ADC.
		// Step 1: Ensure that ADEN (ADC Enable bit) is not set before initiating calibration
if ((ADC1->CR & ADC_CR_ADEN) != 0) {
    // Step 2: Clear the ADEN bit by setting ADDIS (ADC Disable bit)
    ADC1->CR |= ADC_CR_ADDIS;
}
// Wait until ADEN is cleared to ensure ADC is fully disabled
while ((ADC1->CR & ADC_CR_ADEN) != 0) {
    // For a robust implementation, add here time-out management to avoid getting stuck in an infinite loop
}
// Step 3: Ensure DMAEN (Direct Memory Access Enable bit) is cleared in the CFGR1 register
ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN;

// Step 4: Start the ADC calibration by setting the ADCAL bit
ADC1->CR |= ADC_CR_ADCAL;

// Step 5: Wait for calibration to complete, indicated by ADCAL bit being cleared
while ((ADC1->CR & ADC_CR_ADCAL) != 0) {
    // Again, for a robust implementation, consider adding time-out management here
}



// ADC enable sequence code

// Step 1: Ensure that ADRDY (ADC Ready flag) is not set
if ((ADC1->ISR & ADC_ISR_ADRDY) != 0) {
    // Step 2: Clear the ADRDY flag by writing 1 to it
    ADC1->ISR |= (1<<0); 
}
// Step 3: Enable the ADC by setting the ADEN bit
ADC1->CR |= ADC_CR_ADEN;
// Step 4: Wait until the ADC is ready, which is indicated when the ADRDY flag is set
while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) {
    // For a robust implementation, consider adding time-out management here to avoid infinite loops
}

ADC1->CR |= (1<<2);
  while (1)
  {
		int value;
		
		value = ADC1->DR;
		// Red
		if (value < 32) 
		{
			GPIOC->ODR &= ~(1 << 6);  
		}
		else 
		{
			GPIOC->ODR |= (1 << 6);
		}
		
		//Green
		if (value < 64) 
		{
			GPIOC->ODR &= ~(1 << 9);  
		}
		else 
		{
			GPIOC->ODR |= (1 << 9);
		}
		
		// Blue
		if (value < 128) 
		{
			GPIOC->ODR &= ~(1 << 7);  
		}
		else 
		{
			GPIOC->ODR |= (1 << 7);
		}
		
		//Orange
		if (value < 192) 
		{
			GPIOC->ODR &= ~(1 << 8);  
		}
		else 
		{
			GPIOC->ODR |= (1 << 8);
		}
		
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
