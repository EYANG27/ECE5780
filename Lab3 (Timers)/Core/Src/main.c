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
	 //Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
	// Enables the 2nd timer peripheral
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	//Enables the 3rd timer peripheral
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	// Enable peripheral clock to Port C
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	// Set the Moder to general-purpose output mode
	GPIOC->MODER |= (1<<16);  // Setting bit 16 to 1
	GPIOC->MODER |= (1<<18);	// Setting bit 18 to 1
	
	// Set the blue and red to alternate function mode
	GPIOC -> MODER |= (1<<15);
	GPIOC -> MODER &= ~(1<<14);
	GPIOC -> MODER |= (1<<13);
	GPIOC -> MODER &= ~(1<<12);
	
	GPIOC->OTYPER &= ~(1<<9);	 // Setting the 9th Pin to 0
	GPIOC->OTYPER &= ~(1<<8);	 // Setting the 8th Pin to 0
	GPIOC->OTYPER &= ~(1<<7);	 // Setting the 7th Pin to 0
	GPIOC->OTYPER &= ~(1<<6);	 // Setting the 6th Pin to 0


	GPIOC->OSPEEDR &= ~(1<<16);	// Setting the 16th bit to 0, while the 15th bit is no set
	GPIOC->OSPEEDR &= ~(1<<18);	// Setting the 18th bit to 0, while the 19th bit is no set
	GPIOC->OSPEEDR &= ~(1<<14);	// Setting the 14th bit to 0, while the 15th bit is no set
	GPIOC->OSPEEDR &= ~(1<<12);	// Setting the 12th bit to 0, while the 13th bit is no set

	GPIOC->PUPDR  &= ~(1<<12); // Setting the 12th bit to 0
	GPIOC->PUPDR  &= ~(1<<13); // Setting the 13th bit to 0
	GPIOC->PUPDR  &= ~(1<<14); // Setting the 14th bit to 0
	GPIOC->PUPDR  &= ~(1<<15); // Setting the 15th bit to 0
	GPIOC->PUPDR  &= ~(1<<16); // Setting the 16th bit to 0
	GPIOC->PUPDR  &= ~(1<<17); // Setting the 17th bit to 0
	GPIOC->PUPDR  &= ~(1<<18); // Setting the 18th bit to 0
	GPIOC->PUPDR  &= ~(1<<19); // Setting the 19th bit to 0
	
	// Sets the PSR to 7999
	TIM2->PSC = 7999;

	//Sets the TIM@ ARR to 250
	TIM2->ARR = 250;

 	// Sets the PSR to 79 for timer 3, 1250 otherise
	TIM3 -> PSC = 7;
	// Set the ARR to 125 for a 800 Hz Signal
	TIM3 -> ARR = 1250;
	
	// Set the CCMR1 to output for Timer 3
	TIM3 -> CCMR1 &= ~(1<<0);
	TIM3 -> CCMR1 &= ~(1<<1);
	TIM3 -> CCMR1 |= (1<<4);
	TIM3 -> CCMR1 |= (1<<5);
	TIM3 -> CCMR1 |= (1<<6);
	TIM3 -> CCMR1 &= ~(1<<12);
	TIM3 -> CCMR1 |= (1<<13);
	TIM3 -> CCMR1 |= (1<<14);
	TIM3 -> CCMR1 |= (1<<3);
	TIM3 -> CCMR1 |= (1<<11);
	
	GPIOC ->AFR[0] &=~(1<<0);
	GPIOC ->AFR[0] &=~(1<<1);
	GPIOC ->AFR[0] &=~(1<<2);
	GPIOC ->AFR[0] &=~(1<<3);

	TIM3 ->CR1 |= (1<<0);
	
	//TIM3 -> CCMR2 &= ~(1<<0);
	//TIM3 -> CCMR2 &= ~(1<<1);
	//TIM3 -> CCMR2 |= (1<<4);
	//TIM3 -> CCMR2 |= (1<<5);
	//TIM3 -> CCMR2 |= (1<<6);
	//TIM3 -> CCMR2 &= ~(1<<12);
	//TIM3 -> CCMR2 |= (1<<13);
	//TIM3 -> CCMR2 |= (1<<14);
	//TIM3 -> CCMR2 |= (1<<3);
	//TIM3 -> CCMR2 |= (1<<11);
	
	TIM3 -> CCER |= (1<<0);
	TIM3 -> CCER |= (1<<4);
	
	TIM3 -> CCR1 = 25;
	TIM3 -> CCR2 = 25;
	
	// Enable the update interrupt
	TIM2 -> DIER |=1;
	
	NVIC_EnableIRQ(TIM2_IRQn); // Enable TIM2 interrupt in NVIC 
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
	
	//Configure and enable/start the timer
	TIM2->CR1 |= (1<<0); 
	
  /* MCU Configuration--------------------------------------------------------*/
	GPIOC->ODR |= (1<<9);  // Set Green as High
	GPIOC->ODR &= ~(1<<8);  // Set Orange as High


  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

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

void TIM2_IRQHandler(void)
{
	GPIOC->ODR ^= (1<<9);  // Set Green as High
	GPIOC->ODR ^= (1<<8);  // Set Orange as Low
  TIM2->SR &= ~(1<<0);   // Clear update interrupt flag
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
