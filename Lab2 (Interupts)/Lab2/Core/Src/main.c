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

void EXTI0_1_IRQHandler(void){
	GPIOC->ODR ^= (1<<8); // Set Green as high
	GPIOC->ODR ^= (1<<9); // Set Orange as high
	volatile int count = 0; // Sets the count for the interrupt
	while(count < 1500000)
	{
		count++;
	}
	EXTI->PR|=(1<<0); 
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
	HAL_Init(); // Reset of all peripherals, init the Flash and Systick
SystemClock_Config(); //Configure the system clock
	
RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable peripheral clock to Port C
RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // Enable peripheral clock to Port A 

// Set the Moder to general-purpose output mode
GPIOC->MODER |= (1<<12);	// Setting bit 12 to 1
GPIOC->MODER &= ~(1<<13);	// Setting bit 13 to 0
GPIOC->MODER |= (1<<14);	// Setting bit 14 to 1
GPIOC->MODER &= ~(1<<15);	// Setting bit 15 to 0
GPIOC->MODER |= (1<<16);  // Setting bit 16 to 1
GPIOC->MODER &= ~(1<<17);	// Setting bit 17 to 0
GPIOC->MODER |= (1<<18);	// Setting bit 18 to 1
GPIOC->MODER &= ~(1<<19);	// Setting bit 19 to 0

// Set the OTyper to Push pull output
GPIOC->OTYPER &= ~(1<<6);	 // Setting the 6th Pin to 0
GPIOC->OTYPER &= ~(1<<9);	 // Setting the 9th Pin to 0
GPIOC->OTYPER &= ~(1<<7);	 // Setting the 7th Pin to 0
GPIOC->OTYPER &= ~(1<<8);	 // Setting the 8th Pin to 0
	
// Set the Ospeedr to low speed
GPIOC->OSPEEDR &= ~(1<<18);	// Setting the 18th bit to 0, while the 15th bit is no set
GPIOC->OSPEEDR &= ~(1<<12); // Setting the 12th bit to 0, while the 13th bit is no set
GPIOC->OSPEEDR &= ~(1<<16);	// Setting the 16th bit to 0, while the 15th bit is no set
GPIOC->OSPEEDR &= ~(1<<14); // Setting the 14th bit to 0, while the 13th bit is no set

// Set the PUPDR to no pull-up/down resistors 
GPIOC->PUPDR  &= ~(1<<12); // Setting the 12th bit to 0
GPIOC->PUPDR  &= ~(1<<13); // Setting the 13th bit to 0
GPIOC->PUPDR  &= ~(1<<14); // Setting the 14th bit to 0
GPIOC->PUPDR  &= ~(1<<15); // Setting the 15th bit to 0
GPIOC->PUPDR  &= ~(1<<16); // Setting the 12th bit to 0
GPIOC->PUPDR  &= ~(1<<17); // Setting the 13th bit to 0
GPIOC->PUPDR  &= ~(1<<18); // Setting the 14th bit to 0
GPIOC->PUPDR  &= ~(1<<19); // Setting the 15th bit to 0

GPIOC->ODR |= (1<<9);  // Set Red as High
GPIOC->ODR &= ~(1<<8);  // Set Green as Low
GPIOC->ODR &= ~(1<<7);  // Set Orange as Low
GPIOC->ODR &= ~(1<<6);  // Set Blue as Low

/**
Button Input - Enable Port A
**/
// Set the moder to input mode
GPIOA->MODER &= ~(1<<0); // Set the 0 bit to 0
GPIOA->MODER &= ~(1<<1); // Set the 1st bit to 0	
		
// Set the speed to low speed
GPIOA->OSPEEDR &= ~(1<<0);	// Set to 0th bit to 0, while the first bit is no set
 
// Set the Pull-Up Pull Down to Pull-Down Resistor 
GPIOA->PUPDR  |= (1<<1); // Set the 1st bit to 1
GPIOA->PUPDR  &= ~(1<<1); // Set the 0 bit to 0

RCC->APB2RSTR |= RCC_APB2RSTR_SYSCFGRST;

EXTI->IMR |= (1<<0); // Unmasks the 1st bit
EXTI->RTSR |= (1<<0); // Enables Rising-Edge Trigger

SYSCFG->EXTICR[0] &= ~(1<<0); // Sets the 0th bit for EXTICR to 0
SYSCFG->EXTICR[0] |= ~(1<<1); // Sets the 1st for EXTICR to 0
SYSCFG->EXTICR[0] |= ~(1<<2); // Sets the 2nd bit for EXTICR to 0

NVIC_EnableIRQ(EXTI0_1_IRQn); // Enables the EXTI0_1 interupt
NVIC_SetPriority(EXTI0_1_IRQn,3); // Set priority to 3, which is lowest for EXTI0_1
NVIC_SetPriority(SysTick_IRQn,2); // Set the SYSTICK priority to 2.

while(1) {
	HAL_Delay(400); // Delay 400ms
	//GPIOC->ODR ^= (1<<7);  // Set Red as low
	GPIOC->ODR ^= (1<<6); // Set Red as high
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
