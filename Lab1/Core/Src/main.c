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
//int main(void)
//{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
 // HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
 // SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  while (1)
 // {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//  }
  /* USER CODE END 3 */
//}
int main(void) {
HAL_Init(); // Reset of all peripherals, init the Flash and Systick
SystemClock_Config(); //Configure the system clock
	
RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable peripheral clock to Port C
RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // Enable peripheral clock to Port A 

	// GPIO for Port C
// Set the Moder to general-purpose output mode
GPIOC->MODER |= (1<<12);	// Setting bit 12 to 1
GPIOC->MODER &= ~(1<<13);	// Setting bit 13 to 0
GPIOC->MODER |= (1<<14);	// Setting bit 14 to 1
GPIOC->MODER &= ~(1<<15);	// Setting bit 15 to 0

// Set the OTyper to Push pull output
GPIOC->OTYPER &= ~(1<<6);	 // Setting the 6th Pin to 0
GPIOC->OTYPER &= ~(1<<7);	 // Setting the 6th Pin to 1
	
// Set the Ospeedr to low speed
GPIOC->OSPEEDR &= ~(1<<14);	// Setting the 14th bit to 0, while the 15th bit is no set
GPIOC->OSPEEDR &= ~(1<<12); // Setting the 12th bit to 0, while the 13th bit is no set

// Set the PUPDR to no pull-up/down resistors 
GPIOC->PUPDR  &= ~(1<<12); // Setting the 12th bit to 0
GPIOC->PUPDR  &= ~(1<<13); // Setting the 13th bit to 0
GPIOC->PUPDR  &= ~(1<<14); // Setting the 14th bit to 0
GPIOC->PUPDR  &= ~(1<<15); // Setting the 15th bit to 0

GPIOC->ODR |= (1<<7);  // Set Red as High
GPIOC->ODR &= ~(1<<6); // Set Blue as low
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
GPIOA->PUPDR  &= ~(1<<0); // Set the 0 bit to 0

uint32_t debouncer = 0;
while(1) {
// Toggle the output state of both PC8 and PC9
debouncer = (debouncer << 1); // Always shift every loop iteration
if (GPIOA->IDR & 0x1) { // If input signal is set/high
debouncer |= 0x01; // Set lowest bit of bit-vector
}
if (debouncer == 0xFFFFFFFF) {
// This code triggers repeatedly when button is steady high!
}
if (debouncer == 0x00000000) {
// This code triggers repeatedly when button is steady low!
}
if (debouncer == 0x7FFFFFFF) {
	GPIOC->ODR ^= (1<<7);  // Set Red as low
	GPIOC->ODR ^= (1<<6); // Set Blue as high
}
// When button is bouncing the bit-vector value is random since bits are set when the button is high and not when it bounces low.
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
