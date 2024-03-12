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
SystemClock_Config();

RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
  RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;

  GPIOB -> MODER |= (1 << 23); // PB11 Alternate function
  GPIOB -> MODER &= ~(1 << 22); // PB11 Alternate function

  GPIOB -> OTYPER |= (1 << 11); // PB11 Output open-drain

  GPIOB -> AFR[1] |= (1<<12); // Set alternate function mode
	GPIOB -> AFR[1] &= ~(1<<13);
	GPIOB -> AFR[1] &= ~(1<<14);
	GPIOB -> AFR[1] &= ~(1<<15);

  GPIOB -> MODER |= (1 << 27); // PB13 Alternate function
  GPIOB -> MODER &= ~(1 << 26); // PB13 Alternate function

  GPIOB -> OTYPER |= (1 << 13); // PB13 Output open-drain

  GPIOB -> AFR[1] |= (1<<20); // Set alternate function mode
	GPIOB -> AFR[1] &= ~(1<<21);
	GPIOB -> AFR[1] |= (1<<22);
	GPIOB -> AFR[1] &= ~(1<<23);

  GPIOB -> MODER |= (1 << 28); // PB14 output mode
  GPIOB -> MODER &= ~(1 << 29); // PB14 ouput mode

  GPIOB -> OTYPER &= ~(1 << 14); // PB14 push pull

  GPIOB -> ODR |= (1 << 14); // Set pin 14 to high

  GPIOC -> MODER |= (1 << 0); // PC0 output mode
  GPIOC -> MODER &= ~(1 << 1); // PC0 ouput mode
 
  GPIOB -> OTYPER &= ~(1 << 0); // PC0 push pull

  GPIOB -> ODR |= (1 << 0); // Set pin 0 to high

  // Setting I2C2 to 1kHz
  I2C2 -> TIMINGR |= 0x13;
  I2C2 -> TIMINGR |= (0xF << 8);
  I2C2 -> TIMINGR |= (0x2 << 16);
  I2C2 -> TIMINGR |= (0x4 << 20);
  I2C2 -> TIMINGR |= (1 << 28);

  I2C2 -> CR1 |= (1 << 0);

  
  // Setting slave address
  I2C2 -> CR2 |= (0x69 << 1);
  I2C2 -> CR2 |= (1 << 16);
  I2C2 -> CR2 &= ~(1 << 10);
  I2C2 -> CR2 |= (1 << 13);

  // Wait for TXIS and NAXKF
  while(1) {
    if((I2C2->ISR & (1<<1)) == (1<<1)) {
      break;
    }
    if ((I2C2->ISR & (1<<4)) == (1<<4)) {
      continue;
    }
  }
  
  I2C2 -> TXDR |= 0x0F; // WHO_AM_I

  // Wait for transfer complete
  while(1) {
    if((I2C2->ISR & (1<<6)) == (1<<6)) {
      break;
    }
  }

  // Setting slave address
  I2C2 -> CR2 |= (0x69 << 1);
  I2C2 -> CR2 |= (1 << 16);
  I2C2 -> CR2 |= I2C_CR2_RD_WRN;
  I2C2 -> CR2 |= I2C_CR2_START;

  // Wait for RXNE
  while(1) {
    if(I2C2->ISR & I2C_ISR_RXNE) {
      break;
    }
    if((I2C2->ISR & (1<<4)) == (1<<4)) {
      continue;
    }
  }

  // Wait for transfer complete
  while(1) {
    if((I2C2->ISR & (1<<6)) == (1<<6)) {
      break;
    }
  }

  // if(I2C2 -> RXDR == 0xD3) {
  //   I2C2 -> CR2 |= (1 << 14);
  // }

  I2C2 -> CR2 |= (1 << 14);

  while(1) {}
  
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
