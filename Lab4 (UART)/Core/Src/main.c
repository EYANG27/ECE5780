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

// Assume USART_TransmitChar is a function you've previously defined to transmit a single character
void USART_TransmitChar(char c) {
    // Wait for the transmit data register to be empty
    while (!(USART3->ISR &  USART_ISR_TXE));  
    USART3->TDR = c;  // Send the character
}

// Function to transmit a string over USART
void USART_TransmitString(const char* str) {
    while (*str != '\0') {  // Loop until the null character is encountered
        USART_TransmitChar(*str);  // Transmit the current character
        str++;  // Move to the next character in the string
    }
}

// Assume USART_TransmitChar is a function you've previously defined to transmit a single character
char USART_ReceiveChar(void) {
    // Wait for the transmit data register to be empty
    while (!(USART3->ISR & USART_ISR_RXNE));  // Replace 'USARTx' with your USART instance
    return (char)(USART3 -> RDR & 0xFF);
}


int main(void)
{
  HAL_Init();

	// Enable peripheral clock to Port B
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	
	// Enable peripheral clock to Port C
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

	 // Enable the system clock to the USART peripheral
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;  
	
  SystemClock_Config();
	
	GPIOC -> MODER |= 0x55;
	GPIOC -> OTYPER |= 0x0000;
	GPIOC -> PUPDR |= 0x00000000;
	GPIOC -> OSPEEDR |= 0x00000000;
	
	// Set pins 11 & 10 to alternate function mode
	GPIOB -> MODER &= ~(1<<20);
	GPIOB -> MODER |= (1<<21);
	GPIOB -> MODER &= ~(1<<22);
	GPIOB -> MODER |= (1<<23);
	GPIOB -> MODER &= ~(1<<22);	
	GPIOB ->AFR[1] |= (1<<10);
	GPIOB ->AFR[1] |= (1<<14);
	
	// Set the Baud Rate
  //USART3->BRR = HAL_RCC_GetHCLKFreq() / 9600; // This gets the 115200 baud rate
	USART3->BRR = HAL_RCC_GetHCLKFreq() / 115200; // This gets the 115200 baud rate

	// Enable the Transmitter and Receiver
  //USART3->CR1 |= (USART_CR1_TE | USART_CR1_RE);  // Enable transmitter and receiver
	
	//USART3->CR1 |= (1<<5);

	USART3->CR1 |= (1<<2);
	USART3->CR1 |= (1<<3);
	
	// Enable the USART Peripheral
  //USART3->CR1 |= USART_CR1_UE;  // Enable USART
	USART3->CR1 |= (1<<0);
	
	char received;
  while (1)
  {
		HAL_Delay(1500);
		USART_TransmitChar('a');
		/*received = USART_ReceiveChar();
		
 // Ensure that USART3->RDR contains new data by checking USART3's status register (e.g., SR, ISR, or similar, depending on the MCU)
    //if (USART3->ISR & USART_ISR_RXNE) {  // This line might need to be adjusted based on your specific MCU
        switch (received) {
            case 'g': // Green LED
                GPIOC->ODR ^= (1 << 9);
                break; // Added break statement
            case 'o': // Orange LED
                GPIOC->ODR ^= (1 << 8);
                break; // Added break statement
            case 'b': // Blue LED
                GPIOC->ODR ^= (1 << 7);
                break; // Added break statement
            case 'r': // Red LED
                GPIOC->ODR ^= (1 << 6);
                break; // Added break statement
            default:
                USART_TransmitString("Error: Unrecognized command\r\n");  // Print error message
                // Ensure that printf is correctly setup for your environment
		*/
      //}
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
