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

#include "main.h" // Include the main header file

volatile char receivedData;       // Global variable to store received data
volatile uint8_t newDataFlag = 0; // Global flag to indicate new data reception
// Function Prototypes
void SystemClock_Config(void);
void USART_TransmitChar(char c);
void USART_TransmitString(const char *str);
char USART_ReceiveChar(void);
void ProcessCommand(char color, char action);
//void USART3_4_IRQHandler(void);


// USART3 Interrupt Handler
void USART3_4_IRQHandler(void)
{
    if (USART3->ISR & USART_ISR_RXNE)
    {                               // Check if RXNE flag is set
        receivedData = USART3->RDR; // Read data, RXNE flag is cleared automatically
        newDataFlag = 1;            // Set new data flag
    }
}

int main(void)
{
    HAL_Init();
	
    // Enable peripheral clock to Port B
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    // Enable peripheral clock to Port C
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

    // Enable the system clock to the USART3 peripheral
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

    SystemClock_Config();

    // The following lines configure GPIO pins 6-9 on port C for output mode to control LEDs

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
    GPIOC->ODR |= (1 << 9);
    GPIOC->ODR |= (1 << 8);
    GPIOC->ODR |= (1 << 7);
    GPIOC->ODR |= (1 << 6);

    // Set pins 11 & 10 to alternate function mode
    GPIOB->MODER &= ~(1 << 20);
    GPIOB->MODER |= (1 << 21);
    GPIOB->MODER &= ~(1 << 22);
    GPIOB->MODER |= (1 << 23);
    GPIOB->MODER &= ~(1 << 22);
    GPIOB->AFR[1] |= (1 << 10); // Set alternate function for USART3 (AF4) on pins 10
    GPIOB->AFR[1] |= (1 << 14); // Set alternate function for USART3 (AF4) on pins 11

    // Set the Baud Rate
    // USART3->BRR = HAL_RCC_GetHCLKFreq() / 9600; // This gets the 9600 baud rate
    USART3->BRR = HAL_RCC_GetHCLKFreq() / 115200; // This gets the 115200 baud rate

    // Enable the Transmitter and Receiver
    // USART3->CR1 |= (USART_CR1_TE | USART_CR1_RE);
    USART3->CR1 |= (1 << 2);
    USART3->CR1 |= (1 << 3);

    // Enable the USART Peripheral
    // USART3->CR1 |= USART_CR1_RXNEIE;
    USART3->CR1 |= (1 << 5); // Enable RXNE interrupt
    // USART3->CR1 |= USART_CR1_UE;
    USART3->CR1 |= (1 << 0); // Enable USART

    // Sets the Baud Rate of 115200.
    USART3->BRR = HAL_RCC_GetHCLKFreq() / 115200; // Set baud rate

    // NVIC configuration for USART3 interrupt
    NVIC_SetPriority(USART3_4_IRQn, 3); // Set priority level, adjust IRQn as per your MCU
    NVIC_EnableIRQ(USART3_4_IRQn);      // Enable USART3 interrupt in NVIC

    // GPIO and USART initialization code here (not shown for brevity)

    char command[3] = {0}; // Array to store the two-character command and null terminator
    int commandIndex = 0;  // Index for command buffer

    USART_TransmitString("CMD?\r\n"); // Prompt for user input

    while (1)
    {
        char receivedChar = USART_ReceiveChar(); // Receive a character

        // Process received character only if it's a letter or a digit
        if ((receivedChar >= 'a' && receivedChar <= 'z') || (receivedChar >= '0' && receivedChar <= '2'))
        {
            command[commandIndex++] = receivedChar; // Store received character in command array

            if (commandIndex == 2)
            {                                           // If two characters have been received
                ProcessCommand(command[0], command[1]); // Process the command
                commandIndex = 0;                       // Reset command index for next command
                USART_TransmitString("CMD?\r\n");       // Prompt for next command
            }
        }
        else
        {
            USART_TransmitString("Error: Unrecognized command\r\nCMD?\r\n"); // Error message for invalid character
            commandIndex = 0;                                                // Reset command index
        }
    }
}

void ProcessCommand(char color, char action)
{
    uint32_t ledPin = 0; // Variable to hold the LED pin based on the color

    // Determine the LED pin based on the color character
    switch (color)
    {
    case 'g':
        ledPin = GPIO_PIN_9;
        break; // Green LED
    case 'o':
        ledPin = GPIO_PIN_8;
        break; // Orange LED
    case 'b':
        ledPin = GPIO_PIN_7;
        break; // Blue LED
    case 'r':
        ledPin = GPIO_PIN_6;
        break; // Red LED
    default:
        USART_TransmitString("Error: Invalid color\r\n");
        return; // Invalid color character
    }

    // Perform the action based on the action character
    switch (action)
    {
    case '0':
        GPIOC->ODR &= ~ledPin;
        USART_TransmitString("LED Off\r\n");
        break; // Turn off the LED
    case '1':
        GPIOC->ODR |= ledPin;
        USART_TransmitString("LED On\r\n");
        break; // Turn on the LED
    case '2':
        GPIOC->ODR ^= ledPin;
        USART_TransmitString("LED Toggled\r\n");
        break; // Toggle the LED
    default:
        USART_TransmitString("Error: Invalid action\r\n"); // Invalid action character
    }
}

// Assume USART_TransmitChar is a function you've previously defined to transmit a single character
void USART_TransmitChar(char c)
{
    // Wait for the transmit data register to be empty
    while (!(USART3->ISR & USART_ISR_TXE))
        ;
    USART3->TDR = c; // Send the character
}

// Function to transmit a string over USART
void USART_TransmitString(const char *str)
{
    while (*str != '\0')
    {                             // Loop until the null character is encountered
        USART_TransmitChar(*str); // Transmit the current character
        str++;                    // Move to the next character in the string
    }
}
// Assume USART_TransmitChar is a function you've previously defined to transmit a single character
char USART_ReceiveChar(void)
{
    // Wait for the transmit data register to be empty
    while (!(USART3->ISR & USART_ISR_RXNE))
        ; // Replace 'USARTx' with your USART instance
    return (char)(USART3->RDR & 0xFF);
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
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
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

#ifdef USE_FULL_ASSERT
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

