/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c - Edison Yang
 * @brief          : Main program body for Lab5: I2C
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
#include <stdint.h>
#include <stdlib.h>

void SystemClock_Config(void);

// Helper Methods
void Gyroscope(int slaveAddress, int numBytes, int *bytes, int writeRead, int registerAddress);
int bitChecker(int FlagBit);

#define LED_RED (1 << 6)	// PC6
#define LED_BLUE (1 << 7)	// PC7
#define LED_ORANGE (1 << 8) // PC8
#define LED_GREEN (1 << 9)	// PC9
#define SET_LED(ON_LED)                                                                             \
	do                                                                                              \
	{                                                                                               \
		GPIOC->ODR = (GPIOC->ODR & ~((1 << 6) | (1 << 7) | (1 << 8) | (1 << 9))) | (1 << (ON_LED)); \
	} while (0)

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	HAL_Init();

	SystemClock_Config();

	// Enables the GPIOB and GPIOC perpiherals
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

	// Enables the I2C perpiherals
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;

	// Setup red LED (PC6)
	GPIOC->MODER |= (1 << 12);
	// Setup blue LED (PC7)
	GPIOC->MODER |= (1 << 14);
	// Setup orange LED (PC8)
	GPIOC->MODER |= (1 << 16);
	// Setup green LED (PC9)
	GPIOC->MODER |= (1 << 18);

	// PB11
	GPIOB->MODER |= (1 << 23);
	GPIOB->MODER &= ~(1 << 22);
	GPIOB->OTYPER |= (1 << 11);
	GPIOB->AFR[1] |= (1 << 12);

	// PB13
	GPIOB->MODER |= (1 << 27);
	GPIOB->MODER &= ~(1 << 26);
	GPIOB->OTYPER |= (1 << 13);
	GPIOB->AFR[1] |= (1 << 20);
	GPIOB->AFR[1] &= ~(1 << 21);
	GPIOB->AFR[1] |= (1 << 22);
	GPIOB->AFR[1] &= ~(1 << 23);

	// PB14
	GPIOB->MODER &= ~(1 << 29);
	GPIOB->MODER |= (1 << 28);
	GPIOB->OTYPER &= ~(1 << 14);
	GPIOB->ODR |= (1 << 14);

	// PC0
	GPIOC->MODER |= (1 << 0);
	GPIOC->ODR |= (1 << 0);

	// Setting I2C2 to 1kHz
	I2C2->TIMINGR |= 0x13;
	I2C2->TIMINGR |= (0xF << 8);
	I2C2->TIMINGR |= (0x2 << 16);
	I2C2->TIMINGR |= (0x4 << 20);
	I2C2->TIMINGR |= (1 << 28);

	// enable CR1 peripheral
	I2C2->CR1 |= (1 << 0);

	// Enables the PD, Xen, Yen (1011) in binary
	int ctrl_reg1_num = 11;

	Gyroscope(0x69, 1, &ctrl_reg1_num, 0, 0x20);

	int axis[4];

	while (1)
	{
		HAL_Delay(100);
		
		Gyroscope(0x69, 4, axis, 1, 0x28);

		int16_t x = (axis[1] << 8) | axis[0];
		int16_t y = (axis[3] << 8) | axis[2];

		// Tilt detection and LED control

		if (y >= 500)
		{
			if (x >= 500)
			{
				// Tilted towards the positive X and Y
				SET_LED(x > y ? 9 : 6); // GREEN or RED LED
			}
			else if (x <= -500)
			{
				// Tilted towards the negative X and positive Y
				SET_LED(abs(x) > y ? 8 : 6); // ORANGE or RED LED
			}
		}
		else if (y <= -500)
		{
			if (x <= -500)
			{
				// Tilted towards the negative X and Y
				SET_LED(abs(x) > abs(y) ? 8 : 7); // ORANGE or BLUE LED
			}
			else if (x >= 500)
			{
				// Tilted towards the positive X and negative Y
				SET_LED(x > abs(y) ? 9 : 7); // GREEN or BLUE LED
			}
		}
	}
}

/**
 * Monitors specific I2C2 interrupt flags and error conditions.
 * 
 * This function continuously checks for either the NACKF (Not Acknowledge Received Flag) 
 * or a specified operational flag (TXIS or RXNE) in the I2C2 ISR (Interrupt and Status Register).
 * TXIS (Transmit Interrupt Status) is represented by bit 1, indicating the transmit data register is empty.
 * RXNE (Receive Buffer Not Empty) is represented by bit 2, indicating the receive data register is full.
 * 
 * If the NACKF flag (bit 4), indicating a non-acknowledge response, is set during this process, 
 * an error is signaled by illuminating a red LED (assumed to be connected to PC6).
 * 
 * @param FlagBit The bit position of the flag to check (1 for TXIS or 2 for RXNE).
 * @return 1 if the specified operational flag is set, indicating success; 0 if NACKF is set, indicating failure.
 */
int bitChecker(int FlagBit) {
    // Continuously monitor until either NACKF or the specified operational flag is set
    while (!(I2C2->ISR & (1 << 4)) && !(I2C2->ISR & (1 << FlagBit))) {
        // Wait here until one of the conditions is met
    }

    // Check if NACKF is set, indicating a failure in communication
    if (I2C2->ISR & (1 << 4)) {
        GPIOC->ODR |= (1 << 6); // Activate red LED to signal an error condition
        return 0;               // Return 0 to indicate a failure due to NACKF being set
    }

    return 1; // Return 1 to indicate success (TXIS or RXNE flag is set)
}

/**
 * Configures and initiates data transfer with a gyroscope via the I2C2 interface.
 * Depending on the operation mode (read or write), it sets up the I2C2_CR2 register,
 * sends the register address to the gyroscope, and then either reads data from or writes data to the gyroscope.
 *
 * @param slaveAddress The 7-bit I2C slave address of the gyroscope.
 * @param numBytes The number of bytes to read from or write to the gyroscope.
 * @param bytes Pointer to the data buffer for read or write operations.
 * @param writeRead Flag indicating the operation mode: 0 for write, 1 for read.
 * @param registerAddress The address of the gyroscope register to start the read/write operation.
 */
void Gyroscope(int slaveAddress, int numBytes, int *bytes, int writeRead, int registerAddress) {
    // Define constants for bit positions to enhance code readability
    const uint32_t NBYTES_POS = 16; // Position of NBYTES in I2C2_CR2
    const uint32_t SADD_POS = 1;    // Position of SADD in I2C2_CR2
    const uint32_t AUTO_INCREMENT = 128; // Used for register address auto-incrementing

    // Reset necessary bits in the I2C2_CR2 register to prepare for new transaction settings
    I2C2->CR2 &= ~((0xFF << NBYTES_POS) | (0x3FF) | (1 << 10)); // Clear NBYTES, SADD, and RD_WRN

    // Configure the I2C2_CR2 register for the specific operation (read or write)
    if (writeRead) {
        // For read operations, only the register address byte is initially transmitted
        I2C2->CR2 |= (1 << NBYTES_POS) | (slaveAddress << SADD_POS);
    } else {
        // For write operations, set the number of bytes to be written including the register address
        I2C2->CR2 |= ((numBytes + 1) << NBYTES_POS) | (slaveAddress << SADD_POS);
    }

    // Initiate the transaction by setting the START bit
    I2C2->CR2 |= (1 << 13);

    // Wait for acknowledgment from the gyro before proceeding
    if (bitChecker(1)) {
        // Send the register address; set MSB for auto-increment if multiple bytes are involved
        I2C2->TXDR = registerAddress | (numBytes > 1 ? AUTO_INCREMENT : 0);

        if (writeRead) {
            // For read operations: wait for the register address transmission to complete
            while (!(I2C2->ISR & (1 << 6))); // Wait for transmission completion

            // Reconfigure I2C2_CR2 for reading data from the gyroscope
            I2C2->CR2 &= ~((0xFF << NBYTES_POS) | (0x3FF));
            I2C2->CR2 |= (numBytes << NBYTES_POS) | (slaveAddress << SADD_POS) | (1 << 10);

            // Restart the I2C transaction for reading
            I2C2->CR2 |= (1 << 13);

            // Read the specified number of bytes into the provided buffer
            for (int i = 0; i < numBytes; i++) {
                while (!bitChecker(2)); // Wait for the data to be ready
                bytes[i] = I2C2->RXDR;  // Store the received byte
            }
        } else {
            // For write operations: transmit all specified bytes to the gyroscope
            for (int i = 0; i < numBytes; i++) {
                while (!bitChecker(1)); // Wait for the transmit buffer to be ready
                I2C2->TXDR = bytes[i];  // Send the byte
            }
        }
    }

    // Wait for the entire transaction to complete before releasing the bus
    while (!(I2C2->ISR & (1 << 6)));

    // Signal the end of the I2C transaction by setting the STOP bit
    I2C2->CR2 |= (1 << 14);
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