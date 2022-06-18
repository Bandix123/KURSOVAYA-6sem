/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <ctype.h>
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
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
int8_t led_array[39][8] = {  // все по строчкам

		{ 0x3C, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x3C },  //0
				{ 0x10, 0x30, 0x50, 0x10, 0x10, 0x10, 0x10, 0x7c },  //1
				{ 0x7E, 0x2, 0x2, 0x7E, 0x40, 0x40, 0x40, 0x7E },  //2
				{ 0x7E, 0x2, 0x2, 0x7E, 0x2, 0x2, 0x2, 0x7E },  //3
				{ 0x4, 0xC, 0x14, 0x24, 0x7E, 0x4, 0x4, 0x4 },  //4
				{ 0x7E, 0x40, 0x40, 0x7E, 0x2, 0x2, 0x2, 0x7E },  //5
				{ 0x7E, 0x40, 0x40, 0x40, 0x7E, 0x42, 0x42, 0x7E },  //6
				{ 0x3E, 0x22, 0x4, 0x8, 0x8, 0x8, 0x8, 0x8 },  //7
				{ 0x7E, 0x42, 0x42, 0x7E, 0x42, 0x42, 0x42, 0x7E },  //8
				{ 0x3E, 0x22, 0x22, 0x3E, 0x2, 0x2, 0x2, 0x3E },  //9
				{ 0x18, 0x24, 0x42, 0x42, 0x7E, 0x42, 0x42, 0x42 }, //A
				{ 0x7C, 0x42, 0x42, 0x7C, 0x42, 0x42, 0x42, 0x7C }, //B
				{ 0x3C, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x3C }, //C
				{ 0x7C, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x7C }, //D
				{ 0x7C, 0x40, 0x40, 0x7C, 0x40, 0x40, 0x40, 0x7C }, //E
				{ 0x7C, 0x40, 0x40, 0x7C, 0x40, 0x40, 0x40, 0x40 }, //F
				{ 0x3C, 0x40, 0x40, 0x40, 0x4c, 0x44, 0x44, 0x3C }, //G
				{ 0x44, 0x44, 0x44, 0x7C, 0x44, 0x44, 0x44, 0x44 }, //H
				{ 0x7C, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x7C }, //I
				{ 0x3C, 0x8, 0x8, 0x8, 0x8, 0x8, 0x48, 0x30 }, //J
				{ 0x44, 0x48, 0x50, 0x60, 0x60, 0x50, 0x48, 0x44 }, //K
				{ 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x7C }, //L
				{ 0x42, 0x66, 0x5A, 0x42, 0x42, 0x42, 0x42, 0x42 }, //M
				{ 0x42, 0x42, 0x62, 0x72, 0x4E, 0x46, 0x42, 0x42 }, //N
				{ 0x3C, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x3C }, //O
				{ 0x3C, 0x22, 0x22, 0x22, 0x3C, 0x20, 0x20, 0x20 }, //P
				{ 0x1C, 0x22, 0x22, 0x22, 0x22, 0x26, 0x22, 0x1D }, //Q
				{ 0x3C, 0x22, 0x22, 0x22, 0x3C, 0x24, 0x22, 0x21 }, //R
				{ 0x3E, 0x40, 0x40, 0x30, 0xC, 0x2, 0x2, 0x7C }, //S
				{ 0x3E, 0x8, 0x8, 0x8, 0x8, 0x8, 0x8, 0x8 }, //T
				{ 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x3C }, //U
				{ 0x42, 0x42, 0x42, 0x42, 0x42, 0x24, 0x24, 0x18 }, //V
				{ 0x42, 0x42, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x3C }, //W
				{ 0x42, 0x42, 0x24, 0x18, 0x18, 0x24, 0x42, 0x42 }, //X
				{ 0x41, 0x22, 0x14, 0x8, 0x8, 0x8, 0x8, 0x8 }, //Y
				{ 0x7E, 0x2, 0x4, 0x8, 0x10, 0x20, 0x40, 0x7E }, //Z
				{ 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 } // space
		};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart1, (uint8_t*) &ch, 1, 0xFFFF);
	return 0;
}
void Spi1_init(void) {
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	GPIOA->MODER |= GPIO_MODER_MODER6_0;
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_6;
	GPIOA->MODER &= ~(GPIO_MODER_MODER5 | GPIO_MODER_MODER7);
	GPIOA->MODER |= GPIO_MODER_MODER5_1 | GPIO_MODER_MODER7_1;
	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL5 | GPIO_AFRL_AFRL7);

	SPI1->CR1 |= SPI_CR1_CPOL | SPI_CR1_CPHA; // задаем режим работы SPI (полярность и фазу)
	SPI1->CR1 |= SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE;     // задаем режим передачи

	SPI1->CR2 |= SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2; // задаем формат данных (разрядность)
	SPI1->CR1 &= ~SPI_CR1_LSBFIRST;    // задаем способ передачи (формат фрейма)
	SPI1->CR1 |= SPI_CR1_SSM; // задаем режим управления NSS (аппаратное или программное)
	SPI1->CR1 |= SPI_CR1_SSI; // задаем режим управления NSS (аппаратное или программное)
	SPI1->CR2 &= ~SPI_CR2_SSOE;              // задаем режим внешнего вывода NSS
	SPI1->CR1 |= SPI_CR1_MSTR; // задаем режим работы модул SPI (ведущий или ведомый)
	SPI1->CR1 |= SPI_CR1_SPE;                             // включаем модуль SPI
}
void write_byte(uint8_t byte) {
	for (int i = 0; i < 8; i++) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);        // set the clock pin LOW
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, byte & 0x80); // write the MSB bit to the data pin
		byte = byte << 1;                                   // shift left
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);       // set the clock pin HIGH
	}
}

void write_matrix(uint8_t address, uint8_t data) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);  // pull the CS pin LOW
	write_byte(address);
	write_byte(data);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);  // pull the CS pin HIGH
}
void matrix_init(void) {
	write_matrix(0x09, 0x00);       //  no decoding
	write_matrix(0x0a, 0x05);       //  brightness
	write_matrix(0x0b, 0x0F);       //  scan limit = 16 LEDs
	write_matrix(0x0c, 0x01);       //  power down =0,normal mode = 1
	write_matrix(0x0f, 0x00);       //  no display test
}

void write_string(char *str) {
	while (*str) {
		for (int i = 1; i < 9; i++) {
			write_matrix(i, led_array[(*str - 55)][i - 1]);
		}
		str++;
		HAL_Delay(500);
	}
}

int8_t* get_symbol_line(char symbol) { //костыль для маппинга
	if (symbol >= 'A' && symbol <= 'Z') {
		return led_array[symbol - 55];
	} else if (symbol >= '0' && symbol <= '9') {
		return led_array[symbol - 48];
	} else if (symbol == ' ') {
		return led_array[38];
	}
}

uint16_t compute_sum(char *str, int i) { //получаем указатель на символ, она смотрит, если последний, то возвращяет его, и справа 8 бит.
// Если
	uint16_t sum = 0;
	if (*(str + 1)) //понимаем, что не последний символ
	{
		sum = (((uint16_t) get_symbol_line(*str)[i - 1]) << 8)
				| (uint16_t) get_symbol_line(*(str + 1))[i - 1];
	} else {
		sum = (((uint16_t) get_symbol_line(*str)[i - 1]) << 8);
	}
	return sum;
}

void run_string(char *str) {
	int j = 0;
	uint16_t sum = 0;
	int k = 0;
	while (*str) {
		if (k == 0) {
			for (int j = -8; j < 9; j++) //чтобы начать с пустой матрицы
					{
				for (int i = 1; i < 9; i++) {
					sum = compute_sum(str, i);
					write_matrix(i, sum >> (8 - j));
				}

				HAL_Delay(100);
			}
		} else {
			for (int j = 1; j < 9; j++) //чтобы закончить на пустой матрице (доехали до середины
					{
				for (int i = 1; i < 9; i++) {
					sum = compute_sum(str, i);
					write_matrix(i, sum >> (8 - j));
				}
				HAL_Delay(100);
			}
		}
		k++;
		str++;
	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */

	matrix_init();

	run_string(" ");
//	printf("k");
//	char u[] =
//			"osudfodnfks;djf;idsbf;kajsbdfk;asbd;kfjasdjfeo;ibfkjdsnf;awebuf';dk;fasbdfkjwbe;fiwubfksbafliawubf';asdfkblwafhb";
//	HAL_UART_Transmit(&huart1, (uint8_t *) u, strlen(u), -1);
//	char k[10] = { 0 };
//	HAL_UART_Receive(&huart1, k, 10, -1);
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
//	run_string(k);
	int i = 0;
	char buf[64] = { 0 };
	char c = 'A';
	while (c != '\r') {

		HAL_UART_Receive(&huart1, (uint8_t*) &c, 1, -1);
		buf[i] = c;

		i++;
		HAL_UART_Transmit(&huart1, (uint8_t*) &c, 1, -1);

	}
//	   	   HAL_UART_Transmit(&huart1, "YA \r\n", 6, -1);
//	   	   HAL_UART_Transmit(&huart1, ( *)&i, 8, -1);
//	  	   printf("\r\n %d \r\n", i);

	buf[i] = ' ';
	buf[i + 1] = '\0';

	//printf("kek");
//	HAL_UART_Transmit(&huart1, 'x', 1, -1);
//	for (i = 0; i < 63 && !isspace(c); i++) {
//		HAL_UART_Receive(&huart1, (uint8_t*) &c, 1, -1);
//		buf[i] = toupper(c);
//
////		HAL_UART_Transmit(&huart1, (uint8_t*) &c, 1, -1);
//	}
//	buf[i] = ' ';
//	buf[i + 1] = '\0';

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	//char buf[64] = {0};
	//char c = 0;
	while (1) {

//	   HAL_UART_Transmit(&huart1, "hello worls", strlen("hello worls"), -1);
//	write_string(buf);
			run_string(buf);

//		run_string("ABOBA");
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, Clock_Pin | CS_Pin | DATA_Pin | LED_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pins : Clock_Pin CS_Pin DATA_Pin LED_Pin */
	GPIO_InitStruct.Pin = Clock_Pin | CS_Pin | DATA_Pin | LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

