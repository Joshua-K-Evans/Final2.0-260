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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/*** Globals *********/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "reyax_driver.h"
#define BUFFER_SIZE 50

/*  The network will be the same for these two devices
 The Network ID in LoRa communication, such as the one set by LoRa_SetNetworkID(15)
 is a parameter used to define a group of devices that can communicate with each
 other within a LoRa network.
 */
#define LoRa_NetworkID 13

#define alphaAddress 1
#define betaAddress 2
bool button_flag = false;
// Alpha and Beta are buffers for the two Reyax
char alphaRX_Buffer[BUFFER_SIZE] = { 0 };
char alphaTX_Buffer[BUFFER_SIZE] = { 0 };

char betaTX_Buffer[BUFFER_SIZE] = { 0 };
char betaRX_Buffer[BUFFER_SIZE] = { 0 };
char from_alpha_message[] = "TOGGLE LED";
char from_beta_message[] = " Greetings from beta";
// TTYRX_Buffer comes from the terminal console
char TTYRX_Buffer[BUFFER_SIZE] = { 0 };

UART_HandleTypeDef *alphaUART, *betaUART;

uint8_t alphaRX_Index = 0;  // Index for the buffer
uint8_t betaRX_Index = 0;  // Index for the buffer
uint8_t alphabyteReceived = 0; // Store the last received byte
uint8_t betabyteReceived = 0; // Store the last received byte

uint8_t recvd_data; // byte in from USART
volatile uint8_t betaDataReadyFlag = 0; // Global flag to signal data received
MessageElement alpha_element, beta_element;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define I2C_ADDR 0x27 // I2C address of the PCF8574
#define RS_BIT 0 // Register select bit
#define EN_BIT 2 // Enable bit
#define BL_BIT 3 // Backlight bit
#define D4_BIT 4 // Data 4 bit
#define D5_BIT 5 // Data 5 bit
#define D6_BIT 6 // Data 6 bit
#define D7_BIT 7 // Data 7 bit
#define LCD_ROWS 2 // Number of rows on the LCD
#define LCD_COLS 16 // Number of columns on the LCD
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = { .name = "defaultTask",
		.stack_size = 512 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for AlphaRXTask */
osThreadId_t AlphaRXTaskHandle;
const osThreadAttr_t AlphaRXTask_attributes = { .name = "AlphaRXTask",
		.stack_size = 512 * 4, .priority = (osPriority_t) osPriorityLow, };
/* Definitions for BetaRXTask */
osThreadId_t BetaRXTaskHandle;
const osThreadAttr_t BetaRXTask_attributes = { .name = "BetaRXTask",
		.stack_size = 512 * 4, .priority = (osPriority_t) osPriorityLow, };
/* Definitions for toggleLED */
osThreadId_t toggleLEDHandle;
const osThreadAttr_t toggleLED_attributes = { .name = "toggleLED", .stack_size =
		128 * 4, .priority = (osPriority_t) osPriorityLow, };
/* Definitions for toggleLED2task */
osThreadId_t toggleLED2taskHandle;
const osThreadAttr_t toggleLED2task_attributes = { .name = "toggleLED2task",
		.stack_size = 128 * 4, .priority = (osPriority_t) osPriorityLow, };
/* Definitions for got_alpha_message */
osSemaphoreId_t got_alpha_messageHandle;
const osSemaphoreAttr_t got_alpha_message_attributes = { .name =
		"got_alpha_message" };
/* Definitions for got_beta_message */
osSemaphoreId_t got_beta_messageHandle;
const osSemaphoreAttr_t got_beta_message_attributes = { .name =
		"got_beta_message" };
/* Definitions for toggleLEDSem */
osSemaphoreId_t toggleLEDSemHandle;
const osSemaphoreAttr_t toggleLEDSem_attributes = { .name = "toggleLEDSem" };
/* Definitions for toggleLED2sem */
osSemaphoreId_t toggleLED2semHandle;
const osSemaphoreAttr_t toggleLED2sem_attributes = { .name = "toggleLED2sem" };
/* USER CODE BEGIN PV */
uint8_t backlight_state = 1;

uint32_t value_adc[2];
const int adcChannelCount = sizeof(value_adc) / sizeof(value_adc[0]);
volatile int adcConversionComplete = 0;
uint16_t x = 250;
uint16_t y = 250;
bool run = true;

char xInt[] = "";
char yInt[] = "";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void *argument);
void GetAlphaRXTask(void *argument);
void GetBetaRXTask(void *argument);
void ToggleLED(void *argument);
void toggleLED2(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void lcd_write_nibble(uint8_t nibble, uint8_t rs) {
	uint8_t data = nibble << D4_BIT;
	data |= rs << RS_BIT;
	data |= backlight_state << BL_BIT; // Include backlight state in data
	data |= 1 << EN_BIT;
	HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDR << 1, &data, 1, 100);
	HAL_Delay(1);
	data &= ~(1 << EN_BIT);
	HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDR << 1, &data, 1, 100);
}
void lcd_send_cmd(uint8_t cmd) {
	uint8_t upper_nibble = cmd >> 4;
	uint8_t lower_nibble = cmd & 0x0F;
	lcd_write_nibble(upper_nibble, 0);
	lcd_write_nibble(lower_nibble, 0);
	if (cmd == 0x01 || cmd == 0x02) {
		HAL_Delay(2);
	}
}
void lcd_send_data(uint8_t data) {
	uint8_t upper_nibble = data >> 4;
	uint8_t lower_nibble = data & 0x0F;
	lcd_write_nibble(upper_nibble, 1);
	lcd_write_nibble(lower_nibble, 1);
}
void lcd_init() {
	HAL_Delay(50);
	lcd_write_nibble(0x03, 0);
	HAL_Delay(5);
	lcd_write_nibble(0x03, 0);
	HAL_Delay(1);
	lcd_write_nibble(0x03, 0);
	HAL_Delay(1);
	lcd_write_nibble(0x02, 0);
	lcd_send_cmd(0x28);
	lcd_send_cmd(0x0C);
	lcd_send_cmd(0x06);
	lcd_send_cmd(0x01);
	HAL_Delay(2);
}
void lcd_write_string(char *str) {
	while (*str) {
		lcd_send_data(*str++);
	}
}
void lcd_set_cursor(uint8_t row, uint8_t column) {
	uint8_t address;
	switch (row) {
	case 0:
		address = 0x00;
		break;
	case 1:
		address = 0x40;
		break;
	default:
		address = 0x00;
	}
	address += column;
	lcd_send_cmd(0x80 | address);
}
void lcd_clear(void) {
	lcd_send_cmd(0x01);
	HAL_Delay(2);
}
void lcd_backlight(uint8_t state) {
	if (state) {
		backlight_state = 1;
	} else {
		backlight_state = 0;
	}
}

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

void Read_and_Transmit_Task() {
	uint8_t receive_byte;
	uint8_t bytes_in = 0;
	uint8_t xmitmsg[] = "\n\rInput Line to Send from Alpha ->";
	uint8_t sndmsg[] = "\n\rSending -> ";
	uint8_t *xmitmsg_ptr = xmitmsg;
	uint8_t *sndmsg_ptr = sndmsg;

	bytes_in = 0;
	receive_byte = 0;
	// Send out the
	HAL_UART_Transmit(&huart2, xmitmsg_ptr, 23, HAL_MAX_DELAY);

	/* This task reads a line from the Serial/USB port and
	 * transmits out to the REYAX using the drivers
	 * Note that this is polling!  One byte at a time.  Very inefficient
	 */
	while (receive_byte != '\r') {
		while (HAL_UART_Receive(&huart2, &receive_byte, 1, 10) != HAL_OK)
			HAL_Delay(1);
		/* Now we have a byte, if it's a carriage return, send the string
		 * If not, put it on the buffer
		 */
		TTYRX_Buffer[bytes_in] = receive_byte;
		HAL_UART_Transmit(&huart2, (uint8_t*) &TTYRX_Buffer[bytes_in++], 1,
		HAL_MAX_DELAY);  //echo each one as it's typed
	}

	TTYRX_Buffer[bytes_in++] = '\n'; // Add a line_feed
	// Tell the User what we got and what we're sending
	HAL_UART_Transmit(&huart2, sndmsg_ptr, 13, HAL_MAX_DELAY);
	//HAL_UART_Transmit(&huart2, receive_buffer_ptr, bytes_in, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*) &TTYRX_Buffer, bytes_in,
	HAL_MAX_DELAY);

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
//  HAL_ADC_Start_DMA(&hadc1,(uint32_t*) value_adc, adcChannelCount);
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */
	osKernelInitialize();
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_USART1_UART_Init();
	MX_USART3_UART_Init();
	MX_TIM2_Init();
	MX_TIM1_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */

	alphaUART = &huart1;
	betaUART = &huart3;
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

//  red();
//  Set_Brightness(45);
//  WS2812_Send();

	// I2C pull-up resistors
	GPIOB->PUPDR |= 0b01 << (8 * 2);
	GPIOB->PUPDR |= 0b01 << (9 * 2);
	// Initialize the LCD
	lcd_init();
	lcd_backlight(1); // Turn on backlight
	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* Create the semaphores(s) */
	/* creation of got_alpha_message */
	got_alpha_messageHandle = osSemaphoreNew(1, 1,
			&got_alpha_message_attributes);

	/* creation of got_beta_message */
	got_beta_messageHandle = osSemaphoreNew(1, 1, &got_beta_message_attributes);

	/* creation of toggleLEDSem */
	toggleLEDSemHandle = osSemaphoreNew(1, 0, &toggleLEDSem_attributes);

	/* creation of toggleLED2sem */
	toggleLED2semHandle = osSemaphoreNew(1, 0, &toggleLED2sem_attributes);

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* For a normal example, wouldn't need both alpha and beta queues.  Each side is going to
	 * have it's own processor and a single receive queue
	 */

	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of defaultTask */
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL,
			&defaultTask_attributes);

	/* creation of AlphaRXTask */
	AlphaRXTaskHandle = osThreadNew(GetAlphaRXTask, NULL,
			&AlphaRXTask_attributes);

	/* creation of BetaRXTask */
	BetaRXTaskHandle = osThreadNew(GetBetaRXTask, NULL, &BetaRXTask_attributes);

	/* creation of toggleLED */
	toggleLEDHandle = osThreadNew(ToggleLED, NULL, &toggleLED_attributes);

	/* creation of toggleLED2task */
	toggleLED2taskHandle = osThreadNew(toggleLED2, NULL,
			&toggleLED2task_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* creation of AlphaRXTask */
	AlphaRXTaskHandle = osThreadNew(GetAlphaRXTask, NULL,
			&AlphaRXTask_attributes);
	/* creation of BetaRXTask */
	BetaRXTaskHandle = osThreadNew(GetBetaRXTask, NULL, &BetaRXTask_attributes);

	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
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

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 10;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x10D19CE4;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 90 - 1;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 160 - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 4294967295;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

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
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LED2_Pin | LED1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : Button_Push_Pin */
	GPIO_InitStruct.Pin = Button_Push_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(Button_Push_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LED2_Pin LED1_Pin */
	GPIO_InitStruct.Pin = LED2_Pin | LED1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : ToggleButton_Pin */
	GPIO_InitStruct.Pin = ToggleButton_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(ToggleButton_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	char from_alpha_message[] = "TOGGLE LED";
	MessageElement sample_from_alpha;
	sample_from_alpha.theType = OK_MSG;
	if (GPIO_Pin == ToggleButton_Pin) {

		strncpy(sample_from_alpha.theBuffer, "TOGGLE LED",
		MAX_MESSAGE_LENGTH - 1);
		HAL_UART_Receive_IT(betaUART, &betabyteReceived, 1); //start receiving messages before a command
		LoRa_SendMessage(alpha, betaAddress, from_alpha_message);
	}
	if (GPIO_Pin == Button_Push_Pin) {
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		button_flag = true;
	}

//	if (GPIO_Pin == SENSOR_Pin) {  // Check if the sensor detects the laser
//			hit = 1;
//
//
//	   } else {
//	      // Sensor did not detect the laser, turn off the blue LED and Laser
//		   hit = 0;
//
//	    }

}
/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
PUTCHAR_PROTOTYPE {
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART1 and Loop until the end of transmission */
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF);

	return ch;
}
/*          Interrupt service routine for UART RX
 *
 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
#define TERMINATION_CHAR '\n'
	if (huart == alphaUART) {
		// Store the received byte in the buffer
		if (alphabyteReceived == TERMINATION_CHAR) { // Don't store it, just put it all on the queue
			alpha_element = processReceivedData(alphaRX_Buffer);
			alphaRX_Index = 0;
		} else {
			//HAL_UART_Transmit(&huart2, &alphabyteReceived, 1, HAL_MAX_DELAY);
			alphaRX_Buffer[alphaRX_Index++] = alphabyteReceived; // this will add alpha bytes received onto the buffer, increasing the index number every time
			HAL_UART_Receive_IT(alphaUART, &alphabyteReceived, 1);
			alphaRX_Buffer[alphaRX_Index] = 0;
		}
	}
	if (huart == betaUART) {
		if (betabyteReceived == TERMINATION_CHAR) {
			beta_element = processReceivedData(betaRX_Buffer);
			betaRX_Index = 0;
			betaDataReadyFlag = 1;
			//osSemaphoreRelease(got_beta_messageHandle);
		} else {
			//HAL_UART_Transmit(&huart2, &betabyteReceived, 1, HAL_MAX_DELAY); /* do not use this. it messes up the transmission*/

			betaRX_Buffer[betaRX_Index++] = betabyteReceived;
			HAL_UART_Receive_IT(betaUART, &betabyteReceived, 1);
			betaRX_Buffer[betaRX_Index] = 0;
			//betaRX_Index = 0;
		}
	}

}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument) {
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	printf("\033\143"); //Clear the terminal

	MessageElement sample_from_alpha, sample_from_beta;

	sample_from_alpha.theType = OK_MSG;
	sample_from_beta.theType = OK_MSG;
	strncpy(sample_from_alpha.theBuffer, "TOGGLE LED", MAX_MESSAGE_LENGTH - 1);
	strncpy(sample_from_beta.theBuffer, "Greetings from beta",
	MAX_MESSAGE_LENGTH - 1);

	printf("Welcome to the REYAX Demo\n\r");
	lcd_write_string("Welcome");

	sample_from_alpha.theType = ADDRESS_MSG;
	sample_from_beta.theType = ADDRESS_MSG;

// HAL_UART_Receive_IT(&huart1, &alphabyteReceived, 1); //start receiving messages before a command
// LoRa_SendCommand(alphaUART,"AT");

// setup the network and the address for both:
// Alpha
	LoRa_SetNetworkID(alpha, NETWORK_ID);
	LoRa_SetAddress(alpha, ALPHA_ADDRESS);
	LoRa_SetNetworkID(beta, NETWORK_ID);
	LoRa_SetAddress(beta, BETA_ADDRESS);

	HAL_UART_Receive_IT(betaUART, &betabyteReceived, 1); //start receiving messages before a command
//	LoRa_SendMessage(alpha, betaAddress, from_alpha_message);
	HAL_UART_Receive_IT(alphaUART, &alphabyteReceived, 1); //start receiving messages before a command
	osDelay(1500);	// added a delay in between the two send commands
//	LoRa_SendMessage(beta, alphaAddress, from_beta_message);
//	while(1);	// commented out

// HAL_UART_Receive_IT(&huart3, &betabyteReceived, 1); //start receiving messages before a command
// LoRa_SendCommand(betaUART,"AT");

	for (;;) {

		if (button_flag){
			HAL_UART_Receive_IT(betaUART, &betabyteReceived, 1);
			strncpy(sample_from_alpha.theBuffer, "Push Button",
			MAX_MESSAGE_LENGTH - 1);
			LoRa_SendMessage(alpha, betaAddress, "Push Button");
			button_flag = false;
		}
	}

	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_GetAlphaRXTask */
/**
 * @brief Function implementing the AlphaRXTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_GetAlphaRXTask */
void GetAlphaRXTask(void *argument) {
	/* USER CODE BEGIN GetAlphaRXTask */

	/* Infinite loop */
	for (;;) {
		if (osSemaphoreAcquire(got_alpha_messageHandle, osWaitForever)
				== osOK) {
			// Process the received message
//			lcd_clear();
//			lcd_write_string(alpha_element.theBuffer);

			osSemaphoreRelease(got_alpha_messageHandle);
		}
		osDelay(10);
	}
//		osDelay(1);
}
/* USER CODE END GetAlphaRXTask */

/* USER CODE BEGIN Header_GetBetaRXTask */
/**
 * @brief Function implementing the BetaRXTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_GetBetaRXTask */
void GetBetaRXTask(void *argument) {
	/* USER CODE BEGIN GetBetaRXTask */
	/* Infinite loop */
	int localCount = 0;
	char msgBuf[20];
	for (;;) {
		// Wait for a message to be available
		osSemaphoreAcquire(got_beta_messageHandle, osWaitForever); // Wait till there's a message

		// Check if the message is "Toggle Pin"
		if (strncmp(beta_element.theBuffer, "Push Button", 11) == 0) {
			snprintf(msgBuf, 19,  "Pressed: %d", ++localCount);
			lcd_clear();
			lcd_write_string(msgBuf);
		}

//		osSemaphoreRelease(got_beta_messageHandle);

		// Semaphore is taken
		osDelay(1);
	}
	/* USER CODE END GetBetaRXTask */
}
	/* USER CODE END GetBetaRXTask */


/* USER CODE BEGIN Header_ToggleLED */
/**
 * @brief Function implementing the toggleLED thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_ToggleLED */
void ToggleLED(void *argument) {
	/* USER CODE BEGIN ToggleLED */

	for (;;) {
		osSemaphoreAcquire(toggleLEDSemHandle, osWaitForever);
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin); // Toggle LED
	}
	/* USER CODE END ToggleLED */
}

/* USER CODE BEGIN Header_toggleLED2 */
/**
 * @brief Function implementing the toggleLED2task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_toggleLED2 */
void toggleLED2(void *argument) {
	/* USER CODE BEGIN toggleLED2 */
	/* Infinite loop */
	for (;;) {
		osSemaphoreAcquire(toggleLED2semHandle, osWaitForever);
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin); // Toggle LED
	}
	/* USER CODE END toggleLED2 */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM8 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM8) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
