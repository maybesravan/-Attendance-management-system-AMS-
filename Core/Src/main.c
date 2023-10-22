/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include"as608.h"
#include<stdbool.h>
#include"string.h"
#include"stdio.h"
#include "retarget.h"
#include<stdlib.h>
#include "lcd1602.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"

#define RXBUFFER_SIZE	50
#define MAX_PEOPLE 256
#define MAX_NAME_LENGTH 256
#define MAX_NUM_TEMPLATES 254
#define timer htim3
#define MAX_RETRIES 3 // Maximum number of retries
extern TIM_HandleTypeDef timer;
void delay (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&timer, 0);
	while (__HAL_TIM_GET_COUNTER(&timer) < us);
}

extern uint8_t Tone_Status;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
int stored_fingerprint[5][100];
int fingerprint_data;
int id = 0;

bool removeFingerPlaceFinger = false;
bool fingerprintStored = false;
uint8_t fingerprintStatus = FINGERPRINT_NOFINGER;
bool fingerprintOk = false;
bool fingerprintDeleted = false;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
GPIO_InitTypeDef GPIO_InitStructPrivate = {0};
uint32_t previousMillis = 0;
uint32_t currentMillis = 0;
uint8_t keyPressed = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int row=0;
int col=0;
typedef struct
{
	uint16_t start_code; ///< "Wakeup" code for packet detection
	uint8_t address[4]; ///< 32-bit Fingerprint sensor address
	uint8_t type; ///< Type of packet
	uint16_t length; ///< Length of packet
	uint8_t data[64]; ///< The raw buffer for packet payload
}as608Packet_t;

static as608Packet_t packet;
static uint8_t rxPacketBuffer[RXBUFFER_SIZE];
uint8_t FindFingerprint(void);
void EnrollFinger(uint8_t id);
uint8_t AS608_TEMPLATECOUNT(void);
uint8_t AS608_StoreModel(uint16_t id);
void InputFinger(void);
void DeleteFinger(void);
void playTone(uint16_t frequency,uint16_t ms,uint8_t dc);
void stopTone(void);


uint16_t matchingPageID;
uint16_t f_status;
uint8_t Rxbuff = 0;
uint8_t RxbuffCntr = 0;
uint8_t txCplt = SET;
uint8_t RxCplt = 0;
uint8_t keypressed =  0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim3);
	lcd_init ();
	lcd_clear();
	RetargetInit(&huart2);
	HAL_UART_Receive_IT(&huart1, (uint8_t *)&Rxbuff, sizeof(Rxbuff));
	HAL_UART_Receive_IT(&huart2, (uint8_t *)&Rxbuff, sizeof(Rxbuff));
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	printf("\nchoose\nA.Enroll\nB.Find\nC.Delete\nD.DeleteAll\n");
 while (1)
  {
    /* USER CODE END WHILE */
	 if (keyPressed!=0x00)
	 		{
	 			HAL_UART_Transmit(&huart2,(uint8_t*)&keyPressed,1,100);
	 			keyPressed=0;
	 		}
	 		HAL_Delay(1000);
	 		lcd_clear();
	 		lcd_put_cur(0, 0);
	 		HAL_Delay(1500);
	 		lcd_clear();
	 		lcd_init ();
	 		if (keyPressed == 0)
	 		{
	 			lcd_send_string("1enr2fnd3del4delA");
	 			HAL_Delay(100);
	 		}
	 		else if(keyPressed ==65 )
	 		{
	 			keyPressed = 0;
	 			InputFinger();
	 		}
	 		else if(keyPressed == 66)
	 		{
	 			keyPressed = 0;
	 			FindFingerprint();
	 		}
	 		else if(keyPressed ==67 )
	 		{
	 			keyPressed = 0;
	 			DeleteFinger();
	 		}
	 		else if (keyPressed == 68){
	 			keyPressed = 0;
	 			AS608_EmptyDatabase();
	 			lcd_send_string("Allfingerdeleted");
	 			printf("ALL DELETED");
	 			playTone(800,1000,20);
	 			stopTone();
	 		}
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
	htim3.Init.Prescaler = 72-1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 0xffff-1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.RepetitionCounter = 0;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 57600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
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
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 57600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RS_Pin|D5_Pin|D6_Pin|D7_Pin
                          |RW_Pin|EN_Pin|D4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : RS_Pin D5_Pin D6_Pin D7_Pin
                           RW_Pin EN_Pin D4_Pin */
  GPIO_InitStruct.Pin = RS_Pin|D5_Pin|D6_Pin|D7_Pin
                          |RW_Pin|EN_Pin|D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

	HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);

	HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);

	HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);

	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint8_t FindFingerprint(void)
{
	HAL_Delay(1000); // Delay for 1 second

	uint8_t f_status = AS608_GetImage(); // Attempt to capture a fingerprint image

	if (f_status != FINGERPRINT_OK) {
		printf("error in the getimage"); // Print an error message to the console
		lcd_send_string("errorinfinger"); // Send an error message to the LCD
		HAL_Delay(1000); // Delay for 1 second
		lcd_clear(); // Clear the LCD display
		return f_status; // Return the status code
	}

	HAL_Delay(1000); // Delay for 1 second

	f_status = AS608_Image2Tz(1); // Convert the captured image to a template

	if (f_status != FINGERPRINT_OK) {
		printf("convert fail"); // Print an error message to the console
		lcd_send_string("convertinfinger"); // Send an error message to the LCD
		HAL_Delay(1000); // Delay for 1 second
		lcd_clear(); // Clear the LCD display
		return f_status; // Return the status code
	}

	HAL_Delay(1000); // Delay for 1 second

	int pointer[3]; // Create an array to store 2 integers
	char idString[4]; // Create a character array to hold the string representation of the ID

	f_status = AS608_FingerFastSearch(pointer); // Search for a matching fingerprint

	id = pointer[0]; // Assign the ID to the variable 'id'
	printf("id=%d", id); // Print the ID to the console
	sprintf(idString, "%d", id); // Convert the integer to a string

	if (f_status == FINGERPRINT_OK) {
		HAL_Delay(100); // Delay for 0.1 second
		printf("Fingerprint found with id = %d\n", pointer[0]); // Print the ID of the found fingerprint to the console
		lcd_clear(); // Clear the LCD display
		HAL_Delay(1500); // Delay for 1.5 seconds
		lcd_send_string("Finger Found\n"); // Send a message to the LCD
		lcd_send_string(idString); // Send the ID string to the LCD
		 playTone(500,1000,20);
		stopTone();
		HAL_Delay(1500); // Delay for 1.5 seconds
		lcd_clear(); // Clear the LCD display
		return f_status; // Return the status code
	} else {
		printf("Fingerprint not found\n"); // Print a message indicating the fingerprint was not found
		lcd_send_string("finger not found"); // Send a message to the LCD
		lcd_send_string(idString); // Send the ID string to the LCD
		 playTone(500,1000,20);
		stopTone();
		HAL_Delay(2000); // Delay for 2 seconds to display the message
		lcd_clear(); // Clear the LCD display
		// HAL_Delay(1000); // Delay for 1 second to display the message
	}

	return f_status; // Return the status code
}


void DeleteFinger(void)
{
	HAL_Delay(1000); // Delay for 1 second

	char ids[3]; // Create an array to store 2 characters
	uint8_t i = 0; // Initialize a counter variable
	// Loop to get 2 key presses
	while (i < 2)
	{
		HAL_Delay(1000); // Delay for 1 seconds

		//		// Start UART reception in interrupt mode to receive 2 characters
		//		HAL_UART_Receive_IT(&huart2, (uint8_t *)&Rxbuff, 2);

		printf("press the keys\n"); // Prompt the user to press keys
		HAL_Delay(1000);
		lcd_send_string("press the keys");
		HAL_Delay(1000); // Delay for 1 seconds
		lcd_clear();
		ids[i] = keyPressed; // Store the key press in 'ids' array
		i++; // Increment the counter
		HAL_Delay(1000); // Delay for 1 seconds
	}

	ids[2] = '\0'; // Add a null-terminator to the end of the 'ids' string

	id = atoi(ids); // Convert the two-character string 'ids' to an integer 'id'

	// Call a function to delete the fingerprint model associated with 'id'
	AS608_DeleteModel(id);

	printf("Finger is deleted with id = %d", id); // Print a message indicating successful deletion
	lcd_send_string("DElFINGER="); // Send a message "DElFINGER=" to the LCD
	lcd_send_string(ids); // Send the string 'ids' to the LCD
	   playTone(500,1000,20);
			  		        stopTone();
	HAL_Delay(1500); // Delay for 1.5 seconds (1500 milliseconds)
	lcd_clear(); // Clear the LCD display
	keypressed = 0; // Reset a variable 'keypressed' to 0

}

char ids[3]; // Create an array to store 3 characters
void InputFinger(void)
{
	HAL_Delay(1000); // Delay for 1 second
	printf("press the first key\n"); // Prompt the user to press the first key
	lcd_send_string("pressthefirstkey\n");
	HAL_Delay(2000); // Delay for 1 seconds
	lcd_clear();
	ids[0] = keyPressed; // Store the key press in the first position of 'ids'
	printf("press the second key\n"); // Prompt the user to press the second key
	lcd_send_string("press2ndkey");
	HAL_Delay(1000); // Delay for 1 seconds
	lcd_clear();
	HAL_Delay(2000); // Delay for 2 seconds
	ids[1] = keyPressed; // Store the key press in the second position of 'ids'

	//	// Start UART reception in interrupt mode to receive 2 characters
	//	HAL_UART_Receive_IT(&huart2, (uint8_t *)&ids, 2);
	HAL_Delay(1000); // Delay for 1 second
	id = atoi(ids); // Convert the two-character string 'ids' to an integer 'id'
	if (id >= 0 && id <= 99) {
		EnrollFinger(id); // Call a function (EnrollFinger) with the 'id' as a parameter
	} else {
		printf("\nInvalid id. Please enter an id between 1 and 100.");
		lcd_send_string("Invalid id");
		HAL_Delay(1000); // Delay for 1 seconds
		lcd_clear();
	}
	keypressed = 0; // Reset a variable 'keypressed' to 0
}

void EnrollFinger(uint8_t id) {
	// Step 1: Prompt the user to place their finger on the sensor
	printf("Place finger on the sensor\n");
	lcd_send_string("Place finger");
	HAL_Delay(1000); // Delay for 1 seconds
	lcd_clear();

	// Step 2: Capture an image of the fingerprint
	while (AS608_GetImage() != FINGERPRINT_OK) { HAL_Delay(1000);break;}

	// Step 3: Convert the captured image to a template
	while (AS608_Image2Tz(1) != FINGERPRINT_OK) {HAL_Delay(1000);break; }

	// Step 4: Prompt the user to remove and re-place the same finger
	printf("Remove finger and place the same finger again\n");
	lcd_send_string("Remove and place");
	HAL_Delay(1000); // Delay for 1 seconds
	lcd_clear();
	removeFingerPlaceFinger = true;

	// Step 5: Capture another image of the fingerprint
	while (AS608_GetImage() != FINGERPRINT_OK) { HAL_Delay(1000);break;}

	// Step 6: Convert the second captured image to a template
	while (AS608_Image2Tz(2) != FINGERPRINT_OK) {HAL_Delay(1000); break;}

	removeFingerPlaceFinger = false;

	// Step 7: Create a fingerprint model from the two templates
	if (AS608_CreateModel() == FINGERPRINT_OK) {
		// Step 8: Choose the appropriate buffer ID and page ID
		uint16_t pageID = id;
		char idString[10]; // Create a character array to hold the string representation of the integer
		sprintf(idString, "%d", id); // Convert the integer to a string

		// Step 9: Store the fingerprint model in the specified buffer
		if (AS608_StoreModel( pageID) == FINGERPRINT_OK) {
			printf("Fingerprint enrolled for ID: %d\n", id); // Print a message indicating successful fingerprint enrollment along with the 'id'
			lcd_send_string("ENROLL ID="); // Send "ENROLL ID=" to the LCD
			lcd_send_string(idString); // Send the string representation of 'id' to the LCD
			 playTone(800,1000,20);
				 	 	     stopTone();

			HAL_Delay(1500); // Delay for 1.5 seconds
			lcd_clear(); // Clear the LCD display
		}
	}
	else {
        fingerprintStored = false;  // Prints unmatched
        lcd_clear();
        lcd_send_string("fingernotenroled");
        playTone(800,100,20);

	 	     stopTone();
	}
}

//Static functions
static void PacketInit(uint8_t type,uint16_t length,uint8_t* data)
{
	packet.start_code = FINGERPRINT_STARTCODE;
	packet.type = type;
	packet.length = length;
	for(uint8_t i = 0; i < 4; i++)
	{
		packet.address[i] = 0xFF;
	}
	if(length < 50)
	{
		memcpy(packet.data,data,length);

	}
	else
	{
		memcpy(packet.data,data,64);
	}

}

/**
	@brief Process a packet and send it over UART to the sensor
 */
static void WriteStructuredPacket(void)
{
	uint8_t leftshiftstartcode;
	uint8_t bitwiseand;
	uint8_t leftshiftwirelength;
	uint8_t bitwiseandwirelength;
	uint8_t leftshiftsum;
	uint8_t bitwiseandsum;

	leftshiftstartcode = packet.start_code >> 8;
	bitwiseand = packet.start_code & 0xFF;
	HAL_UART_Transmit(&huart1,(&leftshiftstartcode),1,HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1,(&bitwiseand),1,HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1,(&packet.address[0]),4,4);
	HAL_UART_Transmit(&huart1,(&packet.type),1,4);
	uint16_t wireLength = packet.length + 2;
	leftshiftwirelength = wireLength >> 8;
	bitwiseandwirelength = wireLength & 0xFF;
	HAL_UART_Transmit(&huart1,(&leftshiftwirelength),1,HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1,(&bitwiseandwirelength),1,HAL_MAX_DELAY);
	uint16_t sum = wireLength + packet.type;
	for (uint8_t i = 0; i < packet.length; i++)
	{
		HAL_UART_Transmit(&huart1,(&packet.data[i]),1,HAL_MAX_DELAY);
		sum += packet.data[i];
	}
	leftshiftsum = sum >> 8;
	bitwiseandsum = sum & 0xFF;
	HAL_UART_Transmit(&huart1,(&leftshiftsum),1,HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1,(&bitwiseandsum),1,HAL_MAX_DELAY);
}

/**
	@brief Receives data over UART from the sensor via DMA then processes
	the data into a packet/
	@return <code>FINGERPRINT_OK</code> on success
	@return <code>FINGERPRINT_TIMEOUT</code> or
					<code>FINGERPRINT_BADPACKET</code> on failure
 */
static uint8_t GetStructuredPacket(void)
{
	uint8_t nReceivedBytes = 0;
	while(1)
	{
		if(RxCplt == SET)
		{
			RxCplt = RESET;
			nReceivedBytes = RxbuffCntr;
			RxbuffCntr = 0;
			break;
		}
	}
	if(rxPacketBuffer[0] == (FINGERPRINT_STARTCODE >> 8))
	{
		packet.start_code = (uint16_t)rxPacketBuffer[0] << 8;
	}
	packet.start_code |= rxPacketBuffer[1];
	if(packet.start_code != FINGERPRINT_STARTCODE)
	{
		memset(rxPacketBuffer,0,nReceivedBytes); //clear the rx packet buffer
		return FINGERPRINT_BADPACKET;
	}
	for(uint8_t i = 2; i <= 5; i++)
	{
		packet.address[i - 2] = rxPacketBuffer[i];
	}
	packet.type = rxPacketBuffer[6];
	packet.length = (uint16_t)rxPacketBuffer[7] << 8;
	packet.length |= rxPacketBuffer[8];
	for(uint8_t i = 9; i < nReceivedBytes; i++)
	{
		packet.data[i - 9] = rxPacketBuffer[i];
		if((i - 8) == packet.length)
		{
			memset(rxPacketBuffer,0,nReceivedBytes); //clear the rx packet buffer
			return FINGERPRINT_OK;
		}
	}
	return FINGERPRINT_BADPACKET;
}

//Variadic macros
/**
 * @brief Gets the command packet
 */
#define GET_CMD_PACKET(...)                                                    \
		do {                                                                          \
			uint8_t packet_data[] = {__VA_ARGS__};                                      \
			PacketInit(FINGERPRINT_COMMANDPACKET, sizeof(packet_data), packet_data);    \
			WriteStructuredPacket();                                                    \
			if (GetStructuredPacket() != FINGERPRINT_OK)                                \
			return FINGERPRINT_PACKETRECIEVEERR;                                      \
			if (packet.type != FINGERPRINT_ACKPACKET)                                   \
			return FINGERPRINT_PACKETRECIEVEERR;                                      \
		} while (0)


/**
 * @brief Sends the command packet
 */
#define SEND_CMD_PACKET(...)                                                   \
		GET_CMD_PACKET(__VA_ARGS__);                                                 \
		return packet.data[0];

/**
	@brief Ask the sensor to take an image of the finger pressed on surface
	@return <code>FINGERPRINT_OK</code> on success
	@return <code>FINGERPRINT_NOFINGER</code> if no finger detected
	@return <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
	@return <code>FINGERPRINT_IMAGEFAIL</code> on imaging error
 */
uint8_t AS608_GetImage(void)
{
	SEND_CMD_PACKET(FINGERPRINT_GETIMAGE);
}

/**
	@brief Ask the sensor to convert image to feature template
	@param slot Location to place feature template (put one in 1 and another in
	2 for verification to create model)
	@return <code>FINGERPRINT_OK</code> on success
	@return <code>FINGERPRINT_IMAGEMESS</code> if image is too messy
	@return <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
	@return <code>FINGERPRINT_FEATUREFAIL</code> on failure to identify
	fingerprint features
	@return <code>FINGERPRINT_INVALIDIMAGE</code> on failure to identify
	fingerprint features
 */
uint8_t AS608_Image2Tz(uint8_t slot)
{
	SEND_CMD_PACKET(FINGERPRINT_IMAGE2TZ,slot );
}

/**
	@brief Ask the sensor to take two print feature template and create a
	model
	@return <code>FINGERPRINT_OK</code> on success
	@return <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
	@return <code>FINGERPRINT_ENROLLMISMATCH</code> on mismatch of fingerprints
 */
uint8_t AS608_CreateModel(void)
{
	SEND_CMD_PACKET(FINGERPRINT_REGMODEL);
}

/**
	@brief Ask the sensor to delete ALL models in memory
	@return <code>FINGERPRINT_OK</code> on success
	@return <code>FINGERPRINT_BADLOCATION</code> if the location is invalid
	@return <code>FINGERPRINT_FLASHERR</code> if the model couldn't be written
	to flash memory
	@return <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
 */
uint8_t AS608_EmptyDatabase(void)
{
	SEND_CMD_PACKET(FINGERPRINT_EMPTY);
}


uint8_t AS608_TEMPLATECOUNT(void)
{
	GET_CMD_PACKET(FINGERPRINT_TEMPLATECOUNT,0x01,7,0x00);
	return packet.data[0];
}
/**
	@brief Ask the sensor to store the calculated model for later matching
	@param location The model location #
	@return <code>FINGERPRINT_OK</code> on success
	@return <code>FINGERPRINT_BADLOCATION</code> if the location is invalid
	@return <code>FINGERPRINT_FLASHERR</code> if the model couldn't be written
	to flash memory
	@return <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
 */

uint8_t AS608_StoreModel(uint16_t id)
{
	SEND_CMD_PACKET(FINGERPRINT_STORE, 0x01, (uint8_t)(id >> 8), (uint8_t)(id & 0xFF));
}

/**
	@brief Ask the sensor to load a fingerprint model from flash into buffer 1
	@param location The model location
	@return <code>FINGERPRINT_OK</code> on success
	@return <code>FINGERPRINT_BADLOCATION</code> if the location is invalid
	@return <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
 */
uint8_t AS608_LoadModel(uint16_t id)
{
	SEND_CMD_PACKET(FINGERPRINT_LOAD, 0x01, (uint8_t)(id >> 8), (uint8_t)(id & 0xFF));
}

/**
	@brief Ask the sensor to transfer 256-byte fingerprint template from the
	buffer to the UART
	@return <code>FINGERPRINT_OK</code> on success
	@return <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
 */
uint8_t AS608_GetModel(void)
{
	SEND_CMD_PACKET(FINGERPRINT_UPLOAD, 0x01);
}


/**
	@brief Ask the sensor to delete a model in memory
	@param location The model location #
	@return <code>FINGERPRINT_OK</code> on success
	@return <code>FINGERPRINT_BADLOCATION</code> if the location is invalid
	@return <code>FINGERPRINT_FLASHERR</code> if the model couldn't be written
	to flash memory
	@return <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
 */
uint8_t AS608_DeleteModel(uint16_t id)
{
	SEND_CMD_PACKET(FINGERPRINT_DELETE, (uint8_t)(id >> 8), (uint8_t)(id & 0xFF), 0x00, 0x01);

}

/**
	@brief Ask the sensor to search the current slot 1 fingerprint features to
	match saved templates. The matching location is stored in <b>fingerID</b> and
	the matching confidence in <b>confidence</b>
	@return <code>FINGERPRINT_OK</code> on fingerprint match success
	@return <code>FINGERPRINT_NOTFOUND</code> no match made
	@return <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
 */
uint8_t AS608_FingerFastSearch(int *ptr)
{
	GET_CMD_PACKET(FINGERPRINT_HISPEEDSEARCH, 0x01, 0x00, 0x00, 0x00, 0xA3);
	//	if(packet.data[0]==0x00)
	//	{
	//		printf("finger found with id=%d\n",packet.data[2]);
	//
	//	}
	*ptr=packet.data[2];
	return packet.data[0];
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	txCplt = SET;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		printf("%c",Rxbuff);

		HAL_UART_Receive_IT(&huart1, (uint8_t *)&Rxbuff, sizeof(Rxbuff));//, HAL_MAX_DELAY);//(&huart1,(uint8_t *)&Rxbuff,1);
		rxPacketBuffer[RxbuffCntr++] = Rxbuff;
		if(RxbuffCntr >= 12)
		{
			RxCplt = SET;
		}
	}
	else if(huart->Instance==USART2)
	{
		HAL_UART_Receive_IT(&huart2, (uint8_t *)&Rxbuff, sizeof(Rxbuff));

		// Check for specific condition to call enroll() for UART2
		if (Rxbuff == 'A')
		{
			keypressed = 'A';
			//InputFinger();
		}
		else if (Rxbuff == 'B'){
			keypressed = 'B';
			//  FindFingerprint();
		}
		else if (Rxbuff == 'C'){
			keypressed = 'C';
			//			  DeleteFinger();
		}
		else if (Rxbuff == 'D'){
			keypressed = 'D';
			//			  AS608_EmptyDatabase();
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	currentMillis = HAL_GetTick();
	if (currentMillis - previousMillis > 1000) {
		/*Configure GPIO pins : PB6 PB7 PB8 PB9 to GPIO_INPUT*/
		GPIO_InitStructPrivate.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
		GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
		GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
		GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStructPrivate);

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
		if(GPIO_Pin == GPIO_PIN_6 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))
		{
			keyPressed = 49; //ASCII value of D
		}
		else if(GPIO_Pin == GPIO_PIN_7 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7))
		{
			keyPressed = 50; //ASCII value of C
		}
		else if(GPIO_Pin == GPIO_PIN_8 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8))
		{
			keyPressed = 51; //ASCII value of B
		}
		else if(GPIO_Pin == GPIO_PIN_9 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9))
		{
			keyPressed = 65; //ASCII value of A
		}

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
		if(GPIO_Pin == GPIO_PIN_6 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))
		{
			keyPressed = 52; //ASCII value of #
		}
		else if(GPIO_Pin == GPIO_PIN_7 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7))
		{
			keyPressed = 53; //ASCII value of 9
		}
		else if(GPIO_Pin == GPIO_PIN_8 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8))
		{
			keyPressed = 54; //ASCII value of 6
		}
		else if(GPIO_Pin == GPIO_PIN_9 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9))
		{
			keyPressed = 66; //ASCII value of 3
		}

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
		if(GPIO_Pin == GPIO_PIN_6 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))
		{
			keyPressed = 55; //ASCII value of 0
		}
		else if(GPIO_Pin == GPIO_PIN_7 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7))
		{
			keyPressed = 56; //ASCII value of 8
		}
		else if(GPIO_Pin == GPIO_PIN_8 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8))
		{
			keyPressed = 57; //ASCII value of 5
		}
		else if(GPIO_Pin == GPIO_PIN_9 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9))
		{
			keyPressed = 67; //ASCII value of 2
		}

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
		if(GPIO_Pin == GPIO_PIN_6 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))
		{
			keyPressed = 42; //ASCII value of *
		}
		else if(GPIO_Pin == GPIO_PIN_7 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7))
		{
			keyPressed = 48; //ASCII value of 7
		}
		else if(GPIO_Pin == GPIO_PIN_8 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8))
		{
			keyPressed = 35; //ASCII value of 4
		}
		else if(GPIO_Pin == GPIO_PIN_9 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9))
		{
			keyPressed = 68; //ASCII value of 1
		}

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
		/*Configure GPIO pins : PB6 PB7 PB8 PB9 back to EXTI*/
		GPIO_InitStructPrivate.Mode = GPIO_MODE_IT_RISING;
		GPIO_InitStructPrivate.Pull = GPIO_PULLDOWN;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStructPrivate);
		previousMillis = currentMillis;
	}
}

void send_to_lcd (char data, int rs)
{
	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, rs);  // rs = 1 for data, rs=0 for command



	/* write the data to the respective pin */
	HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, ((data>>3)&0x01));
	HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, ((data>>2)&0x01));
	HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, ((data>>1)&0x01));
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, ((data>>0)&0x01));

	/* Toggle EN PIN to send the data
	 * if the HCLK > 100 MHz, use the  20 us delay
	 * if the LCD still doesn't work, increase the delay to 50, 80 or 100..
	 */
	delay (100);
	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, 1);
	delay (100);
	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, 0);
}

void lcd_send_cmd (char cmd)
{
	char datatosend;
	/*send upper nibble first */
	datatosend = ((cmd>>4)&0x0f);
	send_to_lcd(datatosend,0);  // RS must be 0 while sending command

	/* send Lower Nibble */
	datatosend = ((cmd)&0x0f);
	send_to_lcd(datatosend, 0);
}

void lcd_send_data (char data)
{
	char datatosend;
	/* send higher nibble */
	datatosend = ((data>>4)&0x0f);
	send_to_lcd(datatosend, 1);  // rs =1 for sending data
	/* send Lower nibble */
	datatosend = ((data)&0x0f);
	send_to_lcd(datatosend, 1);
}

void lcd_clear (void)
{
	lcd_send_cmd(0x01);
	HAL_Delay(2);
}

void lcd_put_cur(int row, int col)
{
	switch (row)
	{
	case 0:
		col |= 0x80;
		break;
	case 1:
		col |= 0xC0;
		break;
	}
	lcd_send_cmd (col);
}

void lcd_init (void)
{
	// 4 bit initialisation
	HAL_Delay(50); // wait for >40ms
	lcd_send_cmd (0x30); //command for 4 bit initialization.
	HAL_Delay(5);  // wait for >4.1ms
	lcd_send_cmd (0x30); //command for 4 bit initialization.
	HAL_Delay(1);  // wait for >100us
	lcd_send_cmd (0x30); //command for 4 bit initialization.
	HAL_Delay(10);
	lcd_send_cmd (0x20);  // 4bit mode
	HAL_Delay(10);

	// dislay initialisation
	lcd_send_cmd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	HAL_Delay(1);
	lcd_send_cmd (0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
	HAL_Delay(1);
	lcd_send_cmd (0x01);  // clear display
	HAL_Delay(1);
	HAL_Delay(1);
	lcd_send_cmd (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	HAL_Delay(1);
	lcd_send_cmd (0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}

void lcd_send_string (char *str)
{
	while (*str) lcd_send_data (*str++);
}

void playTone(uint16_t frequency,uint16_t ms,uint8_t dc)
{
	// HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, SET);
	//int duration = 1000/frequency;//calculate the duration of tone
	__HAL_TIM_SET_PRESCALER(&htim2,84000000/(frequency*frequency));//set the prescaler to generate the desired frequency.
	__HAL_TIM_SET_AUTORELOAD(&htim2,ms);//set the auto-reload value to generate the desired duration

	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	htim2.Instance->CCR1 = dc;//vary the duty cycle
	HAL_Delay(ms);
	/**** FOR STOPPING THE BUZZER********/
	//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
	// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, RESET);
	//  HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);

}

void stopTone(void)
{
	HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);

}


void eraseFlashSector(uint32_t sector)
{
	FLASH_EraseInitTypeDef eraseInitStruct;
	uint32_t sectorError = 0;
	HAL_StatusTypeDef status;

	// Unlock the Flash memory
	HAL_FLASH_Unlock();

	// Fill erase structure
	eraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	eraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	eraseInitStruct.Sector = sector;
	eraseInitStruct.NbSectors = 1;

	// Erase the desired sector
	status = HAL_FLASHEx_Erase(&eraseInitStruct, &sectorError);

	// Lock the Flash memory
	HAL_FLASH_Lock();

	// Check if erase operation was successful
	if (status != HAL_OK)
	{
		// Handle erase error
		// ...
	}
}
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
