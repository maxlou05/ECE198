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
#include "i2c-lcd.h"
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  // Initialize the I2C connection mode and the LCD display using the library
  lcd_init();

  /* IMPORTANT!!! Addresses of the display lines:
   * Line 1: 0x80|0x00 --> from 0x00(0)  to 0x13(19)
   * Line 3: 0x80|0x14 --> from 0x14(20) to 0x27(39)
   * Line 2: 0x80|0x40 --> from 0x40(40) to 0x53(59)
   * Line 4: 0x80|0x54 --> from 0x54(60) to 0x67(79)
   * NOTE!!! You always need to apply OR with 0x80 (1000 0000) (set bit 8 to 1)
   * 	ie. lcd_send_cmd(0x80|0x00); to move the cursor to line 1 position 1
  */
  unsigned char LINE_1_START = 0x80|0x00;
  unsigned char LINE_2_START = 0x80|0x40;
  unsigned char LINE_3_START = 0x80|0x14;
  unsigned char LINE_4_START = 0x80|0x54;
  unsigned char CLEAR_SCREEN = 0x1;
  unsigned char CURSOR_ON = 0xE;
  unsigned char CURSOR_OFF = 0xC;
  unsigned char CURSOR_BLINK_ON = 0xD;
  unsigned char CURSOR_BLINK_OFF = 0xC;
  unsigned char DISPLAY_OFF = 0x08;
  unsigned char DISPLAY_ON = CURSOR_OFF;

  lcd_send_cmd(CURSOR_BLINK_ON);
//  lcd_send_cmd(LINE_1_START);
  lcd_set_cursor(0,0); // Set cursor to line 1 start
  lcd_send_string("HELLO WORLD!");  // Print text at cursor (just make sure this isn't more than 20 chars, or else it can't display it)
//  lcd_send_cmd(LINE_2_START);
  lcd_set_cursor(1, 0);
  lcd_send_string("LCD 20x4 demo");
//  lcd_send_cmd(LINE_3_START + 9);  // Since it's just a number, you can just add stuff to make it go to the next character
  lcd_set_cursor(2, 9);
  lcd_send_string("by");
//  lcd_send_cmd(LINE_4_START);
  lcd_set_cursor(3, 0);
  lcd_send_string("Controllers Tech");
  HAL_Delay(3000);

  // Custom characters (can only store up to 8 at a time on the board though)
  char cc1[] = {0x00, 0x00, 0x0A, 0x00, 0x11, 0x0E, 0x00, 0x00};  // smiley
  char cc2[] = {0x0E, 0x0E, 0x04, 0x0E, 0x15, 0x04, 0x0A, 0x0A};  // Robo
  char cc3[] = {0x08, 0x0C, 0x0E, 0x0F, 0x0E, 0x0C, 0x08, 0x00};  // arrow
  char cc4[] = {0x00, 0x04, 0x0E, 0x0E, 0x0E, 0x1F, 0x04, 0x00};  // bell
  char cc5[] = {0x00, 0x00, 0x0A, 0x15, 0x11, 0x0E, 0x04, 0x00};  // Heart
  char cc6[] = {0x00, 0x0E, 0x11, 0x11, 0x11, 0x0A, 0x1B, 0x00};  // omega
  char cc7[] = {0x0E, 0x10, 0x17, 0x12, 0x12, 0x12, 0x10, 0x0E};  // CT
  char cc8[] = {0x04, 0x04, 0x1F, 0x04, 0x04, 0x00, 0x1F, 0x00};  // +-

  // store the chars into the CGRAM (up to 8)
  lcd_send_cmd(0x40);
  for (int i=0; i<8; i++) lcd_send_data(cc1[i]);

  lcd_send_cmd(0x40+8);
  for (int i=0; i<8; i++) lcd_send_data(cc2[i]);

  lcd_send_cmd(0x40+16);
  for (int i=0; i<8; i++) lcd_send_data(cc3[i]);

  lcd_send_cmd(0x40+24);
  for (int i=0; i<8; i++) lcd_send_data(cc4[i]);

  lcd_send_cmd(0x40+32);
  for (int i=0; i<8; i++) lcd_send_data(cc5[i]);

  lcd_send_cmd(0x40+40);
  for (int i=0; i<8; i++) lcd_send_data(cc6[i]);

  lcd_send_cmd(0x40+48);
  for (int i=0; i<8; i++) lcd_send_data(cc7[i]);

  lcd_send_cmd(0x40+56);
  for (int i=0; i<8; i++) lcd_send_data(cc8[i]);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  lcd_send_cmd(CURSOR_ON);
	  lcd_send_cmd(CLEAR_SCREEN);
	  lcd_set_cursor(1, 0);
	  lcd_send_string("my string!@)+#$%^&");
	  lcd_set_cursor(2, 0);
	  lcd_send_data(0);
	  lcd_send_data(4);
	  HAL_Delay(3000);
	  lcd_set_cursor(1, 0);
	  HAL_Delay(1000);
	  lcd_send_cmd(DISPLAY_OFF);
	  HAL_Delay(1000);
	  lcd_send_cmd(CURSOR_OFF);
	  lcd_set_cursor(1, 0);
	  lcd_send_data(96);  // You can also just send a char like this too
	  lcd_send_string("no cursor{}[]\\|*");  // Some characters do not display properly
	  HAL_Delay(3000);
    /* USER CODE END WHILE */

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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart2.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
