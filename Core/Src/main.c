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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "I2CMultiplex.h"
#include "IMU.h"
#include "Force.h"
#include "EEPROM.h"
#include "CLI.h"
#include "WIFI.h"
#include "Motor.h"
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
systick Systime;
uint8_t SW_Select, SW_flag, flag, minute_cnt = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		uart_hal_rx.buffer[uart_hal_rx.input_p++] = uart_hal_rx.temp;
		if(uart_hal_rx.input_p >= UART_RX_BUFFER_SIZE)
		{
			uart_hal_rx.input_p = 0;
		}
		HAL_UART_Receive_IT(&WiFi_UART, &uart_hal_rx.temp, 1);
	}
	else if(huart->Instance == USART6)
	{

		uart_hal_rx_CLI.cli_buffer[uart_hal_rx_CLI.cli_input_p++] = uart_hal_rx_CLI.cli_temp;
		if(uart_hal_rx_CLI.cli_input_p >= UART_RX_CLI_BUFFER_SIZE)
		{
			uart_hal_rx_CLI.cli_input_p = 0;
		}
		HAL_UART_Receive_IT(&CLI_UART, &uart_hal_rx_CLI.cli_temp, 1);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == SW_Pin)
	{
		SW_flag = 1;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM10)
	{
	    Systime.Flag_1ms = 1;
		Systime.Count_1ms++;
		if((Systime.Count_1ms % 10) == 0) Systime.Flag_10ms = 1;
		if((Systime.Count_1ms % 20) == 0) Systime.Flag_20ms = 1;
		if((Systime.Count_1ms % 50) == 0) Systime.Flag_50ms = 1;
		if((Systime.Count_1ms % 100) == 0) Systime.Flag_100ms = 1;
		if((Systime.Count_1ms % 200) == 0) Systime.Flag_200ms = 1;
		if((Systime.Count_1ms % 250) == 0) Systime.Flag_250ms = 1;
		if((Systime.Count_1ms % 500) == 0) Systime.Flag_500ms = 1;
		if(Systime.Count_1ms == 1000)
		{
			Systime.Flag_1000ms = 1;
			Systime.Count_1ms = 0;
		}
	}
}
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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM2_Init();
  MX_TIM10_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim10);
#ifdef Glove_mode
  IMU_I2CMultiplexer0_RST();
#endif
  Force_I2CMultiplexer1_RST();
//  Wifi_HW_Reset();

  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);

  HAL_UART_Receive_IT(&WiFi_UART, &uart_hal_rx.temp, 1);
  uart_hal_rx_buffer_init();

  CalculateSModelLine(fre, period, ACCELERATED_SPEED_LENGTH, FRE_MAX, FRE_MIN, flex);

  Motor_PWM_init();
#ifdef Glove_mode
  HAL_Delay(1000);
  ALL_IMU_Initialization();
#endif
//  SW_Select = HAL_GPIO_ReadPin(SW_GPIO_Port, SW_Pin);
  SW_Select = 0;
  if(SW_Select)
  {
	  flag = 1;
  }
  else
  {
	  flag = 0;
	  HAL_UART_Receive_IT(&CLI_UART, &uart_hal_rx_CLI.cli_temp, 1);
	  uart_hal_rx_CLI_buffer_init();
	  Menu_Tree(0);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#if 0
	  TIM2->CCR1 = 350;
	  HAL_Delay(1000);
	  TIM2->CCR1 = 1000;
	  HAL_Delay(1000);
#else
	  if(Systime.Flag_1000ms)
	  {
		  Systime.Flag_1000ms = 0;
		  if(!Realtime_Start)
		  {
			  minute_cnt++;
			  if(minute_cnt % 300 == 0)
			  {
				  WifiStatus();
				  minute_cnt = 0;
			  }
		  }
	  }

	  if(flag)
	  {

	  }
	  else
	  {
		  if(Sensor_Start)
		  {
#ifdef Glove_mode
				IMU_process();
#endif
				Force_Process();
		  }

		  if(Motor_Start)
		  {
//			  Runhand_Motor_Process();
			  One_fingur_2Motor(1);
			  Stepper_motor(0, Roll_buf[1]);
			  Stepper_motor(1, Roll_buf[2]);
			  Stepper_motor(2, Roll_buf[3]);
			  Motor_Start = 0;
		  }
		  if(Realtime_Start)
		  {
//			  Realtime_Motor_Process();
//			  Motor_Test();
			  One_fingur_2Motor(0);
			  Stepper_motor(0, f_Roll1);
			  Stepper_motor(1, f_Roll2);
			  Stepper_motor(2, f_Roll3);
//			  Stepper_motor(3, f_Roll4);
		  }

		  if(Realtime_recv)
		  {
			  Realtime_recv = 0;
			  Realtime_Error = Realtime_Recieve_data();
			  if(Realtime_Error)
			  {
				  printf("Overflow Error\r\n");
				  memset(uart_hal_rx.buffer_temp, 0, sizeof(uart_hal_rx.buffer_temp));
			  }
			  else
			  {
				  Ready_Realtime_rx_Send();
			  }
			  uart_hal_rx.buf_p = 0;
			  Realtime_flag = 1;
		  }
		  uart_hal_rx_CLI_monitor();
	  }

	  uart_hal_rx_monitor();
#endif
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
	printf("ERROR\n");
//	HAL_Delay(100);
//	NVIC_SystemReset();
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
