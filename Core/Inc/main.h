/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct
{
	uint32_t	Flag_1ms	:1;
	uint32_t	Flag_10ms	:1;
	uint32_t	Flag_20ms	:1;
	uint32_t	Flag_50ms	:1;
	uint32_t	Flag_100ms	:1;
	uint32_t	Flag_200ms	:1;
	uint32_t	Flag_250ms	:1;
	uint32_t	Flag_500ms	:1;
	uint32_t	Flag_1000ms	:1;
	uint32_t	Count_1ms;
} systick;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SW_Pin GPIO_PIN_13
#define SW_GPIO_Port GPIOC
#define SW_EXTI_IRQn EXTI15_10_IRQn
#define LED_G_Pin GPIO_PIN_5
#define LED_G_GPIO_Port GPIOA
#define Direction1_Pin GPIO_PIN_5
#define Direction1_GPIO_Port GPIOC
#define I2CMP_RST_Pin GPIO_PIN_14
#define I2CMP_RST_GPIO_Port GPIOB
#define I2CMP1_RST_Pin GPIO_PIN_15
#define I2CMP1_RST_GPIO_Port GPIOB
#define Direction0_Pin GPIO_PIN_8
#define Direction0_GPIO_Port GPIOC
#define Direction2_Pin GPIO_PIN_12
#define Direction2_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
