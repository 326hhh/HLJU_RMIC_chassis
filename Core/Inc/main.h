/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

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
#define L_STBY_Pin GPIO_PIN_4
#define L_STBY_GPIO_Port GPIOA
#define L_AIN1_Pin GPIO_PIN_4
#define L_AIN1_GPIO_Port GPIOC
#define L_AIN2_Pin GPIO_PIN_5
#define L_AIN2_GPIO_Port GPIOC
#define L_BIN1_Pin GPIO_PIN_0
#define L_BIN1_GPIO_Port GPIOB
#define L_BIN2_Pin GPIO_PIN_1
#define L_BIN2_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_2
#define LED_GPIO_Port GPIOB
#define R_STBY_Pin GPIO_PIN_7
#define R_STBY_GPIO_Port GPIOE
#define R_AIN1_Pin GPIO_PIN_8
#define R_AIN1_GPIO_Port GPIOE
#define R_AIN2_Pin GPIO_PIN_10
#define R_AIN2_GPIO_Port GPIOE
#define R_BIN1_Pin GPIO_PIN_12
#define R_BIN1_GPIO_Port GPIOE
#define R_BIN2_Pin GPIO_PIN_13
#define R_BIN2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
