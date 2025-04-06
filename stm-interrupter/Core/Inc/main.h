/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32c0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "int.h"
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
#define BLED_Pin GPIO_PIN_7
#define BLED_GPIO_Port GPIOB
#define BATT_LED_Pin GPIO_PIN_0
#define BATT_LED_GPIO_Port GPIOA
#define BATT_SENSE_Pin GPIO_PIN_1
#define BATT_SENSE_GPIO_Port GPIOA
#define SW_SINGLESHOT_Pin GPIO_PIN_4
#define SW_SINGLESHOT_GPIO_Port GPIOA
#define FIRE_Pin GPIO_PIN_5
#define FIRE_GPIO_Port GPIOA
#define E_OT_Pin GPIO_PIN_6
#define E_OT_GPIO_Port GPIOA
#define E_BPS_Pin GPIO_PIN_7
#define E_BPS_GPIO_Port GPIOA
#define E_VMAX_Pin GPIO_PIN_8
#define E_VMAX_GPIO_Port GPIOA
#define I_OCD_Pin GPIO_PIN_11
#define I_OCD_GPIO_Port GPIOA
#define I_DTC_Pin GPIO_PIN_12
#define I_DTC_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
