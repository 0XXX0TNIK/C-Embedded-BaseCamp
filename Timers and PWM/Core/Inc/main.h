/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define G_LED_Pin GPIO_PIN_12
#define G_LED_GPIO_Port GPIOD
#define O_LED_Pin GPIO_PIN_13
#define O_LED_GPIO_Port GPIOD
#define R_LED_Pin GPIO_PIN_14
#define R_LED_GPIO_Port GPIOD
#define Y_LED_Pin GPIO_PIN_15
#define Y_LED_GPIO_Port GPIOD
#define MORE_DUTY_Pin GPIO_PIN_6
#define MORE_DUTY_GPIO_Port GPIOC
#define MORE_DUTY_EXTI_IRQn EXTI9_5_IRQn
#define LESS_DUTY_Pin GPIO_PIN_8
#define LESS_DUTY_GPIO_Port GPIOC
#define LESS_DUTY_EXTI_IRQn EXTI9_5_IRQn
#define LESS_FREQ_Pin GPIO_PIN_9
#define LESS_FREQ_GPIO_Port GPIOC
#define LESS_FREQ_EXTI_IRQn EXTI9_5_IRQn
#define NEXT_LED_Pin GPIO_PIN_15
#define NEXT_LED_GPIO_Port GPIOA
#define NEXT_LED_EXTI_IRQn EXTI15_10_IRQn
#define MORE_FREQ_Pin GPIO_PIN_11
#define MORE_FREQ_GPIO_Port GPIOC
#define MORE_FREQ_EXTI_IRQn EXTI15_10_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
