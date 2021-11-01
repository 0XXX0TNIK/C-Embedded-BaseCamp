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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Green_Pin GPIO_PIN_12
#define LED_Green_GPIO_Port GPIOD
#define LED_Orange_Pin GPIO_PIN_13
#define LED_Orange_GPIO_Port GPIOD
#define LED_Red_Pin GPIO_PIN_14
#define LED_Red_GPIO_Port GPIOD
#define LED_Blue_Pin GPIO_PIN_15
#define LED_Blue_GPIO_Port GPIOD
#define BUTTON_SpeedUp_Pin GPIO_PIN_6
#define BUTTON_SpeedUp_GPIO_Port GPIOC
#define BUTTON_SpeedUp_EXTI_IRQn EXTI9_5_IRQn
#define BUTTON_SpeedDown_Pin GPIO_PIN_8
#define BUTTON_SpeedDown_GPIO_Port GPIOC
#define BUTTON_SpeedDown_EXTI_IRQn EXTI9_5_IRQn
#define BUTTON_PrevScheme_Pin GPIO_PIN_9
#define BUTTON_PrevScheme_GPIO_Port GPIOC
#define BUTTON_PrevScheme_EXTI_IRQn EXTI9_5_IRQn
#define BUTTON_Power_Pin GPIO_PIN_15
#define BUTTON_Power_GPIO_Port GPIOA
#define BUTTON_Power_EXTI_IRQn EXTI15_10_IRQn
#define BUTTON_NextScheme_Pin GPIO_PIN_11
#define BUTTON_NextScheme_GPIO_Port GPIOC
#define BUTTON_NextScheme_EXTI_IRQn EXTI15_10_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
