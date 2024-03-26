/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#define Segment_PC13_Pin GPIO_PIN_13
#define Segment_PC13_GPIO_Port GPIOC
#define Segment_PC14_Pin GPIO_PIN_14
#define Segment_PC14_GPIO_Port GPIOC
#define Segment_PE8_Pin GPIO_PIN_8
#define Segment_PE8_GPIO_Port GPIOE
#define Segment_PE9_Pin GPIO_PIN_9
#define Segment_PE9_GPIO_Port GPIOE
#define Segment_PE10_Pin GPIO_PIN_10
#define Segment_PE10_GPIO_Port GPIOE
#define Segment_PE11_Pin GPIO_PIN_11
#define Segment_PE11_GPIO_Port GPIOE
#define Segment_PE12_Pin GPIO_PIN_12
#define Segment_PE12_GPIO_Port GPIOE
#define Segment_PE13_Pin GPIO_PIN_13
#define Segment_PE13_GPIO_Port GPIOE
#define Segment_PE14_Pin GPIO_PIN_14
#define Segment_PE14_GPIO_Port GPIOE
#define Segment_PE15_Pin GPIO_PIN_15
#define Segment_PE15_GPIO_Port GPIOE
#define Segment_PD11_Pin GPIO_PIN_11
#define Segment_PD11_GPIO_Port GPIOD
#define Segment_PD15_Pin GPIO_PIN_15
#define Segment_PD15_GPIO_Port GPIOD
#define Segment_PA8_Pin GPIO_PIN_8
#define Segment_PA8_GPIO_Port GPIOA
#define Segment_PD5_Pin GPIO_PIN_5
#define Segment_PD5_GPIO_Port GPIOD
#define Segment_PD6_Pin GPIO_PIN_6
#define Segment_PD6_GPIO_Port GPIOD
#define Segment_PD7_Pin GPIO_PIN_7
#define Segment_PD7_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
