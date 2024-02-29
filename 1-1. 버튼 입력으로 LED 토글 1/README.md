## main 소스코드

```C
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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} GPIO_PinConfig;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// switch 상태를 읽어서 저장할 변수
GPIO_PinState switch_state[3];

// 마지막 스위치 상태를 저장하는 변수
GPIO_PinState last_switch_state[3] = {GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_RESET};

// LED 상태를 저장하는 변수
GPIO_PinState led_state[2][3] = {
		{GPIO_PIN_SET,GPIO_PIN_SET,GPIO_PIN_SET},
		{GPIO_PIN_SET,GPIO_PIN_SET,GPIO_PIN_SET}
};

// 스위치 및 LED 핀 설정 배열
GPIO_PinConfig switches[3] = {
    {GPIOE, GPIO_PIN_3},
    {GPIOC, GPIO_PIN_15},
    {GPIOD, GPIO_PIN_4}
};
GPIO_PinConfig leds[2][3] = {
    {{GPIOD, GPIO_PIN_12}, {GPIOD, GPIO_PIN_13}, {GPIOD, GPIO_PIN_14}},
    {{GPIOC, GPIO_PIN_6}, {GPIOB, GPIO_PIN_5}, {GPIOB, GPIO_PIN_0}}
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  /* USER CODE BEGIN 2 */

  //초기 LED를 모두 OFF
  for (int i = 0; i < 2; i++)
  {
	  for (int j = 0; j < 3; j++)
	  {
		  HAL_GPIO_WritePin(leds[i][j].port, leds[i][j].pin, GPIO_PIN_SET);
	  }
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

      // 스위치 상태 읽기
      for (int i = 0; i < 3; ++i) {
          switch_state[i] = HAL_GPIO_ReadPin(switches[i].port, switches[i].pin);
      }

      // 스위치 상태에 따라 LED 토글
      for (int i = 0; i < 3; ++i) {
          if (switch_state[i] != last_switch_state[i]) {
              if (last_switch_state[i] == GPIO_PIN_RESET && switch_state[i] == GPIO_PIN_SET) {
                  // PUSH 상태
                  if (led_state[0][i] == GPIO_PIN_SET) {
                      HAL_GPIO_WritePin(leds[0][i].port, leds[0][i].pin, GPIO_PIN_RESET);
                      led_state[0][i] = GPIO_PIN_RESET;
                  } else {
                      HAL_GPIO_WritePin(leds[0][i].port, leds[0][i].pin, GPIO_PIN_SET);
                      led_state[0][i] = GPIO_PIN_SET;
                  }
              } else if (last_switch_state[i] == GPIO_PIN_SET && switch_state[i] == GPIO_PIN_RESET) {
                  // PULL 상태
                  if (led_state[1][i] == GPIO_PIN_SET) {
                      HAL_GPIO_WritePin(leds[1][i].port, leds[1][i].pin, GPIO_PIN_RESET);
                      led_state[1][i] = GPIO_PIN_RESET;
                  } else {
                      HAL_GPIO_WritePin(leds[1][i].port, leds[1][i].pin, GPIO_PIN_SET);
                      led_state[1][i] = GPIO_PIN_SET;
                  }
              }
          }

          last_switch_state[i] = switch_state[i]; // 스위치 상태 업데이트
      }
  }
  /* USER CODE END 3 */
}
```