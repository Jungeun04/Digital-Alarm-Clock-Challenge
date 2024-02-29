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
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
