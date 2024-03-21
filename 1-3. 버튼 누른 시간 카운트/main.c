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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "7SEG.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} GPIO_PinConfig;

typedef enum {
	LEFT_LED,
	RIGHT_LED
}SelectLED;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

GPIO_PinState Button_state[4];

GPIO_PinState last_Button_state[4] = {GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_RESET};

GPIO_PinState led_state[2][3] = {
		{GPIO_PIN_SET,GPIO_PIN_SET,GPIO_PIN_SET},
		{GPIO_PIN_SET,GPIO_PIN_SET,GPIO_PIN_SET}
};

GPIO_PinConfig Buttones[4] = {
    {GPIOE, GPIO_PIN_3},
    {GPIOC, GPIO_PIN_15},
    {GPIOD, GPIO_PIN_4},
	{GPIOD, GPIO_PIN_10}
};

GPIO_PinConfig leds[2][3] = {
    {{GPIOD, GPIO_PIN_12}, {GPIOD, GPIO_PIN_13}, {GPIOD, GPIO_PIN_14}},
    {{GPIOC, GPIO_PIN_6}, {GPIOB, GPIO_PIN_5}, {GPIOB, GPIO_PIN_0}}
};

SelectLED SelectedLED = LEFT_LED;

uint8_t uart3_rx_data;
uint8_t uart3_rx_flag;

uint8_t tim6_counter = 0;
uint8_t dpFlashFlag = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
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
  MX_USART3_UART_Init();
  MX_TIM6_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&huart3, &uart3_rx_data, sizeof(uart3_rx_data));

  for (uint8_t i = 0; i < 2; i++)
  {
	  for (uint8_t j = 0; j < 3; j++)
	  {
		  HAL_GPIO_WritePin(leds[i][j].port, leds[i][j].pin, GPIO_PIN_SET);
	  }
  }

  //uint8_t led_rx_data;
  uint8_t segment_DP = 0;

  _7SEG_GPIO_Init();              // 7세그먼트 초기화
  HAL_TIM_Base_Start_IT(&htim6);  // 타이머 시작 명령
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* USER CODE END WHILE */

	  /* USER CODE BEGIN 3 */
	  for (uint8_t i = 0; i < 1; ++i) {
		  Button_state[i] = HAL_GPIO_ReadPin(Buttones[i].port, Buttones[i].pin);
	  }

	  for (uint8_t i = 0; i < 1; ++i) {
		  if (Button_state[i] != last_Button_state[i]) {

			  if (last_Button_state[i] == GPIO_PIN_RESET && Button_state[i] == GPIO_PIN_SET) {
				  // PUSH
				  tim6_counter = 0;

				  if (led_state[0][i] == GPIO_PIN_SET) {
					  HAL_GPIO_WritePin(leds[LEFT_LED][i].port, leds[LEFT_LED][i].pin, GPIO_PIN_RESET);
					  led_state[0][i] = GPIO_PIN_RESET;
				  } else {
					  HAL_GPIO_WritePin(leds[LEFT_LED][i].port, leds[LEFT_LED][i].pin, GPIO_PIN_SET);
					  led_state[0][i] = GPIO_PIN_SET;
				  }
			  } else if (last_Button_state[i] == GPIO_PIN_SET && Button_state[i] == GPIO_PIN_RESET) {
				  // PULL
				  if (led_state[1][i] == GPIO_PIN_SET) {
					  HAL_GPIO_WritePin(leds[RIGHT_LED][i].port, leds[RIGHT_LED][i].pin, GPIO_PIN_RESET);
					  led_state[1][i] = GPIO_PIN_RESET;
				  } else {
					  HAL_GPIO_WritePin(leds[RIGHT_LED][i].port, leds[RIGHT_LED][i].pin, GPIO_PIN_SET);
					  led_state[1][i] = GPIO_PIN_SET;
				  }
			  }

			  last_Button_state[i] = Button_state[i];

		  }
      /* 누르고 있는 상태 감지 */
		  else if(Button_state[i] == GPIO_PIN_SET)
		  {
			  if(tim6_counter>100) tim6_counter = 0;    // 99초가 넘으면 0으로 초기화
			  if(tim6_counter%5 == 0 && dpFlashFlag)    // 0.5초 마다 DGT1의 DP 토글 
			  {
				  dpFlashFlag = 0;
				  if(segment_DP == 0)
					  segment_DP = 1;
				  else
					  segment_DP = 0;
			  }
			  _7SEG_SetNumber(DGT1, tim6_counter/10, segment_DP); // 1초 단위 표시
			  _7SEG_SetNumber(DGT2, tim6_counter%10, OFF);        // 0.1초 단위 표시

		  }

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

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM6_DAC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART3) {
		HAL_UART_Receive_IT(&huart3, &uart3_rx_data, sizeof(uart3_rx_data));
		uart3_rx_flag = 1;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM6)
	{
		tim6_counter += 1;  // 0.1초마다 카운트
		dpFlashFlag = 1;    // 0.1초에 DP토글이 한번만 되게 하는 플래그
	}
}

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
