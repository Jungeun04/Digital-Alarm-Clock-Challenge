# [디지털 알람시계 프로그래밍 챌린지] 1-3. 버튼 누른 시간 카운트

## 목표
- Timer를 사용하여 정확한 시간 생성
- 버튼 상태 추가

## 사용된 핀 설정
- LED1
    - (PORTD,12)
- Button
    - (PORTE,3)
- TIMER6
    - 0.1초 마다 발생
    - Prescaler: 999
    - Counter Period: 8399
    - global interrupt : enable
    - 84Mhz 소스
- 7-Segments
```text
      SR-2056A (Common Anode 2digits 7segment. 18pin)

                A(PD11)                        A(PE9)
               _________                     _________
              |         |                   |         |
      (PC13)F |         | B(PA8)     (PE8)F |         | B(PE10)
              |         |                   |         |
               _________                     _________
              | G(PD15) |                   | G(PE13) |
      (PC14)E |         | C(PD6)    (PE15)E |         | C(PE12)
              |         |                   |         |
               _________  @ DP(PD7)          _________  @ DP(PE11)
                D(PD5)                        D(PE14)
          
                DIGIT_1ST		     DIGIT_2ND

      Set: turn off
      Reset: turn on
```
## 설명
이번 과제는 1-1에서 timer만 추가하고 약간 수정하면 쉽게 해결되었습니다.  
하지만 버튼이 증가하면 상태 확인이 난잡해 질 것으로 예상되어, 과제에서 설명하신 버튼 상태 변수를 따로 만들어 관리해야 할 것 같습니다.


## 작동 영상
[![Video Label](https://img.youtube.com/vi/dGf95cAWKCI/0.jpg)](https://youtu.be/dGf95cAWKCI?si=PF5exUGLYcfVVh1I)

## main 소스코드

자동생성 주석을 제거한 main.c 파일입니다.
```C

#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "7SEG.h"

// 포트와 핀을 쉽게 관리하기 위한 구조체 선언
typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} GPIO_PinConfig;

// LED 선택을 하기 위한 전용 타입
typedef enum {
	LEFT_LED,
	RIGHT_LED
}SelectLED;

// switch 상태를 읽어서 저장할 변수
GPIO_PinState Button_state[4];

// 마지막 버튼 상태를 저장하는 변수
GPIO_PinState last_Button_state[4] = {GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_RESET};

// LED 상태를 저장하는 변수
GPIO_PinState led_state[2][3] = {
		{GPIO_PIN_SET,GPIO_PIN_SET,GPIO_PIN_SET},
		{GPIO_PIN_SET,GPIO_PIN_SET,GPIO_PIN_SET}
};

// 버튼 및 LED 핀 설정 배열
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

void SystemClock_Config(void);
static void MX_NVIC_Init(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_TIM6_Init();

  MX_NVIC_Init();

  HAL_UART_Receive_IT(&huart3, &uart3_rx_data, sizeof(uart3_rx_data));

  for (uint8_t i = 0; i < 2; i++)
  {
	  for (uint8_t j = 0; j < 3; j++)
	  {
		  HAL_GPIO_WritePin(leds[i][j].port, leds[i][j].pin, GPIO_PIN_SET);
	  }
  }

  uint8_t segment_DP = 0;

  _7SEG_GPIO_Init();              // 7세그먼트 초기화
  HAL_TIM_Base_Start_IT(&htim6);  // 타이머 시작 명령

  while (1)
  {
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
}
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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

static void MX_NVIC_Init(void)
{
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
}

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

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}


```
