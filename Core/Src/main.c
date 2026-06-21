/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "cmsis_os.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "driver_encoder.h"
#include "locater_config.h"

void SystemClock_Config(void);
void MX_FREERTOS_Init(void);

#if LOCATER_USART1_BOOT_TEST_ENABLE
static void USART1_BootTrace(uint32_t marker)
{
  static const uint8_t boot_111[] = "111.000,1.000,0.000,0.000,0.000\r\n";
  static const uint8_t boot_222[] = "222.000,2.000,0.000,0.000,0.000\r\n";
  static const uint8_t boot_333[] = "333.000,3.000,0.000,0.000,0.000\r\n";
  const uint8_t *frame = boot_111;
  uint16_t len = (uint16_t)(sizeof(boot_111) - 1U);

  if (marker == 222U) {
    frame = boot_222;
    len = (uint16_t)(sizeof(boot_222) - 1U);
  } else if (marker == 333U) {
    frame = boot_333;
    len = (uint16_t)(sizeof(boot_333) - 1U);
  }

  for (uint32_t i = 0U; i < 5U; i++) {
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)frame, len, 100U);
  }
}
#endif

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
#if LOCATER_USART1_BOOT_TEST_ENABLE
  USART1_BootTrace(111U);
#endif
  MX_UART4_Init();
  MX_UART5_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
#if LOCATER_USART1_BOOT_TEST_ENABLE
  USART1_BootTrace(222U);
#endif

  osKernelInitialize();
  MX_FREERTOS_Init();
#if LOCATER_USART1_BOOT_TEST_ENABLE
  USART1_BootTrace(333U);
#endif
  osKernelStart();

  while (1)
  {
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV12;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  Driver_Encoder_EXTI_Callback(GPIO_Pin);
}

void Error_Handler(void)
{
#if LOCATER_USART1_BOOT_TEST_ENABLE
  if (huart1.Instance == USART1) {
    static const uint8_t error_frame[] = "999.000,9.000,0.000,0.000,0.000\r\n";
    for (uint32_t i = 0U; i < 5U; i++) {
      (void)HAL_UART_Transmit(&huart1, (uint8_t *)error_frame, (uint16_t)(sizeof(error_frame) - 1U), 100U);
    }
  }
#endif
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */
