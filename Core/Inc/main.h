/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

void Error_Handler(void);

#define ENCODER_X_Z_Pin        GPIO_PIN_0
#define ENCODER_X_Z_GPIO_Port  GPIOB
#define ENCODER_X_Z_EXTI_IRQn  EXTI0_IRQn

#define ENCODER_Y_Z_Pin        GPIO_PIN_1
#define ENCODER_Y_Z_GPIO_Port  GPIOB
#define ENCODER_Y_Z_EXTI_IRQn  EXTI1_IRQn

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
