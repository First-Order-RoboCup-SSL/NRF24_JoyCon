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
#include "stm32f1xx_hal.h"
#include <stdbool.h>

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
#define KEY5_Pin GPIO_PIN_6
#define KEY5_GPIO_Port GPIOA
#define KEY5_EXTI_IRQn EXTI9_5_IRQn
#define KEY6_Pin GPIO_PIN_7
#define KEY6_GPIO_Port GPIOA
#define KEY6_EXTI_IRQn EXTI9_5_IRQn
#define KEY7_Pin GPIO_PIN_0
#define KEY7_GPIO_Port GPIOB
#define KEY7_EXTI_IRQn EXTI0_IRQn
#define KEY8_Pin GPIO_PIN_1
#define KEY8_GPIO_Port GPIOB
#define KEY8_EXTI_IRQn EXTI1_IRQn
#define KEY2_Pin GPIO_PIN_11
#define KEY2_GPIO_Port GPIOB
#define KEY2_EXTI_IRQn EXTI15_10_IRQn
#define CSN_Pin GPIO_PIN_12
#define CSN_GPIO_Port GPIOB
#define IRQ_Pin GPIO_PIN_8
#define IRQ_GPIO_Port GPIOA
#define IRQ_EXTI_IRQn EXTI9_5_IRQn
#define CE_Pin GPIO_PIN_11
#define CE_GPIO_Port GPIOA
#define KEY1_Pin GPIO_PIN_15
#define KEY1_GPIO_Port GPIOA
#define KEY1_EXTI_IRQn EXTI15_10_IRQn
#define BUZZER_PIN GPIO_PIN_12
#define BUZZER_PORT GPIOA

// Beep control structure
typedef struct {
    uint32_t start_time;    // When beep started
    uint32_t duration;      // How long to beep
    bool is_beeping;        // Currently beeping?
} Beep_Control_t;

extern Beep_Control_t beep_control;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
