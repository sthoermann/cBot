/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_gpio.h"

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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define Button3_Pin GPIO_PIN_14
#define Button3_GPIO_Port GPIOC
#define Button4_Pin GPIO_PIN_15
#define Button4_GPIO_Port GPIOC
#define ML1_Pin GPIO_PIN_0
#define ML1_GPIO_Port GPIOA
#define ML2_Pin GPIO_PIN_1
#define ML2_GPIO_Port GPIOA
#define ML3_Pin GPIO_PIN_2
#define ML3_GPIO_Port GPIOA
#define ML4_Pin GPIO_PIN_3
#define ML4_GPIO_Port GPIOA
#define MR1_Pin GPIO_PIN_4
#define MR1_GPIO_Port GPIOA
#define MR2_Pin GPIO_PIN_5
#define MR2_GPIO_Port GPIOA
#define MR3_Pin GPIO_PIN_6
#define MR3_GPIO_Port GPIOA
#define MR4_Pin GPIO_PIN_7
#define MR4_GPIO_Port GPIOA
#define rangeSensor0Trigger_Pin GPIO_PIN_10
#define rangeSensor0Trigger_GPIO_Port GPIOB
#define rangeSensor0Echo_Pin GPIO_PIN_11
#define rangeSensor0Echo_GPIO_Port GPIOB
#define rangeSensor0Echo_EXTI_IRQn EXTI15_10_IRQn
#define rangeSensor2Trigger_Pin GPIO_PIN_12
#define rangeSensor2Trigger_GPIO_Port GPIOB
#define rangeSensor2Echo_Pin GPIO_PIN_13
#define rangeSensor2Echo_GPIO_Port GPIOB
#define rangeSensor2Echo_EXTI_IRQn EXTI15_10_IRQn
#define rangeSensor1Trigger_Pin GPIO_PIN_14
#define rangeSensor1Trigger_GPIO_Port GPIOB
#define rangeSensor1Echo_Pin GPIO_PIN_15
#define rangeSensor1Echo_GPIO_Port GPIOB
#define rangeSensor1Echo_EXTI_IRQn EXTI15_10_IRQn
#define Button1_Pin GPIO_PIN_3
#define Button1_GPIO_Port GPIOB
#define Button2_Pin GPIO_PIN_5
#define Button2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
