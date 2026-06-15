/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32l0xx_hal.h"

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
void Rotary_Encoder_Interrupt_Handler(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Current_sensor_Pin GPIO_PIN_0
#define Current_sensor_GPIO_Port GPIOC
#define PDO5_Pin GPIO_PIN_1
#define PDO5_GPIO_Port GPIOC
#define Button_Pin GPIO_PIN_0
#define Button_GPIO_Port GPIOA
#define PDO2_Pin GPIO_PIN_1
#define PDO2_GPIO_Port GPIOA
#define PDO3_Pin GPIO_PIN_4
#define PDO3_GPIO_Port GPIOA
#define DMA_Debug_Pin GPIO_PIN_5
#define DMA_Debug_GPIO_Port GPIOA
#define Voltage_sensor_Pin GPIO_PIN_6
#define Voltage_sensor_GPIO_Port GPIOA
#define Voltage_input_sensor_Pin GPIO_PIN_7
#define Voltage_input_sensor_GPIO_Port GPIOA
#define PDO4_Pin GPIO_PIN_0
#define PDO4_GPIO_Port GPIOB
#define ROT_CHB_Pin GPIO_PIN_8
#define ROT_CHB_GPIO_Port GPIOA
#define ROT_CHB_EXTI_IRQn EXTI4_15_IRQn
#define SHDN_Pin GPIO_PIN_10
#define SHDN_GPIO_Port GPIOA
#define PWM_TIM2_CH2_Pin GPIO_PIN_3
#define PWM_TIM2_CH2_GPIO_Port GPIOB
#define ROT_CHA_Pin GPIO_PIN_4
#define ROT_CHA_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_5
#define LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
