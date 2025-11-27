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
#include "stm32h7xx.h"  // For direct register access on STM32H7xx

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
// Function to handle errors
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
// Pin definitions for battery voltage
#define Bat_Voltage_Pin GPIO_PIN_0
#define Bat_Voltage_GPIO_Port GPIOC
// Pin definitions for MAP sensor
#define Sensor_MAP_Pin GPIO_PIN_1
#define Sensor_MAP_GPIO_Port GPIOC
// Pin definitions for CKP
#define CKP_Pin GPIO_PIN_0
#define CKP_GPIO_Port GPIOA
// Pin definitions for CMP
#define CMP_Pin GPIO_PIN_1
#define CMP_GPIO_Port GPIOA
// Pin definitions for PWM output 4
#define PWM_MCU_OUT_4_Pin GPIO_PIN_2
#define PWM_MCU_OUT_4_GPIO_Port GPIOA
// Pin definitions for PWM output 5
#define PWM_MCU_OUT_5_Pin GPIO_PIN_3
#define PWM_MCU_OUT_5_GPIO_Port GPIOA
// Pin definitions for SPI CJ125 SCK
#define SPI_CJ125_SCK_Pin GPIO_PIN_5
#define SPI_CJ125_SCK_GPIO_Port GPIOA
// Pin definitions for Injector 1
#define Injector_1_Pin GPIO_PIN_9
#define Injector_1_GPIO_Port GPIOE
// Pin definitions for Injector 2
#define Injector_2_Pin GPIO_PIN_11
#define Injector_2_GPIO_Port GPIOE
// Pin definitions for Injector 3
#define Injector_3_Pin GPIO_PIN_13
#define Injector_3_GPIO_Port GPIOE
// Pin definitions for Injector 4
#define Injector_4_Pin GPIO_PIN_14
#define Injector_4_GPIO_Port GPIOE
// Pin definitions for CMP output
#define CMP_OUT_Pin GPIO_PIN_11
#define CMP_OUT_GPIO_Port GPIOD
// Pin definitions for Input Ig 1
#define Input_Ig_1_Pin GPIO_PIN_12
#define Input_Ig_1_GPIO_Port GPIOD
// Pin definitions for Input Ig 2
#define Input_Ig_2_Pin GPIO_PIN_13
#define Input_Ig_2_GPIO_Port GPIOD
// Pin definitions for Input Ig 3
#define Input_Ig_3_Pin GPIO_PIN_14
#define Input_Ig_3_GPIO_Port GPIOD
// Pin definitions for Input Ig 4
#define Input_Ig_4_Pin GPIO_PIN_15
#define Input_Ig_4_GPIO_Port GPIOD
// Pin definitions for PWM output 1
#define PWM_MCU_OUT_1_Pin GPIO_PIN_6
#define PWM_MCU_OUT_1_GPIO_Port GPIOC
// Pin definitions for PWM output 2
#define PWM_MCU_OUT_2_Pin GPIO_PIN_7
#define PWM_MCU_OUT_2_GPIO_Port GPIOC
// Pin definitions for SPI CJ125 MOSI
#define SPI_CJ125_MOSI_Pin GPIO_PIN_7
#define SPI_CJ125_MOSI_GPIO_Port GPIOD
// Pin definitions for SPI CJ125 MISO
#define SPI_CJ125_MISO_Pin GPIO_PIN_4
#define SPI_CJ125_MISO_GPIO_Port GPIOB
// Pin definitions for PWM output 3
#define PWM_MCU_OUT_3_Pin GPIO_PIN_5
#define PWM_MCU_OUT_3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */