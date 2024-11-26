/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

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
#define ESTOP2_Pin GPIO_PIN_4
#define ESTOP2_GPIO_Port GPIOE
#define IND1_Pin GPIO_PIN_14
#define IND1_GPIO_Port GPIOC
#define IND2_Pin GPIO_PIN_15
#define IND2_GPIO_Port GPIOC
#define GPIO4_Pin GPIO_PIN_0
#define GPIO4_GPIO_Port GPIOC
#define FSYNC_Pin GPIO_PIN_0
#define FSYNC_GPIO_Port GPIOA
#define LSYNC_Pin GPIO_PIN_1
#define LSYNC_GPIO_Port GPIOA
#define GPIO5_Pin GPIO_PIN_4
#define GPIO5_GPIO_Port GPIOA
#define FAN1_TACH_Pin GPIO_PIN_0
#define FAN1_TACH_GPIO_Port GPIOB
#define FAN2_TACH_Pin GPIO_PIN_1
#define FAN2_TACH_GPIO_Port GPIOB
#define FAN1_PWM_Pin GPIO_PIN_9
#define FAN1_PWM_GPIO_Port GPIOE
#define FAN2_PWM_Pin GPIO_PIN_11
#define FAN2_PWM_GPIO_Port GPIOE
#define ESTOP1_Pin GPIO_PIN_13
#define ESTOP1_GPIO_Port GPIOE
#define HUB_RESET_Pin GPIO_PIN_15
#define HUB_RESET_GPIO_Port GPIOB
#define SCL_CFG_Pin GPIO_PIN_12
#define SCL_CFG_GPIO_Port GPIOD
#define SDA_REM_Pin GPIO_PIN_13
#define SDA_REM_GPIO_Port GPIOD
#define GPIO0_Pin GPIO_PIN_8
#define GPIO0_GPIO_Port GPIOC
#define IND3_Pin GPIO_PIN_9
#define IND3_GPIO_Port GPIOA
#define GPIO3_Pin GPIO_PIN_10
#define GPIO3_GPIO_Port GPIOC
#define GPIO2_Pin GPIO_PIN_11
#define GPIO2_GPIO_Port GPIOC
#define GPIO1_Pin GPIO_PIN_12
#define GPIO1_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern CRC_HandleTypeDef   hcrc;
extern UART_HandleTypeDef huart4;
#define DEBUG_UART huart4
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
