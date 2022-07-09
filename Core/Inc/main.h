/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f0xx_hal.h"

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
#define VHED_Pin GPIO_PIN_2
#define VHED_GPIO_Port GPIOA
#define VMED_Pin GPIO_PIN_3
#define VMED_GPIO_Port GPIOA
#define A0_Pin GPIO_PIN_4
#define A0_GPIO_Port GPIOA
#define A1_Pin GPIO_PIN_5
#define A1_GPIO_Port GPIOA
#define CREST_Pin GPIO_PIN_6
#define CREST_GPIO_Port GPIOA
#define CVIN_Pin GPIO_PIN_7
#define CVIN_GPIO_Port GPIOA
#define LOG2_Pin GPIO_PIN_12
#define LOG2_GPIO_Port GPIOB
#define LOG1_Pin GPIO_PIN_13
#define LOG1_GPIO_Port GPIOB
#define REFN_Pin GPIO_PIN_8
#define REFN_GPIO_Port GPIOA
#define REFP_Pin GPIO_PIN_9
#define REFP_GPIO_Port GPIOA
#define VZERO_Pin GPIO_PIN_10
#define VZERO_GPIO_Port GPIOA
#define CT3_Pin GPIO_PIN_3
#define CT3_GPIO_Port GPIOB
#define CT2_Pin GPIO_PIN_4
#define CT2_GPIO_Port GPIOB
#define CT1_Pin GPIO_PIN_5
#define CT1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
