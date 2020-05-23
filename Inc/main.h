/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define REFCCR TIM2->CCR2
#define REFNCCR TIM2->CCR1
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PC4_Pin GPIO_PIN_13
#define PC4_GPIO_Port GPIOC
#define PC3_Pin GPIO_PIN_14
#define PC3_GPIO_Port GPIOC
#define PC5_Pin GPIO_PIN_15
#define PC5_GPIO_Port GPIOC
#define CREFN_Pin GPIO_PIN_0
#define CREFN_GPIO_Port GPIOA
#define CREFP_Pin GPIO_PIN_1
#define CREFP_GPIO_Port GPIOA
#define CVIN_Pin GPIO_PIN_2
#define CVIN_GPIO_Port GPIOA
#define CZERO_Pin GPIO_PIN_3
#define CZERO_GPIO_Port GPIOA
#define VCENT_Pin GPIO_PIN_4
#define VCENT_GPIO_Port GPIOA
#define CADC_Pin GPIO_PIN_5
#define CADC_GPIO_Port GPIOA
#define VPREZ_Pin GPIO_PIN_0
#define VPREZ_GPIO_Port GPIOB
#define VZERO_Pin GPIO_PIN_1
#define VZERO_GPIO_Port GPIOB
#define CLOG_Pin GPIO_PIN_2
#define CLOG_GPIO_Port GPIOB
#define CLOG2_Pin GPIO_PIN_10
#define CLOG2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
