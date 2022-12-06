/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_it.h"
#include "adc_config.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
uint32_t run = 1;
__IO uint32_t runDown = 0;
__IO uint32_t haveRunDown = 0;
__IO uint32_t refp = 0;
__IO uint32_t refn = 0;
__IO int32_t ct = 20000;
uint16_t adcbuf[256];
uint16_t adcindex = 0;
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim15;
extern DMA_HandleTypeDef hdma_usart1_rx;
/* USER CODE BEGIN EV */
extern ADC_HandleTypeDef hadc;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel 2 and 3 interrupts.
  */
void DMA1_Channel2_3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_3_IRQn 0 */

  /* USER CODE END DMA1_Channel2_3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA1_Channel2_3_IRQn 1 */

  /* USER CODE END DMA1_Channel2_3_IRQn 1 */
}

/**
  * @brief This function handles TIM1 break, update, trigger and commutation interrupts.
  */
void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim1);
}	

void ADC1_IRQHandler(void) {
	LOG2_GPIO_Port->BSRR = (uint32_t)LOG2_Pin;
	LOG2_GPIO_Port->BRR = (uint32_t)LOG2_Pin;
	
	__IO uint32_t tmp = hadc.Instance->DR;
			if(run) {
				//for ADC, CCR1 refn, ccr2 refp
				if(tmp > ADH) {
					htim1.Instance->CCR1 = TZS;
					htim1.Instance->CCR2 = TZW;
					refp += TZW;
					refn += TZS;
				}
				else if( tmp < ADL) {
					htim1.Instance->CCR1 = TZW;
					htim1.Instance->CCR2 = TZS;
					refn += TZW;
					refp += TZS;
				}
				else {
					htim1.Instance->CCR1 = TZS;
					htim1.Instance->CCR2 = TZS;
					refp += TZS;
					refn += TZS;
				}
				ct--;
		}
		else {
			if(!haveRunDown) {
				//__HAL_TIM_DISABLE(&htim1);
				htim1.Instance->ARR = 62000;
				while(htim1.Instance->CNT < 500);
				htim1.Instance->CCMR1 = 0x4040;
				htim1.Instance->CCR4 = 62000;
				htim1.Instance->CCR2 = 62000;
				runDown = 1;
				haveRunDown = 1;
				//HAL_NVIC_DisableIRQ(TIM1_CC_IRQn);
				LOG1_GPIO_Port->BSRR = (uint32_t)LOG1_Pin;
				LOG1_GPIO_Port->BRR = (uint32_t)LOG1_Pin;
				LOG1_GPIO_Port->BSRR = (uint32_t)LOG1_Pin;
				LOG1_GPIO_Port->BRR = (uint32_t)LOG1_Pin;
			}
			else {
				if(tmp>1000) {
					adcbuf[adcindex++] = tmp;
				}
				else {
					htim15.Instance->CR1 &= ~(TIM_CR1_CEN);
				}
				LOG1_GPIO_Port->BSRR = (uint32_t)LOG1_Pin;
				LOG1_GPIO_Port->BRR = (uint32_t)LOG1_Pin;
			}
		}
		//HAL_GPIO_WritePin(LOG2_GPIO_Port,LOG2_Pin,GPIO_PIN_SET);
		//HAL_GPIO_WritePin(LOG2_GPIO_Port,LOG2_Pin,GPIO_PIN_RESET);
		//HAL_ADC_IRQHandler(&hadc);
		__HAL_ADC_CLEAR_FLAG(&hadc, ADC_FLAG_OVR);
}

void TIM1_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_UP_TRG_COM_IRQn 0 */
			//if(ct>0) {
			/*if(run) {
				//for ADC, CCR1 refn, ccr2 refp
				__IO uint32_t tmp = hadc.Instance->DR;
				if(tmp > ADH) {
					htim1.Instance->CCR1 = TZS;
					htim1.Instance->CCR2 = TZW;
					refp += TZW;
					refn += TZS;
				}
				else if( tmp < ADL) {
					htim1.Instance->CCR1 = TZW;
					htim1.Instance->CCR2 = TZS;
					refn += TZW;
					refp += TZS;
				}
				else {
					htim1.Instance->CCR1 = TZS;
					htim1.Instance->CCR2 = TZS;
					refp += TZS;
					refn += TZS;
				}
				HAL_GPIO_WritePin(LOG1_GPIO_Port,LOG1_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(LOG1_GPIO_Port,LOG1_Pin,GPIO_PIN_RESET);
				ct--;
		}
		else {
			if(!haveRunDown) {
				//__HAL_TIM_DISABLE(&htim1);
				htim1.Instance->ARR = 62000;
				while(htim1.Instance->CNT < 500);
				htim1.Instance->CCMR1 = 0x4040;
				htim1.Instance->CCR4 = 62000;
				htim1.Instance->CCR2 = 62000;
				runDown = 1;
				haveRunDown = 1;
				//HAL_NVIC_DisableIRQ(TIM1_CC_IRQn);
				HAL_GPIO_WritePin(LOG1_GPIO_Port,LOG1_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(LOG1_GPIO_Port,LOG1_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LOG1_GPIO_Port,LOG1_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(LOG1_GPIO_Port,LOG1_Pin,GPIO_PIN_RESET);
			}
		}*/
		__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC4);
  /* USER CODE END TIM1_BRK_UP_TRG_COM_IRQn 0 */
  //HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_BRK_UP_TRG_COM_IRQn 1 */

  /* USER CODE END TIM1_BRK_UP_TRG_COM_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
	if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE) != RESET)
  {
    if (__HAL_TIM_GET_IT_SOURCE(&htim3, TIM_IT_UPDATE) != RESET)
    {
      __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
			//htim1.Instance->CR1 |= (TIM_CR1_CEN);
			run = 1;
			refp = 0;
			refn = 0;
			/*__HAL_ADC_DISABLE(&hadc);
			hadc.Instance->CFGR1 &= ~(0x7<<6);
			hadc.Instance->CFGR1 |= (0x1<<6);
			__HAL_ADC_ENABLE(&hadc);*/
			__HAL_ADC_CLEAR_FLAG(&hadc, ADC_FLAG_OVR);
			__HAL_ADC_CLEAR_FLAG(&hadc, ADC_ISR_EOC|ADC_ISR_EOS);
			adcindex = 0;
			htim1.Instance->ARR = TZCLK;
			htim1.Instance->CNT = 50;
			htim1.Instance->CCR4 = TZCC;
			htim1.Instance->CCR1 = TZS;
			htim1.Instance->CCR2 = TZS;
			htim1.Instance->CCMR1 = 0x6868;
			runDown = 0;
			haveRunDown = 0;
			ct = 20000;
			
			//HAL_GPIO_WritePin(LOG2_GPIO_Port,LOG2_Pin,GPIO_PIN_SET);
			//HAL_GPIO_WritePin(LOG2_GPIO_Port,LOG2_Pin,GPIO_PIN_RESET);
			//HAL_GPIO_WritePin(LOG2_GPIO_Port,LOG2_Pin,GPIO_PIN_SET);
			//HAL_GPIO_WritePin(LOG2_GPIO_Port,LOG2_Pin,GPIO_PIN_RESET);
		}
	}
  if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_CC2) != RESET) {
		 if (__HAL_TIM_GET_IT_SOURCE(&htim3, TIM_IT_CC2) != RESET)
    {
      __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC2);
			run = 0;
			//HAL_GPIO_WritePin(LOG2_GPIO_Port,LOG2_Pin,GPIO_PIN_SET);
			//HAL_GPIO_WritePin(LOG2_GPIO_Port,LOG2_Pin,GPIO_PIN_RESET);
		}
	}
	
  /* USER CODE END TIM3_IRQn 0 */
  //HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
