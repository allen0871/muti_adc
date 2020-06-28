/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

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
extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

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
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
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
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles ADC1 global interrupt.
  */
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */

  /* USER CODE END ADC_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC_IRQn 1 */

  /* USER CODE END ADC_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

extern ADC_HandleTypeDef hadc1;
/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/* USER CODE BEGIN 1 */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern volatile char init;

volatile char flag = 0;
volatile unsigned int* ccra;
volatile unsigned int* ccrb;
void (*time2_handler)(void);
int tim2needStop;
int tim2needStart;
extern volatile uint32_t refTime;
extern volatile uint32_t refnTime;
extern volatile uint32_t curRefCount;
extern volatile uint32_t curRefnCount;
extern volatile uint32_t debugRef;
extern volatile uint32_t debugRefn;
extern volatile uint32_t adc_status;
extern volatile uint16_t *adc_log;
extern volatile uint32_t adc_index;
extern volatile uint32_t adc_reflast;
extern volatile uint32_t adc_refnlast;
extern volatile int32_t adc_count;
extern volatile uint32_t adc_sumTime;

void TIM2_IRQHandler(void)
{
	time2_handler();
}
int t;


void TIM2_IRQClean(void)
{
	if(__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET)
	{
			 REFCCR = 750;
			 REFNCCR = 150;
		   
			TIM2->CCR4 = 750;
		  TIM2->ARR = 1679;
			TIM2->CCMR1 = 0x1010; 
			//time2_handler = TIM2_IRQStart;
		adc_status = 0;
		curRefCount = 750;
		curRefnCount = 150;
		refTime = 0;
		refnTime = 0;
		
		HAL_GPIO_WritePin(CADC_GPIO_Port,CADC_Pin,GPIO_PIN_RESET);
		__HAL_TIM_CLEAR_IT(&htim2, TIM_FLAG_UPDATE);
		HAL_GPIO_TogglePin(CLOG_GPIO_Port, CLOG_Pin);
		HAL_GPIO_TogglePin(CLOG_GPIO_Port, CLOG_Pin);
				HAL_GPIO_TogglePin(CLOG_GPIO_Port, CLOG_Pin);
		HAL_GPIO_TogglePin(CLOG_GPIO_Port, CLOG_Pin);
	}
	else if(__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC4) != RESET)
	{
		if(adc_status == 0) {
			adc_status = 1;
		}
		else if(adc_status == 2){
			debugRef = TIM2->CCR4;
			REFCCR = TIM2->CNT + 80;
			REFNCCR = REFCCR;
			TIM2->CCMR1 = 0x2020;
			HAL_GPIO_TogglePin(CLOG_GPIO_Port, CLOG_Pin);
			HAL_GPIO_TogglePin(CLOG_GPIO_Port, CLOG_Pin);
			
			uint32_t tmp = (debugRef-curRefCount);
			refTime += tmp;
			adc_reflast = tmp;
			tmp = (debugRef-curRefnCount);
			adc_refnlast = tmp;
			refnTime += tmp;
			adc_sumTime = debugRef;
			adc_status = 3;
			//REFCCR = TIM2->CNT + 80;
			//REFNCCR = REFCCR;
			//TIM2->CCMR1 = 0x2020;
		}
		else if(adc_status == 4) {
			HAL_GPIO_WritePin(CADC_GPIO_Port,CADC_Pin,GPIO_PIN_RESET);
		}
		__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC4);
	}
}

void TIM2_IRQStart(void)
{
	 if(__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC4) != RESET)
  {
		 //HAL_GPIO_TogglePin(CLOG_GPIO_Port, CLOG_Pin);
		 
		 if(VCENT_GPIO_Port->IDR & VCENT_Pin)
		 {
			 REFCCR = 1650;
			 REFNCCR = 1050;
			 TIM2->CCMR1 = 0x2020;//强制低
			 uint16_t tmp = (1650-curRefCount);
			 refTime += tmp;
			 //adc_log[adc_index++] = tmp;
			 tmp = (1050 - curRefnCount);
			 //adc_log[adc_index++] = tmp;
			 refnTime += tmp;
		 }
		 else
		 {
			 REFCCR = 1050;
			 REFNCCR = 1650;
			 TIM2->CCMR1 = 0x2020;//强制低
			 uint16_t tmp = (1050-curRefCount);
			 refTime += tmp;
			 //adc_log[adc_index++] = tmp;
			 tmp = (1650 - curRefnCount);
			 //adc_log[adc_index++] = tmp;
			 refnTime += tmp;
		 }
		 	HAL_GPIO_TogglePin(CLOG_GPIO_Port, CLOG_Pin);
			HAL_GPIO_TogglePin(CLOG_GPIO_Port, CLOG_Pin);
		 adc_count--;
		//HAL_GPIO_TogglePin(CLOG_GPIO_Port, CLOG_Pin);
     __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC4);


  }
	if(__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET)
  {
		 //if(tim2needStop && !tim2needStart)
		 if(adc_count <= 0) 
		 {
			 //tim2needStop = 0;
			 
			 REFCCR = 150;
			 REFNCCR = 419699;
			 TIM2->CCMR1 = 0x1020;  //OC1M/2M 强制设高
			 TIM2->ARR = 419599;
			 time2_handler = TIM2_IRQClean;
			 curRefCount = 150;
		   TIM2->CCR4 = 350;
			tim2needStop = 0;
					HAL_GPIO_TogglePin(CLOG2_GPIO_Port, CLOG2_Pin);
		 HAL_GPIO_TogglePin(CLOG2_GPIO_Port, CLOG2_Pin);
			 		HAL_GPIO_TogglePin(CLOG2_GPIO_Port, CLOG2_Pin);
		 HAL_GPIO_TogglePin(CLOG2_GPIO_Port, CLOG2_Pin);
			 		HAL_GPIO_TogglePin(CLOG2_GPIO_Port, CLOG2_Pin);
		 HAL_GPIO_TogglePin(CLOG2_GPIO_Port, CLOG2_Pin);
		 }
		 else {
			 if(VCENT_GPIO_Port->IDR & VCENT_Pin)
			 {
				 REFCCR = 150;
				 REFNCCR = 750;
				 TIM2->CCMR1 = 0x1010;  //OC1M/2M 强制设高
				 curRefCount = 150;
				 curRefnCount = 750;
			 }
			 else
			 {
				 REFCCR = 750;
				 REFNCCR = 150;
				 TIM2->CCMR1 = 0x1010;  //OC1M/2M 强制设高
				 curRefCount = 750;
				 curRefnCount = 150;
			 }
		HAL_GPIO_TogglePin(CLOG2_GPIO_Port, CLOG2_Pin);
		 HAL_GPIO_TogglePin(CLOG2_GPIO_Port, CLOG2_Pin);
	 }
		__HAL_TIM_CLEAR_IT(&htim2, TIM_FLAG_UPDATE);

  }
}

void TIM5_IRQHandler(void) 
{
	if(__HAL_TIM_GET_FLAG(&htim5, TIM_FLAG_UPDATE) != RESET)
	{
		__HAL_TIM_CLEAR_IT(&htim5, TIM_FLAG_UPDATE);
					 REFCCR = 750;
			 REFNCCR = 150;
		   
			TIM2->CCR4 = 750;
		  TIM2->ARR = 1679;
			TIM2->CCMR1 = 0x1010; 
			//time2_handler = TIM2_IRQStart;
		adc_status = 0;
		curRefCount = 750;
		curRefnCount = 150;
		refTime = 0;
		refnTime = 0;
		TIM2->CNT=0;
		time2_handler = TIM2_IRQStart;
		tim2needStart = 1;
	}
	else if(__HAL_TIM_GET_FLAG(&htim5, TIM_FLAG_CC3) != RESET)
	{
		__HAL_TIM_CLEAR_IT(&htim5, TIM_FLAG_CC3);
		tim2needStop = 1;
		tim2needStart = 0;
		HAL_GPIO_TogglePin(CLOG2_GPIO_Port, CLOG2_Pin);
		 HAL_GPIO_TogglePin(CLOG2_GPIO_Port, CLOG2_Pin);
		HAL_GPIO_TogglePin(CLOG2_GPIO_Port, CLOG2_Pin);
		 HAL_GPIO_TogglePin(CLOG2_GPIO_Port, CLOG2_Pin);
	}
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
