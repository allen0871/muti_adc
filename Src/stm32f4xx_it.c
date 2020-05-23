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
volatile char adcstart = 0;
 uint32_t adcbuf[10240] = {0};
extern ADC_HandleTypeDef hadc1;
/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
	adcstart = 0;
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
	adcstart = 0;
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

void TIM2_IRQStop(void);

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
			time2_handler = TIM2_IRQStart;
		adc_status = 0;
		curRefCount = 0;
		curRefnCount = 0;
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
			adc_status = 3;
			refTime += (debugRef-curRefCount);
			refnTime += (debugRef-curRefnCount);
		}
		/*
		//等等变负
		while(VZERO_GPIO_Port->IDR & VZERO_Pin);
		//160 cnt后正ref关闭
		REFCCR = TIM2->CNT + 100;
		//负ref打开,上升
		REFNCCR = REFCCR+100; 
		TIM2->CCMR1 = 0x2030; 
		//等待变正
		while(!(VZERO_GPIO_Port->IDR & VZERO_Pin));
		HAL_GPIO_WritePin(CADC_GPIO_Port,CADC_Pin,GPIO_PIN_SET);
		if(!adcstart) {
			adcstart = 1;
			//HAL_ADC_Start_DMA(&hadc1,adcbuf,1024);
		}
		REFCCR = TIM2->CNT + 60;
		TIM2->CCMR1 = 0x3010;
		while(VZERO_GPIO_Port->IDR & VZERO_Pin);
		
		REFCCR = TIM2->CNT + 80;
		REFNCCR = REFCCR;
		TIM2->CCMR1 = 0x3030;
		
		//TIM2->CCR4 = 5500;
		__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC4);
					 		 HAL_GPIO_TogglePin(CLOG2_GPIO_Port, CLOG2_Pin);
			HAL_GPIO_TogglePin(CLOG2_GPIO_Port, CLOG2_Pin);
					 		 HAL_GPIO_TogglePin(CLOG2_GPIO_Port, CLOG2_Pin);
		 HAL_GPIO_TogglePin(CLOG2_GPIO_Port, CLOG2_Pin);
					 		 HAL_GPIO_TogglePin(CLOG2_GPIO_Port, CLOG2_Pin);
		 HAL_GPIO_TogglePin(CLOG2_GPIO_Port, CLOG2_Pin);
		//TIM2->CCR1 = 750;
		//TIM2->ARR = 1679; */
	}
}

void TIM2_IRQStart(void)
{
/*	if(TIM2->CCMR1 != 0x3030)
	{
		//设置OCMode从force low为toggle
		
		TIM2->CCMR1 = 0x3030;
	}*/
	 if(__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC4) != RESET)
  {
		 //HAL_GPIO_TogglePin(CLOG_GPIO_Port, CLOG_Pin);
		 
		 if(VCENT_GPIO_Port->IDR & VCENT_Pin)
		 //if(0)
		 {
			 REFCCR = 1650;
			 REFNCCR = 1050;
			 TIM2->CCMR1 = 0x2020;//强制低
			 refTime += (1650-curRefCount);
			 refnTime += (1050 - curRefnCount);
		 }
		 else
		 {
			 REFCCR = 1050;
			 REFNCCR = 1650;
			 TIM2->CCMR1 = 0x2020;//强制低
			 refTime += (1650-curRefCount);
			 refnTime += (1050 - curRefnCount);
		 }
		//HAL_GPIO_TogglePin(CLOG_GPIO_Port, CLOG_Pin);
     __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC4);


  }
	if(__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET)
  {
     if(VCENT_GPIO_Port->IDR & VCENT_Pin)
		 //if(1)
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
		 
		 //HAL_GPIO_TogglePin(CLOG2_GPIO_Port, CLOG2_Pin);
		 //HAL_GPIO_TogglePin(CLOG2_GPIO_Port, CLOG2_Pin);
		__HAL_TIM_CLEAR_IT(&htim2, TIM_FLAG_UPDATE);
		 		 if(tim2needStop && !tim2needStart)
		 {
			 //tim2needStop = 0;
			 time2_handler = TIM2_IRQClean;
			 REFCCR = 150;
			 curRefCount = 150;
			 REFNCCR = 200000;
		   TIM2->ARR = 419999;
		   TIM2->CCMR1 = 0x1010;
		   TIM2->CCR4 = 150;
			tim2needStop = 0;
			 		 HAL_GPIO_TogglePin(CLOG2_GPIO_Port, CLOG2_Pin);
			HAL_GPIO_TogglePin(CLOG2_GPIO_Port, CLOG2_Pin);
		 }
  }
}

void TIM2_IRQStop(void)
{
	TIM2->CCMR1 = 0x4040;
	TIM2->ARR = 839;
	if(__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET)
	{
		__HAL_TIM_CLEAR_IT(&htim2, TIM_FLAG_UPDATE);
	}
	else if(__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC4) != RESET)
	{
		__HAL_TIM_CLEAR_IT(&htim2, TIM_FLAG_CC4);
		if(tim2needStart)
		{
			tim2needStart = 0;
			time2_handler = TIM2_IRQStart;
		}
	}
}

void TIM5_IRQHandler(void) 
{
	if(__HAL_TIM_GET_FLAG(&htim5, TIM_FLAG_UPDATE) != RESET)
	{
		__HAL_TIM_CLEAR_IT(&htim5, TIM_FLAG_UPDATE);
		//time2_handler = TIM2_IRQStart;
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
