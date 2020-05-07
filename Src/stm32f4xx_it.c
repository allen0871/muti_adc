/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern volatile char init;
#define REFCCR TIM2->CCR2
#define REFNCCR TIM2->CCR1
volatile char flag = 0;
volatile unsigned int* ccra;
volatile unsigned int* ccrb;
void (*time2_handler)(void);
int tim2needStop;
int tim2needStart;

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
			 REFCCR = 150;
			 REFNCCR = 1000;
		   TIM2->ARR = 10000;
		   TIM2->CCMR1 = 0x1010;
		__HAL_TIM_CLEAR_IT(&htim2, TIM_FLAG_UPDATE);
	}
	else if(__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC4) != RESET)
	{
		
		t = 1;
		while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4))
		{
			t++;
			if(t>100)
			{
				t = 0;
				break;
			}
		}
		REFCCR = TIM2->CNT + 60;
		//REFCCR = 800;
		TIM2->CCMR1 = 0x3040; 
		__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC4);
		time2_handler = TIM2_IRQStop;
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
		 HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		 
		 if(GPIOB->IDR & GPIO_PIN_0)
		 //if(0)
		 {
			 REFCCR = 800;
			 REFNCCR = 550;
		 }
		 else
		 {
			 REFCCR = 550;
			 REFNCCR = 800;
		 }
		 TIM2->CCMR1 = 0x3030;  //toggle
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
     __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC4);
		 if(tim2needStop)
		 {
			 tim2needStop = 0;
			 time2_handler = TIM2_IRQClean;
		 }

  }
	if(__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET)
  {
     if(GPIOB->IDR & GPIO_PIN_0)
		 //if(1)
		 {
			 REFCCR = 150;
			 REFNCCR = 400;
		 }
		 else
		 {
			 REFCCR = 400;
			 REFNCCR = 150;
		 }
		 TIM2->CCMR1 = 0x1010;  //OC1M/2M 强制设高
				 HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		 HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		 		 HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		 HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		__HAL_TIM_CLEAR_IT(&htim2, TIM_FLAG_UPDATE);
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

void TIM3_IRQHandler(void) 
{
	if(__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE) != RESET)
	{
		__HAL_TIM_CLEAR_IT(&htim3, TIM_FLAG_UPDATE);
		//time2_handler = TIM2_IRQStart;
		tim2needStart = 1;
	}
	else if(__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_CC1) != RESET)
	{
		__HAL_TIM_CLEAR_IT(&htim3, TIM_FLAG_CC1);
		tim2needStop = 1;
		tim2needStart = 0;
	}
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
