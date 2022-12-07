/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "adc_config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern __IO uint32_t runDown;
extern __IO uint32_t refp;
extern __IO uint32_t refn;
extern __IO uint32_t nopn;
__IO uint32_t totalCT = 0;
__IO uint32_t totalNPL = 0;
__IO uint32_t tmpNPL = 0;
uint16_t adc_dma_buf[1024];

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim15;
DMA_HandleTypeDef hdma_adc;
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM15_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay_us(uint32_t value) {
	__IO uint32_t tmp = value;
	while(tmp--);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	HAL_NVIC_DisableIRQ(SysTick_IRQn);
	SysTick->CTRL=0x00;
	SysTick->VAL=0x00;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
	__HAL_RCC_TIM15_CLK_ENABLE();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
	MX_TIM15_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_NVIC_EnableIRQ(ADC1_IRQn);
	__HAL_ADC_ENABLE_IT(&hadc, ADC_IT_EOC);
	__HAL_ADC_ENABLE(&hadc);
	hadc.Instance->CR |= ADC_CR_ADSTART;
	//HAL_ADC_Start_IT(&hadc);
	//slop FB1
	HAL_GPIO_WritePin(A0_GPIO_Port,A0_Pin,GPIO_PIN_RESET);
	//set>>VIN, reset>>REFGND
	HAL_GPIO_WritePin(A1_GPIO_Port,A1_Pin,GPIO_PIN_SET);
	//HAL_GPIO_WritePin(VHED_GPIO_Port,VHED_Pin,GPIO_PIN_SET);
	
	//HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
	//HAL_NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
	
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_2);
	HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_1);
	__HAL_TIM_ENABLE(&htim15);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		static double preValue;
		static int32_t preV = 0;
		if(runDown) {
			uint32_t rundownp1 = 0;
			uint32_t rundownp2 = 0;
			__IO uint32_t runupn1 = 0;
			__IO uint32_t tmp = 0;
			__IO uint32_t tmp2 = 0;
			htim1.Instance->CNT = 0;
			//40>>forec inactive, 10 active on match,20>>inactive on match
			htim1.Instance->CCMR1 = 0x1040;
			htim1.Instance->CCR2 = 300;
			//等待积分电压<0
			while((VZERO_GPIO_Port->IDR & VZERO_Pin) && (htim1.Instance->CNT<50000));
			//50 cnt后refp关闭,同时打开refn,积分电压上升
			if(htim1.Instance->CNT<50000) {
				htim1.Instance->CCR2 = htim1.Instance->CNT + 50;
				htim1.Instance->CCR1 = htim1.Instance->CCR2;
				htim1.Instance->CCMR1 = 0x2010;
				rundownp1 = htim1.Instance->CCR2;
			}
			else {
				goto Error;
			}
			//等待积分电压>0
			//while(htim1.Instance->CCR2>htim1.Instance->CNT);
			while(!(VZERO_GPIO_Port->IDR & VZERO_Pin)  && (htim1.Instance->CNT<50000));
			if(htim1.Instance->CNT<50000) {
				htim1.Instance->CCR1 = htim1.Instance->CNT+30;
				htim1.Instance->CCMR1 = 0x4020;
				HAL_GPIO_WritePin(A0_GPIO_Port,A0_Pin,GPIO_PIN_SET);
				//runup时间
				runupn1 = htim1.Instance->CCR1 - rundownp1;
				//CCR2-300是第一次rundown时间
				rundownp1 = rundownp1 - 300;
			}
			else {
				goto Error;
			}
			//等待积分电压>0
			while((!(VZERO_GPIO_Port->IDR & VZERO_Pin)) && (htim1.Instance->CNT<50000));
			if(htim1.Instance->CNT<50000) {
				//正负基准同时开启,因为副基准绝对值较小, 积分电压将缓慢下降
				htim1.Instance->CCR2 = htim1.Instance->CNT+500;
				htim1.Instance->CCR1 = htim1.Instance->CCR2;
				htim1.Instance->CCMR1 = 0x1010;
				tmp = htim1.Instance->CCR1 + 250;

				//开启tim15驱动ADC DMA, 存入adc_dma_buf, 监控积分波形, 找出慢速电压与时间关系
				hadc.Instance->CFGR1 &= ~(0x7<<6);
				hadc.Instance->CFGR1 |= (0x4<<6);
				__HAL_ADC_CLEAR_FLAG(&hadc, (ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR));
				/* Disable the DMA */
				hdma_adc.Instance->CCR &= ~DMA_CCR_EN;
				/* Clear all flags */
			  hdma_adc.DmaBaseAddress->IFCR  = (DMA_FLAG_GL1 << hdma_adc.ChannelIndex);
				/* Configure DMA Channel data length */
				hdma_adc.Instance->CNDTR = 1024;
				/* Configure DMA Channel source address */
				hdma_adc.Instance->CPAR = (uint32_t)&hadc.Instance->DR;
				/* Configure DMA Channel destination address */
				hdma_adc.Instance->CMAR = (uint32_t)adc_dma_buf;
				/* Enable the Peripheral */
				hdma_adc.Instance->CCR |= DMA_CCR_EN;
				//enable dma mode
				hadc.Instance->CFGR1 |= ADC_CFGR1_DMAEN;
				//disable adc it
				__HAL_ADC_DISABLE_IT(&hadc, ADC_IT_EOC);
				//enable timer
				htim15.Instance->CR1|=(TIM_CR1_CEN);

				tmp = htim1.Instance->CCR2;
			}
			else {
				goto Error;
			}
			//等待正负基准开启
			while((htim1.Instance->CNT < tmp) && (htim1.Instance->CNT<50000));
			//等待积分(电压x20)<1000
			while((hadc.Instance->DR>1000) && (htim1.Instance->CNT<50000));
			LOG1_GPIO_Port->BSRR = (uint32_t)LOG1_Pin;
			LOG1_GPIO_Port->BRR = (uint32_t)LOG1_Pin;
			if(htim1.Instance->CNT<50000) {
				//tmp2中存入dma传输个数
				tmp2 = hdma_adc.Instance->CNDTR;
				tmp2 = 1024 - tmp2;
				//关闭正负基准,积分电压保持不变
				htim1.Instance->CCR2 = htim1.Instance->CNT+50;
				htim1.Instance->CCR1 = htim1.Instance->CCR2;
				htim1.Instance->CCMR1 = 0x2020;
				//第二次rundown时间,慢速
				rundownp2 = htim1.Instance->CCR2 - tmp; 
				tmp = htim1.Instance->CCR1 + 3000;
			}
			else {
				goto Error;
			}
			while((htim1.Instance->CNT < tmp) && (htim1.Instance->CNT<50000));
			if(runDown) {
				//__HAL_TIM_DISABLE(&htim15);
				//__HAL_ADC_DISABLE(&hadc);
				htim15.Instance->CR1 &= ~(TIM_CR1_CEN);
				hdma_adc.Instance->CCR &= ~DMA_CCR_EN;
				hadc.Instance->CFGR1 &= ~ADC_CFGR1_DMAEN;
				hadc.Instance->CFGR1 &= ~(0x7<<6);
				hadc.Instance->CFGR1 |= (0x1<<6);
				LOG1_GPIO_Port->BSRR = (uint32_t)LOG1_Pin;
				LOG1_GPIO_Port->BRR = (uint32_t)LOG1_Pin;
				__HAL_ADC_ENABLE_IT(&hadc, ADC_IT_EOC);
				HAL_GPIO_WritePin(A0_GPIO_Port,A0_Pin,GPIO_PIN_RESET);
				runDown = 0;
			  uint16_t t = htim3.Instance->CCMR1;
				t = t & 0xFF00;
				htim3.Instance->CCMR1 = t | 0x50;
				delay_us(100);
				htim3.Instance->CCR1 = htim3.Instance->ARR - 1;
				htim3.Instance->CCMR1 = t | 0x20;
				uint32_t trefp,trefn;
				//临时关闭tm3,进行输出
				htim3.Instance->CR1 &= ~(TIM_CR1_CEN);
				tmp = hdma_adc.Instance->CNDTR;
				tmp = 1024 - tmp;
				int16_t prev = 0;
				int32_t sum = 0,sum2=0;
				int32_t count = 0,count2=0;
				int32_t step = 116;
				int32_t remainV = 0;
				double cha = 0;
				for(int i=0;i<tmp;i++) {
					if(adc_dma_buf[i]<2800) {
						if(i>tmp2) {
							if(count == 16) {
								step = sum>>4;
							}
							tmp2 = tmp;
						}
						if(tmp2 != tmp) {
							if(count<16) {
								sum+= prev-adc_dma_buf[i];
								count++;
							}
						}
						else {
							if(i!=tmp-1) {
								sum2+=adc_dma_buf[i];
								count2++;
							}
						}
					}
					prev = adc_dma_buf[i];
				}
				remainV = sum2/count2;
				cha = remainV - preV;
				cha = (cha/step)*125.0;
				preV = remainV;
				double ws = (rundownp2+cha)/55.55;
				trefp = refp + rundownp1;
				trefn = refn + runupn1;
				double ttt = trefp + ws -trefn;
				ttt = ttt *(-14100000);
				ttt = ttt/totalNPL;
				printf("step=%d remain=%d %d %d %d %d %d %d %d %.2f %.2f %.2f %.2f\n",step,remainV,refp,refn,rundownp1, runupn1, rundownp2,refp+rundownp1, refn+runupn1, rundownp2+cha, ttt, ttt - preValue, cha);
				preValue = ttt;
				htim3.Instance->CCR1 = htim3.Instance->ARR - 1;
				htim3.Instance->CR1|=(TIM_CR1_CEN);
				continue;
			}
Error:
			runDown = 0;
			htim15.Instance->CR1 &= ~(TIM_CR1_CEN);
			hdma_adc.Instance->CCR &= ~DMA_CCR_EN;
			hadc.Instance->CFGR1 &= ~ADC_CFGR1_DMAEN;
			hadc.Instance->CFGR1 &= ~(0x7<<6);
			hadc.Instance->CFGR1 |= (0x1<<6);
			__HAL_ADC_ENABLE_IT(&hadc, ADC_IT_EOC);
			//disableTim2OCInput();
			HAL_GPIO_WritePin(A0_GPIO_Port,A0_Pin,GPIO_PIN_RESET);
			printf("Error -1\n");
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC1_2_EXTERNALTRIG_T1_CC4;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = TZCLK;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  /*sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR2;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }*/
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = TZS;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  //__HAL_TIM_DISABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_1);
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  //__HAL_TIM_DISABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_2);
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = TZCC;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */
  totalCT = NPLCCT*TIMCLKDIV;
  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = TIMCLKDIV-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = NPLCCT + RUNDOWN - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_FORCED_INACTIVE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
	__HAL_TIM_DISABLE_OCxPRELOAD(&htim3, TIM_CHANNEL_1);
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = NPLCCT;
	totalNPL = NPLCCT*TIMCLKDIV;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = SYSMHZ*5-1;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  //HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  //HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, A0_Pin|A1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LOG2_Pin|LOG1_Pin|CT3_Pin|CT2_Pin
                          |CT1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : VHED_Pin VMED_Pin */
  GPIO_InitStruct.Pin = VHED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = VHED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : A0_Pin A1_Pin */
  GPIO_InitStruct.Pin = A0_Pin|A1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LOG2_Pin LOG1_Pin CT3_Pin CT2_Pin
                           CT1_Pin */
  GPIO_InitStruct.Pin = LOG2_Pin|LOG1_Pin|CT3_Pin|CT2_Pin
                          |CT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
int fputc(int ch, FILE *f)
{
      HAL_UART_Transmit(&huart1, (uint8_t *)&ch,1, 0xFFFF);
      return ch;
}

int fgetc(FILE *f)
{
  uint8_t  ch;
	HAL_UART_Receive(&huart1,(uint8_t *)&ch, 1, 0xFFFF);
	return  ch;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
