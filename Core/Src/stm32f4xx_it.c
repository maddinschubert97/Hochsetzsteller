/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
extern uint32_t	value_adc1[2];	//
extern uint32_t	value_adc2[2];	//
extern uint32_t	value_adc3[2];	//
extern double U_in;
extern double L;
extern double T;
double U_sum;
double U_sum_neu;
extern uint16_t safety_dist;
extern double a;
extern double a2;
extern double U_soll;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
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
extern DMA_HandleTypeDef hdma_adc2;
extern DMA_HandleTypeDef hdma_adc3;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;
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
  * @brief This function handles TIM1 capture compare interrupt.
  */
void TIM1_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_CC_IRQn 0 */
//schaltet M2. Überprüfen, ob M1 aus ist!
	if(TIM1->SR & TIM_SR_CC1IF)// && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)==GPIO_PIN_RESET)	//Neue Periode
	{
		GPIOE->BSRR=(uint32_t)GPIO_PIN_9 << 16U;
		asm("NOP");
		asm("NOP");
		asm("NOP");
		GPIOB->BSRR=GPIO_PIN_15;
		//HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
		__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC1);
	}
	else if(TIM1->SR & TIM_SR_CC2IF)
	{
		GPIOB->BSRR=(uint32_t)GPIO_PIN_15 << 16U;
		//HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
		asm("NOP");
		asm("NOP");
		asm("NOP");
		GPIOE->BSRR=GPIO_PIN_9;
		__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC2);
	}
	/*else if(TIM1->SR & TIM_SR_CC3IF)
	{
		GPIOB->BSRR=GPIO_PIN_15;
		//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
		__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC3);
	}*/
	else if(TIM1->SR & TIM_SR_CC4IF)
	{
		GPIOE->BSRR=(uint32_t)GPIO_PIN_9 << 16U;
		//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
		__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC4);
	}
  /* USER CODE END TIM1_CC_IRQn 0 */
  //HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_CC_IRQn 1 */
  //int status=TIM1->CNT;
  /* USER CODE END TIM1_CC_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
//U_out einlesen
	HAL_ADC_Start_DMA(&hadc1,value_adc1,2);
	HAL_ADC_Start_DMA(&hadc2,value_adc2,2);
	HAL_ADC_Start_DMA(&hadc3,value_adc3,2);
	double U_out=value_adc1[0]*720/4095;
	U_in=value_adc1[1]*720/4095;
	int U_diff=U_soll-U_out;
	double i_const=(double)(value_adc3[1]-value_adc3[0])*25*2/4095;
	double delta_i=U_in/L*T*a;
	double i_max_ber=i_const+delta_i;
	double i_max=(value_adc2[1]-value_adc2[0])*25*2/4095;
	if(i_max_ber>24 || i_const>15)
		a=a-0.2;
	uint16_t K_p=1/600;
	uint16_t K_i=100;
	if(abs(U_diff)>50)	//Große Differenz -> Durchgriff auf Stellgröße
	{
		a=1-U_in/U_soll;
		a2=1-a;
		/*if(delta_i>2*i_const)		//Lückgrenze
		{
			a=sqrt(i_const/U_soll*2*L/T*(U_soll/U_in-1));	//sqrt(((U_soll/U_in)^2-U_soll/U_in)*2*L/(R*branches*T));
			a2=a*U_in/(U_soll-U_in);
		}*/
	}
	else	//PI-Regler
	{
			U_sum_neu=U_sum+U_diff;	//U_sum global
			a=K_p*U_diff+K_i*T*U_sum;
			if(a<0)
				a=0;
			if(a>0.95)
				a=0.95;
			else
				U_sum=U_sum_neu;
			if(delta_i>2*i_const)	//Lückbetrieb
				a2=a*U_in/(U_soll-U_in);
			else
				a2=1-a;
	}
	uint16_t period=TIM1->ARR;
	TIM1->CCR1=0;	//evtl beim UG interrupt
	TIM1->CCR2=a*period;
	//TIM1->CCR3=a*period;
	if((a+a2)*period>5010)
		TIM1->CCR4=5010;
	else
		TIM1->CCR4=(a+a2)*period;
	if(period-TIM1->CCR4<safety_dist)
		__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC4);
	//else
		//__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC4);
	//TIM1->CNT=TIM8->CNT;
  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles TIM8 capture compare interrupt.
  */
void TIM8_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_CC_IRQn 0 */
	//schaltet M1. Überprüfen, ob M2 aus ist!
	if(TIM8->SR & TIM_SR_CC1IF)// && HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9)==GPIO_PIN_RESET)	//Neue Periode
	{
		//_NOP();	//müsste automatisch eingestellt werden (PWM channel 1)
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
	}
	else if(TIM8->SR & TIM_SR_CC2IF)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	}
  /* USER CODE END TIM8_CC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  /* USER CODE BEGIN TIM8_CC_IRQn 1 */

  /* USER CODE END TIM8_CC_IRQn 1 */
}

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
  * @brief This function handles DMA2 stream1 global interrupt.
  */
void DMA2_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */

  /* USER CODE END DMA2_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc3);
  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

  /* USER CODE END DMA2_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream3 global interrupt.
  */
void DMA2_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream3_IRQn 0 */

  /* USER CODE END DMA2_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc2);
  /* USER CODE BEGIN DMA2_Stream3_IRQn 1 */

  /* USER CODE END DMA2_Stream3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/