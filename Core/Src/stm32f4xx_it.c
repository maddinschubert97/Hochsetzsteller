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
#define buf_size 10
extern uint32_t	value_adc1[2];	//
extern uint32_t	value_adc2[2];	//
extern uint32_t	value_adc3[2];	//
extern double U_in;
extern double L;
extern double T;
double U_sum;
double U_sum_neu;
double U_out;
double U_out_mid;
extern double U_in;
extern double U_in_mid;
double i_const;
double i_max;
int reset_cnt;
extern uint16_t safety_dist;
extern double a;
extern double a2;
extern double U_soll;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
uint32_t	lem1[buf_size];	//
uint32_t	lem2[buf_size];	//
uint32_t	lem1_ref[buf_size];	//
uint32_t	lem2_ref[buf_size];
uint32_t	lem_count;
double	U_out_buf[buf_size];
extern double U_in_buf[buf_size];
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
		asm("NOP");
		asm("NOP");
		GPIOB->BSRR=GPIO_PIN_15;
		//HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
		__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC1);
	}
	else if(TIM1->SR & TIM_SR_CC2IF)
	{
		HAL_ADC_Start_DMA(&hadc2,value_adc2,2);
		GPIOB->BSRR=(uint32_t)GPIO_PIN_15 << 16U;
		//HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
		asm("NOP");
		asm("NOP");
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
	//HAL_ADC_Start_DMA(&hadc2,value_adc2,2);
	HAL_ADC_Start_DMA(&hadc3,value_adc3,2);
	/*voltages*/
	U_out=1.1*(double)value_adc1[0]*720/4095+1.7;	//y=1,1*x+1,7
	U_in=1.093*(double)value_adc1[1]*720/4095+0.073;	//casting schlecht für Rechenzeit?
	/*currents*/
	U_out_buf[lem_count]=U_out;
	U_in_buf[lem_count]=U_in;
	lem1[lem_count]=value_adc3[1];
	lem2[lem_count]=value_adc2[1];
	lem1_ref[lem_count]=value_adc3[0];
	lem2_ref[lem_count]=value_adc2[0];
	/*lem buffer mit buf_size Einträgen -> Mittelwert*/
	if(lem_count>buf_size-1)
		lem_count=0;
	else
		lem_count++;
	uint32_t lem1_mid=0,lem2_mid=0,lem1_ref_mid=0,lem2_ref_mid=0;
	for(int i=0;i<buf_size;i++)
	{
		lem1_ref_mid+=lem1_ref[i];
		lem2_ref_mid+=lem2_ref[i];
		lem1_mid+=lem1[i];
		lem2_mid+=lem2[i];
		U_out_mid+=U_out_buf[i];
		U_in_mid+=U_in_buf[i];
	}
	lem1_ref_mid/=buf_size;
	lem2_ref_mid/=buf_size;
	lem1_mid/=buf_size;
	lem2_mid/=buf_size;
	U_out_mid/=buf_size;
	U_in_mid/=buf_size;
	//U_in_mid=20;
	double U_diff=U_soll-U_out_mid;
	//double i_const=(double)(value_adc3[1]-value_adc3[0])*25*2/4095;
	/*Maximaler Strom vom LEM: 25A≙5V, Vref=0A≙2,5V.
	 * Hälfte der 4095 Schritte steht zur Verfügung (da Rest negativ..)
	 * Ausgleichsgeraden I_const: y=1.1*x+0.057, I_max: y=1.1*x-0.15
	 * Faktor: 2*25/(4095*10)=50/4095*/
	i_const=1.1*((double)lem1_mid-(double)lem1_ref_mid)*50/4095-0.057;
	double delta_i=U_in_mid/L*T*a;
	double i_max_ber=i_const+delta_i;
	//double i_max=(double)(value_adc2[1]-value_adc2[0])*25*2/4095;
	i_max=1.1*((double)lem2_mid-(double)lem2_ref_mid)*50/4095+0.15;
	/*Sicherheitsfunktion -> U_out anpassen!*/
	if(i_max_ber>24 || i_const>15 || U_out_mid>120)
	{
		a=0;//a=a-0.2;
		HAL_TIM_OC_Stop_IT(&htim4, TIM_CHANNEL_1);
		TIM1->CCR1=0;	//evtl beim UG interrupt
		TIM1->CCR2=5010;
	}
	double K_p=0.0002;
	double K_i=50;
	if(labs(U_diff+0.5)>0.1*U_soll && U_in_mid>10)	//Große Differenz -> Durchgriff auf Stellgröße
	{
		double a_neu=1-U_in_mid/U_soll;
		double a2_neu=1-a_neu;
		/*if(delta_i>2*i_const)		//Lückgrenze
		{
			a_neu=sqrt((1/U_in_mid-1/U_soll)*2*L*i_const/T);
			//a=sqrt(i_const/U_soll*2*L/T*(U_soll/U_in_mid-1));	//sqrt(((U_soll/U_in)^2-U_soll/U_in)*2*L/(R*branches*T));
			a2=a*U_in/(U_soll-U_in_mid);
		}*/
		if(a_neu<0)
			a=0;
		else if(a_neu>0.95)
			a=0.95;
		else
			a=a_neu;
		if(a2_neu<0)
			a2=0;
		else if(a2_neu>0.95)
			a2=0.95;
		else
			a2=a2_neu;
	}
	else if(U_in_mid>10)	//PI-Regler
	{
			U_sum_neu=U_sum+U_diff*T;	//U_sum global
			double a_neu=K_p*U_diff+K_i*U_sum;
			double a2_neu;
			if(delta_i-2*i_const>0.5)	//Lückbetrieb
				a2_neu=a_neu*U_in_mid/(U_soll-U_in_mid);
			else
				a2_neu=1-a_neu;
			if(a_neu<0)
				a=0;
			else if(a_neu>0.95)
			{
				a=0.95;
				reset_cnt++; //wenn zu lange in Sättigung, U_sum zurücksetzen
				if(reset_cnt>100)
					U_sum=0;
			}
			else
			{
				U_sum=U_sum_neu;	//wenn in Sättigung, nicht integrieren
				a=a_neu;
				reset_cnt=0;
			}
			if(a2_neu<0)
				a2=0;
			else if(a2_neu>0.95)
				a2=0.95;
			else if(a2_neu+a>1)	//zur Sicherheit
				a2_neu=1-a;
			else
				a2=a2_neu;
	}
	else
	{
		a=0.5;
		a2=0.5;
	}
	uint16_t period=TIM1->ARR;
	TIM1->CCR1=0;	//evtl beim UG interrupt
	TIM1->CCR2=a*period;
	//TIM1->CCR3=a*period;
	/*if((a+a2)*period>5010)
		TIM1->CCR4=5010;
	else
		TIM1->CCR4=(a+a2)*period;
	if(period-TIM1->CCR4<safety_dist)
		__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC4);
	else
		__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC4);*/
	//TIM1->CNT=TIM8->CNT;
  /* USER CODE END TIM4_IRQn 0 */
  //HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */
	__HAL_TIM_CLEAR_IT(&htim4, TIM_IT_CC1);
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
