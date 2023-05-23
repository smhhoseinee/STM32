/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_it.h"
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
int state=0;
extern int input_completed;
extern int is_similar;
extern double similarity_percentage;
extern int number_of_words1;
extern int number_of_words2;
extern int restart = 0;
int switch_delay = 10;
int final_delay = 4;

void num1(int a){
	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, 0);
	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1,1);
	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 1);
	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, 1);
	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0);
	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4 ,a%10);
	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7,((a%100)-a%10)/10);
	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, ((a%1000)-a%10-(((a%100)-a%10)/10))/100);
	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, ((a%10000)-a%10-(((a%100)-a%10)/10)-(((a%1000)-a%10-(((a%100)-a%10)/10))/100))/1000);
}

void num2(int b){
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, 1);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1,1);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 1);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, 0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4 ,b%10);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7,((b%100)-b%10)/10);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, ((b%1000)-b%10-(((b%100)-b%10)/10))/100);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, ((b%10000)-b%10-(((b%100)-b%10)/10)-(((b%1000)-b%10-(((b%100)-b%10)/10))/100))/1000);
}

void num3(int c){
	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, 1);
	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1,1);
	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 0);
	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, 1);
	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1);
	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4 ,c%10);
	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7,((c%100)-c%10)/10);
	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, ((c%1000)-c%10-(((c%100)-c%10)/10))/100);
	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, ((c%10000)-c%10-(((c%100)-c%10)/10)-(((c%1000)-c%10-(((c%100)-c%10)/10))/100))/1000);
}
void num4(int d){
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, 1);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, 0);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 1);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, 1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4 ,d%10);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7,((d%100)-d%10)/10);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, ((d%1000)-d%10-(((d%100)-d%10)/10))/100);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, ((d%10000)-d%10-(((d%100)-d%10)/10)-(((d%1000)-d%10-(((d%100)-d%10)/10))/100))/1000);
}

int decimalToBinary(int num) {
    if (num == 0) {
        return 0;
    }

   // Stores binary representation of number.
   int binaryNum[4]={0,0,0,0}; // Assuming 32 bit integer.
   int i=0;

   for ( ;num > 0; ){
      binaryNum[i++] = num % 2;
      num /= 2;
   }
   int k=0;
   k=binaryNum[3]*1000+binaryNum[2]*100+binaryNum[1]*10+binaryNum[0];
   return k;
}


/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
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
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
//	similarity_percentage = 67.89;
	int a = (int)similarity_percentage / 10;
	int b = (int)(similarity_percentage - (a*10));
	int c = (int)((similarity_percentage - (a*10 + b))*10);
	int d = (int)((similarity_percentage - (a*10 + b + c*0.1))*100);

//	if(is_similar){
	if(1){
		if (state == 0) {
			num1(decimalToBinary(a));
			state = 1;
		} else if (state == 1) {
			num2(decimalToBinary(b));
			state = 2;
		} else if(state == 2) {
			num3(decimalToBinary(c));
			state = 3;
		} else if (state == 3) {
			num4(decimalToBinary(d));
			state = 0;
		}
	}


  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
	if (input_completed == 0) {
		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_7);
//		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, 1);

	}else{
		if(number_of_words1 > 0){
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_7);
			number_of_words1--;
		}else{
			if(switch_delay > 0){
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, 0);
				switch_delay--;
			}else{
				if(number_of_words2 > 0){
					HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_7);
					number_of_words2--;
				}else{
					if(is_similar){
						if(final_delay > 0){
							HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, 1);
							final_delay--;
						}else{
							HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, 0);
							HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, 0);

							restart = 1;
							input_completed = 0;
							is_similar = 0;
							final_delay = 4;
							switch_delay = 10;
						}
					}
				}
			}
		}
	}
	if(is_similar){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, 1);
	}else{
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, 0);
	}


  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
