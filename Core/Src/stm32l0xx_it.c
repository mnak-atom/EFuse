/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32l0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdio.h>
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
extern UART_HandleTypeDef huart2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

// void HardFault_Handler_C(uint32_t *fault_stack_addr);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable Interrupt.
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
/* STM32L0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l0xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */
void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}

// __attribute__((naked)) void HardFault_Handler(void)
// {
//     __asm volatile
//     (
//         "movs r1, #4        \n"  /* r1 = 4 */
//         "mov  r0, lr        \n"  /* r0 = EXC_RETURN */
//         "tst  r0, r1        \n"  /* test bit 2 */
//         "beq 1f             \n"  /* if 0 -> MSP */
//         "mrs  r0, psp       \n"  /* r0 = PSP */
//         "b    2f            \n"
//         "1:                 \n"
//         "mrs  r0, msp       \n"  /* r0 = MSP */
//         "2:                 \n"
//         "b    HardFault_Handler_C \n"
//     );
// }

// void HardFault_Handler_C(uint32_t *sp)
// {
//     uint32_t r0  = sp[0];
//     uint32_t r1  = sp[1];
//     uint32_t r2  = sp[2];
//     uint32_t r3  = sp[3];
//     uint32_t r12 = sp[4];
//     uint32_t lr  = sp[5];
//     uint32_t pc  = sp[6];
//     uint32_t psr = sp[7];

//     printf("\r\n[HardFault]\r\n");
//     printf(" R0 = 0x%08lX  R1 = 0x%08lX  R2  = 0x%08lX  R3  = 0x%08lX\r\n",
//            (unsigned long)r0, (unsigned long)r1, (unsigned long)r2, (unsigned long)r3);
//     printf(" R12= 0x%08lX  LR = 0x%08lX  PC  = 0x%08lX  PSR = 0x%08lX\r\n",
//            (unsigned long)r12, (unsigned long)lr, (unsigned long)pc, (unsigned long)psr);

//     while (1) { __NOP(); }
// }

/* USER CODE END 1 */
