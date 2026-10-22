/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @brief   Interrupt Service Routines for STM32L412KBU6 + DW3210
  ******************************************************************************
  */
#include "stm32l4xx_hal.h"
#include "stm32l4xx.h"
#include "stm32l4xx_it.h"
#include "port.h"
#include <example_selection.h>

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
#if defined(TEST_SS_TWR_RESPONDER) || defined(TEST_TDOA_TAG) || defined(TEST_FUSION_TAG) || defined(TEST_DOWNLINK_TAG) || defined(FUSION_USART1)
extern UART_HandleTypeDef huart1;
#else
extern UART_HandleTypeDef huart2;
#endif

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */
/******************************************************************************/

void SVC_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
    HAL_SYSTICK_IRQHandler();
}

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/******************************************************************************/

/**
  * @brief TIM1 capture compare interrupt handler
  */
void TIM1_CC_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim1);
}

/**
  * @brief USART1/2 global interrupt handler (debug output)
  */
#if defined(TEST_SS_TWR_RESPONDER) || defined(TEST_TDOA_TAG) || defined(TEST_FUSION_TAG) || defined(TEST_DOWNLINK_TAG) || defined(FUSION_USART1)
void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}
#else
void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}
#endif

/**
  * @brief EXTI9_5 interrupt handler — DW_RESET (PB6 → EXTI6)
  *        Fires when DW3210 reset completes (rising edge on RESET pin)
  */
void EXTI9_5_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(DW_RESET_Pin) != RESET)
    {
        HAL_GPIO_EXTI_IRQHandler(DW_RESET_Pin);
    }
}

/**
  * @brief EXTI15_10 interrupt handler — DW_IRQn (PA12 → EXTI12)
  *        Fires when DW3210 asserts its IRQ line
  */
void EXTI15_10_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(DW_IRQn_Pin) != RESET)
    {
        HAL_GPIO_EXTI_IRQHandler(DW_IRQn_Pin);
    }
}
