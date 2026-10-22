/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @brief   IRQ handlers for Gateway — USART2 + DW3000 EXTI
  ******************************************************************************
  */
#include "stm32l4xx_hal.h"
#include "stm32l4xx.h"
#include "stm32l4xx_it.h"
#include "port.h"
#include "main.h"
#include <deca_device_api.h>

extern UART_HandleTypeDef huart2;

void SVC_Handler(void) {}
void PendSV_Handler(void) {}

void SysTick_Handler(void)
{
    HAL_SYSTICK_IRQHandler();
}

void HAL_SYSTICK_Callback(void)
{
    HAL_IncTick();
}

void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}

void EXTI9_5_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(DW_RESET_Pin) != RESET)
    {
        HAL_GPIO_EXTI_IRQHandler(DW_RESET_Pin);
    }
}

/* UWB RX interrupt — highest priority (0).
 * Gateway uses polling loop for RX; this handler only clears the
 * pending bit so EXTI doesn't re-trigger continuously. */
void EXTI15_10_IRQHandler(void)
{
    EXTI->PR1 = (1u << 12);  /* clear EXTI12 pending */

    if (__HAL_GPIO_EXTI_GET_IT(DW_IRQn_Pin) != RESET)
    {
        HAL_GPIO_EXTI_IRQHandler(DW_IRQn_Pin);
    }
}
