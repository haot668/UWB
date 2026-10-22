/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @brief   IRQ handlers for Anchor — USART2 + DW3000 EXTI
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
 * When a DW3000 frame arrives, we immediately stop any ranging TX and
 * clear stale EXTI pending flags so the frame processor runs clean. */
void EXTI15_10_IRQHandler(void)
{
    /* Abort any in-progress DW3000 delayed TX (beacon/blink).
     * Frame reception must not be delayed by a pending transmission.
     * Sets DW3000 to IDLE so the callback can process the RX buffer. */
    dwt_forcetrxoff();

    /* Clear the EXTI pending bit for DW_IRQn (EXTI line 12).
     * Writing 1 to the pending bit clears it. This prevents re-entry
     * from a stale edge that occurred before forcetrxoff. */
    EXTI->PR1 = (1u << 12);

    if (__HAL_GPIO_EXTI_GET_IT(DW_IRQn_Pin) != RESET)
    {
        HAL_GPIO_EXTI_IRQHandler(DW_IRQn_Pin);
    }
}
