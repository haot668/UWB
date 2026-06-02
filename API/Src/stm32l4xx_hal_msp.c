/**
  ******************************************************************************
  * @file    stm32l4xx_hal_msp.c
  * @brief   HAL MSP for STM32L412KBU6 + DW3210
  *          SPI1: PB3(SCK), PB4(MISO), PB5(MOSI), PA15(NSS-SW)
  *          USART2: PA2(TX), PA3(RX)
  ******************************************************************************
  */
#include "stm32l4xx_hal.h"
#include "main.h"
#include <example_selection.h>

extern void _Error_Handler(char *, int);

void HAL_MspInit(void)
{
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
    HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
    HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
    HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
    HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
    HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 MSP: PB3=SCK(AF5), PB4=MISO(AF5), PB5=MOSI(AF5) */
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    if (hspi->Instance == SPI1)
    {
        __HAL_RCC_SPI1_CLK_ENABLE();

        /* PB3=SCK, PB4=MISO, PB5=MOSI → AF5 */
        GPIO_InitStruct.Pin = DW_SCK_Pin | DW_MISO_Pin | DW_MOSI_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{
    if (hspi->Instance == SPI1)
    {
        __HAL_RCC_SPI1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, DW_SCK_Pin | DW_MISO_Pin | DW_MOSI_Pin);
    }
}

/* TIM1 base */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
    if (htim_base->Instance == TIM1)
    {
        __HAL_RCC_TIM1_CLK_ENABLE();
    }
}

/* TIM1 is used for DW3210 timestamp capture (internal, no output pin needed) */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
    UNUSED(htim);
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
    if (htim_base->Instance == TIM1)
    {
        __HAL_RCC_TIM1_CLK_DISABLE();
        HAL_NVIC_DisableIRQ(TIM1_CC_IRQn);
    }
}

/* USART1/2 MSP */
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
    GPIO_InitTypeDef GPIO_InitStruct;

#if defined(TEST_SS_TWR_RESPONDER) || defined(TEST_TDOA_TAG) || defined(TEST_FUSION_TAG) || defined(TEST_DOWNLINK_TAG) || defined(FUSION_USART1)
    if (huart->Instance == USART1)
    {
        __HAL_RCC_USART1_CLK_ENABLE();

        /* PA9=TX(AF7), PA10=RX(AF7) */
        GPIO_InitStruct.Pin = DBG1_TX_Pin | DBG1_RX_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
#else
    if (huart->Instance == USART2)
    {
        __HAL_RCC_USART2_CLK_ENABLE();

        /* PA2=TX(AF7), PA3=RX(AF7) */
        GPIO_InitStruct.Pin = DBG_TX_Pin | DBG_RX_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
#endif
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
#if defined(TEST_SS_TWR_RESPONDER) || defined(TEST_TDOA_TAG) || defined(TEST_FUSION_TAG) || defined(TEST_DOWNLINK_TAG) || defined(FUSION_USART1)
    if (huart->Instance == USART1)
    {
        __HAL_RCC_USART1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, DBG1_TX_Pin | DBG1_RX_Pin);
        HAL_NVIC_DisableIRQ(USART1_IRQn);
    }
#else
    if (huart->Instance == USART2)
    {
        __HAL_RCC_USART2_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, DBG_TX_Pin | DBG_RX_Pin);
        HAL_NVIC_DisableIRQ(USART2_IRQn);
    }
#endif
}
