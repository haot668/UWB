/**
  ******************************************************************************
  * @file    stm32l4xx_hal_timebase_TIM.c
  * @brief   HAL time base based on TIM6 for STM32L4xx.
  *          Provides 1ms tick using TIM6.
  ******************************************************************************
  */

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_tim.h"

TIM_HandleTypeDef        htim6;
uint32_t                 uwIncrementState = 0;

/**
  * @brief  Configure TIM6 as the time base source (1ms tick).
  * @param  TickPriority: Tick interrupt priority.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
    RCC_ClkInitTypeDef    clkconfig;
    uint32_t              uwTimclock = 0;
    uint32_t              uwPrescalerValue = 0;
    uint32_t              pFLatency;

    /* Configure the TIM6 IRQ priority */
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, TickPriority, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

    /* Enable TIM6 clock */
    __HAL_RCC_TIM6_CLK_ENABLE();

    /* Get clock configuration */
    HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);

    /* Compute TIM6 clock (APB1 timer clock = 2 * APB1 when prescaler > 1 or = APB1) */
    uwTimclock = HAL_RCC_GetPCLK1Freq();
    if (clkconfig.APB1CLKDivider != RCC_HCLK_DIV1)
    {
        uwTimclock *= 2;
    }

    /* Compute prescaler to get 1MHz timer clock */
    uwPrescalerValue = (uint32_t)((uwTimclock / 1000000U) - 1U);

    /* Initialize TIM6 */
    htim6.Instance = TIM6;

    /* Period = (1MHz / 1000) - 1 = 999 → 1kHz → 1ms */
    htim6.Init.Period = (1000000U / 1000U) - 1U;
    htim6.Init.Prescaler = uwPrescalerValue;
    htim6.Init.ClockDivision = 0;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;

    if (HAL_TIM_Base_Init(&htim6) == HAL_OK)
    {
        return HAL_TIM_Base_Start_IT(&htim6);
    }

    return HAL_ERROR;
}

/**
  * @brief  Suspend Tick increment.
  */
void HAL_SuspendTick(void)
{
    __HAL_TIM_DISABLE_IT(&htim6, TIM_IT_UPDATE);
}

/**
  * @brief  Resume Tick increment.
  */
void HAL_ResumeTick(void)
{
    __HAL_TIM_ENABLE_IT(&htim6, TIM_IT_UPDATE);
}
