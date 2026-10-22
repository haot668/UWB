/**
  ******************************************************************************
  * @file    stm32l4xx_it.h
  * @brief   Interrupt handler declarations for STM32L412KBU6.
  ******************************************************************************
  */

#ifndef __STM32L4xx_IT_H
#define __STM32L4xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif

void SVC_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void TIM1_CC_IRQHandler(void);
void USART2_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void EXTI15_10_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32L4xx_IT_H */
