/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Pin definitions for DW3210 on STM32L412KBU6
  *
  *   STM32L412KB          DW3210
  *   ──────────           ──────────
  *   PA2 (TX)  ─────────> UART_RX (debug)
  *   PA3 (RX)  <───────── UART_TX
  *   PB3 (SCK) ─────────> SCK
  *   PB4 (MISO)<───────── MISO
  *   PB5 (MOSI)─────────> MOSI
  *   PA15(nCS)──────────> nCS
  *   PB6 (RSTn)─────────> RSTn
  *   PB7 (WAKE)─────────> WAKE
  *   PA12(IRQ) <───────── IRQ
  *   PA5 (LED1) ────────  LED1
  *   PA11(LED2) ────────  LED2
  ******************************************************************************
  */

#ifndef __MAIN_H__
#define __MAIN_H__

/* ---- Debug UART (USART2: PA2=TX, PA3=RX) ---- */
#define DBG_TX_Pin                  GPIO_PIN_2
#define DBG_TX_GPIO_Port            GPIOA
#define DBG_RX_Pin                  GPIO_PIN_3
#define DBG_RX_GPIO_Port            GPIOA

/* ---- Debug UART1 (USART1: PA9=TX, PA10=RX) ---- */
#define DBG1_TX_Pin                 GPIO_PIN_9
#define DBG1_TX_GPIO_Port           GPIOA
#define DBG1_RX_Pin                 GPIO_PIN_10
#define DBG1_RX_GPIO_Port           GPIOA

/* ---- SPI1: DW3210 (PB3=SCK, PB4=MISO, PB5=MOSI) ---- */
#define DW_SCK_Pin                  GPIO_PIN_3
#define DW_SCK_GPIO_Port            GPIOB
#define DW_MISO_Pin                 GPIO_PIN_4
#define DW_MISO_GPIO_Port           GPIOB
#define DW_MOSI_Pin                 GPIO_PIN_5
#define DW_MOSI_GPIO_Port           GPIOB

/* ---- SPI1 NSS (PA15, SW control) ---- */
#define DW_NSS_Pin                  GPIO_PIN_15
#define DW_NSS_GPIO_Port            GPIOA

/* ---- DW3210 IRQ input (PA12 → EXTI12, EXTI15_10_IRQn) ---- */
#define DW_IRQn_Pin                 GPIO_PIN_12
#define DW_IRQn_GPIO_Port           GPIOA
#define DECAIRQ_EXTI_IRQn           (EXTI15_10_IRQn)

/* ---- DW3210 RESET output (PB6) ---- */
#define DW_RESET_Pin                GPIO_PIN_6
#define DW_RESET_GPIO_Port          GPIOB
/* RSTn EXTI for reset-done detection: PB6 → EXTI6 → EXTI9_5_IRQn */
#define DW_RESET_EXTI_IRQn          (EXTI9_5_IRQn)

/* ---- DW3210 WAKEUP output (PB7) ---- */
#define DW_WAKEUP_Pin               GPIO_PIN_7
#define DW_WAKEUP_GPIO_Port         GPIOB

/* ---- LEDs (PA5=LED1, PA11=LED2) ---- */
#define LED1_Pin                    GPIO_PIN_5
#define LED1_GPIO_Port              GPIOA
#define LED2_Pin                    GPIO_PIN_11
#define LED2_GPIO_Port              GPIOA

/* ---- Optional: measurement time debug pin ---- */
#define DW_MEAS_TIME_Pin            GPIO_PIN_0
#define DW_MEAS_TIME_GPIO_Port      GPIOA

/* ########################## Assert Selection ############################## */
/* #define USE_FULL_ASSERT    1U */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);
#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */
