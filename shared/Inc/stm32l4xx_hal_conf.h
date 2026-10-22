/**
  ******************************************************************************
  * @file    stm32l4xx_hal_conf.h
  * @brief   HAL configuration for STM32L412KBU6 + DW3210
  *
  *   Modules used:
  *     - SPI1  (DW3210 communication, PB3/PB4/PB5, PA15 NSS)
  *     - USART2 (debug output, PA2/PA3)
  *     - TIM1  (DW3210 timestamp capture)
  *     - GPIO  (pin control, LEDs, IRQ, WAKEUP)
  *     - PWR   (voltage scaling for 80MHz)
  *     - RCC   (clock configuration: MSI → PLL → 80MHz)
  *     - CORTEX (NVIC, SysTick)
  *
  *   NOT present on STM32L412KB:
  *     - No hardware RNG  (software LFSR used instead)
  *     - No USB OTG       (USART2 used for debug output)
  *     - No DMA used      (SPI and UART in polled mode)
  ******************************************************************************
  */

#ifndef __STM32L4xx_HAL_CONF_H
#define __STM32L4xx_HAL_CONF_H

#ifdef __cplusplus
 extern "C" {
#endif

/* ########################## Module Selection ############################## */
#define HAL_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_SPI_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
/* #define HAL_RNG_MODULE_ENABLED */
/* #define HAL_PCD_MODULE_ENABLED */
#define HAL_FLASH_MODULE_ENABLED
/* #define HAL_I2C_MODULE_ENABLED */
/* #define HAL_CRC_MODULE_ENABLED */

/* ########################## Oscillator Values ############################# */
#if !defined(MSI_VALUE)
  #define MSI_VALUE    4000000U  /* MSI configured to 4 MHz (MSIRANGE_6) */
#endif

#if !defined(HSI_VALUE)
  #define HSI_VALUE    16000000U /* 16 MHz internal RC */
#endif

#if !defined(HSE_VALUE)
  #define HSE_VALUE    8000000U  /* Not used — MSI is the clock source */
#endif

#if !defined(LSE_VALUE)
  #define LSE_VALUE    32768U
#endif

#if !defined(LSI_VALUE)
  #define LSI_VALUE    32000U
#endif

#if !defined(HSI48_VALUE)
  #define HSI48_VALUE  48000000U /* 48 MHz internal RC, not used on L412 */
#endif

/* ########################### System Configuration ######################### */
#define HSE_STARTUP_TIMEOUT  100U
#define HSI_STARTUP_TIMEOUT  100U
#define LSE_STARTUP_TIMEOUT  5000U
#define LSI_STARTUP_TIMEOUT  100U
#define VDD_VALUE                3300U
#define TICK_INT_PRIORITY        0U
#define USE_RTOS                 0U
#define PREFETCH_ENABLE          1U
#define INSTRUCTION_CACHE_ENABLE 1U
#define DATA_CACHE_ENABLE        1U

/* ########################## Assert Selection ############################## */
/* #define USE_FULL_ASSERT    1U */
#ifdef USE_FULL_ASSERT
  #define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
#else
  #define assert_param(expr) ((void)0U)
#endif

/* ################## SPI peripheral configuration ########################## */
#define USE_SPI_CRC     0U

/* Includes ------------------------------------------------------------------*/
#include <stm32l4xx_hal_def.h>
#include <stm32l4xx_hal_cortex.h>
#include <stm32l4xx_hal_dma.h>
#include <stm32l4xx_hal_flash.h>
#include <stm32l4xx_hal_gpio.h>
#include <stm32l4xx_hal_pwr.h>
#include <stm32l4xx_hal_rcc.h>
#include <stm32l4xx_hal_spi.h>
#include <stm32l4xx_hal_tim.h>
#include <stm32l4xx_hal_uart.h>

#ifdef __cplusplus
}
#endif

#endif /* __STM32L4xx_HAL_CONF_H */
