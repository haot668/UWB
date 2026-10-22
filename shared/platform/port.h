/*! ----------------------------------------------------------------------------
 * @file    port.h
 * @brief   HW specific definitions for STM32L412KBU6 + DW3210
 *
 *   Pin mapping:
 *     PB3=SPI1_SCK, PB4=SPI1_MISO, PB5=SPI1_MOSI, PA15=SPI1_NSS
 *     PA12=IRQ (EXTI15_10), PB6=RSTn, PB7=WAKE
 *     PA5=LED1, PA11=LED2
 *     USART2: PA2=TX, PA3=RX (debug output)
 *
 * @attention
 * Copyright 2015-2020 (c) DecaWave Ltd, Dublin, Ireland.
 * All rights reserved.
 */

#ifndef PORT_H_
#define PORT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>
#include <stm32l4xx_hal.h>

typedef unsigned int useconds_t;

#define EVB1000_LED_SUPPORT 0
#define EVB1000_LCD_SUPPORT 0

typedef void (*port_dwic_isr_t)(void);

void port_set_dwic_isr(port_dwic_isr_t isr);

#define BUFFLEN     (4096+128)
#define BUF_SIZE    (64)

typedef __packed struct
{
    uint16_t        usblen;
    uint8_t         usbbuf[BUF_SIZE*3];
} app_t;

extern app_t    app;

#ifndef FALSE
#define FALSE               0
#endif
#ifndef TRUE
#define TRUE                1
#endif

typedef enum
{
    LED1 = 0,
    LED2,
    LED_ALL,
    LEDn
} led_t;

/* =====================================================================
 *   PIN DEFINITIONS — must match main.h
 * ===================================================================== */

/* ---- SPI1 pins (AF5) ---- */
#define DW_SCK_Pin                  GPIO_PIN_3
#define DW_SCK_GPIO_Port            GPIOB
#define DW_MISO_Pin                 GPIO_PIN_4
#define DW_MISO_GPIO_Port           GPIOB
#define DW_MOSI_Pin                 GPIO_PIN_5
#define DW_MOSI_GPIO_Port           GPIOB

/* ---- SPI NSS (SW controlled) ---- */
#define DW_NSS_Pin                  GPIO_PIN_15
#define DW_NSS_GPIO_Port            GPIOA

/* ---- DW3210 IRQ (PA12 → EXTI12 → EXTI15_10_IRQn) ---- */
#define DW_IRQn_Pin                 GPIO_PIN_12
#define DW_IRQn_GPIO_Port           GPIOA
#define DECAIRQ_EXTI_IRQn           (EXTI15_10_IRQn)

/* ---- DW3210 RESET (PB6 → EXTI6 → EXTI9_5_IRQn for input) ---- */
#define DW_RESET_Pin                GPIO_PIN_6
#define DW_RESET_GPIO_Port          GPIOB
#define DW_RESET_EXTI_IRQn          (EXTI9_5_IRQn)

/* ---- DW3210 WAKEUP (PB7) ---- */
#define DW_WAKEUP_Pin               GPIO_PIN_7
#define DW_WAKEUP_GPIO_Port         GPIOB

/* ---- LEDs ---- */
#define LED1_Pin                    GPIO_PIN_5
#define LED1_GPIO_Port              GPIOA
#define LED2_Pin                    GPIO_PIN_11
#define LED2_GPIO_Port              GPIOA

/* ---- Debug UART (USART2) ---- */
#define DBG_TX_Pin                  GPIO_PIN_2
#define DBG_TX_GPIO_Port            GPIOA
#define DBG_RX_Pin                  GPIO_PIN_3
#define DBG_RX_GPIO_Port            GPIOA

/* ---- RF Switch SW8112QU4: ENT=PA7(TX_EN), ENR=PA6(RX_EN) ---- */
#define RF_SW_TX_EN_Pin             GPIO_PIN_7   /* PA7 = ENT */
#define RF_SW_TX_EN_GPIO_Port       GPIOA
#define RF_SW_RX_EN_Pin             GPIO_PIN_6   /* PA6 = ENR */
#define RF_SW_RX_EN_GPIO_Port       GPIOA

/* ---- Measurement time debug pin ---- */
#define DW_MEAS_TIME_Pin            GPIO_PIN_0
#define DW_MEAS_TIME_GPIO_Port      GPIOA

/****************************************************************************//**
 *                              MACRO functions
 *******************************************************************************/
#define GPIO_ResetBits(x,y)             HAL_GPIO_WritePin(x,y, RESET)
#define GPIO_SetBits(x,y)               HAL_GPIO_WritePin(x,y, SET)
#define GPIO_ReadInputDataBit(x,y)      HAL_GPIO_ReadPin (x,y)

#define port_SPIx_set_chip_select()     HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_SET)
#define port_SPIx_clear_chip_select()   HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_RESET)

/****************************************************************************//**
 *                              Function prototypes
 *******************************************************************************/
int usleep(useconds_t usec);
void Sleep(uint32_t Delay);
uint32_t portGetTickCnt(void);

void port_set_dw_ic_spi_slowrate(void);
void port_set_dw_ic_spi_fastrate(void);

void process_deca_irq(void);
void led_on(led_t led);
void led_off(led_t led);

int  peripherals_init(void);
void spi_peripheral_init(void);
void setup_DWICRSTnIRQ(int enable);
void reset_DWIC(void);

ITStatus EXTI_GetITEnStatus(IRQn_Type x);
uint32_t port_GetEXT_IRQStatus(void);
uint32_t port_CheckEXT_IRQ(void);
void port_DisableEXT_IRQ(void);
void port_EnableEXT_IRQ(void);
extern uint32_t HAL_GetTick(void);

void wakeup_device_with_io(void);
void make_very_short_wakeup_io(void);

void uart_tx_str(const char *str);
void uart_tx_data(uint8_t *data, uint16_t len);

void port_rf_switch_init(void);
void port_rf_switch_tx(void);
void port_rf_switch_rx(void);
void port_rf_switch_disable(void);

/* Measurement / Wakeup pin toggles */
#define SET_MEAS_PIN_IO_LOW     HAL_GPIO_WritePin(DW_MEAS_TIME_GPIO_Port, DW_MEAS_TIME_Pin, GPIO_PIN_RESET)
#define SET_MEAS_PIN_IO_HIGH    HAL_GPIO_WritePin(DW_MEAS_TIME_GPIO_Port, DW_MEAS_TIME_Pin, GPIO_PIN_SET)

#define SET_WAKEUP_PIN_IO_LOW     HAL_GPIO_WritePin(DW_WAKEUP_GPIO_Port, DW_WAKEUP_Pin, GPIO_PIN_RESET)
#define SET_WAKEUP_PIN_IO_HIGH    HAL_GPIO_WritePin(DW_WAKEUP_GPIO_Port, DW_WAKEUP_Pin, GPIO_PIN_SET)

#define WAIT_500uSEC    Sleep(1)

/* LCD functions disabled */
#define writetoLCD(x)
#define lcd_display_str(x) ((void)0)
#define lcd_display_str2(x) ((void)0)

#ifdef __cplusplus
}
#endif

#endif /* PORT_H_ */


#ifndef _LINUX_CIRC_BUF_H
#define _LINUX_CIRC_BUF_H 1

struct circ_buf {
    char *buf;
    int head;
    int tail;
};

#define CIRC_CNT(head,tail,size) (((head) - (tail)) & ((size)-1))
#define CIRC_SPACE(head,tail,size) CIRC_CNT((tail),((head)+1),(size))
#define CIRC_CNT_TO_END(head,tail,size) \
    ({int end = (size) - (tail); \
      int n = ((head) + end) & ((size)-1); \
      n < end ? n : end;})
#define CIRC_SPACE_TO_END(head,tail,size) \
    ({int end = (size) - 1 - (head); \
      int n = (end + (tail)) & ((size)-1); \
      n <= end ? n : end+1;})

#endif /* _LINUX_CIRC_BUF_H  */
