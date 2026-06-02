/*! ----------------------------------------------------------------------------
 * @file    port.c
 * @brief   HW specific functions for STM32L412KBU6 + DW3210
 *
 * @attention
 * Copyright 2015-2020 (c) DecaWave Ltd, Dublin, Ireland.
 * All rights reserved.12
 */

#include <port.h>
#include <stm32l4xx_hal.h>
#include <example_selection.h>

/****************************************************************************//**
 *
 *                              APP global variables
 *
 *******************************************************************************/
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;

/****************************************************************************//**
 *
 *                  Port private variables and function prototypes
 *
 *******************************************************************************/
static volatile uint32_t signalResetDone;

/* DW IC IRQ handler definition. */
static port_dwic_isr_t port_dwic_isr = NULL;

/****************************************************************************//**
 *
 *                              Time section
 *
 *******************************************************************************/

/* @fn    portGetTickCnt
 * @brief wrapper for to read a SysTickTimer, which is incremented with
 *        CLOCKS_PER_SEC frequency.
 *        The resolution is usually 1/1000 sec.
 * */
uint32_t portGetTickCnt(void)
{
    return HAL_GetTick();
}

/* @fn    usleep
 * @brief precise usleep() delay using NOP loop.
 *        Calibrated for ~80MHz STM32L4.
 * */
int usleep(useconds_t usec)
{
    unsigned int i;

    usec *= 6;  /* tuned for ~80MHz L4 core */
    for (i = 0; i < usec; i++)
    {
        __NOP();
    }
    return 0;
}

/* @fn    Sleep
 * @brief Sleep delay in ms using NOP loop (does NOT depend on SysTick).
 *        Calibrated for ~80MHz STM32L4.
 * */
void Sleep(uint32_t x)
{
    volatile uint32_t i;
    while (x--)
    {
        for (i = 0; i < 13333; i++)  /* ~1ms at 80MHz */
        {
            __NOP();
        }
    }
}

/****************************************************************************//**
 *
 *                              END OF Time section
 *
 *******************************************************************************/

/****************************************************************************//**
 *
 *                              Configuration section
 *
 *******************************************************************************/

/* @fn    peripherals_init
 * */
int peripherals_init(void)
{
    /* All peripherals initialized in CubeMX code (main.c) */
    return 0;
}

/* @fn    spi_peripheral_init
 * */
void spi_peripheral_init()
{
    /* SPI initialized in CubeMX code (main.c) */
}

/**
  * @brief  Checks whether the specified IRQn line is enabled or not.
  * @param  IRQn: specifies the IRQn line to check.
  * @return "0" when IRQn is "not enabled" and !0 otherwise
  */
ITStatus EXTI_GetITEnStatus(IRQn_Type IRQn)
{
    return ((NVIC->ISER[(((uint32_t)(int32_t)IRQn) >> 5UL)] &
        (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL))) == (uint32_t)RESET) ? (RESET) : (SET);
}

/****************************************************************************//**
 *
 *                          End of configuration section
 *
 *******************************************************************************/

/****************************************************************************//**
 *
 *                          DW IC port section
 *
 *******************************************************************************/

/* @fn      reset_DWIC
 * @brief   DW_RESET pin on DW IC has 2 functions.
 *          In general it is output, but it also can be used to reset the digital
 *          part of DW IC by driving this pin low.
 *          Note: the DW_RESET pin should not be driven high externally.
 * */
void reset_DWIC(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* Enable GPIO used for DW3210 reset as open collector output */
    GPIO_InitStruct.Pin = DW_RESET_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DW_RESET_GPIO_Port, &GPIO_InitStruct);

    /* Drive the RSTn pin low */
    HAL_GPIO_WritePin(DW_RESET_GPIO_Port, DW_RESET_Pin, GPIO_PIN_RESET);

    usleep(1);

    /* Put the pin back to output open-drain (not active) */
    setup_DWICRSTnIRQ(0);
    Sleep(2);
}

/* @fn      setup_DWICRSTnIRQ
 * @brief   setup the DW_RESET pin mode
 *          0 - output Open collector mode
 *          !0 - input mode with connected EXTI IRQ
 *
 * NOTE: DW_RESET_Pin (PB6) → EXTI6 → EXTI9_5_IRQn on STM32L412KB.
 * */
void setup_DWICRSTnIRQ(int enable)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    if (enable)
    {
        /* Enable GPIO used as DW RESET for interrupt (rising edge) */
        GPIO_InitStruct.Pin = DW_RESET_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(DW_RESET_GPIO_Port, &GPIO_InitStruct);

        HAL_NVIC_EnableIRQ(DW_RESET_EXTI_IRQn);
        HAL_NVIC_SetPriority(DW_RESET_EXTI_IRQn, 5, 0);
    }
    else
    {
        HAL_NVIC_DisableIRQ(DW_RESET_EXTI_IRQn);

        /* Put the pin back to output push-pull (actively drive HIGH to release reset) */
        GPIO_InitStruct.Pin = DW_RESET_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(DW_RESET_GPIO_Port, &GPIO_InitStruct);
        HAL_GPIO_WritePin(DW_RESET_GPIO_Port, DW_RESET_Pin, GPIO_PIN_SET);
    }
}

/* @fn      wakeup_device_with_io()
 * @brief   Wakes up the device by toggling WAKEUP pin with a delay.
 * */
void wakeup_device_with_io(void)
{
    SET_WAKEUP_PIN_IO_HIGH;
    WAIT_500uSEC;
    SET_WAKEUP_PIN_IO_LOW;
}

/* @fn      make_very_short_wakeup_io()
 * @brief   Toggle the wakeup pin for a very short time. Device should not wake up.
 * */
void make_very_short_wakeup_io(void)
{
    uint8_t cnt;

    SET_WAKEUP_PIN_IO_HIGH;
    for (cnt = 0; cnt < 10; cnt++)
        __NOP();
    SET_WAKEUP_PIN_IO_LOW;
}

/* @fn      led_off
 * @brief   switch off the led
 * */
void led_off(led_t led)
{
    switch (led)
    {
    case LED1:
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
        break;
    case LED2:
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
        break;
    case LED_ALL:
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
        break;
    default:
        break;
    }
}

/* @fn      led_on
 * @brief   switch on the led
 * */
void led_on(led_t led)
{
    switch (led)
    {
    case LED1:
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
        break;
    case LED2:
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
        break;
    case LED_ALL:
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
        break;
    default:
        break;
    }
}

/* @fn      port_set_dw_ic_spi_slowrate
 * @brief   Set slow SPI rate (~2.5MHz)
 *          hspi1 is clocked from APB2 (80MHz on STM32L412)
 * */
void port_set_dw_ic_spi_slowrate(void)
{
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;  /* 80/32 = 2.5MHz */
    HAL_SPI_Init(&hspi1);
}

/* @fn      port_set_dw_ic_spi_fastrate
 * @brief   Set fast SPI rate (~20MHz)
 *          hspi1 is clocked from APB2 (80MHz on STM32L412)
 * */
void port_set_dw_ic_spi_fastrate(void)
{
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;   /* 80/4 = 20MHz */
    HAL_SPI_Init(&hspi1);
}

/* @fn      port_rf_switch_init
 * @brief   Initialize STM32 GPIOs for external RF switch SW8112QU4 control.
 *          Pin mapping to be confirmed by hardware.
 * */
void port_rf_switch_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* Ensure GPIOA clock is on (belt-and-suspenders) */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = RF_SW_TX_EN_Pin | RF_SW_RX_EN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;  /* max drive strength */
    HAL_GPIO_Init(RF_SW_TX_EN_GPIO_Port, &GPIO_InitStruct);

    /* Default to RX mode (safe): TX_EN=0, RX_EN=1 */
    port_rf_switch_rx();
}

/* @fn      port_rf_switch_tx
 * @brief   Set RF switch to TX path: TX_EN=1, RX_EN=0
 * */
void port_rf_switch_tx(void)
{
    HAL_GPIO_WritePin(RF_SW_TX_EN_GPIO_Port, RF_SW_TX_EN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RF_SW_RX_EN_GPIO_Port, RF_SW_RX_EN_Pin, GPIO_PIN_RESET);
}

/* @fn      port_rf_switch_rx
 * @brief   Set RF switch to RX path: TX_EN=0, RX_EN=1
 * */
void port_rf_switch_rx(void)
{
    HAL_GPIO_WritePin(RF_SW_TX_EN_GPIO_Port, RF_SW_TX_EN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RF_SW_RX_EN_GPIO_Port, RF_SW_RX_EN_Pin, GPIO_PIN_SET);
}

/****************************************************************************//**
 *
 *                          End APP port section
 *
 *******************************************************************************/

/****************************************************************************//**
 *
 *                              IRQ section
 *
 *******************************************************************************/

/* @fn         HAL_GPIO_EXTI_Callback
 * @brief      EXTI line detection callback from HAL layer.
 *             Handles DW_RESET and DW_IRQn pins.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin)
    {
    case DW_RESET_Pin:
        signalResetDone = 1;
        break;

    case DW_IRQn_Pin:
    {
        process_deca_irq();
        break;
    }

    default:
        break;
    }
}

/* @fn      process_deca_irq
 * @brief   Main call-back for processing DW3210 IRQ.
 *          Re-enters the IRQ routine and processes all events.
 *          After all events are processed, DW3210 clears the IRQ line.
 * */
void process_deca_irq(void)
{
    while (port_CheckEXT_IRQ() != 0)
    {
        if (port_dwic_isr)
        {
            port_dwic_isr();
        }
    }
}

/* @fn      port_DisableEXT_IRQ
 * @brief   Disable DW_IRQ pin interrupt
 * */
__INLINE void port_DisableEXT_IRQ(void)
{
    NVIC_DisableIRQ(DECAIRQ_EXTI_IRQn);
}

/* @fn      port_EnableEXT_IRQ
 * @brief   Enable DW_IRQ pin interrupt
 * */
__INLINE void port_EnableEXT_IRQ(void)
{
    NVIC_EnableIRQ(DECAIRQ_EXTI_IRQn);
}

/* @fn      port_GetEXT_IRQStatus
 * @brief   Read DW_IRQ pin interrupt enable status
 * */
uint32_t port_GetEXT_IRQStatus(void)
{
    return EXTI_GetITEnStatus(DECAIRQ_EXTI_IRQn);
}

/* @fn      port_CheckEXT_IRQ
 * @brief   Read DW_IRQ input pin state
 * */
__INLINE uint32_t port_CheckEXT_IRQ(void)
{
    return HAL_GPIO_ReadPin(DW_IRQn_GPIO_Port, DW_IRQn_Pin);
}

/****************************************************************************//**
 *
 *                              END OF IRQ section
 *
 *******************************************************************************/

/****************************************************************************//**
 *
 *                              UART debug output section
 *
 *******************************************************************************/

/* @fn      uart_tx_str
 * @brief   Transmit a null-terminated string via USART2 (debug output).
 * */
void uart_tx_str(const char *str)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 1000);
}

/* @fn      uart_tx_data
 * @brief   Transmit raw data via USART (debug output).
 * */
void uart_tx_data(uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(&huart2, data, len, 1000);
}

/****************************************************************************//**
 *
 *                              END OF UART debug section
 *
 *******************************************************************************/

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn port_set_dwic_isr()
 *
 * @brief Install the handling function for DW IC IRQ.
 *
 * NOTE:
 *   - The user application must ensure a proper handler is set by calling this function before any DW IC IRQ occurs!
 *   - This function deactivates the DW IC IRQ line while the handler is installed.
 *
 * @param dwic_isr function pointer to DW IC interrupt handler to install
 *
 * @return none
 */
void port_set_dwic_isr(port_dwic_isr_t dwic_isr)
{
    uint32_t en = port_GetEXT_IRQStatus();

    port_DisableEXT_IRQ();

    port_dwic_isr = dwic_isr;

    if (!en)
    {
        port_EnableEXT_IRQ();
    }
}
