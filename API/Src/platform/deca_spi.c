/*! ----------------------------------------------------------------------------
 * @file    deca_spi.c
 * @brief   SPI access functions
 *          Ported from STM32F4 to STM32L4
 *
 * @attention
 *
 * Copyright 2015-2020 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */

#include <deca_spi.h>
#include <deca_device_api.h>
#include <port.h>
#include <stm32l4xx_hal_def.h>

extern SPI_HandleTypeDef hspi1;

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: openspi()
 *
 * Low level abstract function to open and initialise access to the SPI device.
 * returns 0 for success, or -1 for error
 */
int openspi(void)
{
    return 0;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: closespi()
 *
 * Low level abstract function to close the SPI device.
 * returns 0 for success, or -1 for error
 */
int closespi(void)
{
    return 0;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: writetospiwithcrc()
 *
 * Low level abstract function to write to the SPI when SPI CRC mode is used.
 * Takes two separate byte buffers for write header and write data, and a CRC8 byte written last.
 * returns 0 for success, or -1 for error
 */
int writetospiwithcrc(
                uint16_t      headerLength,
                const uint8_t *headerBuffer,
                uint16_t      bodyLength,
                const uint8_t *bodyBuffer,
                uint8_t       crc8)
{
    decaIrqStatus_t stat;
    stat = decamutexon();

    while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);

    HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_RESET);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)headerBuffer, headerLength, 10);
    HAL_SPI_Transmit(&hspi1, (uint8_t *)bodyBuffer, bodyLength, 10);
    HAL_SPI_Transmit(&hspi1, (uint8_t *)&crc8, 1, 10);

    HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_SET);

    decamutexoff(stat);
    return 0;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: writetospi()
 *
 * Low level abstract function to write to the SPI.
 * Takes two separate byte buffers for write header and write data.
 * returns 0 for success, or -1 for error
 */
int writetospi(uint16_t       headerLength,
               const uint8_t  *headerBuffer,
               uint16_t       bodyLength,
               const uint8_t  *bodyBuffer)
{
    decaIrqStatus_t stat;
    stat = decamutexon();

    while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);

    HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_RESET);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)headerBuffer, headerLength, HAL_MAX_DELAY);

    if (bodyLength != 0)
        HAL_SPI_Transmit(&hspi1, (uint8_t *)bodyBuffer, bodyLength, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_SET);

    decamutexoff(stat);
    return 0;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn spi_cs_low_delay()
 *
 * @brief Sets the CS low for ms delay, then raises it.
 *
 * input parameters:
 * @param delay_ms - The delay for CS to be low
 *
 * no return value
 */
uint16_t spi_cs_low_delay(uint16_t delay_ms)
{
    while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);

    __HAL_LOCK(&hspi1);
    HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_RESET);
    Sleep(delay_ms);
    HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_SET);
    __HAL_UNLOCK(&hspi1);

    return 0;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: readfromspi()
 *
 * Low level abstract function to read from the SPI.
 * Takes two separate byte buffers for write header and read data.
 * returns the offset into read buffer where first byte of read data may be found,
 * or returns -1 if there was an error
 */
int readfromspi(uint16_t  headerLength,
                uint8_t   *headerBuffer,
                uint16_t  readlength,
                uint8_t   *readBuffer)
{
    int i;

    decaIrqStatus_t stat;
    stat = decamutexon();

    while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);

    HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_RESET);

    /* Send header */
    for (i = 0; i < headerLength; i++)
    {
        HAL_SPI_Transmit(&hspi1, (uint8_t*)&headerBuffer[i], 1, HAL_MAX_DELAY);
    }

    /* Read data one byte at a time.
     * Use 8-bit access to DR so each read transfers exactly one byte.
     * 32-bit access can cause multi-byte FIFO pops on STM32L4 even in 8-bit mode,
     * resulting in every-other-byte or every-4th-byte skip patterns. */
    while (readlength-- > 0)
    {
        uint8_t dummy_tx = 0;

        /* Wait until TXE flag is set to send data */
        while (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_TXE) == RESET)
        {
        }

        *((__IO uint8_t *)&hspi1.Instance->DR) = dummy_tx;

        /* Wait until RXNE flag is set to read data */
        while (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_RXNE) == RESET)
        {
        }

        *readBuffer++ = *((__IO uint8_t *)&hspi1.Instance->DR);
    }

    HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_SET);

    decamutexoff(stat);

    return 0;
}
