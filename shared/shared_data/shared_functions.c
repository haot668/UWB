/*! ----------------------------------------------------------------------------
 * @file    shared_functions.c
 * @brief   DW3000 timestamp read/write helpers
 *
 * @attention
 * Copyright 2013-2020 (c) Decawave Ltd, Dublin, Ireland.
 * All rights reserved.
 */

#include <deca_types.h>
#include <deca_regs.h>
#include <deca_device_api.h>
#include <shared_defines.h>
#include <shared_functions.h>

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
uint64_t get_tx_timestamp_u64(void)
{
    uint8_t ts_tab[5];
    uint64_t ts = 0;
    int8_t i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
uint64_t get_rx_timestamp_u64(void)
{
    uint8_t ts_tab[5];
    uint64_t ts = 0;
    int8_t i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_sys_timestamp_u64()
 *
 * @brief Read the DW3000 40-bit system time counter (always monotonic).
 *        The SYS_TIME register (0x1C) is 4 bytes (32-bit) storing bits[31:1].
 *        actual_40bit_time = register_value << 1.
 *
 *        NOTE: If the DW3000 is in sleep/deep-sleep mode, SYS_TIME may be frozen
 *        and not incrementing. In that case, use get_tx_timestamp_u64() or
 *        get_rx_timestamp_u64() as alternative time references.
 *
 * @param  none
 *
 * @return  64-bit value of the system time (upper 40 bits of 64-bit container).
 */
uint64_t get_sys_timestamp_u64(void)
{
    /* SYS_TIME_ID (0x1C) is 4 bytes storing bits[39:8] of the 40-bit counter.
     * Reading as 32-bit and shifting left 8 gives the correct 40-bit time. */
    return ((uint64_t)dwt_read32bitreg(SYS_TIME_ID)) << 8;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn pack_40bit_ts()
 *
 * @brief Pack a 40-bit DW3000 timestamp into 5 bytes (little-endian, LSB first).
 *
 * @param  buf  pointer to 5-byte output buffer
 *         ts   40-bit timestamp value
 *
 * @return none
 */
void pack_40bit_ts(uint8_t *buf, uint64_t ts)
{
    buf[0] = (uint8_t)(ts & 0xFF);
    buf[1] = (uint8_t)((ts >> 8) & 0xFF);
    buf[2] = (uint8_t)((ts >> 16) & 0xFF);
    buf[3] = (uint8_t)((ts >> 24) & 0xFF);
    buf[4] = (uint8_t)((ts >> 32) & 0xFF);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn unpack_40bit_ts()
 *
 * @brief Unpack a 40-bit DW3000 timestamp from 5 bytes (little-endian, LSB first).
 *
 * @param  buf  pointer to 5-byte input buffer
 *
 * @return  64-bit timestamp value
 */
uint64_t unpack_40bit_ts(const uint8_t *buf)
{
    return (uint64_t)buf[0]
         | ((uint64_t)buf[1] << 8)
         | ((uint64_t)buf[2] << 16)
         | ((uint64_t)buf[3] << 24)
         | ((uint64_t)buf[4] << 32);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn pack_64bit_ts()
 *
 * @brief Pack a 64-bit extended timestamp into 8 bytes (little-endian, LSB first).
 *        Used for cross-anchor timestamp comparison to avoid 40-bit wrap ambiguity.
 *
 * @param  buf  pointer to 8-byte output buffer
 *         ts   64-bit extended timestamp
 *
 * @return none
 */
void pack_64bit_ts(uint8_t *buf, uint64_t ts)
{
    buf[0] = (uint8_t)(ts & 0xFF);
    buf[1] = (uint8_t)((ts >> 8) & 0xFF);
    buf[2] = (uint8_t)((ts >> 16) & 0xFF);
    buf[3] = (uint8_t)((ts >> 24) & 0xFF);
    buf[4] = (uint8_t)((ts >> 32) & 0xFF);
    buf[5] = (uint8_t)((ts >> 40) & 0xFF);
    buf[6] = (uint8_t)((ts >> 48) & 0xFF);
    buf[7] = (uint8_t)((ts >> 56) & 0xFF);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn unpack_64bit_ts()
 *
 * @brief Unpack a 64-bit extended timestamp from 8 bytes (little-endian, LSB first).
 *
 * @param  buf  pointer to 8-byte input buffer
 *
 * @return  64-bit extended timestamp
 */
uint64_t unpack_64bit_ts(const uint8_t *buf)
{
    return (uint64_t)buf[0]
         | ((uint64_t)buf[1] << 8)
         | ((uint64_t)buf[2] << 16)
         | ((uint64_t)buf[3] << 24)
         | ((uint64_t)buf[4] << 32)
         | ((uint64_t)buf[5] << 40)
         | ((uint64_t)buf[6] << 48)
         | ((uint64_t)buf[7] << 56);
}
