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
 *        Unlike TX timestamp which can be unreliable on some DW3210 revisions,
 *        the system time is a free-running counter at ~63.9 GHz resolution.
 *        Reads 5 bytes from SYS_TIME_ID (0x1C).
 *
 * @param  none
 *
 * @return  64-bit value of the system time.
 */
uint64_t get_sys_timestamp_u64(void)
{
    uint8_t ts_tab[5];
    uint64_t ts = 0;
    int8_t i;
    dwt_readfromdevice(SYS_TIME_ID, 0, 5, ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
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
