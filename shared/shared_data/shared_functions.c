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

/*============================================================================
 * DS-TWR compute (Decawave official formula)
 *
 *   Tr1 = T_resp_rx_tag    - T_poll_tx_tag    (tag clock)
 *   Tre = T_resp_tx_anchor - T_poll_rx_anchor  (anchor clock)
 *   Tr2 = T_final_rx_anchor - T_resp_tx_anchor (anchor clock)
 *   Tfe = T_final_tx_tag   - T_resp_rx_tag    (tag clock)
 *
 *   TOF_dtu = (Tr1*Tr2 - Tre*Tfe) / (Tr1 + Tr2 + Tre + Tfe)
 *
 *   Tr1 and Tfe are in TAG's clock; Tr2 and Tre are in ANCHOR's clock.
 *   The numerator/denominator product with both clocks cancels the
 *   first-order clock-rate error.  Result is in DW3210 time units.
 *
 *   distance_mm = TOF_dtu × 4.690782 mm/DTU
 *                 (1 DTU at 63.8976 GHz = c*1000/63.8976e9 mm)
 *
 *   Returns 0 on success, -1 if any of the round-trip / reply intervals is
 *   zero or the denominator is zero (degenerate measurement).
 *============================================================================*/
int dstwr_compute(const dst_twr_timestamps_t *ts,
                  int32_t *distance_mm, uint32_t *tof_dtu)
{
    /* Sanity: any zero timestamp means we didn't RX all required frames. */
    if (ts->T_poll_rx == 0 || ts->T_resp_tx == 0 ||
        ts->T_resp_rx == 0 || ts->T_poll_tx == 0 ||
        ts->T_final_rx == 0 || ts->T_final_tx == 0)
        return -1;

    /* Cast each delta to int64_t (signed) — intervals are small (~ms), but
     * wraparound at 17.2 s system rollover must not produce silent underflow. */
    int64_t Tr1 = (int64_t)(ts->T_resp_rx - ts->T_poll_tx);   /* tag-clock */
    int64_t Tre = (int64_t)(ts->T_resp_tx - ts->T_poll_rx);   /* anchor-clock */
    int64_t Tr2 = (int64_t)(ts->T_final_rx - ts->T_resp_tx);  /* anchor-clock */
    int64_t Tfe = (int64_t)(ts->T_final_tx - ts->T_resp_rx);  /* tag-clock */

    /* Guard against wraparound (timestamps are 40-bit, 17.2s rollover):
     * any negative delta > 8s indicates wraparound happened during this
     * cycle — reject measurement rather than output garbage. */
    if (Tr1 < 0) Tr1 += (int64_t)1 << 40;
    if (Tre < 0) Tre += (int64_t)1 << 40;
    if (Tr2 < 0) Tr2 += (int64_t)1 << 40;
    if (Tfe < 0) Tfe += (int64_t)1 << 40;

    int64_t num = Tr1 * Tr2 - Tre * Tfe;
    int64_t den = Tr1 + Tr2 + Tre + Tfe;
    if (den <= 0) return -1;

    /* Round to nearest DTU (avoid systematic bias from integer truncation). */
    int64_t tof = (num + (den >> 1)) / den;
    if (tof < 0) tof = 0;        /* can't be negative — clamp */

    /* Convert DTU → mm: 1 DTU = c × 1000 / 63.8976e9 mm ≈ 4.690782 mm. */
    const uint32_t MM_PER_DTU_Q8 = 1201;   /* 4.690782 × 256 ≈ 1200.84 */
    uint32_t d_mm = (uint32_t)((uint64_t)tof * MM_PER_DTU_Q8 >> 8);

    *tof_dtu      = (uint32_t)tof;
    *distance_mm  = (int32_t)d_mm;
    return 0;
}
