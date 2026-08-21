/*! ----------------------------------------------------------------------------
 * @file    clock_sync.h
 * @brief   Per-anchor clock synchronization state (BD8NCF-style Kalman + RC)
 *
 *  Each AnchorClockSync holds:
 *    - Kalman filter tracking factor = slave_local_clock / remote_clock ratio
 *    - RC low-pass filtering the offset (intercept)
 *    - Quality metric (sync_count, RSSI-derived, refreshed each update)
 *
 *  Adapted from BD8NCF's CLOCKSOURCE_SYNC_INFO (https://skytracksoft.com
 *  RTLS-2 article).  Implementation in clock_sync.c.
 *
 * @attention
 * Copyright 2026 (c) UWB project authors.
 * --------------------------------------------------------------------------- */
#ifndef CLOCK_SYNC_H
#define CLOCK_SYNC_H

#include <stdint.h>
#include <stdbool.h>

/* Default Kalman tuning --------------------------------------------------*/
#define CS_DEFAULT_FACTOR_PROC_VAR 1e-12   /* factor process variance per second */
#define CS_DEFAULT_FACTOR_MEAS_VAR 1e-6    /* factor measurement variance       */
#define CS_DEFAULT_RC_ALPHA        0.20f   /* RC smoothing factor (0..1)         */

/* DW3000 clockoffset register is 16-bit signed representing the ratio of the
 * two clocks.  Datasheet says it's relative to the configured RX/TX clock.
 * Conversion: factor = 1.0 + (double)cfo / 65536.0   (conservative estimate)
 * Some references cite 2^26; if your crystal is highly stable, use 2^26. */
#define CS_CFO_TO_FACTOR_DIVISOR  65536.0

typedef struct {
    /* Kalman state: factor (1D scalar) */
    double  factor;             /* estimated ratio (slave/remote) */
    double  factor_var;         /* current factor variance          */
    double  factor_proc_var;    /* process variance per second     */
    double  factor_meas_var;    /* measurement noise variance      */

    /* RC filter: offset_dtu = local_dt - remote_dt (in DTU)             */
    double  offset_dtu;
    double  rc_alpha;

    /* Bookkeeping */
    uint64_t last_remote_tx_ts;  /* last remote TX timestamp (40-bit extended)  */
    uint64_t last_local_rx_ts;   /* last local RX timestamp (u64)                */
    int64_t  last_update_ms;
    uint32_t update_count;
    uint32_t outlier_count;
    float    quality;            /* 0..1 */
    bool     initialized;
} AnchorClockSync;

/* Initialize with initial factor (= 1.0 nominally) and offset (= 0).
 * Pass NULL alpha/var to use defaults. */
void clock_sync_init(AnchorClockSync *s,
                     double init_factor,
                     double init_offset_dtu,
                     double rc_alpha,
                     double factor_proc_var,
                     double factor_meas_var);

/* Update with new measurement.
 *   cfo_raw   : raw value from dwt_readclockoffset() (16-bit signed)
 *   rx_local  : this device's RX timestamp at the moment the remote's
 *               TX timestamp tx_remote was received.
 *   tx_remote : the remote's TX timestamp extracted from the TimeSync payload.
 *   dt_ms     : elapsed milliseconds since previous update (for Q scaling).
 * Returns true if update was applied (not rejected as outlier). */
bool clock_sync_update(AnchorClockSync *s,
                       int16_t cfo_raw,
                       uint64_t rx_local,
                       uint64_t tx_remote,
                       uint32_t dt_ms);

/* Convert a local timestamp into the remote's "global" time frame (DTU).
 *   global = factor * local + offset_dtu
 * If the sync state is uninitialized, returns local unchanged. */
uint64_t clock_sync_to_global(const AnchorClockSync *s, uint64_t local_dt);

/* Compute the implied DTU offset between two anchors A and B from this
 * device's perspective, using their respective sync states.
 *   delta_AB = to_global(sync_B, rx_local_B) - to_global(sync_A, rx_local_A)
 * rx_local_A/B should be the same wall moment (the tag's local RX time for
 * each anchor's TimeSync packet). */
int64_t clock_sync_tdoa_dtu(const AnchorClockSync *sync_A,
                            const AnchorClockSync *sync_B,
                            uint64_t rx_local_A,
                            uint64_t rx_local_B);

#endif /* CLOCK_SYNC_H */