/*! ----------------------------------------------------------------------------
 * @file    clock_sync.h
 * @brief   Per-anchor clock synchronization state (rate + offset filter)
 *
 *  Each AnchorClockSync holds:
 *    - A Kalman filter tracking the RATE ratio between this device's clock
 *      and the remote's clock (factor = local_clock / remote_clock).
 *    - An RC low-pass filter on the OFFSET (the intercept).
 *    - A quality metric and RX statistics.
 *
 *  ── Why the rate is measured from timestamp pairs ──────────────────────────
 *  The DW3000 CFO register (CIA_DIAG_0[31:16]) is a 13-bit signed value in
 *  s[-15:-26] fixed-point format:
 *        factor = 1.0 + (cfo_raw / 2^26)
 *  Divisor is 2^26 = 67,108,864 (NOT 2^16).  The CFO is a *frequency*
 *  estimate with a noise of a few ppm — good enough to converge quickly,
 *  but not accurate enough to place a beacon on a grid: at slot 7 (35 ms)
 *  a 10 ppm error displaces the beacon by 350 ns ≈ 10 cm.
 *
 *  The far better rate source is the timestamp pair itself:
 *        factor = Δrx_local / Δtx_remote
 *  The RX timestamp is hardware-latched (jitter ~1 ns) and the TX timestamp
 *  is exact, so over a window of ~1.3 s the rate is known to ~1 ppb — three
 *  orders of magnitude better than the CFO, and it needs no calibration.
 *  The CFO is kept for the warm-up (the window is empty right after a
 *  reset) and as a cross-check.
 *
 *  ── Why the offset is maintained incrementally ────────────────────────────
 *  The textbook form  offset = rx_local - factor * tx_remote  multiplies the
 *  factor error by the ABSOLUTE timestamp, which grows without bound (the
 *  40-bit stamp reaches 1e12 DTU after ~4 h).  A 1 ppm error at that point
 *  is 1e6 DTU = 15 µs = 4.7 m.  The offset is therefore updated with the
 *  DIFFERENCES between consecutive samples, so the factor error only ever
 *  acts on one inter-frame interval.
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
/* Process variance per second: the crystal drifts ~0.1 ppm/s with
 * temperature, i.e. (1e-7)^2 = 1e-14 .. 1e-13. */
#define CS_DEFAULT_FACTOR_PROC_VAR 1e-13
/* CFO measurement variance: the CFO noise is ~2 ppm = (2e-6)^2 = 4e-12. */
#define CS_DEFAULT_FACTOR_MEAS_VAR 4e-12
/* Timestamp-pair measurement variance: ~0.1 ppb of rate uncertainty plus
 * the channel-delay wander over the window → 1e-13 is a safe bound. */
#define CS_DEFAULT_FACTOR_MEAS_VAR_TS 1e-13
#define CS_DEFAULT_RC_ALPHA        0.30f

/* Rate window (number of beacons).  16 beacons = 16 x 80 ms = 1.28 s. */
#define CS_RATE_WINDOW_SAMPLES   16u
/* Minimum number of samples before the timestamp-pair rate is trusted.
 * 8 beacons = 0.64 s -> the rate is known to ~1.5 ppb, which already puts
 * the slot placement error (80 ms x 1.5 ppb = 0.12 ns = 3.6 cm) well below
 * the 20 cm target.  The CFO (a few ppm) covers the warm-up before that. */
#define CS_RATE_MIN_SAMPLES      8u

/* Outlier / clamp thresholds on the rate innovation (dimensionless factor
 * units; 1e-6 = 1 ppm). */
#define CS_FACTOR_OUTLIER_CFO    5e-6    /* 5 ppm  */
#define CS_FACTOR_OUTLIER_TS     1e-6    /* 1 ppm  */
/* Maximum rate correction per update once the filter has settled.  2 ppm is
 * 0.16 ns over one 80 ms interval — small enough that even a mis-classified
 * sample cannot move the grid, while a genuine 20 ppm crystal still needs
 * only a few updates to be tracked. */
#define CS_FACTOR_CLAMP          2e-6    /* 2 ppm  */
/* While the filter is still settling (first CS_WARMUP_UPDATES updates) the
 * clamp is loose, otherwise the factor could not reach its true value. */
#define CS_FACTOR_CLAMP_WARMUP  50e-6    /* 50 ppm */
#define CS_WARMUP_UPDATES        8u
/* A factor correction larger than this re-anchors the absolute offset
 * (1 ppb — the incremental offset update cannot follow a factor change). */
#define CS_FACTOR_REANCHOR      1e-9

/* Sample-level sanity gate (DTU).  For two static anchors the interval
 * residual  d_rx - factor * d_tx  is the change of the path length, i.e.
 * essentially zero.  A frame with a corrupt timestamp breaks it by
 * microseconds — and because the rate window only uses its two ENDPOINTS,
 * such a sample would poison the rate for a whole window.
 *
 * IMPORTANT: the gate must NOT compare against the filter's own factor —
 * that is a circular dependency (the factor is learned from the window, the
 * window is fed by the gate): with a far-off estimate the gate would reject
 * every sample and the filter could never converge (the CFO was the only
 * way out, and the CFO is noisy).  The gate therefore compares the
 * candidate's INTERVAL RATE against the median of the interval rates
 * already in the window — both come from the timestamps themselves, so the
 * test is independent of the filter state. */
#define CS_SAMPLE_MIN_DTU        20000u  /* 313 ns */
/* Number of samples in the window before the gate can work (it needs two
 * interval rates to form a median). */
#define CS_GATE_MIN_INTERVALS    3u
/* After this many consecutive gate rejections the window is considered
 * stale (the link's rate has changed for good) and is dropped, so the
 * filter re-acquires the rate from scratch. */
#define CS_GATE_MAX_REJECT       8u

/* DW3000 CIA clock offset register (CIA_DIAG_0[31:16] COE_PPM) is a 13-bit
 * signed value in s[-15:-26] fixed-point format, representing the ratio of
 * the two clocks.  Per DW3000 User Manual Page 184:
 *   factor = 1.0 + (cfo_raw / 2^26)
 * Divisor is 2^26 = 67,108,864, NOT 2^16.
 * The old 2^16 value was an incorrect estimate. */
#define CS_CFO_TO_FACTOR_DIVISOR  67108864.0

typedef struct {
    /* Kalman state: factor (1D scalar) */
    double  factor;             /* estimated ratio (local/remote)          */
    double  factor_var;         /* current factor variance                 */
    double  factor_proc_var;    /* process variance per second             */
    double  factor_meas_var;    /* CFO measurement noise variance          */
    double  factor_meas_var_ts; /* timestamp-pair measurement noise var.   */

    /* RC filter: offset_dtu = local_dt - remote_dt (in DTU)             */
    double  offset_dtu;
    double  rc_alpha;

    /* Bookkeeping */
    uint64_t last_remote_tx_ts;  /* last remote TX timestamp (40-bit)       */
    uint64_t last_local_rx_ts;   /* last local RX timestamp (u64)            */
    int64_t  last_update_ms;     /* HAL tick of the last update             */
    uint32_t update_count;
    uint32_t outlier_count;      /* outliers in the last 32 updates         */
    float    quality;            /* 0..1 */
    bool     initialized;
    uint16_t neighbor_rank;     /* rank of this neighbor (Anchor-specific) */

    /* Rate estimation from timestamp pairs — a REAL sliding window over the
     * last CS_RATE_WINDOW_SAMPLES beacons (16 x 80 ms = 1.28 s).  The old
     * code kept only the two endpoints and slid them one step at a time,
     * so once the window was "full" it degenerated to a two-sample
     * difference (80 ms) — 16x worse than the intended 1 ppb, i.e. a slot
     * placement error of ~30 cm. */
    uint64_t rate_tx[CS_RATE_WINDOW_SAMPLES];  /* remote TX stamps         */
    uint64_t rate_rx[CS_RATE_WINDOW_SAMPLES];  /* local RX stamps          */
    uint8_t  rate_head;         /* index of the next write                 */
    uint32_t rate_samples;      /* samples currently in the window         */

    /* Robust scale of the rate innovation: an exponential average of the
     * innovation magnitude.  Used to widen the outlier threshold when the
     * link is noisy (and to narrow it when it is clean). */
    double   innov_abs_avg;
    /* Counter for a PERSISTENT large innovation: a step that repeats in the
     * same direction is a genuine rate change, not an outlier. */
    uint8_t  innov_step_count;
    int8_t   innov_step_sign;
    /* Consecutive accepted updates with a small innovation.  The sample
     * gate (see CS_SAMPLE_MIN_DTU) is only trustworthy while the filter is
     * actually tracking — with a far-off estimate it would reject every
     * sample and the filter could never converge. */
    uint8_t  stable_count;
    /* Consecutive samples the sanity gate rejected — a safety valve so the
     * filter can never be locked out completely. */
    uint8_t  gate_reject_count;

    /* Outlier history: one bit per update, MSB = newest (32 updates)      */
    uint32_t outlier_hist;
    int16_t  last_cfo_raw;       /* last accepted CFO value                 */

    /* Statistics for SYNC_METRICS output */
    uint32_t rx_total;          /* total RX attempts (OK + ERR) */
    uint32_t rx_ok;             /* successful RX frames */
    uint32_t freeze_count;      /* GTX freeze events */
    uint32_t dly_fail_count;   /* delayed TX failures */
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
 *   now_ms    : current monotonic tick (HAL_GetTick()) — used for the
 *               process-noise scaling and for the "last update" stamp.
 * Returns true if the update was applied (not rejected as outlier). */
bool clock_sync_update(AnchorClockSync *s,
                       int16_t cfo_raw,
                       uint64_t rx_local,
                       uint64_t tx_remote,
                       uint32_t now_ms);

/* Report a TX-side event (GTX freeze / delayed-TX failure) of the link this
 * sync state belongs to. */
void clock_sync_note_tx_event(AnchorClockSync *s,
                              uint8_t freeze,
                              uint8_t dly_fail);

/* Convert a local timestamp into the remote's "global" time frame (DTU).
 *   global = factor * local + offset_dtu
 * If the sync state is uninitialized, returns local unchanged.
 *
 * NOTE: this helper is NOT part of the positioning path.  The TDOA solve
 * uses the grid/slot approach (anchors are placed on the Root's grid by the
 * DS-TWR loop, and the Tag compensates the slot spacing with the rate
 * ratio it measures towards the Root).  This form multiplies the factor
 * error with the absolute timestamp and is therefore only good enough for
 * diagnostics. */
uint64_t clock_sync_to_global(const AnchorClockSync *s, uint64_t local_dt);

/* Compute the implied DTU offset between two anchors A and B from this
 * device's perspective, using their respective sync states.
 *   delta_AB = to_global(sync_B, rx_local_B) - to_global(sync_A, rx_local_A)
 * rx_local_A/B should be the same wall moment (the tag's local RX time for
 * each anchor's TimeSync packet).
 * Diagnostic only — see the note on clock_sync_to_global(). */
int64_t clock_sync_tdoa_dtu(const AnchorClockSync *sync_A,
                            const AnchorClockSync *sync_B,
                            uint64_t rx_local_A,
                            uint64_t rx_local_B);

#endif /* CLOCK_SYNC_H */
