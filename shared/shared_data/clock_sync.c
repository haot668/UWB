/*! ----------------------------------------------------------------------------
 * @file    clock_sync.c
 * @brief   Kalman + RC implementation for BD8NCF-style anchor clock sync.
 * --------------------------------------------------------------------------- */
#include "clock_sync.h"
#include <math.h>

/* Population count of a 32-bit word (the outlier history). */
static uint32_t cs_popcount32(uint32_t v)
{
    uint32_t n = 0;
    while (v) { n += (v & 1u); v >>= 1; }
    return n;
}

void clock_sync_init(AnchorClockSync *s,
                     double init_factor,
                     double init_offset_dtu,
                     double rc_alpha,
                     double factor_proc_var,
                     double factor_meas_var)
{
    s->factor          = (init_factor > 0.0) ? init_factor : 1.0;
    s->factor_var      = 1e-6;
    s->factor_proc_var = (factor_proc_var > 0.0) ? factor_proc_var
                                                  : CS_DEFAULT_FACTOR_PROC_VAR;
    s->factor_meas_var = (factor_meas_var > 0.0) ? factor_meas_var
                                                  : CS_DEFAULT_FACTOR_MEAS_VAR;
    s->factor_meas_var_ts = CS_DEFAULT_FACTOR_MEAS_VAR_TS;
    s->offset_dtu      = init_offset_dtu;
    s->rc_alpha        = (rc_alpha > 0.0 && rc_alpha <= 1.0)
                            ? rc_alpha : CS_DEFAULT_RC_ALPHA;

    s->last_remote_tx_ts = 0;
    s->last_local_rx_ts  = 0;
    s->last_update_ms    = 0;
    s->update_count      = 0;
    s->outlier_count     = 0;
    s->quality           = 0.0f;
    s->initialized       = true;
    s->rx_total         = 0;
    s->rx_ok            = 0;
    s->freeze_count     = 0;
    s->dly_fail_count   = 0;

    s->rate_tx[0] = s->rate_tx[1] = 0;
    s->rate_rx[0] = s->rate_rx[1] = 0;
    s->rate_fill    = 0;
    s->rate_samples = 0;
    s->outlier_hist = 0;
    s->last_cfo_raw = 0;
}

/* Feed one (tx_remote, rx_local) pair into the sliding rate window.
 * Returns the rate estimate when the window holds enough samples, else 0. */
static double cs_rate_from_timestamps(AnchorClockSync *s,
                                      uint64_t tx_remote, uint64_t rx_local)
{
    if (s->rate_fill == 0) {
        s->rate_tx[0] = tx_remote;
        s->rate_rx[0] = rx_local;
        s->rate_fill    = 1;
        s->rate_samples = 1;
        return 0.0;
    }

    if (s->rate_samples < CS_RATE_WINDOW_SAMPLES) {
        /* still filling: keep the oldest endpoint, move the newest one */
        s->rate_tx[1] = tx_remote;
        s->rate_rx[1] = rx_local;
        s->rate_fill    = 2;
        s->rate_samples++;
    } else {
        /* window full: slide it by one beacon */
        s->rate_tx[0] = s->rate_tx[1];
        s->rate_rx[0] = s->rate_rx[1];
        s->rate_tx[1] = tx_remote;
        s->rate_rx[1] = rx_local;
    }

    if (s->rate_fill < 2 || s->rate_samples < CS_RATE_MIN_SAMPLES) return 0.0;

    /* Δtx is a 40-bit modular difference; the window is ~1.3 s, far below
     * the 17.2 s wrap period, so the mask is exact. */
    uint64_t d_tx = (s->rate_tx[1] - s->rate_tx[0]) & 0xFFFFFFFFFFULL;
    if (d_tx == 0) return 0.0;

    /* Δrx is taken on our own 64-bit extended clock — monotonic, so a plain
     * subtraction is correct. */
    int64_t d_rx = (int64_t)(s->rate_rx[1] - s->rate_rx[0]);
    if (d_rx <= 0) return 0.0;

    return (double)d_rx / (double)d_tx;
}

bool clock_sync_update(AnchorClockSync *s,
                       int16_t cfo_raw,
                       uint64_t rx_local,
                       uint64_t tx_remote,
                       uint32_t now_ms)
{
    if (!s || !s->initialized) return false;

    s->rx_total++;

    uint32_t dt_ms = 100;
    if (s->last_update_ms != 0 && now_ms >= (uint32_t)s->last_update_ms) {
        dt_ms = now_ms - (uint32_t)s->last_update_ms;
    }
    if (dt_ms == 0) dt_ms = 1;

    /* ── 1) Rate measurement ───────────────────────────────────────────────
     * (a) the CFO register — fast, but noisy (a few ppm).  It carries the
     *     warm-up while the timestamp window is still empty.
     * (b) the timestamp pairs — Δrx_local / Δtx_remote over the window.
     *     Accurate to ~1 ppb, which is what the grid placement needs. */
    double factor_cfo = 1.0 + (double)cfo_raw / CS_CFO_TO_FACTOR_DIVISOR;
    double factor_ts  = cs_rate_from_timestamps(s, tx_remote, rx_local);

    bool     from_ts  = (factor_ts > 0.0);
    double   factor_meas = from_ts ? factor_ts : factor_cfo;
    double   meas_var   = from_ts ? s->factor_meas_var_ts : s->factor_meas_var;
    double   out_thr    = from_ts ? CS_FACTOR_OUTLIER_TS : CS_FACTOR_OUTLIER_CFO;

    /* ── 2) Predict (process noise scaled by the elapsed time) ──────────── */
    double dt_s = (double)dt_ms / 1000.0;
    s->factor_var += s->factor_proc_var * dt_s;

    /* ── 3) Innovation, outlier test, Kalman update ───────────────────────
     * The old thresholds (±0.005 / 0.003) were ten times the FULL SCALE of
     * the CFO register (488 ppm = 4.9e-4): every measurement passed, the
     * outlier counter was therefore always 0 and the "quality" metric was
     * meaningless. */
    double innovation = factor_meas - s->factor;
    if (innovation >  CS_FACTOR_CLAMP) innovation =  CS_FACTOR_CLAMP;
    if (innovation < -CS_FACTOR_CLAMP) innovation = -CS_FACTOR_CLAMP;

    bool outlier = (fabs(innovation) > out_thr);

    double K = s->factor_var / (s->factor_var + meas_var);
    if (K > 1.0) K = 1.0;
    s->factor     += K * innovation;
    s->factor_var *= (1.0 - K);

    /* ── 4) Offset (RC), maintained on DIFFERENCES ────────────────────────
     * offset_dtu represents local_dt - remote_dt.  Updating it with the
     * inter-sample differences keeps the factor error from being multiplied
     * by the absolute timestamp (see the header for the numbers). */
    if (s->update_count == 0) {
        s->offset_dtu = (double)rx_local - s->factor * (double)tx_remote;
    } else {
        double d_rx = (double)(rx_local - s->last_local_rx_ts);
        double d_tx = (double)(int64_t)((tx_remote - s->last_remote_tx_ts)
                                        & 0xFFFFFFFFFFULL);
        if (d_tx > 8.0e12) d_tx -= 1.099511627776e12;   /* 2^40 wrap */
        s->offset_dtu += s->rc_alpha * (d_rx - s->factor * d_tx);
    }

    /* ── 5) Bookkeeping ──────────────────────────────────────────────────── */
    s->last_remote_tx_ts = tx_remote;
    s->last_local_rx_ts  = rx_local;
    s->last_update_ms    = (int64_t)now_ms;
    s->update_count++;
    s->last_cfo_raw      = cfo_raw;

    /* Outlier history: one bit per update, MSB = newest.  The old code
     * reset the counter to 0 on every good sample, so "outlier_count" was
     * really "were the last two samples outliers". */
    s->outlier_hist = (s->outlier_hist << 1) | (outlier ? 1u : 0u);
    s->outlier_count = cs_popcount32(s->outlier_hist);
    if (!outlier) s->rx_ok++;

    /* ── 6) Quality metric ────────────────────────────────────────────────
     *  q1: how well the rate is known (sigma_factor in ppm)
     *  q2: the outlier ratio of the last 32 updates
     *  q3: the update count (the filter needs a few samples to settle) */
    double sigma_ppm = sqrt(s->factor_var) * 1.0e6;
    double q1 = 1.0 - sigma_ppm / 5.0;
    if (q1 < 0.0) q1 = 0.0;
    if (q1 > 1.0) q1 = 1.0;
    double q2 = 1.0 - ((double)s->outlier_count / 16.0);
    if (q2 < 0.0) q2 = 0.0;
    double q3 = (s->update_count < 10) ? ((double)s->update_count / 10.0) : 1.0;
    s->quality = (float)(q1 * q2 * q3);

    return !outlier;
}

void clock_sync_note_tx_event(AnchorClockSync *s,
                              uint8_t freeze,
                              uint8_t dly_fail)
{
    if (!s) return;
    if (freeze)    s->freeze_count++;
    if (dly_fail)  s->dly_fail_count++;
}

uint64_t clock_sync_to_global(const AnchorClockSync *s, uint64_t local_dt)
{
    if (!s->initialized) return local_dt;
    double g = s->factor * (double)local_dt + s->offset_dtu;
    if (g < 0.0) g = 0.0;
    if (g > (double)UINT64_MAX) g = (double)UINT64_MAX;
    return (uint64_t)g;
}

int64_t clock_sync_tdoa_dtu(const AnchorClockSync *sync_A,
                            const AnchorClockSync *sync_B,
                            uint64_t rx_local_A,
                            uint64_t rx_local_B)
{
    uint64_t g_A = clock_sync_to_global(sync_A, rx_local_A);
    uint64_t g_B = clock_sync_to_global(sync_B, rx_local_B);
    return (int64_t)g_B - (int64_t)g_A;
}
