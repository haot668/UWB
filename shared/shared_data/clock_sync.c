/*! ----------------------------------------------------------------------------
 * @file    clock_sync.c
 * @brief   Kalman + RC implementation for BD8NCF-style anchor clock sync.
 * --------------------------------------------------------------------------- */
#include "clock_sync.h"
#include "downlink_protocol.h"
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

    s->ref_tx = 0;
    s->ref_rx = 0;

    s->rate_head    = 0;
    s->rate_samples = 0;
    s->innov_abs_avg = 0.0;
    s->innov_step_count = 0;
    s->innov_step_sign  = 0;
    s->stable_count     = 0;
    s->gate_reject_count = 0;
    s->outlier_hist = 0;
    s->last_cfo_raw = 0;
}

/* ── Robust helpers for the sample gate ─────────────────────────────────── */

/* Median of a small array (insertion sort; n <= CS_RATE_WINDOW_SAMPLES).
 * The array is sorted in place. */
static double cs_median(double *v, uint32_t n)
{
    for (uint32_t i = 1; i < n; i++) {
        double x = v[i];
        uint32_t j = i;
        while (j > 0 && v[j - 1] > x) { v[j] = v[j - 1]; j--; }
        v[j] = x;
    }
    if (n == 0) return 0.0;
    if (n & 1u) return v[n / 2];
    return 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

/* Median absolute deviation around the median — a robust sigma that a
 * single corrupt sample cannot inflate. */
static double cs_mad(const double *v, uint32_t n, double med)
{
    if (n == 0) return 0.0;
    double d[CS_RATE_WINDOW_SAMPLES];
    for (uint32_t i = 0; i < n; i++) d[i] = fabs(v[i] - med);
    return cs_median(d, n);
}

/* Measure the rate over the sliding window, INCLUDING a candidate sample
 * that has not been stored yet.
 *
 * The window is a real ring buffer of the last CS_RATE_WINDOW_SAMPLES
 * accepted samples; the rate is the quotient of the two endpoints:
 *
 *      factor = (rx_newest - rx_oldest) / (tx_newest - tx_oldest)
 *
 * With 16 samples at 80 ms the span is 1.28 s, so a 1 ns timestamp jitter
 * gives a rate uncertainty of ~0.8 ppb (the old two-endpoint sliding
 * implementation degenerated to an 80 ms span and was 16x worse).
 *
 * Returns 0.0 when the window does not hold enough samples yet. */
static double cs_rate_measure(AnchorClockSync *s,
                               uint64_t tx_remote, uint64_t rx_local)
{
    uint32_t n = s->rate_samples;
    if (n < CS_RATE_MIN_SAMPLES) return 0.0;
    if (n > (CS_RATE_WINDOW_SAMPLES - 1u)) n = CS_RATE_WINDOW_SAMPLES - 1u;

    /* The oldest sample of the window (the newest one is the candidate). */
    uint8_t oldest = (uint8_t)((s->rate_head + CS_RATE_WINDOW_SAMPLES - n)
                                % CS_RATE_WINDOW_SAMPLES);

    /* Δtx is a 40-bit modular difference; the window is ~1.3 s, far below
     * the 17.2 s wrap period, so the mask is exact. */
    uint64_t d_tx = (tx_remote - s->rate_tx[oldest]) & 0xFFFFFFFFFFULL;
    if (d_tx == 0) return 0.0;

    /* Δrx is taken on our own 64-bit extended clock — monotonic, so a plain
     * subtraction is correct.  (The remote's stamps are 40-bit and wrap
     * every 17.2 s, which is why Δtx needs the mask and Δrx does not.)
     *
     * The ratio bound below is a safety net: a caller that hands in the RAW
     * 40-bit hardware stamp would produce a Δrx of 2^40 at the wrap, and
     * that must not be allowed to enter the filter as a rate measurement. */
    int64_t d_rx = (int64_t)(rx_local - s->rate_rx[oldest]);
    if (d_rx <= 0) return 0.0;
    if ((uint64_t)d_rx > (d_tx * 4u)) return 0.0;   /* rate outside 0.25..4 */

    return (double)d_rx / (double)d_tx;
}

/* Store an accepted sample in the rate window (ring buffer). */
static void cs_rate_push(AnchorClockSync *s, uint64_t tx_remote, uint64_t rx_local)
{
    s->rate_tx[s->rate_head] = tx_remote;
    s->rate_rx[s->rate_head] = rx_local;
    s->rate_head = (uint8_t)((s->rate_head + 1u) % CS_RATE_WINDOW_SAMPLES);
    if (s->rate_samples < CS_RATE_WINDOW_SAMPLES) s->rate_samples++;
}

/* Sample-level sanity gate.
 *
 * For two static anchors the interval rate  d_rx / d_tx  is the ratio of
 * the two clocks and is therefore constant.  A frame with a corrupt
 * timestamp breaks it immediately (by microseconds), and because the rate
 * window only uses its two ENDPOINTS such a sample would poison the rate
 * for a whole window (1.28 s) — a rate error of hundreds of ppm, which
 * destroys the slot spacing.
 *
 * The test compares the CANDIDATE's interval rate with the MEDIAN of the
 * interval rates already in the window.  Both quantities come from the
 * timestamps themselves, so the gate does NOT depend on the filter state —
 * otherwise the filter could never learn the rate it is being compared
 * against (with the CFO noisy or absent, the old gate was a deadlock).
 *
 * Returns true when the candidate must be rejected. */
static bool cs_gate_check(AnchorClockSync *s,
                          uint64_t tx_remote, uint64_t rx_local)
{
    uint32_t n = s->rate_samples;
    if (n < CS_GATE_MIN_INTERVALS) return false;   /* not enough history */

    /* The window is a ring: the oldest sample sits at rate_head - n. */
    uint8_t oldest = (uint8_t)((s->rate_head + CS_RATE_WINDOW_SAMPLES - n)
                                % CS_RATE_WINDOW_SAMPLES);

    /* Interval rates of the samples already in the window. */
    double ir[CS_RATE_WINDOW_SAMPLES];
    uint32_t m = 0;
    for (uint32_t k = 0; k + 1u < n; k++) {
        uint8_t i0 = (uint8_t)((oldest + k) % CS_RATE_WINDOW_SAMPLES);
        uint8_t i1 = (uint8_t)((i0 + 1u) % CS_RATE_WINDOW_SAMPLES);
        uint64_t d_tx = (s->rate_tx[i1] - s->rate_tx[i0]) & 0xFFFFFFFFFFULL;
        int64_t  d_rx = (int64_t)(s->rate_rx[i1] - s->rate_rx[i0]);
        if (d_tx == 0 || d_rx <= 0) continue;
        if ((uint64_t)d_rx > (d_tx * 4u)) continue;  /* rate outside 0.25..4 */
        ir[m++] = (double)d_rx / (double)d_tx;
    }
    if (m < 2) return false;

    /* The candidate's interval, measured against the LAST STORED sample
     * (the interval may span more than one frame if a sample was rejected
     * before — that is fine, the rate is the same). */
    uint8_t  last = (uint8_t)((s->rate_head + CS_RATE_WINDOW_SAMPLES - 1u)
                               % CS_RATE_WINDOW_SAMPLES);
    uint64_t d_tx = (tx_remote - s->rate_tx[last]) & 0xFFFFFFFFFFULL;
    int64_t  d_rx = (int64_t)(rx_local - s->rate_rx[last]);
    if (d_tx == 0 || d_rx <= 0) return true;    /* inconsistent -> reject */
    if ((uint64_t)d_rx > (d_tx * 4u)) return true;  /* rate outside 0.25..4 */

    double med   = cs_median(ir, m);
    double sigma = 1.4826 * cs_mad(ir, m, med);
    if (sigma < 1e-9) sigma = 1e-9;             /* floor: 1 ppb */

    /* Residual of the candidate against the median rate, expressed in DTU. */
    double dev = (double)d_rx - med * (double)d_tx;
    double tol = 4.0 * sigma * (double)d_tx + (double)CS_SAMPLE_MIN_DTU;
    return (fabs(dev) > tol);
}

bool clock_sync_update(AnchorClockSync *s,
                       int16_t cfo_raw,
                       uint64_t rx_local,
                       uint64_t tx_remote,
                       uint32_t now_ms)
{
    if (!s || !s->initialized) return false;

    s->rx_total++;

    /* P1-10: the beacons arrive once per TDMA round — DL_TIMESYNC_PERIOD_MS
     * (8 slots x 6.25 ms = 50 ms).  The value only scales the process
     * noise, so it has to match the real beacon period: the hard-coded 80
     * came from the old 2-round/100 ms grid and made the filter's noise
     * band too wide for the actual cadence.
     * P2-9: use the protocol constant instead of a literal. */
    uint32_t dt_ms = DL_TIMESYNC_PERIOD_MS;
    if (s->last_update_ms != 0 && now_ms >= (uint32_t)s->last_update_ms) {
        dt_ms = now_ms - (uint32_t)s->last_update_ms;
    }
    if (dt_ms == 0) dt_ms = 1;

    /* ── 1) Sample-level sanity gate ──────────────────────────────────────
     * For two static anchors the interval residual  d_rx - factor * d_tx  is
     * the change of the path length — essentially zero.  A frame with a
     * corrupt timestamp breaks it by microseconds, and because the rate
     * window only uses its two ENDPOINTS such a sample would poison the
     * rate for a whole window (1.28 s) — a 780 ppm error, which destroys
     * the slot spacing.  The sample is therefore dropped BEFORE it can
     * enter the window. */
    bool gate_fail = cs_gate_check(s, tx_remote, rx_local);
    if (gate_fail) {
        if (s->gate_reject_count < 255) s->gate_reject_count++;
    } else {
        s->gate_reject_count = 0;
    }
    /* Safety valve: if the gate rejects everything for good, the window is
     * stale — most likely the link's rate has changed (a different
     * neighbour, a re-synchronisation).  The stale window is then DROPPED
     * so the filter re-acquires the rate from scratch instead of waiting
     * for an old sample to roll out.  The reject counter is cleared so the
     * valve closes again and the window can refill. */
    bool gate_relaxed = (s->gate_reject_count >= CS_GATE_MAX_REJECT);
    if (gate_relaxed) {
        s->rate_head         = 0;
        s->rate_samples      = 0;
        s->gate_reject_count = 0;
    }
    bool sample_ok = (!gate_fail || gate_relaxed);

    /* ── 2) Rate measurement ───────────────────────────────────────────────
     * (a) the CFO register — fast, but noisy (a few ppm).  It carries the
     *     warm-up while the timestamp window is still empty.
     * (b) the timestamp pairs — Δrx_local / Δtx_remote over the window.
     *     Accurate to ~1 ppb, which is what the grid placement needs.
     * A sample the gate rejected must not contribute to the rate. */
    double factor_cfo = 1.0 + (double)cfo_raw / CS_CFO_TO_FACTOR_DIVISOR;
    double factor_ts  = (!gate_fail) ? cs_rate_measure(s, tx_remote, rx_local) : 0.0;

    bool     from_ts  = (factor_ts > 0.0);
    double   factor_meas = from_ts ? factor_ts : factor_cfo;
    double   meas_var   = from_ts ? s->factor_meas_var_ts : s->factor_meas_var;
    double   out_thr    = from_ts ? CS_FACTOR_OUTLIER_TS : CS_FACTOR_OUTLIER_CFO;

    /* ── 3) Predict (process noise scaled by the elapsed time) ──────────── */
    double dt_s = (double)dt_ms / 1000.0;
    s->factor_var += s->factor_proc_var * dt_s;

    /* ── 4) Innovation, outlier test, Kalman update ───────────────────────
     * The old thresholds (±0.005 / 0.003) were ten times the FULL SCALE of
     * the CFO register (488 ppm = 4.9e-4): every measurement passed, the
     * outlier counter was therefore always 0 and the "quality" metric was
     * meaningless.
     *
     * P0-3: an outlier must NOT be written into the filter.  The old code
     * clamped the innovation and then applied it anyway — the counter moved
     * but the state was corrupted, so one wild frame could shift the rate
     * by the clamp (20 ppm) and poison the offset with it.
     *
     * The threshold is ADAPTIVE: the CFO is noisy (a few ppm) while the
     * timestamp rate is accurate to ~1 ppb, so a fixed 1 ppm gate would
     * throw away every timestamp measurement right after the warm-up.  The
     * robust scale of the recent innovations (EMA of |innovation|) adapts
     * the gate to the actual noise of the link. */
    double innovation = factor_meas - s->factor;      /* unclamped */
    double thr = out_thr;
    double sigma = 1.2533 * s->innov_abs_avg;         /* robust scale */
    if (4.0 * sigma > thr) thr = 4.0 * sigma;

    bool warmup = (s->update_count < CS_WARMUP_UPDATES);

    /* A large innovation that keeps repeating in the same direction is a
     * genuine rate step (the estimate was far off, e.g. right after a
     * reset), not an outlier.  Without this rule a filter that starts
     * 50 ppm off would reject every measurement and never converge.
     * The counter is maintained during the warm-up as well — otherwise the
     * step would be "forgotten" exactly when the warm-up ends. */
    bool big = (fabs(innovation) > thr);
    bool step_ok = false;
    if (big) {
        int sign = (innovation > 0.0) ? 1 : -1;
        if (sign == (int)s->innov_step_sign) {
            s->innov_step_count++;
            step_ok = (s->innov_step_count >= 2);
        } else {
            s->innov_step_sign  = (int8_t)sign;
            s->innov_step_count = 1;
        }
    } else {
        s->innov_step_count = 0;
    }

    bool outlier = (!sample_ok)
                || (!warmup && !step_ok && big);

    if (outlier) {
        /* Keep the state untouched — factor, variance and offset stay as
         * they were.  The reference timestamps are NOT advanced either: the
         * next accepted sample then spans the whole interval and the
         * rejected value never enters a difference. */
        s->outlier_hist = (s->outlier_hist << 1) | 1u;
        s->outlier_count = cs_popcount32(s->outlier_hist);
        return false;
    }

    /* Clamp the correction that is actually applied. */
    double clamp = (warmup || step_ok) ? CS_FACTOR_CLAMP_WARMUP : CS_FACTOR_CLAMP;
    double applied = innovation;
    if (applied >  clamp) applied =  clamp;
    if (applied < -clamp) applied = -clamp;

    double K = s->factor_var / (s->factor_var + meas_var);
    if (K > 1.0) K = 1.0;
    s->factor     += K * applied;
    s->factor_var *= (1.0 - K);

    /* The robust scale is fed ONLY by accepted samples — otherwise the
     * outliers would inflate their own threshold. */
    s->innov_abs_avg = 0.9 * s->innov_abs_avg + 0.1 * fabs(innovation);

    /* Track how well the filter is following: only while it tracks can the
     * sample gate be trusted (see CS_SAMPLE_MIN_DTU). */
    if (!big) {
        if (s->stable_count < 255) s->stable_count++;
    } else {
        s->stable_count = 0;
    }

    /* ── 5) Offset (RC), maintained RELATIVE to a reference sample ───────
     * The model is
     *
     *      local(remote) = ref_rx + factor * (remote - ref_tx)
     *
     * and offset_dtu is that model written absolutely
     * (offset_dtu = ref_rx - factor * ref_tx), so that the consumers'
     * formula  local = factor * ts + offset_dtu  stays valid.
     *
     * P0-6: with the ABSOLUTE form (offset = rx_local - factor * tx_remote)
     * the model error is  factor_error * tx_absolute  — the remote
     * timestamp grows without bound, so a link that has been up for
     * 10 minutes carries metres of offset error (0.2 ppb x 3.8e13 DTU =
     * 7.6 m), and the incremental update cannot repair it either (a
     * constant offset error is invisible in the differences).
     * With the reference form the factor error is only multiplied by the
     * ELAPSED time since the reference, which CS_REF_MAX_AGE_DTU bounds
     * (0.47 s -> 6 DTU at 0.2 ppb).
     *
     * The reference is moved forward - WITH continuity, so that the model
     * does not jump - whenever it has grown too old. */
    if (s->update_count == 0) {
        s->ref_tx = tx_remote;
        s->ref_rx = rx_local;
    }
    /* Δ to the reference: modular (40 bit), and always far below the wrap
     * period because the reference is refreshed every CS_REF_MAX_AGE_DTU. */
    double d_tx_ref = (double)((tx_remote - s->ref_tx) & 0xFFFFFFFFFFULL);
    double pred = (double)s->ref_rx + s->factor * d_tx_ref;
    /* P1-6: the old "if (d_tx > 8.0e12) d_tx -= 1.099511627776e12" was
     * dead code — the 40-bit mask above already handles the wrap
     * (2^40 = 1.0995e12), the mask is exact for any window. */
    s->ref_rx += s->rc_alpha * ((double)rx_local - pred);
    if (d_tx_ref > CS_REF_MAX_AGE_DTU) {
        /* Move the reference forward, keeping the model continuous. */
        s->ref_rx += s->factor * d_tx_ref;
        s->ref_tx  = tx_remote;
    }
    s->offset_dtu = (double)s->ref_rx - s->factor * (double)s->ref_tx;

    /* ── 6) Bookkeeping ──────────────────────────────────────────────────── */
    s->last_remote_tx_ts = tx_remote;
    s->last_local_rx_ts  = rx_local;
    s->last_update_ms    = (int64_t)now_ms;
    s->update_count++;
    s->last_cfo_raw      = cfo_raw;

    /* The accepted sample enters the rate window — but a sample the gate
     * rejected never does (that is what keeps a corrupt timestamp from
     * poisoning the window's endpoints).  When the safety valve opened the
     * window was reset, so the sample starts the new one. */
    if (!gate_fail || gate_relaxed) cs_rate_push(s, tx_remote, rx_local);

    /* Outlier history: one bit per update, newest in the LSB.  The old code
     * reset the counter to 0 on every good sample, so "outlier_count" was
     * really "were the last two samples outliers". */
    s->outlier_hist = (s->outlier_hist << 1);
    s->outlier_count = cs_popcount32(s->outlier_hist);
    s->rx_ok++;

    /* ── 7) Quality metric ────────────────────────────────────────────────
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

    return true;
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
