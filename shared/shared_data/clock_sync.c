/*! ----------------------------------------------------------------------------
 * @file    clock_sync.c
 * @brief   Kalman + RC implementation for BD8NCF-style anchor clock sync.
 * --------------------------------------------------------------------------- */
#include "clock_sync.h"
#include <math.h>

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
}

bool clock_sync_update(AnchorClockSync *s,
                       int16_t cfo_raw,
                       uint64_t rx_local,
                       uint64_t tx_remote,
                       uint32_t dt_ms)
{
    if (!s->initialized) return false;

    /* 1) Measurement: factor_meas = 1 + cfo / divisor */
    double factor_meas = 1.0 + (double)cfo_raw / CS_CFO_TO_FACTOR_DIVISOR;

    /* 2) Predict factor with process noise proportional to dt */
    double dt_s = (dt_ms > 0 ? (double)dt_ms : 1.0) / 1000.0;
    s->factor_var += s->factor_proc_var * dt_s;

    /* 3) Kalman gain & update */
    double innovation = factor_meas - s->factor;
    if (innovation >  0.01) innovation =  0.01;   /* clamp outliers */
    if (innovation < -0.01) innovation = -0.01;

    bool outlier = (fabs(innovation) > 0.005);
    if (outlier) s->outlier_count++;
    else s->outlier_count = 0;

    double K = s->factor_var / (s->factor_var + s->factor_meas_var);
    s->factor     += K * innovation;
    s->factor_var *= (1.0 - K);

    /* 4) RC filter for offset_dtu:
     *    offset_dtu represents local_dt - remote_dt.  Given the TX/RX pair:
     *      rx_local ≈ factor * tx_remote + offset_dtu
     *      offset_dtu ≈ rx_local - factor * tx_remote
     */
    double new_offset = (double)rx_local - s->factor * (double)tx_remote;
    if (s->update_count == 0) {
        s->offset_dtu = new_offset;
    } else {
        s->offset_dtu = s->rc_alpha * new_offset
                      + (1.0 - s->rc_alpha) * s->offset_dtu;
    }

    /* 5) Bookkeeping */
    s->last_remote_tx_ts = tx_remote;
    s->last_local_rx_ts  = rx_local;
    s->last_update_ms   += dt_ms;
    s->update_count++;

    /* 6) Quality metric: higher when factor variance shrinks and outliers low */
    double q_var = 1.0 / (1.0 + s->factor_var * 1e6);
    double q_out = 1.0 - (double)s->outlier_count * 0.1;
    if (q_out < 0.0) q_out = 0.0;
    s->quality = (float)(q_var * q_out);

    return !outlier;
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