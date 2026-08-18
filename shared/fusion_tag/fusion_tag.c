/*! ----------------------------------------------------------------------------
 *  @file    fusion_tag.c
 *  @brief   Downlink TDOA Tag — receives anchor beacons + blinks, solves position
 *
 *           Each cycle:
 *             1. RX Downlink Beacon (0xEE) → anchor position table + timing ref
 *             2. RX Downlink Blinks  (0xEF) → TDOA timestamps (one per slave)
 *             3. tdoa_solve() → Gauss-Newton multilateration (local FPU)
 *             4. UART output: P,SEQ,X=<m>,Y=<m>,Z=<m>,Q=<rms m>
 *             5. TX Position Report (0xE3) → gateway for +ONLINE/+POS
 *
 *           Requires: FUSION_DOWNLINK_MODE defined in example_selection.h
 *                     tdoa_solver.c compiled into project
 *                     ≥3 anchors with DL_A<n>_X/Y/Z_CM positions configured
 *
 * @attention
 * Copyright 2015-2020 (c) Decawave Ltd, Dublin, Ireland.
 * All rights reserved.
 */
#include <deca_device_api.h>
#include <math.h>
#include <deca_regs.h>
#include <deca_spi.h>
#include <port.h>
#include <string.h>
#include <shared_defines.h>
#include <shared_functions.h>
#include <example_selection.h>
#include "downlink_protocol.h"
#include "gateway_protocol.h"
#include "firmware_version.h"
#include "../gw_ota/gw_ota.h"

#if defined(TEST_FUSION_TAG)

extern void test_run_info(unsigned char *data);
#if !TAG_DEBUG_OUTPUT
extern UART_HandleTypeDef huart2;
#endif

#define APP_NAME "TAG V0.3"

/* Tag serial output mode:
 *   1 = Debug:   human-readable text  (P,X=...,Y=...,Z=...,Q=...)
 *   0 = Release: compact HEX binary   (protocol frame, 14 bytes/cycle)
 * Change to 0 for production use — protocol spec TBD. */
#define TAG_DEBUG_OUTPUT  1

/* Positioning diagnostics:
 *   1 = print anchor table (once) + compensated TDOA (every cycle)
 *   0 = off
 * Used to trace "tag outputs wrong coordinates" — reveals whether the
 * anchor coordinates in the beacon match the physical layout and whether
 * the TDOA measurements are small (near anchors) or large (far/wrong). */
#define TAG_DIAG  1

/* Default communication configuration — Channel 5, 6.8 Mbps, NO STS */
static dwt_config_t config = {
    UWB_CHANNEL,     /* Channel number (5 or 9). */
        DWT_PLEN_128,    /* Preamble length. */
    DWT_PAC8,        /* Preamble acquisition chunk size. */
    9,               /* TX preamble code. */
    9,               /* RX preamble code. */
    1,               /* 0 = standard 8 symbol SFD, 1 = non-standard 8 symbol */
    DWT_BR_6M8,      /* Data rate. */
        DWT_PHRMODE_STD, /* PHY header mode. */
    DWT_PHRRATE_STD, /* PHY header rate. */
    (129 + 8 - 8),   /* SFD timeout */
    DWT_STS_MODE_OFF,
    DWT_STS_LEN_64,
    DWT_PDOA_M0
};

#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define DL_TAG_ID  0x0010u  /* Tags start at 0x10, anchors use 0x00-0x0F */

/* Increase RX buffer — beacon can be up to DL_BEACON_LEN(16)=126 bytes */
#define RX_BUF_LEN  (DL_BEACON_LEN(DL_MAX_ANCHORS) + 4)

/* DW3210 DLY_RS calibration — AUTO-CALIBRATING per-slave.
 * Each DW3210 chip has slightly different hardware TX delay (±3m variation)
 * and the delay drifts with temperature / power-cycle.
 *
 * Auto-calibration: accumulates raw slot-compensated bias (T, value) over
 * CALIB_SAMPLES cycles at startup, then freezes the per-slave DLY_RS.
 * This removes power-on variation without absorbing the true TDOA signal.
 *
 * Defaults are used during the initial calibration window (~10s at 10 Hz). */
#define CALIB_SAMPLES      10      /* cycles to average for calibration   */
#define CALIB_OUTLIER_M    20      /* reject samples > ±20m from running avg */
#define CALIB_MAX_CYCLES   50      /* give up calibration after N total cycles */
#define CALIB_ABS_MAX_DTU  200000  /* reject raw_bias beyond ±~940m (slot mis-sync) */

/* CFO compensation toggle: 1 = subtract the tag↔slave clock-frequency-offset
 * slot-timing error from raw_bias before calibration/compensation.  The DW3000
 * reports CFO via dwt_readclockoffset() (÷2^26 = fractional offset, positive =
 * tag RX clock faster than slave TX).  Over the rank·3ms slot this produces a
 * rank-dependent bias ≈0.9 m/ppm per rank step that a one-shot DLY_RS
 * calibration can only absorb while the crystals are at a stable temperature. */
#define CFO_COMP_ENABLE  0

/* Runtime DLY_RS state — per-slave arrays (frozen after calibration) */
#define DLY_RS_DEFAULT  15800   /* ~74m initial guess for all slaves */
static int64_t dly_rs[DL_MAX_ANCHORS];      /* calibrated DLY_RS per slave */
static int64_t calib_sum[DL_MAX_ANCHORS];   /* accumulated raw_bias per slave */
static int32_t calib_cnt[DL_MAX_ANCHORS];   /* accepted samples per slave */
static int32_t calib_total  = 0;            /* total cycles attempted */
static int     calib_done   = 0;            /* 1 = calibration frozen */

static uint8_t rx_buffer[RX_BUF_LEN];

/* Downlink anchor table + TOA buffer */
static dl_anchor_entry_t anchor_table[DL_MAX_ANCHORS];
static uint64_t toa_dtu[DL_MAX_ANCHORS];
static int64_t  baseline_dtu[DL_MAX_ANCHORS];  /* master-to-anchor[k] distance in DTU */
static uint8_t  num_anchors = 0;
static uint8_t  frame_seq   = 0;

/* Slot-synchronized listening state */
static uint32_t g_next_bcn_tick = 0;   /* portGetTickCnt() of next expected beacon */
static int      g_slot_synced  = 0;    /* 1 = have valid TDMA phase sync            */

#define BCN_TIMEOUT_MS  650u   /* Unsynced beacon search timeout                   */
#define BCN_SYNC_WIN_MS 10u    /* Synced beacon RX window (50ms cycle @20Hz)       */
#define BCN_SYNC_POLL_MS 20u    /* Synced poll: beacon + all blinks (~10ms) + margin */

/* TDOA solver tuning */
#define TDOA_CM_PER_DTU  0.4691f  /* cm per DW3000 device time unit (499.2MHz × 128) */
#define OUTLIER_RMS_CM   10.0f    /* all-anchor RMS (cm) above which leave-one-out is tried */

/* Header match patterns (SN, DST, SRC wildcarded) */
#define HDR_LEN 10u
#define SN_IDX   2u
static uint8_t match_beacon[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0, 0, 0xEE};
static uint8_t match_blink[]  = {0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0, 0, 0xEF};

/*============================================================================
 * Kalman filter — 2D constant-velocity, per-axis 2-state (pos, vel)
 *
 *   Each axis is filtered independently with identical parameters.
 *   dt is in seconds (typically 0.5 s for the 500ms beacon period).
 *   The measurement noise R is scaled by the solver's Q (RMS residual)
 *   so periods with high solver uncertainty are weighted less.
 *============================================================================*/
static float kf_pos[2] = {0, 0};   /* filtered position  [x_cm, y_cm] */
static float kf_vel[2] = {0, 0};   /* filtered velocity  [cm/s]       */
static float kf_Px[4];              /* covariance for x-axis */
static float kf_Py[4];              /* covariance for y-axis */
static int   kf_init_done = 0;
static int   kf_reject_cnt = 0;     /* consecutive rejected measurements  */

/* Tuning constants */
#define KF_Q_POS     0.01f   /* position process noise spectral density (cm²/s)  */
#define KF_Q_VEL    10.0f    /* velocity process noise spectral density (cm²/s³) */
#define KF_R_BASE   100.0f   /* base measurement noise (cm²) when solver RMS=0   */
#define KF_R_MULT    15.0f   /* scale measurement noise by RMS×MULT              */
#define KF_GATE_CM       50000.0f  /* innovation outlier gate (500 m, in cm)   */
#define KF_REINIT_THRESH 3         /* consecutive rejections before re-seeding */

/* kalman_update_2d — run one predict+update cycle.
 *   x_m, y_m : raw measurement from solver (cm)
 *   dt       : time since last update (seconds), or 0 for first fix
 *   rms      : solver residual (cm), used to scale measurement noise
 *   x_f, y_f : [out] filtered position
 * Returns 0 on success, -1 if measurement rejected (>500m from prediction). */
static int kalman_update_2d(float x_m, float y_m, float dt, float rms,
                            float *x_f, float *y_f)
{
    int axis;
    float dt2  = dt * dt;

    /* Measurement noise: base + RMS-scaled */
    float R = KF_R_BASE + KF_R_MULT * rms * rms;
    if (R < 1.0f) R = 1.0f;  /* clamp floor */

    if (!kf_init_done)
    {
        /* First fix — initialize state */
        kf_pos[0] = x_m;  kf_pos[1] = y_m;
        kf_vel[0] = 0.0f; kf_vel[1] = 0.0f;
        /* Initial covariance: position ~10cm, velocity unknown */
        kf_Px[0] = 100.0f; kf_Px[1] = 0.0f;
        kf_Px[2] = 0.0f;   kf_Px[3] = 100.0f;
        kf_Py[0] = 100.0f; kf_Py[1] = 0.0f;
        kf_Py[2] = 0.0f;   kf_Py[3] = 100.0f;
        kf_init_done = 1;
        kf_reject_cnt = 0;
        *x_f = x_m; *y_f = y_m;
        return 0;
    }

    /*=== Stale-state recovery ===
     * If the filter is seeded with a bad first fix (e.g. a warm-up cycle before
     * DLY_RS calibration), every subsequent *correct* measurement looks like a
     * >500m outlier and is rejected forever, freezing the position at the bad
     * value.  Count consecutive 2D rejections; after K in a row, trust the
     * latest measurement and re-seed the filter. */
    {
        float px = kf_pos[0] + kf_vel[0] * dt;
        float py = kf_pos[1] + kf_vel[1] * dt;
        float ix = x_m - px;
        float iy = y_m - py;
        if (ix > KF_GATE_CM || ix < -KF_GATE_CM ||
            iy > KF_GATE_CM || iy < -KF_GATE_CM)
        {
            if (++kf_reject_cnt >= KF_REINIT_THRESH)
            {
                kf_pos[0] = x_m;  kf_pos[1] = y_m;
                kf_vel[0] = 0.0f; kf_vel[1] = 0.0f;
                kf_Px[0] = 100.0f; kf_Px[1] = 0.0f;
                kf_Px[2] = 0.0f;   kf_Px[3] = 100.0f;
                kf_Py[0] = 100.0f; kf_Py[1] = 0.0f;
                kf_Py[2] = 0.0f;   kf_Py[3] = 100.0f;
                kf_reject_cnt = 0;
                *x_f = x_m; *y_f = y_m;
                return 0;
            }
        }
        else
        {
            kf_reject_cnt = 0;
        }
    }

    for (axis = 0; axis < 2; axis++)
    {
        float *P = (axis == 0) ? kf_Px : kf_Py;
        float pos  = kf_pos[axis];
        float vel  = kf_vel[axis];
        float P00  = P[0];  /* P_pos_pos */
        float P01  = P[1];  /* P_pos_vel */
        float P10  = P[2];  /* P_vel_pos */
        float P11  = P[3];  /* P_vel_vel */
        float z    = (axis == 0) ? x_m : y_m;

        /*=== Predict ===*/
        float pos_pred = pos + vel * dt;
        float vel_pred = vel;

        /* P_pred = F*P*F' + Q,  with  F = [[1,dt],[0,1]] */
        float P00_pred = P00 + dt*(P01 + P10) + dt2*P11
                       + (KF_Q_POS*dt + KF_Q_VEL*dt2*dt/3.0f);
        float P01_pred = P01 + dt*P11 + KF_Q_VEL*dt2*0.5f;
        float P10_pred = P10 + dt*P11 + KF_Q_VEL*dt2*0.5f;
        float P11_pred = P11 + KF_Q_VEL*dt;

        /*=== Outlier gate: reject if innovation > 500m ===*/
        float innov = z - pos_pred;
        if (innov > KF_GATE_CM || innov < -KF_GATE_CM)
        {
            /* Measurement rejected — output prediction only */
            kf_pos[axis] = pos_pred;
            kf_vel[axis] = vel_pred;
            P[0] = P00_pred; P[1] = P01_pred;
            P[2] = P10_pred; P[3] = P11_pred;
            if (axis == 0) *x_f = pos_pred; else *y_f = pos_pred;
            continue;
        }

        /*=== Update ===*/
        /* S = H*P_pred*H' + R = P00_pred + R  (since H=[1,0]) */
        float S = P00_pred + R;

        /* K = P_pred * H' / S  → 2×1 vector */
        float K0 = P00_pred / S;
        float K1 = P10_pred / S;

        /* State update */
        kf_pos[axis] = pos_pred + K0 * innov;
        kf_vel[axis] = vel_pred + K1 * innov;

        /* Covariance update: P = (I - K*H) * P_pred */
        float IKH00 = 1.0f - K0;     /* I-K*H = [[1-K0, 0], [-K1, 1]] */
        P[0] = IKH00 * P00_pred;                    /* P_pos_pos */
        P[1] = IKH00 * P01_pred;                    /* P_pos_vel */
        P[2] = P10_pred - K1 * P00_pred;            /* P_vel_pos */
        P[3] = P11_pred - K1 * P01_pred;            /* P_vel_vel */

        if (axis == 0) *x_f = kf_pos[axis];
        else           *y_f = kf_pos[axis];
    }
    return 0;
}

extern dwt_txconfig_t txconfig_options;
extern dwt_txconfig_t txconfig_options_ch9;
#if UWB_CHANNEL == 9
#define TXCONFIG (&txconfig_options_ch9)
#else
#define TXCONFIG (&txconfig_options)
#endif

static const char hex[] = "0123456789ABCDEF";

/*============================================================================
 * LED blink poll (V0.3) — fast blink 2s, off 5s
 *============================================================================*/
static void find_blink_poll(void)
{
    /* LED off during active positioning, heartbeat blink when idle.
     * g_slot_synced is true when tag has valid TDMA phase sync
     * and is actively tracking anchor beacons. */
    if (g_slot_synced) {
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
        return;
    }
    uint32_t t = HAL_GetTick() % 7000u;
    if (t < 2000u) {
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin,
            ((t % 100u) < 50u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
    }
}

/*============================================================================
 * match_header — compare rx buffer against pattern, masking SN/DST/SRC
 *============================================================================*/
static int match_header(uint8_t *rx, uint8_t *pattern, uint8_t len,
                        uint8_t zero_sn, uint8_t zero_dst, uint8_t zero_src)
{
    uint8_t saved_sn = rx[SN_IDX];
    uint8_t saved_d0 = rx[5], saved_d1 = rx[6];
    uint8_t saved_s0 = rx[7], saved_s1 = rx[8];
    if (zero_sn)  rx[SN_IDX] = 0;
    if (zero_dst) { rx[5] = 0; rx[6] = 0; }
    if (zero_src) { rx[7] = 0; rx[8] = 0; }
    int match = (memcmp(rx, pattern, len) == 0);
    if (zero_sn)  rx[SN_IDX] = saved_sn;
    if (zero_dst) { rx[5] = saved_d0; rx[6] = saved_d1; }
    if (zero_src) { rx[7] = saved_s0; rx[8] = saved_s1; }
    return match;
}

/*============================================================================
 * solve_2d_gn — 2D TDOA Gauss-Newton least squares
 *   anchors[]   : anchor positions (cm); anchor 0 = reference (beacon)
 *   tdoa_cm[]   : TDOA measurements in cm (tdoa_cm[k] = d_k - d_0, k=1..N-1)
 *   valid[]     : per-anchor validity mask (NULL = all valid).  A slave whose
 *                 blink was not received this cycle has valid[k]=0 and must be
 *                 excluded, otherwise its zero TOA would read as a bogus huge
 *                 negative TDOA and poison the whole solve.
 *   num_anchors : total anchors N
 *   skip        : slave index to exclude (-1 = use all) for outlier rejection
 *   x0/y0       : initial guess (cm)
 *   x/y/rms_out : solution position + RMS residual (cm)
 *   Returns 0 on success, -1 on singular matrix, -2 on too few anchors.
 *============================================================================*/
static int solve_2d_gn(const dl_anchor_entry_t *anchors,
                       const float *tdoa_cm,
                       const uint8_t *valid,
                       uint8_t num_anchors, int skip,
                       float x0, float y0,
                       float *x_out, float *y_out, float *rms_out)
{
    float x = x0, y = y0, rms = 0.0f;
    int iter, k;
    int used = 0;

    for (k = 1; k < (int)num_anchors; k++)
        if (k != skip && (!valid || valid[k]))
            used++;

    if (used < 2)
        return -2;

    for (iter = 0; iter < 8; iter++)
    {
        float r[DL_MAX_ANCHORS];
        float H00 = 0, H01 = 0, H11 = 0, g0 = 0, g1 = 0;
        float _rms = 0, prev_rms = rms;

        for (k = 0; k < (int)num_anchors; k++)
        {
            float dx = x - (float)anchors[k].x_cm;
            float dy = y - (float)anchors[k].y_cm;
            r[k] = sqrtf(dx * dx + dy * dy);
            if (r[k] < 1.0f) r[k] = 1.0f;
        }

        for (k = 1; k < (int)num_anchors; k++)
        {
            float pred, res, dx0, dy0, dxk, dyk, J0, J1;
            if (k == skip || (valid && !valid[k])) continue;
            pred = r[k] - r[0];
            res  = pred - tdoa_cm[k];
            dx0 = (x - (float)anchors[0].x_cm) / r[0];
            dy0 = (y - (float)anchors[0].y_cm) / r[0];
            dxk = (x - (float)anchors[k].x_cm) / r[k];
            dyk = (y - (float)anchors[k].y_cm) / r[k];
            J0 = dxk - dx0;
            J1 = dyk - dy0;
            H00 += J0 * J0;
            H01 += J0 * J1;
            H11 += J1 * J1;
            g0  -= J0 * res;
            g1  -= J1 * res;
            _rms += res * res;
        }
        _rms = sqrtf(_rms / (float)used);
        rms = _rms;

        {
            float det = H00 * H11 - H01 * H01;
            float dx, dy;
            if (fabsf(det) < 1e-6f) {
                /* Near-singular information matrix: thin anchor layout or
                 * tag outside the convex hull makes the TDOA gradients
                 * nearly collinear.  Add Levenberg-Marquardt damping so H
                 * stays invertible — the step shrinks but the solver still
                 * yields a usable position instead of aborting. */
                H00 += 1.0f;
                H11 += 1.0f;
                det = H00 * H11 - H01 * H01;
            }
            if (fabsf(det) < 1e-9f)
                return -1;
            dx = (g0 * H11 - H01 * g1) / det;
            dy = (H00 * g1 - g0 * H01) / det;
            /* Damping: halve step if residual increased */
            if (iter > 0 && _rms > prev_rms) { dx *= 0.5f; dy *= 0.5f; }
            x += dx; y += dy;
            if (sqrtf(dx * dx + dy * dy) < 0.05f)
                break;
        }
    }

    *x_out = x;
    *y_out = y;
    *rms_out = rms;
    return 0;
}

/*============================================================================
 * wait_rx — Sleep(1)-based RX polling with timeout (ms)
 *============================================================================*/
static int wait_rx(uint32_t *status, uint32_t *flen, uint32_t timeout_ms)
{
    uint32_t elapsed;
    for (elapsed = 0; elapsed < timeout_ms; elapsed++)
    {
        Sleep(1);
        *status = dwt_read32bitreg(SYS_STATUS_ID);
        if (*status & SYS_STATUS_ALL_RX_ERR)
        {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            continue;
        }
        if (*status & SYS_STATUS_RXFCG_BIT_MASK)
        {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
            *flen = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
            return 1;
        }
    }
    return 0;
}

/*============================================================================
 * uart_print_f32 — format a float to N fractional digits into out[].
 *                  Returns number of characters written.
 *============================================================================*/
static int uart_print_f32(char *out, float val, int frac_digits)
{
    int pos = 0;
    int int_part;
    float frac;

    if (val < 0) { out[pos++] = '-'; val = -val; }
    int_part = (int)val;
    frac = val - (float)int_part;

    /* Integer part (simple itoa) */
    if (int_part == 0) { out[pos++] = '0'; }
    else
    {
        char tmp[12]; int tp = 0;
        while (int_part > 0) { tmp[tp++] = '0' + (int_part % 10); int_part /= 10; }
        while (tp > 0) out[pos++] = tmp[--tp];
    }
    out[pos++] = '.';

    /* Fractional digits */
    {
        int d;
        for (d = 0; d < frac_digits; d++)
        {
            frac *= 10.0f;
            int digit = (int)frac;
            out[pos++] = '0' + digit;
            frac -= (float)digit;
        }
    }
    return pos;
}

#if !TAG_DEBUG_OUTPUT
/*============================================================================
 * tx_tag_pos_hex — send position report in robot communication protocol
 *
 *   Frame format per 通信协议.md (47 bytes total, little-endian):
 *     [0-7]    Location_time_us    uint64   定位时间 (μs), bcn_tick×1000
 *     [8-19]   Pos[3]              float[3]  {x, y, z} 单位 米
 *     [20-25]  Vel_cm[3]           int16[3]  {vx, vy, vz} 单位 cm/s
 *     [26-28]  Pos_noise_cm[3]     uint8[3]  位置噪声 单位 cm
 *     [29-31]  Vel_noise_cm[3]     uint8[3]  速度噪声 单位 cm/s (暂无=0)
 *     [32]     Map_id              uint8     地图 ID (暂无=0)
 *     [33]     Error_code+Area_id  uint8     hi4=Error, lo4=Area (暂无=0)
 *     [34]     Reserved            uint8     保留 (0)
 *     [35]     Anchor_count(4bit)  uint8     hi4=锚点数, lo4=padding 0
 *     [36-44]  Anchors[9]          uint8[9]  锚点地址列表 (暂无=0)
 *     [45]     Rx_rssi             uint8     接收信号强度 (暂无=0)
 *     [46]     Rx_rate             uint8     信标接收率 (暂无=0)
 *============================================================================*/
static void tx_tag_pos_hex(uint32_t bcn_tick_ms, uint8_t seq,
                           float x_cm, float y_cm, float z_cm,
                           float rms_cm, int fix_valid)
{
    uint8_t frame[47];
    uint64_t loc_time_us;
    float    pos_m[3];
    int16_t  vel_cm[3];
    uint8_t  noise_cm[3];
    int i;

    (void)seq;  /* seq not in this protocol — reserved for future */

    /* Fill all with zero first ("没有的字节就空着") */
    for (i = 0; i < 47; i++) frame[i] = 0;

    /* 1. Location_time_us — ms → μs */
    loc_time_us = (uint64_t)bcn_tick_ms * 1000ull;
    frame[0] = (uint8_t)(loc_time_us & 0xFF);
    frame[1] = (uint8_t)((loc_time_us >> 8) & 0xFF);
    frame[2] = (uint8_t)((loc_time_us >> 16) & 0xFF);
    frame[3] = (uint8_t)((loc_time_us >> 24) & 0xFF);
    frame[4] = (uint8_t)((loc_time_us >> 32) & 0xFF);
    frame[5] = (uint8_t)((loc_time_us >> 40) & 0xFF);
    frame[6] = (uint8_t)((loc_time_us >> 48) & 0xFF);
    frame[7] = (uint8_t)((loc_time_us >> 56) & 0xFF);

    /* 2. Pos[3] — cm → meters (float, little-endian) */
    if (fix_valid) {
        pos_m[0] = x_cm / 100.0f;
        pos_m[1] = y_cm / 100.0f;
        pos_m[2] = z_cm / 100.0f;
    } else {
        pos_m[0] = 0.0f; pos_m[1] = 0.0f; pos_m[2] = 0.0f;
    }
    {
        uint8_t *p = (uint8_t *)pos_m;
        for (i = 0; i < 12; i++) frame[8 + i] = p[i];
    }

    /* 3. Vel_cm[3] — from Kalman filter (cm/s → int16 LE) */
    if (fix_valid && kf_init_done) {
        vel_cm[0] = (int16_t)kf_vel[0];
        vel_cm[1] = (int16_t)kf_vel[1];
    } else {
        vel_cm[0] = 0; vel_cm[1] = 0;
    }
    vel_cm[2] = 0;  /* Z-velocity: no estimate */
    frame[20] = (uint8_t)(vel_cm[0] & 0xFF);
    frame[21] = (uint8_t)((vel_cm[0] >> 8) & 0xFF);
    frame[22] = (uint8_t)(vel_cm[1] & 0xFF);
    frame[23] = (uint8_t)((vel_cm[1] >> 8) & 0xFF);
    frame[24] = (uint8_t)(vel_cm[2] & 0xFF);
    frame[25] = (uint8_t)((vel_cm[2] >> 8) & 0xFF);

    /* 4. Pos_noise_cm[3] — RMS residual per axis (uint8, clamp to 255) */
    if (fix_valid) {
        float n = rms_cm;
        if (n > 255.0f) n = 255.0f;
        noise_cm[0] = (uint8_t)n;
        noise_cm[1] = (uint8_t)n;
        noise_cm[2] = (uint8_t)n;
    } else {
        noise_cm[0] = 0xFF; noise_cm[1] = 0xFF; noise_cm[2] = 0xFF;
    }
    frame[26] = noise_cm[0];
    frame[27] = noise_cm[1];
    frame[28] = noise_cm[2];

    /* 5. Vel_noise_cm[3] — [29-31], leave as 0 (暂无) */

    /* 6. Map_id — [32], leave as 0 (暂无) */

    /* 7. Error_code(hi4) + Area_id(lo4) — [33], leave as 0 (暂无) */

    /* 8. Reserved — [34], leave as 0 */

    /* 9. Anchor_count(hi4) — [35] */
    frame[35] = (num_anchors & 0x0Fu) << 4;  /* hi4 = count, lo4 = 0 */

    /* 10. Anchors[9] — [36-44], leave as 0 (暂无锚点地址列表) */

    /* 11. Rx_rssi — [45], leave as 0 (暂无) */

    /* 12. Rx_rate — [46], leave as 0 (暂无) */

    HAL_UART_Transmit(&huart2, frame, sizeof(frame), 100);
}
#endif /* !TAG_DEBUG_OUTPUT */

/*============================================================================
 * fusion_tag — downlink TDOA receiver + solver
 *============================================================================*/
int fusion_tag(void)
{
    int timeout;

    test_run_info((unsigned char *)APP_NAME);

    /*--- DW3000 init with three-tier PLL retry ---*/
    {
        int attempt;
        for (attempt = 0; attempt < 3; attempt++)
        {
            port_set_dw_ic_spi_fastrate();
            reset_DWIC();
            Sleep(100);

            timeout = 1000;
            while (!dwt_checkidlerc() && --timeout > 0) { Sleep(1); }
            if (timeout <= 0)
            {
                test_run_info((unsigned char *)"IDLE TIMEOUT");
                continue;
            }
            test_run_info((unsigned char *)"IDLE OK");

            if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
            {
                test_run_info((unsigned char *)"INIT FAILED");
                continue;
            }
            test_run_info((unsigned char *)"INIT OK");

            if (dwt_configure(&config) == DWT_SUCCESS)
            {
                test_run_info((unsigned char *)"CONFIG OK");
                break;
            }

            /* Tier-2: soft PLL re-lock + PGF re-cal (5ms PLL timeout) */
            test_run_info((unsigned char *)"CONFIG FAILED");
            {
                uint8_t pll_cnt, pll_ok = 0;
                dwt_write8bitoffsetreg(PLL_CAL_ID, 0, RF_PLL_CFG_LD);
                dwt_write8bitoffsetreg(SYS_STATUS_ID, 0, SYS_STATUS_CP_LOCK_BIT_MASK);
                dwt_setdwstate(DWT_DW_IDLE);
                for (pll_cnt = 0; pll_cnt < 250; pll_cnt++)
                {
                    deca_usleep(DELAY_20uUSec);
                    if (dwt_read8bitoffsetreg(SYS_STATUS_ID, 0)
                            & SYS_STATUS_CP_LOCK_BIT_MASK)
                    {
                        pll_ok = 1;
                        break;
                    }
                }
                if (pll_ok)
                {
                    test_run_info((unsigned char *)"PLL OK");
                    if (dwt_pgf_cal(1) == DWT_SUCCESS)
                    {
                        test_run_info((unsigned char *)"PGF OK");
                        break;
                    }
                    test_run_info((unsigned char *)"PGF FAILED");
                }
                else
                {
                    test_run_info((unsigned char *)"PLL FAILED");
                }
            }
        }
        if (attempt >= 3)
        {
            test_run_info((unsigned char *)"DW INIT ALL FAIL");
            while (1) { }
        }
    }

    /* Enable CIA diagnostics — improves RX reliability */
    dwt_configciadiag(DW_CIA_DIAG_LOG_ALL);

    /* Enable DW3000 auto RX re-enable (RXAUTR, bit 10 in SYS_CFG).
     * Without this, DW3000 stops RX after each frame and requires an
     * explicit dwt_rxenable() call to re-arm.  With RXAUTR, the radio
     * automatically re-enters RX after each frame reception, which is
     * critical for slot-synchronized multi-frame TDMA cycles. */
    {
        uint32_t sys_cfg = dwt_read32bitreg(SYS_CFG_ID);
        sys_cfg |= SYS_CFG_RXAUTR_BIT_MASK;
        dwt_write32bitreg(SYS_CFG_ID, sys_cfg);
    }

    /* Enable DW3000 GPIO clocks */
    dwt_enablegpioclocks();

    dwt_configuretxrf(TXCONFIG);
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_setfinegraintxseq(0);
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    /* Init per-slave DLY_RS to default */
    { int _s; for(_s=0;_s<DL_MAX_ANCHORS;_s++){ dly_rs[_s]=DLY_RS_DEFAULT; calib_sum[_s]=0; calib_cnt[_s]=0; } }

    test_run_info((unsigned char *)"INIT DONE");
    /* Tag does NOT call gw_ota_confirm_boot() — it has never been OTA'd
     * and the flag page (0x0801F000) might still contain stale data from
     * previous anchor firmware that could trigger unintended flash erases. */
    port_rf_switch_init();
    port_rf_switch_rx();
    /* Heartbeat timer: tag sends a no-fix POS_REPORT every 2s at startup
     * so the Gateway can detect it even before TDOA starts.  Once TDOA
     * is running, actual position reports keep the tag online. */
    uint32_t last_hb_tick = 0;
    #define TAG_HB_INTERVAL_MS  2000u

    /*=== Main downlink TDOA loop — SLOT-SYNCHRONIZED listen, then solve ===*/
    while (1)
    {
        uint32_t status_reg, frame_len;
        uint8_t  cycle_seq = 0;
        int      num_rx = 0;        /* total anchors collected this cycle */
        int      i, p;
        uint64_t tx_ts_dtu[DL_MAX_ANCHORS];  /* embedded TX timestamps */
        int16_t  cfo_raw[DL_MAX_ANCHORS];    /* CFO clock offset (raw, /2^26 = frac, ×1e6 = ppm) */
        int64_t  raw_bias_dtu[DL_MAX_ANCHORS]; /* raw slot-compensated bias (diagnostic) */
        uint32_t bcn_tick = 0;      /* portGetTickCnt() snapshot at beacon RX */

        for (i = 0; i < DL_MAX_ANCHORS; i++) { toa_dtu[i] = 0; tx_ts_dtu[i] = 0; cfo_raw[i] = 0; raw_bias_dtu[i] = 0; }
        num_anchors = 0;

        /*=== Slot-synced pre-sleep: skip ~490ms idle period between cycles ===*/
        if (g_slot_synced)
        {
            int32_t until_bcn = (int32_t)(g_next_bcn_tick - portGetTickCnt());
            if (until_bcn > (int32_t)(BCN_SYNC_WIN_MS / 2))
            {
                int32_t sleep_ms = until_bcn - (int32_t)(BCN_SYNC_WIN_MS / 2);
                if (sleep_ms > 0 && sleep_ms < 600)
                {
                    int _sp;
                    for (_sp = 0; _sp < sleep_ms; _sp++) Sleep(1);
                }
            }
        }

        /*=== Unified RX poll (same structure as proven demo v1.0).
         *   Synced: 100ms window (beacon + all blinks).  Unsynced: 650ms. ===*/
        {
            int poll_max = g_slot_synced ? (int)BCN_SYNC_POLL_MS : (int)BCN_TIMEOUT_MS;
            int first_beacon = 1;

            /* Force IDLE before RX to guarantee a clean state transition.
             * Without this, back-to-back while(1) iterations leave the DW3000
             * in RX (no frame received → RXAUTR never triggered), and calling
             * CMD_RX on an already-RX radio can lock the SPI interface on
             * some DW3000 revisions.  The forctrxoff+rxenable pattern matches
             * the TX path (heartbeat / pos_report) and the DW3000 API guide. */
            dwt_forcetrxoff();
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            for (p = 0; p < poll_max; p++)
            {
                Sleep(1);
                find_blink_poll();
                status_reg = dwt_read32bitreg(SYS_STATUS_ID);
                if (status_reg & SYS_STATUS_ALL_RX_ERR)
                { dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR); dwt_rxenable(DWT_START_RX_IMMEDIATE); continue; }
                if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK)) continue;
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
                frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
                if (frame_len > sizeof(rx_buffer)) { dwt_rxenable(DWT_START_RX_IMMEDIATE); continue; }
                dwt_readrxdata(rx_buffer, frame_len, 0);

                /* Beacon (0xEE): first one only per cycle */
                if (first_beacon && match_header(rx_buffer, match_beacon, HDR_LEN, 1, 0, 1))
                {
                    uint8_t n = (uint8_t)(((int)frame_len - 19) / 7);  /* 7-byte anchor entries */
                    /* Byte 15 = num_anchors | DL_BCN_TDOA_ACTIVE(0x80); mask off bit7 */
                    if (n < 2 || n > DL_MAX_ANCHORS) n = rx_buffer[DL_BCN_NUM_IDX] & 0x7Fu;
                    if (n >= 2 && n <= DL_MAX_ANCHORS)
                    {
                        num_rx    = 1;
                        num_anchors = n;
                        cycle_seq = rx_buffer[DL_BCN_CYCLE_IDX];
                        first_beacon = 0;
                        bcn_tick = portGetTickCnt();
                        /* B,<n>,<seq> */
                        /* { char _b[16]; _b[0]='B'; _b[1]=','; _b[2]=hex[n&0xF]; _b[3]=',';
                          _b[4]=hex[(cycle_seq>>4)&0xF]; _b[5]=hex[cycle_seq&0xF];
                          _b[6]='\r'; _b[7]='\n'; _b[8]='\0';
                          test_run_info((unsigned char *)_b); } */
                        if (cycle_seq != frame_seq || frame_seq == 0)
                        {
                            frame_seq = cycle_seq;
                            for (i = 0; i < (int)n; i++)
                                unpack_anchor_entry(
                                    &rx_buffer[DL_BCN_TABLE_IDX + i * DL_ANCHOR_ENTRY_SIZE],
                                    &anchor_table[i]);
                            baseline_dtu[0] = 0;
                            for (i = 1; i < (int)n; i++)
                            {
                                float _bx = (float)(anchor_table[i].x_cm - anchor_table[0].x_cm);
                                float _by = (float)(anchor_table[i].y_cm - anchor_table[0].y_cm);
                                float _bz = (float)(anchor_table[i].z_cm - anchor_table[0].z_cm);
                                float _bcm = sqrtf(_bx*_bx + _by*_by + _bz*_bz);
                                baseline_dtu[i] = (int64_t)(_bcm / ((float)SPEED_OF_LIGHT * 100.0f / (499.2e6f * 128.0f)));
                            }
#if TAG_DIAG
                            /* One-shot: dump the anchor table the tag is actually
                             * solving against (decimal meters).  If these don't
                             * match the host's configured coordinates, the beacon
                             * is carrying stale/wrong coords (COORD write failed
                             * or defaults mismatch) — that's the root cause of a
                             * far-off fixed position. */
                            {
                                static int _ac_printed = 0;
                                if (!_ac_printed) {
                                    _ac_printed = 1;
                                    char _ac[128]; int _p = 0, _ai;
                                    _ac[_p++] = 'A'; _ac[_p++] = 'C'; _ac[_p++] = ':';
                                    for (_ai = 0; _ai < (int)n && _ai < 4; _ai++) {
                                        _ac[_p++] = ' ';
                                        _ac[_p++] = '#'; _ac[_p++] = '0' + _ai;
                                        _ac[_p++] = '(';
                                        _p += uart_print_f32(&_ac[_p], (float)anchor_table[_ai].x_cm / 100.0f, 2);
                                        _ac[_p++] = ',';
                                        _p += uart_print_f32(&_ac[_p], (float)anchor_table[_ai].y_cm / 100.0f, 2);
                                        _ac[_p++] = ',';
                                        _p += uart_print_f32(&_ac[_p], (float)anchor_table[_ai].z_cm / 100.0f, 2);
                                        _ac[_p++] = ')';
                                    }
                                    _ac[_p++] = '\r'; _ac[_p++] = '\n'; _ac[_p] = '\0';
                                    test_run_info((unsigned char *)_ac);
                                }
                            }
#endif
                        }
                        toa_dtu[0]  = get_rx_timestamp_u64();
                        tx_ts_dtu[0] = unpack_40bit_ts(&rx_buffer[DL_BCN_TX_TS_IDX]);
                        cfo_raw[0]   = dwt_readclockoffset();
                    }
                    dwt_rxenable(DWT_START_RX_IMMEDIATE);
                    continue;
                }

                /* Blink (0xF1): only after beacon, one per rank */
                if (num_rx >= 1
                    && match_header(rx_buffer, match_blink, HDR_LEN, 1, 0, 1)
                    && frame_len >= DL_BLINK_LEN)
                {
                    uint16_t rank16 = (uint16_t)(rx_buffer[7])
                                    | ((uint16_t)(rx_buffer[8]) << 8);
                    if (rank16 > 0 && rank16 < DL_MAX_ANCHORS && toa_dtu[rank16] == 0)
                    {
                        toa_dtu[rank16]  = get_rx_timestamp_u64();
                        tx_ts_dtu[rank16] = unpack_40bit_ts(&rx_buffer[DL_BLINK_TX_TS_IDX]);
                        cfo_raw[rank16]   = dwt_readclockoffset();
                        num_rx++;
                        /* K,<rank>,CFO=<raw> */
                        /* { char _k[18]; int _kp = 0;
                          _k[_kp++]='K'; _k[_kp++]=',';
                          _k[_kp++]=hex[rank16&0xF]; _k[_kp++]=',';
                          { int16_t _cv = cfo_raw[rank16]; uint16_t _uv = (uint16_t)_cv;
                            _k[_kp++]=hex[(_uv>>12)&0xF]; _k[_kp++]=hex[(_uv>>8)&0xF];
                            _k[_kp++]=hex[(_uv>>4)&0xF]; _k[_kp++]=hex[_uv&0xF];
                          }
                          _k[_kp++]='\r'; _k[_kp++]='\n'; _k[_kp]='\0';
                          test_run_info((unsigned char *)_k); } */
                    }
                }
                /* PREP_CMD (0xF0) — only respond if addressed to this tag.
                 * Gateway sends PREP_CMD to Master(0x0000) during COORD write;
                 * tag must ignore those to avoid resetting slot sync. */
                else if (rx_buffer[9] == FUNC_PREP_CMD
                         && (uint16_t)(rx_buffer[5] | (rx_buffer[6] << 8)) == DL_TAG_ID)
                {
                    g_slot_synced = 0;  /* Stop_Ranging: release TDMA slot, free air for OTA */
                    /* ACK with 0xF1 — 3x burst, IRQ protection, 10ms TX timeout */
                    { uint8_t ack[13]; int _r;
                      ack[0]=0x41; ack[1]=0x88; ack[2]=0;
                      ack[3]=0xCA; ack[4]=0xDE; ack[5]=rx_buffer[7]; ack[6]=rx_buffer[8];
                      ack[7]=DL_TAG_ID&0xFF; ack[8]=(DL_TAG_ID>>8)&0xFF;
                      ack[9]=FUNC_PREP_ACK; ack[10]=0xCC; ack[11]=0xDD; ack[12]=0xEE;
                      port_DisableEXT_IRQ();
                      for(_r=0;_r<3;_r++){
                        dwt_forcetrxoff(); port_rf_switch_tx();
                        dwt_writetxdata(sizeof(ack),ack,0); dwt_writetxfctrl(sizeof(ack),0,0);
                        if(dwt_starttx(DWT_START_TX_IMMEDIATE)==DWT_SUCCESS){
                          uint32_t tx_dl = HAL_GetTick() + 10u;
                          while(!(dwt_read32bitreg(SYS_STATUS_ID)&SYS_STATUS_TXFRS_BIT_MASK)){
                            if(HAL_GetTick() > tx_dl) break;
                          }
                          dwt_write32bitreg(SYS_STATUS_ID,SYS_STATUS_TXFRS_BIT_MASK);}
                        port_rf_switch_rx(); if(_r<2) Sleep(5);
                      }
                      dwt_rxenable(DWT_START_RX_IMMEDIATE);
                      EXTI->PR1 = (1u << 12);
                      /* EXTI stays disabled — tag uses polling for RX */ }
                    test_run_info((unsigned char *)"OTA READY\r\n");
                }
                /* OTA_DATA (0xF6) — only if addressed to this tag */
                else if (rx_buffer[9] == FUNC_OTA_DATA && frame_len >= GW_HDR_LEN + 4
                         && (uint16_t)(rx_buffer[5] | (rx_buffer[6] << 8)) == DL_TAG_ID)
                {
                    gw_ota_handle_data(rx_buffer, frame_len, DL_TAG_ID);
                }
                /* OTA_END (0xE4) — only if addressed to this tag */
                else if (rx_buffer[9] == FUNC_OTA_END
                         && (uint16_t)(rx_buffer[5] | (rx_buffer[6] << 8)) == DL_TAG_ID)
                {
                    gw_ota_finish(DL_TAG_ID);
                }
                /* OTA_ABORT (0xE6) — only if addressed to this tag */
                else if (rx_buffer[9] == FUNC_OTA_ABORT
                         && (uint16_t)(rx_buffer[5] | (rx_buffer[6] << 8)) == DL_TAG_ID)
                {
                    gw_ota_abort();
                    test_run_info((unsigned char *)"OTA ABORT\r\n");
                }
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
            }

            /* If synced window ended without any frames, lose sync */
            if (g_slot_synced && num_rx == 0)
                g_slot_synced = 0;
        }

        /* OTA timeout poll — L1 rollback: if no data for 5s, resume ranging */
        if (gw_ota_poll()) {
            test_run_info((unsigned char *)"OTA TIMEOUT\r\n");
        }

        /* During OTA: skip output/solver/position-report TX.
         * Position report TX calls dwt_forcetrxoff() which kills the receiver.
         * Relying on num_rx < 2 alone is not safe — a spurious match could
         * increment num_rx and cause the output phase to run, killing RX for
         * tens of ms. */
        if (gw_ota_is_active())
            continue;

        /*=== Heartbeat: send no-fix POS_REPORT every ~2s.
         *    Must fire BEFORE the num_rx < 2 guard so the tag stays online
         *    even when only the beacon is received (no blinks).  Common
         *    scenarios: config mode (Slave heartbeat blinks are sparse),
         *    post-network-config (few anchors), or before TDOA starts. ===*/
        if (num_anchors < 3) {
            uint32_t now = portGetTickCnt();
            if (now - last_hb_tick > TAG_HB_INTERVAL_MS) {
                last_hb_tick = now;
                {
                    uint8_t hb[DL_POS_REPORT_LEN];
                    hb[0] = 0x41; hb[1] = 0x88;
                    hb[2] = FW_VERSION_BYTE;
                    hb[3] = 0xCA; hb[4] = 0xDE;
                    hb[5] = 0xFF; hb[6] = 0xFF;
                    hb[7] = (DL_TAG_ID >> 0) & 0xFF;
                    hb[8] = (DL_TAG_ID >> 8) & 0xFF;
                    hb[9] = FUNC_DL_POS_REPORT;
                    /* position = (0,0,0), quality = 0 → "no fix" */
                    { int _z; for (_z = 10; _z < DL_POS_REPORT_LEN; _z++) hb[_z] = 0; }
                    /* Tag uses polling mode (EXTI disabled by main.c).
                     * Disable EXTI during TX to prevent IRQ handler's
                     * dwt_forcetrxoff() from aborting the transmission.
                     * After TX, keep EXTI disabled (polling-only mode). */
                    port_DisableEXT_IRQ();
                    dwt_forcetrxoff();
                    port_rf_switch_tx();
                    dwt_writetxdata(sizeof(hb), hb, 0);
                    dwt_writetxfctrl(sizeof(hb), 0, 0);
                    if (dwt_starttx(DWT_START_TX_IMMEDIATE) == DWT_SUCCESS) {
                        uint32_t tx_dl = HAL_GetTick() + 10u;
                        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {
                            if (HAL_GetTick() > tx_dl) break;  /* TX timeout */
                        }
                        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
                    }
                    port_rf_switch_rx();
                    dwt_rxenable(DWT_START_RX_IMMEDIATE);
                    EXTI->PR1 = (1u << 12);
                    /* EXTI stays disabled — tag uses polling for RX */
                }
            }
        }

        /*=== Output result ===*/
        if (num_rx < 2)
        {
            /* Missed too many blinks — try to re-sync next cycle */
            g_slot_synced = 0;
            continue;
        }

#if 0
        /* Debug: slot-compensated TDOA (before per-slave DLY_RS — per-rank bias) */
        { char _d[80]; int _p = 0;
          _d[_p++]='T'; _d[_p++]=',';
          for(i=1;i<(int)num_anchors&&i<4;i++){
            if(toa_dtu[i]!=0){
              int64_t _diff = (int64_t)(toa_dtu[i]-toa_dtu[0]);
              int64_t _slot = (int64_t)i * (int64_t)DL_SLOT_DURATION_UUS * (int64_t)UUS_TO_DWT_TIME;
              int64_t _comp = _diff - _slot - baseline_dtu[i];
              int32_t _m = (int32_t)(_comp / 213);  /* DTU→m (÷213) */
              if(_m<0){_d[_p++]='-';_m=-_m;} else _d[_p++]='+';
              _d[_p++]=hex[(_m>>12)&0xF];_d[_p++]=hex[(_m>>8)&0xF];
              _d[_p++]=hex[(_m>>4)&0xF];_d[_p++]=hex[_m&0xF];
            } else { _d[_p++]='?';_d[_p++]='?';_d[_p++]='?';_d[_p++]='?';_d[_p++]='?'; }
            _d[_p++]=',';
          }
          /* anchor coords */
          for(i=0;i<(int)num_anchors&&i<3;i++){
            _d[_p++]=hex[i&0xF];_d[_p++]=':';
            { int16_t _ax=anchor_table[i].x_cm; if(_ax<0){_d[_p++]='-';_ax=-_ax;} else _d[_p++]='+';
              _d[_p++]=hex[(_ax>>8)&0xF];_d[_p++]=hex[(_ax>>4)&0xF];_d[_p++]=hex[_ax&0xF];}
            _d[_p++]=',';
            { int16_t _ay=anchor_table[i].y_cm; if(_ay<0){_d[_p++]='-';_ay=-_ay;} else _d[_p++]='+';
              _d[_p++]=hex[(_ay>>8)&0xF];_d[_p++]=hex[(_ay>>4)&0xF];_d[_p++]=hex[_ay&0xF];}
            _d[_p++]=' ';
          }
          _d[_p++]='\r';_d[_p++]='\n';_d[_p]='\0';
          test_run_info((unsigned char *)_d); }
#endif

        /*=== Compensate TOAs: slot + baseline + auto-calibrating per-slave DLY_RS.
         *   Master uses DWT_START_TX_IMMEDIATE → no DLY_RS.
         *   Slave  uses DWT_START_TX_DLY_RS     → +DLY_RS delay (~250ns).
         *
         *   Auto-calibration: accumulates raw slot-compensated bias over
         *   CALIB_SAMPLES cycles at power-on, then freezes.  The raw bias
         *   (= T, value in DTU) equals DLY_RS[k] + true_TDOA[k].
         *   Averaging over many cycles suppresses measurement noise while
         *   the true_TDOA term (bounded by anchor spacing ±1.5m) is small
         *   compared to DLY_RS (~75m).  Freezing prevents continuous
         *   adaptation from absorbing the true TDOA signal (= losing accuracy).  ===*/
        {
            uint64_t ref = toa_dtu[0];
            for (i = 1; i < (int)num_anchors && i < DL_MAX_ANCHORS; i++)
            {
                if (toa_dtu[i] != 0)
                {
                    int64_t toa_diff  = (int64_t)(toa_dtu[i] - toa_dtu[0]);
                    int64_t slot_dtu  = (int64_t)i * (int64_t)DL_SLOT_DURATION_UUS * (int64_t)UUS_TO_DWT_TIME;
                    int64_t raw_bias  = toa_diff - slot_dtu - baseline_dtu[i];  /* = DLY_RS + tdoa */
                    raw_bias_dtu[i] = raw_bias;  /* diagnostic: uncompensated raw bias */

#if CFO_COMP_ENABLE
                    /* CFO compensation — remove the slot-timing error caused by
                     * the tag↔slave crystal frequency offset.  cfo_raw[i] is a
                     * signed 12-bit value where value/2^26 = fractional clock
                     * offset (positive = tag RX clock faster than slave TX).
                     * Over the rank·3ms slot the tag therefore counts an extra
                     * slot_dtu × (ε_tag − ε_slave) ticks that are NOT real
                     * propagation — subtract them so the calibration sees a
                     * clean DLY_RS + TDOA instead of a rank-dependent CFO bias
                     * (≈0.9 m/ppm per rank step). */
                    raw_bias -= (slot_dtu * (int64_t)cfo_raw[i]) >> 26;
#endif

                    /* Accumulate for calibration (before DLY_RS subtraction),
                     * with outlier rejection: skip samples > ±CALIB_OUTLIER_M
                     * from the per-anchor running average to prevent a single
                     * spike from corrupting the calibration result. */
                    if (!calib_done && i < DL_MAX_ANCHORS)
                    {
                        int64_t outlier_thresh_dtu = (int64_t)CALIB_OUTLIER_M * 213;
                        int accept = 1;  /* default: accept this sample */

                        /* Absolute sanity bound: raw_bias = DLY_RS latency +
                         * TDOA.  The latency is a fixed hardware value (≪1µs)
                         * and |TDOA| is bounded by anchor spacing, so raw_bias
                         * stays well under ~1µs in a synced cycle.  A value
                         * approaching a whole beacon cycle means the tag's
                         * slot attribution was wrong on that first cycle —
                         * reject it, otherwise it poisons the running average
                         * and every real sample is then discarded as an
                         * outlier, leaving the DLY_RS stuck at its default. */
                        if (raw_bias > CALIB_ABS_MAX_DTU || raw_bias < -CALIB_ABS_MAX_DTU)
                            accept = 0;
                        else if (calib_cnt[i] > 0)
                        {
                            /* Compare against per-anchor running average */
                            int64_t running_avg = calib_sum[i] / calib_cnt[i];
                            int64_t diff = raw_bias - running_avg;
                            if (diff < 0) diff = -diff;
                            if (diff > outlier_thresh_dtu)
                                accept = 0;  /* outlier → reject */
                        }
                        /* else: first valid sample for this anchor, always accept */

                        if (accept)
                        {
                            calib_sum[i] += raw_bias;
                            calib_cnt[i]++;
                        }
                    }

                    /* Use calibrated or default DLY_RS for this slave */
                    int64_t dly_rs_val = (i < DL_MAX_ANCHORS) ? dly_rs[i] : DLY_RS_DEFAULT;
                    int64_t tdoa_comp = raw_bias - dly_rs_val;
                    toa_dtu[i] = ref + (uint64_t)tdoa_comp;
                }
            }

            /* Update calibration state (once per cycle, after all slaves).
             * Completes when ALL in-range slaves (rank < num_anchors)
             * have enough accepted samples, or times out. */
            if (!calib_done)
            {
                int all_done;
                calib_total++;
                all_done = 1;
                { int _r;
                  for (_r = 1; _r < (int)num_anchors && _r < DL_MAX_ANCHORS; _r++)
                  {
                      if (toa_dtu[_r] != 0 && calib_cnt[_r] < CALIB_SAMPLES)
                          { all_done = 0; break; }
                  }
                }
                if (all_done)
                {
                    /* Enough good samples — freeze all per-slave DLY_RS */
                    { int _r;
                      for (_r = 1; _r < (int)num_anchors && _r < DL_MAX_ANCHORS; _r++)
                      {
                          if (calib_cnt[_r] > 0)
                              dly_rs[_r] = calib_sum[_r] / calib_cnt[_r];
                      }
                    }
                    calib_done = 1;
                    /* One-shot: report calibrated DLY_RS (meters).  Reveals
                     * the true RX→TX latency the tag measured — if this is
                     * far from ~74m the DLY_RS_DEFAULT guess was wrong. */
                    { char _dl[64]; int _dp = 0, _di;
                      _dl[_dp++]='D'; _dl[_dp++]='L'; _dl[_dp++]=':';
                      for (_di = 1; _di < (int)num_anchors && _di < 4; _di++) {
                          if (_di > 1) _dl[_dp++] = ',';
                          _dp += uart_print_f32(&_dl[_dp], (float)dly_rs[_di] / 213.0f, 1);
                      }
                      _dl[_dp++]='\r'; _dl[_dp++]='\n'; _dl[_dp]='\0';
                      test_run_info((unsigned char *)_dl); }
                }
                else if (calib_total >= CALIB_MAX_CYCLES)
                {
                    /* Timeout — too many outliers, keep defaults */
                    calib_done = 1;
                    /* One-shot: calibration timed out (all samples rejected). */
                    test_run_info((unsigned char *)"DL:timeout\r\n");
                }
            }
        }

#if TAG_DIAG
        /* CFO + raw_bias diagnostic: prints tag↔slave clock offset (raw int16,
         * ÷2^26 = fractional, ×1e6 = ppm) and the raw slot-compensated bias
         * (meters) for each slave, for the first 80 cycles (~4s @20Hz).  Used
         * to confirm whether a per-slave crystal CFO is the source of the
         * stable far-field TDOA bias. */
        {
            static int _diag_cnt = 0;
            if (_diag_cnt < 80) {
                _diag_cnt++;
                char _d[96]; int _p = 0, _r;
                _d[_p++]='C'; _d[_p++]='F'; _d[_p++]=':';
                for (_r = 1; _r < (int)num_anchors && _r < 4; _r++) {
                    if (_r > 1) _d[_p++] = ',';
                    _p += uart_print_f32(&_d[_p], (float)cfo_raw[_r], 0);
                }
                _d[_p++]=' '; _d[_p++]='R'; _d[_p++]='B'; _d[_p++]=':';
                for (_r = 1; _r < (int)num_anchors && _r < 4; _r++) {
                    if (_r > 1) _d[_p++] = ',';
                    _p += uart_print_f32(&_d[_p], (float)raw_bias_dtu[_r] / 213.0f, 1);
                }
                _d[_p++]='\r'; _d[_p++]='\n'; _d[_p]='\0';
                test_run_info((unsigned char *)_d);
            }
        }
#endif

        /*=== Solve and output ===*/
        {
            float x_cm = 0.0f, y_cm = 0.0f, z_cm = 0.0f, rms = 0.0f;
            int ret = 0;
            int16_t  tx_xi = 0, tx_yi = 0, tx_zi = 0;
            uint16_t tx_qi = 0xFFFFu;

            /*=== 2D TDOA Gauss-Newton solve with leave-one-out outlier rejection ===*/
            {
                float tdoa_cm[DL_MAX_ANCHORS];
                int _si, _k, _attempt;
                static float last_x = 0, last_y = 0;
                static int   has_last = 0;

                /* Extract TDOA measurements (cm, referenced to anchor 0).
                 * Build a validity mask: a slave whose blink was NOT received
                 * this cycle has toa_dtu[_si]==0 and must be excluded, or the
                 * solver reads it as a bogus huge negative TDOA. */
                uint8_t valid[DL_MAX_ANCHORS];
                int n_valid = 0;
                valid[0] = 1;  /* reference anchor (beacon) always present */
                for (_si = 1; _si < (int)num_anchors && _si < DL_MAX_ANCHORS; _si++)
                {
                    valid[_si] = (toa_dtu[_si] != 0);
                    if (valid[_si]) n_valid++;
                    int64_t diff = (int64_t)(toa_dtu[_si] - toa_dtu[0]);
                    tdoa_cm[_si] = (float)diff * TDOA_CM_PER_DTU;
                }

#if TAG_DIAG
                /* Compensated TDOA (meters) — small (≈±3m) means the tag is
                 * inside the anchor hull; tens of meters means either the
                 * anchor coords are wrong or the DLY_RS/slot compensation is
                 * grossly off. */
                {
                    char _td[64]; int _tp = 0, _ti;
                    _td[_tp++] = 'T'; _td[_tp++] = 'D'; _td[_tp++] = ':';
                    for (_ti = 1; _ti < (int)num_anchors && _ti < 4; _ti++) {
                        if (_ti > 1) _td[_tp++] = ',';
                        _tp += uart_print_f32(&_td[_tp], tdoa_cm[_ti] / 100.0f, 2);
                    }
                    _td[_tp++] = '\r'; _td[_tp++] = '\n'; _td[_tp] = '\0';
                    test_run_info((unsigned char *)_td);
                }
#endif

                /* Try up to 2 initial guesses: (1) last good pos, (2) centroid.
                 * Skip the solve entirely unless ≥2 valid TDOA (reference + 2
                 * slaves) are available — a 2D fix needs 2 independent range
                 * differences. */
                ret = -1;
                if (n_valid >= 2)
                {
                for (_attempt = 0; _attempt < 2; _attempt++)
                {
                    float gx, gy;

                    if (_attempt == 0 && has_last)
                    {
                        gx = last_x;
                        gy = last_y;
                    }
                    else
                    {
                        float _sx = 0, _sy = 0;
                        for (_k = 0; _k < (int)num_anchors; _k++)
                        {
                            _sx += (float)anchor_table[_k].x_cm;
                            _sy += (float)anchor_table[_k].y_cm;
                        }
                        gx = _sx / (float)num_anchors;
                        gy = _sy / (float)num_anchors;
                    }

                    ret = solve_2d_gn(anchor_table, tdoa_cm, valid, num_anchors, -1,
                                      gx, gy, &x_cm, &y_cm, &rms);
                    if (ret != 0)
                        continue;   /* retry with centroid init */

                    /* Outlier rejection: a single corrupted blink (multipath /
                     * missed-adjacent) can dominate the residual.  If the all-
                     * anchor RMS is poor and we have ≥3 valid TDOA, try leaving
                     * each slave out in turn and keep the lowest-RMS solution. */
                    if (n_valid >= 3 && rms > OUTLIER_RMS_CM)
                    {
                        float bx = x_cm, by = y_cm, brms = rms;
                        for (_k = 1; _k < (int)num_anchors; _k++)
                        {
                            float tx, ty, trms;
                            if (!valid[_k]) continue;
                            if (solve_2d_gn(anchor_table, tdoa_cm, valid, num_anchors, _k,
                                            gx, gy, &tx, &ty, &trms) == 0 && trms < brms)
                            {
                                bx = tx; by = ty; brms = trms;
                            }
                        }
                        if (brms < rms) { x_cm = bx; y_cm = by; rms = brms; }
                    }

                    last_x = x_cm; last_y = y_cm; has_last = 1;
                    break;
                }
                }
                else
                {
                    ret = -2;   /* <2 valid anchors: no TDOA fix possible */
                }
                z_cm = 0;
            }

#if 0
        /* Debug: print compensated TOA diffs + ret code */
        { char _dd[80]; int _dp = 0;
          _dd[_dp++]='D'; _dd[_dp++]=',';
          for(i=1;i<(int)num_anchors&&i<4;i++){
            int64_t _d = (int64_t)(toa_dtu[i]-toa_dtu[0]);
            int32_t _c = (int32_t)(_d / 213);
            if(_c<0){_dd[_dp++]='-';_c=-_c;} else _dd[_dp++]='+';
            _dd[_dp++]=hex[(_c>>12)&0xF];_dd[_dp++]=hex[(_c>>8)&0xF];
            _dd[_dp++]=hex[(_c>>4)&0xF];_dd[_dp++]=hex[_c&0xF];_dd[_dp++]=',';
          }
          _dd[_dp++]='R';_dd[_dp++]='=';
          _dd[_dp++]=hex[ret & 0xF];_dd[_dp++]=',';
          { union { float f; uint32_t u; } _fu; _fu.f = x_cm;
            int _j; for(_j=28;_j>=0;_j-=4) _dd[_dp++]=hex[(_fu.u>>_j)&0xF]; }
          _dd[_dp++]='\r';_dd[_dp++]='\n';_dd[_dp]='\0';
          test_run_info((unsigned char *)_dd); }
#endif

        if (ret == 0)
            {
                float kf_x, kf_y;

                /* Kalman filter: raw TDOA → smoothed position estimate.
                 * dt = 0.5s (500ms beacon cycle).  Measurement noise
                 * scales with solver RMS residual. */
                kalman_update_2d(x_cm, y_cm, (float)DL_CYCLE_PERIOD_MS / 1000.0f, rms, &kf_x, &kf_y);

#if TAG_DEBUG_OUTPUT
                /* Real 20Hz: the TDOA cycle is now 50ms, so one P line per
                 * cycle is already 20Hz — no repetition needed. */
                {
                    char out[80]; int pos = 0;
                    out[pos++] = 'P'; out[pos++] = ',';
                    out[pos++] = hex[(frame_seq >> 4) & 0xF];
                    out[pos++] = hex[ frame_seq       & 0xF];
                    out[pos++] = ','; out[pos++] = 'X'; out[pos++] = '=';
                    pos += uart_print_f32(&out[pos], kf_x / 100.0f, 2);
                    out[pos++] = ','; out[pos++] = 'Y'; out[pos++] = '=';
                    pos += uart_print_f32(&out[pos], kf_y / 100.0f, 2);
                    out[pos++] = ','; out[pos++] = 'Z'; out[pos++] = '=';
                    pos += uart_print_f32(&out[pos], z_cm / 100.0f, 2);
                    out[pos++] = ','; out[pos++] = 'Q'; out[pos++] = '=';
                    pos += uart_print_f32(&out[pos], rms / 100.0f, 2);
                    out[pos] = '\0';
                    test_run_info((unsigned char *)out);
                }
#else  /* !TAG_DEBUG_OUTPUT — robot protocol frame, 47B/cycle */
                {
                    tx_tag_pos_hex(bcn_tick, frame_seq,
                                   kf_x, kf_y, z_cm, rms, 1);
                }
#endif /* TAG_DEBUG_OUTPUT */
                tx_xi = (int16_t)kf_x; tx_yi = (int16_t)kf_y; tx_zi = (int16_t)z_cm;
                tx_qi = (uint16_t)(rms * 10.0f);
            }
            else
            {
                /* Coast on solver failure: reuse the last filtered position
                 * instead of emitting zeros/ERR.  Keeps the reported position
                 * stable during brief geometry-induced singularities. */
                if (kf_init_done) {
                    tx_xi = (int16_t)kf_pos[0];
                    tx_yi = (int16_t)kf_pos[1];
                    tx_zi = 0;
                    tx_qi = 0xFFFFu;   /* stale / low-confidence marker */
                }
#if TAG_DEBUG_OUTPUT
                test_run_info((unsigned char *)(ret == -2 ? "ERR ANCHORS" : "ERR DIVERGE"));
#else
                tx_tag_pos_hex(0, frame_seq,
                               kf_init_done ? kf_pos[0] : 0.0f,
                               kf_init_done ? kf_pos[1] : 0.0f,
                               0.0f, 255.0f, kf_init_done ? 1 : 0);
#endif
            }

            /* Phase 5: TX report to gateway (always — ensures tag shows online) */
            {
                uint8_t pos_report[DL_POS_REPORT_LEN];
                pos_report[0] = 0x41; pos_report[1] = 0x88;
                pos_report[2] = FW_VERSION_BYTE;  /* SEQ byte carries firmware version */
                pos_report[3] = 0xCA; pos_report[4] = 0xDE;
                pos_report[5] = 0xFF; pos_report[6] = 0xFF;  /* broadcast */
                pos_report[7] = (DL_TAG_ID >> 0) & 0xFF;
                pos_report[8] = (DL_TAG_ID >> 8) & 0xFF;
                pos_report[9] = FUNC_DL_POS_REPORT;
                pos_report[10] = (uint8_t)(tx_xi & 0xFF);
                pos_report[11] = (uint8_t)((tx_xi >> 8) & 0xFF);
                pos_report[12] = (uint8_t)(tx_yi & 0xFF);
                pos_report[13] = (uint8_t)((tx_yi >> 8) & 0xFF);
                pos_report[14] = (uint8_t)(tx_zi & 0xFF);
                pos_report[15] = (uint8_t)((tx_zi >> 8) & 0xFF);
                pos_report[16] = (uint8_t)(tx_qi & 0xFF);
                pos_report[17] = (uint8_t)((tx_qi >> 8) & 0xFF);
                pos_report[18] = (uint8_t)(FW_VERSION_U16 & 0xFF);
                pos_report[19] = (uint8_t)((FW_VERSION_U16 >> 8) & 0xFF);
                /* Tag uses polling mode (EXTI disabled by main.c).
                 * Disable EXTI during TX to prevent IRQ handler's
                 * dwt_forcetrxoff() from aborting the transmission. */
                port_DisableEXT_IRQ();
                dwt_forcetrxoff();
                port_rf_switch_tx();
                dwt_writetxdata(sizeof(pos_report), pos_report, 0);
                dwt_writetxfctrl(sizeof(pos_report), 0, 0);
                if (dwt_starttx(DWT_START_TX_IMMEDIATE) == DWT_SUCCESS)
                {
                    uint32_t tx_dl = HAL_GetTick() + 10u;
                    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {
                        if (HAL_GetTick() > tx_dl) break;  /* TX timeout */
                    }
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
                }
                port_rf_switch_rx();
                dwt_rxenable(DWT_START_RX_IMMEDIATE);  /* re-arm RX immediately after TX */
                EXTI->PR1 = (1u << 12);
                /* EXTI stays disabled — tag uses polling for RX */
            }

        /*=== Update slot sync for next cycle ===
         *   Master sends beacons every 500ms via DLY_TS.  We snapshotted
         *   portGetTickCnt() at beacon RX; the next beacon is expected
         *   exactly 500ms later.  Even if the solver fails (ERR DIVERGE),
         *   we still have valid phase sync from the received beacon.    ===*/
        g_next_bcn_tick = bcn_tick + (uint32_t)DL_CYCLE_PERIOD_MS;
        g_slot_synced  = 1;
        }
    }
}
#endif /* TEST_FUSION_TAG */
