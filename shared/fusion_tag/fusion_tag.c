/*! ----------------------------------------------------------------------------
 *  @file    fusion_tag.c
 *  @brief   BD8NCF-style tag — multi-anchor tracking + pairwise TDOA solve
 *
 *  Each cycle:
 *    1. Continuous RX for ~95 ms — track all visible anchors' TimeSync
 *    2. Per-anchor Kalman clock-sync state (clock_sync.h)
 *    3. Every ~50 ms: collect active trackers, compute pairwise TDOAs,
 *       solve 2D position with Gauss-Newton
 *    4. UART output: P,seq,X=...,Y=...,Z=...,Q=...
 *    5. TX 22-byte Position Report to gateway for +POS display
 *
 *  No TDMA, no slot sync, no DLY_RS calibration.
 *
 * @attention
 * Copyright 2015-2026 (c) UWB project authors.
 * --------------------------------------------------------------------------- */
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
#include "clock_sync.h"

#if defined(TEST_FUSION_TAG)

extern void test_run_info(unsigned char *data);
#if !TAG_DEBUG_OUTPUT
extern UART_HandleTypeDef huart2;
#endif

#define APP_NAME "TAG V1.0"   /* BD8NCF migration */

/* Tag serial output mode: 1=debug text, 0=22B binary protocol */
#define TAG_DEBUG_OUTPUT  1

/* Diagnostics */
#define TAG_DIAG  0

/*============================================================================
 * Hardware config (same as before)
 *============================================================================*/
static dwt_config_t config = {
    UWB_CHANNEL, DWT_PLEN_128, DWT_PAC8,
    9, 9, 1, DWT_BR_6M8, DWT_PHRMODE_STD, DWT_PHRRATE_STD,
    (129 + 8 - 8), DWT_STS_MODE_OFF, DWT_STS_LEN_64, DWT_PDOA_M0
};

#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define DL_TAG_ID  0x0010u

/* Frame match */
static const uint8_t match_timesync[] = {0x41, 0x88, 0,
                                         0xCA, 0xDE, 0xFF, 0xFF,
                                         0,    0,    0xEE};

/*============================================================================
 * Per-anchor tracking state
 *============================================================================*/
typedef struct {
    uint16_t           rank;             /* 1..N-1 (rank 0 reserved for master) */
    uint8_t            cs_level;         /* 0 = root, 1+ = child */
    uint64_t           parent_eui64;
    dl_anchor_entry_t  pos;              /* Latest position from TimeSync */
    AnchorClockSync    sync;             /* Kalman clock filter */
    uint64_t           last_rx_ts_local; /* tag's local RX timestamp */
    uint64_t           last_tx_ts_remote;/* anchor's TX timestamp from TimeSync */
    uint32_t           last_seen_ms;
    uint8_t            is_active;
    int16_t            cfo_raw_last;
    float              quality;
} TagAnchorTracker;

static TagAnchorTracker s_trackers[DL_MAX_TRACKED_ANCHORS];

/*============================================================================
 * Output and protocol helpers
 *============================================================================*/
static const char hex[] = "0123456789ABCDEF";

/* match_header helper — compare rx buffer against pattern, masking SN/DST/SRC */
static int match_header(uint8_t *rx, uint8_t *pattern, uint8_t len,
                       uint8_t skip_sn, uint8_t skip_dst, uint8_t skip_src)
{
    uint8_t i;
    for (i = 0; i < len; i++) {
        if (i == 2 && skip_sn) continue;
        if ((i == 5 || i == 6) && skip_dst) continue;
        if ((i == 7 || i == 8) && skip_src) continue;
        if (rx[i] != pattern[i]) return 0;
    }
    return 1;
}

/*============================================================================
 * Solver (kept from original — Gauss-Newton 2D)
 *============================================================================*/
static int solve_2d_gn(const dl_anchor_entry_t *anchors,
                       const float *tdoa_cm,
                       const uint8_t *valid,
                       uint8_t num_anchors, int skip,
                       float x0, float y0,
                       float *x_out, float *y_out, float *rms_out)
{
    int used[DL_MAX_TRACKED_ANCHORS];
    int n_used = 0;
    int i, j, iter;
    float dx[DL_MAX_TRACKED_ANCHORS], dy[DL_MAX_TRACKED_ANCHORS];
    float dist[DL_MAX_TRACKED_ANCHORS];
    float H[2][2], g[2], step[2];
    float b[DL_MAX_TRACKED_ANCHORS];
    float last_resid = 1e30f, det;
    float new_x = x0, new_y = y0;

    for (i = 0; i < num_anchors; i++) {
        if (!valid[i]) continue;
        if (skip >= 0 && i == skip) continue;
        used[n_used++] = i;
    }
    if (n_used < 2) return -2;

    for (iter = 0; iter < 8; iter++) {
        for (i = 0; i < n_used; i++) {
            int a = used[i];
            dx[a] = new_x - anchors[a].x_cm;
            dy[a] = new_y - anchors[a].y_cm;
            float d = sqrtf(dx[a]*dx[a] + dy[a]*dy[a]);
            if (d < 1.0f) d = 1.0f;
            dist[a] = d;
        }
        H[0][0] = H[0][1] = H[1][0] = H[1][1] = 0;
        g[0] = g[1] = 0;
        float resid = 0;
        int idx = 0;
        for (i = 0; i < n_used; i++) {
            int a = used[i];
            for (j = i + 1; j < n_used; j++) {
                int b_idx = used[j];
                float h_a_x = dx[a] / dist[a];
                float h_a_y = dy[a] / dist[a];
                float h_b_x = dx[b_idx] / dist[b_idx];
                float h_b_y = dy[b_idx] / dist[b_idx];
                H[0][0] += h_a_x*h_a_x + h_b_x*h_b_x;
                H[0][1] += h_a_x*h_a_y + h_b_x*h_b_y;
                H[1][1] += h_a_y*h_a_y + h_b_y*h_b_y;
                float t_pred = dist[a] - dist[b_idx];
                float t_obs  = tdoa_cm[a] - tdoa_cm[b_idx];
                float r = t_obs - t_pred;
                b[idx++] = r;
                resid += r * r;
                g[0] += r * (h_a_x - h_b_x);
                g[1] += r * (h_a_y - h_b_y);
            }
        }
        H[1][0] = H[0][1];

        det = H[0][0] * H[1][1] - H[0][1] * H[1][0];
        if (fabsf(det) < 1e-6f) {
            H[0][0] += 1.0f; H[1][1] += 1.0f;
            det = H[0][0] * H[1][1] - H[0][1] * H[1][0];
        }
        step[0] = (H[1][1] * g[0] - H[0][1] * g[1]) / det;
        step[1] = (H[0][0] * g[1] - H[0][1] * g[0]) / det;

        float step_len = sqrtf(step[0]*step[0] + step[1]*step[1]);
        if (step_len < 0.05f) break;
        if (resid > last_resid) {
            step[0] *= 0.5f; step[1] *= 0.5f;
        }
        last_resid = resid;
        new_x += step[0];
        new_y += step[1];
    }

    *x_out = new_x;
    *y_out = new_y;
    *rms_out = sqrtf(last_resid / (float)(n_used * (n_used - 1) / 2));
    return 0;
}

/*============================================================================
 * Pairwise TDOA computation and solve trigger
 *============================================================================*/
static void tag_solve_and_report(void)
{
    /* Find active trackers (rank 0 reserved as reference) */
    int n_active = 0;
    int active_idx[DL_MAX_TRACKED_ANCHORS];
    int i, j;
    for (i = 0; i < DL_MAX_TRACKED_ANCHORS; i++) {
        if (s_trackers[i].is_active) {
            active_idx[n_active++] = i;
        }
    }
    if (n_active < 3) return;

    /* Choose ref = oldest / highest-quality */
    int ref = active_idx[0];
    float best_q = s_trackers[ref].quality;
    for (i = 1; i < n_active; i++) {
        if (s_trackers[active_idx[i]].quality > best_q) {
            best_q = s_trackers[active_idx[i]].quality;
            ref = active_idx[i];
        }
    }

    /* Build packed TDOA array: anchors are packed 0..n_active-1 so
     * solve_2d_gn receives contiguous data (skip=-1). */
    float tdoa_cm[DL_MAX_TRACKED_ANCHORS];
    uint8_t valid_arr[DL_MAX_TRACKED_ANCHORS];
    dl_anchor_entry_t anchors[DL_MAX_TRACKED_ANCHORS];
    for (i = 0; i < DL_MAX_TRACKED_ANCHORS; i++) { tdoa_cm[i] = 0; valid_arr[i] = 0; }

    int n_valid = 0;
    for (i = 0; i < n_active; i++) {
        int idx = active_idx[i];
        /* Pairwise TDOA relative to ref */
        int64_t tdoa_dtu = clock_sync_tdoa_dtu(
            &s_trackers[ref].sync,
            &s_trackers[idx].sync,
            s_trackers[ref].last_rx_ts_local,
            s_trackers[idx].last_rx_ts_local);
        tdoa_cm[n_valid] = (float)tdoa_dtu * TDOA_CM_PER_DTU;
        anchors[n_valid] = s_trackers[idx].pos;
        valid_arr[n_valid] = 1;
        n_valid++;
    }
    if (n_valid < 3) return;

    /* Initial guess: centroid of all anchors */
    float cx = 0, cy = 0;
    for (i = 0; i < n_valid; i++) {
        cx += anchors[i].x_cm; cy += anchors[i].y_cm;
    }
    cx /= (float)n_valid; cy /= (float)n_valid;

    float x_cm = 0, y_cm = 0, rms = 0;
    int ret = solve_2d_gn(anchors, tdoa_cm, valid_arr, n_valid, -1,
                          cx, cy, &x_cm, &y_cm, &rms);
    if (ret != 0) return;

#if TAG_DEBUG_OUTPUT
    /* ASCII debug: P,SEQ,X=m,Y=m,Z=m,Q=qm */
    {
        char out[80]; int pos = 0;
        out[pos++] = 'P'; out[pos++] = ',';
        uint8_t seq = (uint8_t)(HAL_GetTick() & 0xFF);
        out[pos++] = hex[(seq >> 4) & 0xF];
        out[pos++] = hex[seq & 0xF];
        out[pos++] = ','; out[pos++] = 'X'; out[pos++] = '=';
        int p; char tmp[16];
        int k;
        p = sprintf(tmp, "%.2f", x_cm / 100.0f);
        for (k = 0; k < p; k++) out[pos++] = tmp[k];
        out[pos++] = ','; out[pos++] = 'Y'; out[pos++] = '=';
        p = sprintf(tmp, "%.2f", y_cm / 100.0f);
        for (k = 0; k < p; k++) out[pos++] = tmp[k];
        out[pos++] = ','; out[pos++] = 'Z'; out[pos++] = '=';
        p = sprintf(tmp, "%.2f", 0.0f);
        for (k = 0; k < p; k++) out[pos++] = tmp[k];
        out[pos++] = ','; out[pos++] = 'Q'; out[pos++] = '=';
        p = sprintf(tmp, "%.2f", rms / 100.0f);
        for (k = 0; k < p; k++) out[pos++] = tmp[k];
        out[pos++] = '\r'; out[pos++] = '\n'; out[pos] = 0;
        test_run_info((unsigned char *)out);
    }
#else
    /* 47-byte binary protocol via UART2 */
    {
        uint8_t tx[48];
        tx[0] = 0x41; tx[1] = 0x88;
        tx[2] = FW_VERSION_BYTE;
        tx[3] = 0xCA; tx[4] = 0xDE;
        tx[5] = 0xFF; tx[6] = 0xFF;
        tx[7] = DL_TAG_ID & 0xFF; tx[8] = DL_TAG_ID >> 8;
        tx[9] = FUNC_DL_POS_REPORT;
        int16_t xi = (int16_t)x_cm, yi = (int16_t)y_cm, zi = 0;
        uint16_t qi = (uint16_t)(rms / 100.0f * 1000.0f);
        tx[10] = xi & 0xFF; tx[11] = (xi >> 8) & 0xFF;
        tx[12] = yi & 0xFF; tx[13] = (yi >> 8) & 0xFF;
        tx[14] = zi & 0xFF; tx[15] = (zi >> 8) & 0xFF;
        tx[16] = qi & 0xFF; tx[17] = (qi >> 8) & 0xFF;
        tx[18] = FW_VERSION_U16 & 0xFF; tx[19] = (FW_VERSION_U16 >> 8) & 0xFF;
        for (int k = 20; k < 47; k++) tx[k] = 0;
        tx[47] = 0;
        HAL_UART_Transmit(&huart2, tx, 47, 10);
    }
#endif
}

/*============================================================================
 * TimeSync RX handler
 *============================================================================*/
static int find_tracker_slot(uint16_t rank)
{
    int i;
    for (i = 0; i < DL_MAX_TRACKED_ANCHORS; i++) {
        if (s_trackers[i].rank == rank) return i;
    }
    return -1;
}

static int alloc_tracker_slot(uint16_t rank)
{
    int i;
    /* Find empty slot */
    for (i = 0; i < DL_MAX_TRACKED_ANCHORS; i++) {
        if (!s_trackers[i].is_active && s_trackers[i].rank == 0) {
            s_trackers[i].rank = rank;
            clock_sync_init(&s_trackers[i].sync, 1.0, 0.0, 0.0, 0.0, 0.0);
            return i;
        }
    }
    /* Replace lowest-quality */
    int worst = 0;
    float worst_q = s_trackers[0].quality;
    for (i = 1; i < DL_MAX_TRACKED_ANCHORS; i++) {
        if (s_trackers[i].quality < worst_q) {
            worst_q = s_trackers[i].quality;
            worst = i;
        }
    }
    s_trackers[worst].rank = rank;
    clock_sync_init(&s_trackers[worst].sync, 1.0, 0.0, 0.0, 0.0, 0.0);
    return worst;
}

static void tag_rx_timesync(uint8_t *rx_buf, uint32_t frame_len)
{
    if (frame_len < DL_TIMESYNC_LEN) return;

    uint16_t src_rank = (uint16_t)rx_buf[7] | ((uint16_t)rx_buf[8] << 8);
    if (src_rank == 0 || src_rank > DL_MAX_TRACKED_ANCHORS) return;

    uint64_t tx_remote = unpack_40bit_ts(&rx_buf[DL_TS_TX_TS_IDX]);
    uint64_t rx_local  = get_rx_timestamp_u64();
    /* reject obviously bogus timestamps */
    int64_t diff = (int64_t)(rx_local - tx_remote);
    if (diff < 0 || diff > (int64_t)(200 * 1000 * UUS_TO_DWT_TIME)) return;

    int slot = find_tracker_slot(src_rank);
    if (slot < 0) slot = alloc_tracker_slot(src_rank);

    int16_t cfo_raw = dwt_readclockoffset();
    uint32_t dt_ms = s_trackers[slot].last_seen_ms == 0
                       ? 100
                       : HAL_GetTick() - s_trackers[slot].last_seen_ms;

    bool ok = clock_sync_update(&s_trackers[slot].sync, cfo_raw, rx_local, tx_remote, dt_ms);
    s_trackers[slot].last_rx_ts_local  = rx_local;
    s_trackers[slot].last_tx_ts_remote = tx_remote;
    s_trackers[slot].cs_level          = rx_buf[DL_TS_CS_LEVEL_IDX];
    s_trackers[slot].parent_eui64     = 0;  /* TODO unpack */
    s_trackers[slot].cfo_raw_last      = cfo_raw;
    s_trackers[slot].quality           = s_trackers[slot].sync.quality;
    s_trackers[slot].last_seen_ms      = HAL_GetTick();
    s_trackers[slot].is_active         = 1;

    /* Diagnostics: print sync state after each update
     * Format: TRX[R=rank] CFO=±xxxxx F=xxxxxxx O=xxxxxxxx Q=xx N=xx U=xx
     *   F = factor * 1e6, O = offset_dtu (hex), Q = quality*100, U = update_count */
    {
        char out[80]; int pos = 0;
        const AnchorClockSync *s = &s_trackers[slot].sync;
        pos += sprintf(out + pos, "TRX[R=%u]", src_rank);
        pos += sprintf(out + pos, " CFO=%+dd", cfo_raw);
        pos += sprintf(out + pos, " F=%07lu", (unsigned long)(s->factor * 1e6));
        pos += sprintf(out + pos, " O=%08lX", (unsigned long)s->offset_dtu);
        pos += sprintf(out + pos, " Q=%02u", (unsigned int)(s->quality * 100));
        pos += sprintf(out + pos, " N=%02u", s->outlier_count);
        pos += sprintf(out + pos, " U=%02u", s->update_count);
        pos += sprintf(out + pos, " %s\r\n", ok ? "OK" : "OUTL");
        test_run_info((unsigned char *)out);
    }
}

/*============================================================================
 * Tag main loop (continuous RX + periodic solve)
 *============================================================================*/
int fusion_tag(void)
{
    test_run_info((unsigned char *)APP_NAME "\r\n");
    test_run_info((unsigned char *)"IDLE OK\r\n");

    /* Hardware init (deca + port + LED) */
    port_init_dw_chip();
    test_run_info((unsigned char *)"INIT OK\r\n");
    reset_DWIC();
    Sleep(2);
    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
        test_run_info((unsigned char *)"INIT FAIL\r\n");
        return -1;
    }
    dwt_configure(&config);
    dwt_configuretxrf(&txconfig_options);
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_setleds(DWT_LEDS_ENABLE);
    test_run_info((unsigned char *)"CONFIG OK\r\n");
    test_run_info((unsigned char *)"INIT DONE\r\n");

    /* Init tracker slots */
    int i;
    for (i = 0; i < DL_MAX_TRACKED_ANCHORS; i++) {
        s_trackers[i].rank = 0;
        s_trackers[i].is_active = 0;
        s_trackers[i].last_seen_ms = 0;
        s_trackers[i].quality = 0;
        s_trackers[i].sync.initialized = false;
    }

    while (1) {
        /* Phase 1: Continuous RX for ~95 ms */
        dwt_forcetrxoff();
        port_rf_switch_rx();
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        uint32_t rx_start = HAL_GetTick();
        uint32_t solve_ctr = 0;
        while ((HAL_GetTick() - rx_start) < DL_RX_WINDOW_MS) {
            Sleep(1);
            uint32_t status = dwt_read32bitreg(SYS_STATUS_ID);
            if (status & SYS_STATUS_ALL_RX_ERR) {
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
                continue;
            }
            if (!(status & SYS_STATUS_RXFCG_BIT_MASK)) continue;
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
            uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
            if (frame_len == 0 || frame_len > 128) {
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
                continue;
            }
            uint8_t rx_buf[128];
            dwt_readrxdata(rx_buf, frame_len, 0);
            if (!match_header(rx_buf, match_timesync, 10, 1, 0, 1)) {
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
                continue;
            }
            if (rx_buf[9] == FUNC_DOWNLINK_BEACON) {
                tag_rx_timesync(rx_buf, frame_len);
            }
            dwt_rxenable(DWT_START_RX_IMMEDIATE);

            solve_ctr++;
            if (solve_ctr >= 20) {
                solve_ctr = 0;
                tag_solve_and_report();
            }
        }

#ifndef GATEWAY_ROLE
        if (gw_ota_poll()) {
            /* OTA timeout — resume ranging */
        }
#endif

        /* Periodic tracker summary every ~5 s */
        static uint32_t s_summary_ctr;
        if (++s_summary_ctr >= 50) {
            s_summary_ctr = 0;
            char out[128]; int pos = 0;
            pos += sprintf(out + pos, "TSUM");
            for (int i = 0; i < DL_MAX_TRACKED_ANCHORS; i++) {
                if (!s_trackers[i].is_active) continue;
                const AnchorClockSync *s = &s_trackers[i].sync;
                uint32_t age_ms = (s_trackers[i].last_seen_ms == 0) ? 0
                            : (HAL_GetTick() - s_trackers[i].last_seen_ms);
                pos += sprintf(out + pos, " R%d:F=%07lu:Q=%02u:A=%04u",
                    s_trackers[i].rank,
                    (unsigned long)(s->factor * 1e6),
                    (unsigned int)(s->quality * 100),
                    age_ms);
            }
            pos += sprintf(out + pos, "\r\n");
            test_run_info((unsigned char *)out);
        }
    }
    return 0;
}

#endif /* TEST_FUSION_TAG */