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
/* tdoa_solve provided by shared/downlink/tdoa_solver.c (compiled separately) */
int tdoa_solve(const dl_anchor_entry_t *anchors,
               const uint64_t *toa_dtu,
               uint8_t num_anchors,
               float *x_cm_out, float *y_cm_out, float *z_cm_out,
               float *rms_out);
#include "gateway_protocol.h"
#include "firmware_version.h"
#include "../gw_ota/gw_ota.h"
#include "clock_sync.h"
#include "deca_device_api.h"

#if defined(TEST_FUSION_TAG)

extern void test_run_info(unsigned char *data);
extern dwt_txconfig_t txconfig_options;
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
    dl_anchor_entry_t  pos;              /* Latest position from TimeSync */
    AnchorClockSync    sync;             /* Kalman clock filter */
    uint32_t           last_seen_ms;
    uint8_t            is_active;
    int16_t            cfo_raw_last;
    float              quality;
    uint32_t           rx_cycle;         /* local fallback cycle counter (80 ms) */
    uint16_t           cycle_id;         /* TDMA cycle id carried in the beacon (P0-4) */
    uint8_t            slot;             /* TDMA slot index (255=unknown) */
    uint64_t           last_rx_ts;       /* Raw RX local timestamp for TDOA */
    uint32_t           clock_reject_count; /* clock updates refused as outliers */
} TagAnchorTracker;

static TagAnchorTracker s_trackers[DL_MAX_TRACKED_ANCHORS];

/* P0-4 diagnostics: how many beacons were paired and how many had to be
 * dropped because they belonged to a different TDMA cycle. */
static uint32_t s_n_pair = 0;             /* beacons used in a TDOA pair      */
static uint32_t s_n_cycle_mismatch = 0;   /* beacons dropped: wrong cycle id  */

/* The Root (cs_level = 0) defines the grid; our rate relative to the Root's
 * clock scales the slot spacing in the slot compensation. */
static uint16_t s_root_rank      = 0;
static double   s_root_rate      = 1.0;    /* (our clock)/(Root's clock)       */
static uint8_t  s_root_rate_valid = 0;

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
 * Pairwise TDOA computation and solve trigger
 *============================================================================*/
static void tag_solve_and_report(void)
{
    int i;
    uint32_t now_ms = HAL_GetTick();

    /* Find fresh active trackers (within FRESH_MS_MAX ms) */
    int n_active = 0;
    int active_idx[DL_MAX_TRACKED_ANCHORS];
    for (i = 0; i < DL_MAX_TRACKED_ANCHORS; i++) {
        if (!s_trackers[i].is_active) continue;
        uint32_t age_ms = (s_trackers[i].last_seen_ms == 0) ? 0
                          : (now_ms - s_trackers[i].last_seen_ms);
        if (age_ms > 500) continue;  /* measurement too stale */
        active_idx[n_active++] = i;
    }
    if (n_active < 3) return;

    /* Choose ref = highest quality + newest
     * Score = quality * 1000 - age_ms (higher is better) */
    int ref = active_idx[0];
    uint32_t ref_age = now_ms - s_trackers[ref].last_seen_ms;
    float ref_score = s_trackers[ref].quality * 1000.0f - (float)ref_age;
    for (i = 1; i < n_active; i++) {
        int idx = active_idx[i];
        uint32_t age = now_ms - s_trackers[idx].last_seen_ms;
        float score = s_trackers[idx].quality * 1000.0f - (float)age;
        if (score > ref_score) {
            ref_score = score;
            ref = idx;
        }
    }

    /* Build packed arrays for tdoa_solve:
     * Slot-based TDOA: toa_equiv = rx - slot × 5ms
     * This normalizes all beacons to slot=0 (Root's TX time).
     * No anchor clock sync needed: Tag's local clock is the common reference.
     *
     * DTU bookkeeping (P0-4): one DW3000 device time unit is 15.65 ps, so
     * 1 µs = 63,898 DTU and one 5 ms TDMA slot = 5000 × 63898 = 319,490,000
     * DTU.  The old constant (192,000) belonged to a 38.4 DTU/µs clock and was
     * 1664× too small: the slot compensation could never cancel the 0..75 ms
     * spread between the slots, every TDOA was rejected and the solver
     * collapsed onto the centroid of the anchors. */
    #define DTU_PER_SLOT  ((int64_t)DL_SLOT_DTU_US * (int64_t)UUS_TO_DWT_TIME)

    /* Acceptance window for a TDOA difference.  Light covers 1 m in 3.3356 ns
     * = 213.20 DTU (1 DTU = 15.65 ps), so ±100 m of path difference covers
     * any indoor deployment.  The dominant error source (a beacon from a
     * different TDMA cycle, 80 ms = 5.1 M DTU) is removed by the cycle-id
     * pairing below — and by this window as a second line of defence: with
     * the old (1000x too large) constant the window was ±100 km and let
     * cross-cycle frames through. */
    #define TDOA_DTU_PER_METER  213LL
    #define TDOA_MAX_DTU        (100LL * TDOA_DTU_PER_METER)   /* 21,300 */

    /* ── Slot spacing is defined in the ROOT's clock, not in ours ──────────
     * The anchors place their beacons on the Root's grid: anchor k transmits
     * at  G + slot_k × 5 ms  where G and the 5 ms are the ROOT's clock.
     * Our own crystal differs from the Root's by up to ±20 ppm, so 5 ms is
     * 5 ms × f in our clock, where f is the rate ratio (our clock)/(Root's
     * clock).  Subtracting the nominal 5 ms instead would leave a per-anchor
     * error of slot × 5 ms × 20 ppm = up to 700 ns ≈ 21 cm at slot 7 — far
     * above the 0.67 ns the 20 cm target needs.  f is measured from the
     * Root's own beacon (its in-frame TX timestamp vs. our RX timestamp),
     * which is exactly the clock_sync factor of the Root's tracker. */
    double slot_rate = 1.0;
    if (s_root_rate_valid) slot_rate = s_root_rate;
    if (!(slot_rate > 0.5 && slot_rate < 2.0)) slot_rate = 1.0;
    const int64_t slot_dtu_eff = (int64_t)((double)DTU_PER_SLOT * slot_rate);

    uint64_t toa_dtu[DL_MAX_TRACKED_ANCHORS];
    dl_anchor_entry_t anchors[DL_MAX_TRACKED_ANCHORS];
    int valid_idx[DL_MAX_TRACKED_ANCHORS];  /* tracker index for each valid anchor */

    /* Select anchors with known slot (skip slot=255) */
    int sel_idx[DL_MAX_TRACKED_ANCHORS];
    int n_sel = 0;
    for (i = 0; i < n_active; i++) {
        int idx = active_idx[i];
        if (s_trackers[idx].slot != 255) {
            sel_idx[n_sel++] = idx;
        }
    }

    /* ── P0-4: only beacons of the SAME TDMA cycle may be paired ─────────
     * The slot-compensated TDOA
     *     toa = rx_local - slot × 5 ms
     * cancels the cycle origin only when every beacon belongs to the same
     * cycle — one cycle of skew is 80 ms = 5.1 M DTU and would poison every
     * pair it enters.  Find the cycle most of the visible anchors are
     * currently in and drop the rest. */
    uint16_t best_cycle_sel = 0, best_cycle_all = 0;
    int best_cnt_sel = 0, best_cnt_all = 0, i2, j2, k2;
    for (i2 = 0; i2 < n_sel; i2++) {
        uint16_t c = s_trackers[sel_idx[i2]].cycle_id;
        int cnt = 0;
        for (j2 = 0; j2 < n_sel; j2++)
            if (s_trackers[sel_idx[j2]].cycle_id == c) cnt++;
        if (cnt > best_cnt_sel) { best_cnt_sel = cnt; best_cycle_sel = c; }
    }
    for (i2 = 0; i2 < n_active; i2++) {
        uint16_t c = s_trackers[active_idx[i2]].cycle_id;
        int cnt = 0;
        for (j2 = 0; j2 < n_active; j2++)
            if (s_trackers[active_idx[j2]].cycle_id == c) cnt++;
        if (cnt > best_cnt_all) { best_cnt_all = cnt; best_cycle_all = c; }
    }

    /* Filter the slot-known set down to the dominant cycle */
    k2 = 0;
    for (i2 = 0; i2 < n_sel; i2++) {
        int idx = sel_idx[i2];
        if (s_trackers[idx].cycle_id == best_cycle_sel) {
            sel_idx[k2++] = idx;
        } else {
            s_n_cycle_mismatch++;
            { char dbg[64];
              sprintf(dbg, "TDOA_CYCLE_MISMATCH R=%u cyc=%u!=cyc=%u\r\n",
                  s_trackers[idx].rank, s_trackers[idx].cycle_id, best_cycle_sel);
              test_run_info((unsigned char *)dbg); }
        }
    }
    n_sel = k2;

    int ref_idx;
    int n_valid;
    bool use_slot_compensation = (n_sel >= 3);

    if (use_slot_compensation) {
        /* ── Slot-compensated TDOA ────────────────────────────────────────
         * toa_equiv = rx - slot × 5ms (normalizes all to slot=0)
         * Works when anchors are in Root's TDMA table. */

        /* Select ref = lowest slot number (Root = slot 0) */
        ref_idx = sel_idx[0];
        for (i = 1; i < n_sel; i++) {
            if (s_trackers[sel_idx[i]].slot < s_trackers[ref_idx].slot) {
                ref_idx = sel_idx[i];
            }
        }

        uint8_t ref_slot = s_trackers[ref_idx].slot;
        toa_dtu[0] = (uint64_t)((int64_t)s_trackers[ref_idx].last_rx_ts
                                - (int64_t)ref_slot * slot_dtu_eff);
        anchors[0] = s_trackers[ref_idx].pos;
        valid_idx[0] = ref_idx;

        n_valid = 1;
        for (i = 0; i < n_sel; i++) {
            int idx = sel_idx[i];
            if (idx == ref_idx) continue;

            uint8_t k_slot = s_trackers[idx].slot;
            uint64_t rx_k = s_trackers[idx].last_rx_ts;
            int64_t toa_equiv = (int64_t)rx_k - (int64_t)k_slot * slot_dtu_eff;
            int64_t tdoa_dtu = toa_equiv - (int64_t)toa_dtu[0];

            if (tdoa_dtu > TDOA_MAX_DTU || tdoa_dtu < -TDOA_MAX_DTU) continue;
            s_n_pair++;

            toa_dtu[n_valid] = (uint64_t)toa_equiv;
            anchors[n_valid] = s_trackers[idx].pos;
            valid_idx[n_valid] = idx;
            n_valid++;
        }
    } else {
        /* ── Fallback: raw RX TDOA (no slot compensation) ─────────────────
         * The anchors have no known slot (slot = 255 — the Root never
         * confirmed them, or they are in ALOHA fallback mode), so the slot
         * offset cannot be subtracted.  The raw difference is only valid
         * within ONE cycle, so the same-cycle filter is applied here too. */
        if (best_cnt_all < 3) {
            /* Fewer than three anchors in one cycle — a solve would be
             * geometrically degenerate, so report and wait for the next
             * cycle instead of mixing cycles. */
            { char dbg[48];
              sprintf(dbg, "TDOA_CYCLE_WAIT n=%d\r\n", best_cnt_all);
              test_run_info((unsigned char *)dbg); }
            return;
        }

        ref_idx = -1;
        for (i = 0; i < n_active; i++) {
            int idx = active_idx[i];
            if (s_trackers[idx].cycle_id == best_cycle_all
                && s_trackers[idx].slot == 255) {
                ref_idx = idx;
                break;
            }
        }
        if (ref_idx < 0) {
            /* No off-grid anchor in the dominant cycle — the raw fallback
             * would mix a slot-compensated anchor into the difference. */
            { char dbg[48];
              sprintf(dbg, "TDOA_CYCLE_WAIT n=%d\r\n", best_cnt_all);
              test_run_info((unsigned char *)dbg); }
            return;
        }
        toa_dtu[0] = s_trackers[ref_idx].last_rx_ts;
        anchors[0] = s_trackers[ref_idx].pos;
        valid_idx[0] = ref_idx;

        n_valid = 1;
        for (i = 0; i < n_active; i++) {
            int idx = active_idx[i];
            if (idx == ref_idx) continue;
            if (s_trackers[idx].cycle_id != best_cycle_all) {
                s_n_cycle_mismatch++;
                continue;
            }

            int64_t tdoa_dtu = (int64_t)(s_trackers[idx].last_rx_ts - toa_dtu[0]);
            if (tdoa_dtu > TDOA_MAX_DTU || tdoa_dtu < -TDOA_MAX_DTU) continue;
            s_n_pair++;

            toa_dtu[n_valid] = s_trackers[idx].last_rx_ts;
            anchors[n_valid] = s_trackers[idx].pos;
            valid_idx[n_valid] = idx;
            n_valid++;
        }
    }

    /* P1-7: the diagnostics are printed BEFORE the "not enough anchors"
     * early exit — a solve that is silently dropped looks exactly like a
     * solve that never ran, and there is no way to tell the two apart on
     * the bench. */
    /* Debug: print raw TDOA values */
    { char dbg[128]; int pos = 0;
      pos += sprintf(dbg + pos, "TDOA_DBG REF=R%d TOA=%llu",
          s_trackers[ref_idx].rank, (unsigned long long)toa_dtu[0]);
      for (i = 1; i < n_valid; i++) {
          int idx = valid_idx[i];
          pos += sprintf(dbg + pos, " R%d=%lld",
              anchors[i].id,
              (long long)((int64_t)toa_dtu[i] - (int64_t)toa_dtu[0]));
      }
      pos += sprintf(dbg + pos, "\r\n");
      test_run_info((unsigned char *)dbg); }

    /* P0-4 summary: how many pairs were formed and how many beacons had to
     * be dropped because they belonged to a different TDMA cycle.  A rising
     * mismatch counter means the anchors are not on one common grid. */
    { char dbg[96]; int pos = 0;
      pos += sprintf(dbg + pos, "TSUM n=%d pair=%u mism=%u cyc=%u\r\n",
          n_valid, (unsigned)s_n_pair, (unsigned)s_n_cycle_mismatch,
          (unsigned)s_trackers[ref_idx].cycle_id);
      test_run_info((unsigned char *)dbg); }

    if (n_valid < 3) {
        { char dbg[64];
          sprintf(dbg, "TSUM n=%d SKIP need 3 anchors\r\n", n_valid);
          test_run_info((unsigned char *)dbg); }
        return;
    }

    float x_cm = 0, y_cm = 0, z_cm = 0, rms = 0;
    int ret = tdoa_solve(anchors, toa_dtu, (uint8_t)n_valid,
                         &x_cm, &y_cm, &z_cm, &rms);
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
        p = sprintf(tmp, "%.2f", z_cm / 100.0f);
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
        int k;
        for (k = 20; k < 47; k++) tx[k] = 0;
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
            s_trackers[i].slot = 255;      /* unknown until beacon received */
            s_trackers[i].last_rx_ts = 0;
            s_trackers[i].rx_cycle = 0;
            s_trackers[i].cycle_id = 0;    /* P0-4: TDMA cycle id (0 = unknown) */
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
    s_trackers[worst].slot = 255;
    s_trackers[worst].last_rx_ts = 0;
    s_trackers[worst].rx_cycle = 0;
    s_trackers[worst].cycle_id = 0;      /* P0-4 */
    clock_sync_init(&s_trackers[worst].sync, 1.0, 0.0, 0.0, 0.0, 0.0);
    return worst;
}

static void tag_rx_timesync(uint8_t *rx_buf, uint32_t frame_len, uint64_t rx_hw_ts)
{
    if (frame_len < DL_TIMESYNC_LEN) return;

    uint16_t src_rank = (uint16_t)rx_buf[7] | ((uint16_t)rx_buf[8] << 8);
    if (src_rank == 0) return;

    uint64_t tx_remote = unpack_40bit_ts(&rx_buf[DL_TS_TX_TS_IDX]);
    uint64_t rx_local  = rx_hw_ts;

    /* P0-2 (tag side): only a beacon whose TX instant is known exactly may
     * enter the TDOA solve.  An imprecise frame (bootstrap / ALOHA) was
     * transmitted at an unknown instant — off the grid by up to 30 µs — so
     * its slot compensation is meaningless. */
    uint8_t ts_flags   = (frame_len >= DL_TS_FLAGS_IDX + 1)
                       ? rx_buf[DL_TS_FLAGS_IDX] : 0;
    uint8_t ts_precise = (ts_flags & DL_TS_FLAG_TX_TS_PRECISE) ? 1 : 0;
    if (!ts_precise) {
        static uint32_t s_n_imprecise;
        if ((s_n_imprecise++ % 20) == 0) {
            char dbg[48];
            sprintf(dbg, "TS_IMPRECISE R=%u dropped\r\n", src_rank);
            test_run_info((unsigned char *)dbg);
        }
        return;
    }

    /* Diagnostic: print RX timestamp diff between consecutive frames.
     * If all anchors report identical rx_local, RX timestamp latch is broken. */
    {
        static uint64_t s_last_rx;
        char dbg[96];
        sprintf(dbg, "RX_TS R=%u tx=%llu rx=%llu diff=%lld\r\n",
            src_rank,
            (unsigned long long)tx_remote,
            (unsigned long long)rx_local,
            (long long)(rx_local - s_last_rx));
        test_run_info((unsigned char *)dbg);
        s_last_rx = rx_local;
    }

    int tracker_slot = find_tracker_slot(src_rank);
    if (tracker_slot < 0) tracker_slot = alloc_tracker_slot(src_rank);

    /* Track which TDMA cycle this RX belongs to.  The authoritative value is
     * the cycle id carried in the beacon (P0-4) — every anchor of the same
     * grid cycle stamps the same id, so the Tag can pair beacons that truly
     * belong to the same cycle.  The millisecond counter is only a fallback
     * for beacons that do not carry an id. */
    s_trackers[tracker_slot].rx_cycle = HAL_GetTick() / DL_TIMESYNC_PERIOD_MS;
    /* Record raw RX timestamp for TDOA */
    s_trackers[tracker_slot].last_rx_ts = rx_local;
    /* Parse TDMA slot index from beacon */
    s_trackers[tracker_slot].slot = rx_buf[DL_TS_SLOT_IDX];
    /* P0-4: TDMA cycle id of this beacon */
    s_trackers[tracker_slot].cycle_id = (uint16_t)(rx_buf[DL_TS_CYCLE_IDX]
                                | ((uint16_t)rx_buf[DL_TS_CYCLE_IDX + 1] << 8));

    int16_t cfo_raw = dwt_readclockoffset();

    /* Diagnostic: show the raw CFO value and the rate it implies.
     * The DW3000 register is s[-15:-26] fixed point → the divisor is 2^26
     * (67,108,864), NOT 2^16.  With the old 65536 divisor the printed
     * "factor" was off by a factor of 1024 and hid the real behaviour. */
    {
        static int16_t s_last_cfo;
        char dbg2[64];
        sprintf(dbg2, "CFO cfo_raw=%d factor_meas=%.9f\r\n",
            cfo_raw, 1.0 + (double)cfo_raw / 67108864.0);
        if (cfo_raw != s_last_cfo || s_last_cfo == 0) {
            test_run_info((unsigned char *)dbg2);
            s_last_cfo = cfo_raw;
        }
    }

    /* Force-accept first 5 updates so Kalman filter can converge */
    bool force_accept = (s_trackers[tracker_slot].sync.update_count < 5);

    bool ok = clock_sync_update(&s_trackers[tracker_slot].sync, cfo_raw,
                                rx_local, tx_remote, HAL_GetTick());
    /* P0-4: a refused clock update (outlier) does NOT invalidate the
     * beacon — only the clock model.  Returning here used to make a live
     * anchor disappear from the solve for one bad frame: no position
     * update, no last_seen, no is_active → the TDOA solver lost an anchor
     * and the whole position estimate collapsed onto the remaining ones.
     * The update is simply not applied; the anchor keeps its place and is
     * de-weighted through the quality instead. */
    if (!ok && !force_accept) {
        s_trackers[tracker_slot].clock_reject_count++;
    }

    /* Debug: print raw RX data + Kalman state for first 10 frames per tracker */
    {
        static uint8_t s_print_count[DL_MAX_TRACKED_ANCHORS];
        if (s_print_count[tracker_slot] < 10) {
            s_print_count[tracker_slot]++;
            AnchorClockSync *s = &s_trackers[tracker_slot].sync;
            char dbg[96];
            int len = sprintf(dbg, "RX_RAW R=%u tx=%llu rx=%llu f=%.9f fppm=%.1f ofs=%.0f q=%.0f\r\n",
                src_rank,
                (unsigned long long)tx_remote,
                (unsigned long long)rx_local,
                s->factor,
                (double)(s->factor - 1.0) * 1e6,
                s->offset_dtu,
                s->quality);
            test_run_info((unsigned char *)dbg);
        }
    }

    /* Update tracker position from Timesync frame (coordinates appended after standard TS payload) */
    int16_t x = 0, y = 0, z = 0;
    if (frame_len >= DL_TS_COORD_IDX + 6) {
        x = (int16_t)((uint16_t)rx_buf[DL_TS_COORD_IDX + 0] | ((uint16_t)rx_buf[DL_TS_COORD_IDX + 1] << 8));
        y = (int16_t)((uint16_t)rx_buf[DL_TS_COORD_IDX + 2] | ((uint16_t)rx_buf[DL_TS_COORD_IDX + 3] << 8));
        z = (int16_t)((uint16_t)rx_buf[DL_TS_COORD_IDX + 4] | ((uint16_t)rx_buf[DL_TS_COORD_IDX + 5] << 8));
        s_trackers[tracker_slot].pos.id = src_rank;
        s_trackers[tracker_slot].pos.x_cm = x;
        s_trackers[tracker_slot].pos.y_cm = y;
        s_trackers[tracker_slot].pos.z_cm = z;

        /* Debug: dump anchor coordinates, rate-limited to 1 per 5 s per anchor */
        {
            static uint32_t s_last_coord_print[DL_MAX_TRACKED_ANCHORS];
            uint32_t now = HAL_GetTick();
            if ((now - s_last_coord_print[tracker_slot]) > 5000) {
                s_last_coord_print[tracker_slot] = now;
                char dbg[64]; sprintf(dbg, "ANCHOR_COORD R=%u X=%d Y=%d Z=%d\r\n", src_rank, x, y, z);
                test_run_info((unsigned char *)dbg);
            }
        }
    }

    /* Update tracker metadata */
    s_trackers[tracker_slot].cfo_raw_last      = cfo_raw;
    s_trackers[tracker_slot].quality           = s_trackers[tracker_slot].sync.quality;
    s_trackers[tracker_slot].last_seen_ms      = HAL_GetTick();
    s_trackers[tracker_slot].is_active         = 1;

    /* P0-4: de-weight an anchor whose clock update was refused, instead of
     * dropping it.  The quality feeds the reference-anchor selection, so a
     * single outlier lowers the anchor's influence without removing it. */
    if (!ok && !force_accept) {
        s_trackers[tracker_slot].quality *= 0.5f;
    }

    /* The Root (cs_level = 0) owns the grid the other anchors are placed on,
     * so its clock is the reference for the slot spacing.  Our clock_sync
     * factor towards the Root is the rate ratio (our clock)/(Root's clock)
     * that the slot compensation needs (see the TDOA block). */
    if (frame_len >= DL_TS_LEVEL_IDX + 1 && rx_buf[DL_TS_LEVEL_IDX] == 0) {
        s_root_rank = src_rank;
        s_root_rate = s_trackers[tracker_slot].sync.factor;
        s_root_rate_valid = (s_trackers[tracker_slot].sync.update_count >= 8)
                            ? 1 : 0;
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
    test_run_info((unsigned char *)"INIT OK\r\n");
    reset_DWIC();
    Sleep(2);
    port_rf_switch_init();  /* Initialize PA6/PA7 for RF switch */
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
        uint32_t last_solve_ms = rx_start;
        while ((HAL_GetTick() - rx_start) < DL_RX_WINDOW_MS) {
            Sleep(1);
            uint32_t now = HAL_GetTick();

            /* 20 Hz solve: fixed 50 ms interval (checked every iteration) */
            if ((int32_t)(now - last_solve_ms) >= 50) {
                last_solve_ms += 50;
                tag_solve_and_report();
            }

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
            /* Read RX HW timestamp BEFORE re-enabling RX, while RX_TIME
             * latch still holds the timestamp for the received frame. */
            uint64_t rx_hw_ts = get_rx_timestamp_u64();
            if (!match_header(rx_buf, match_timesync, 10, 1, 0, 1)) {
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
                continue;
            }
            if (rx_buf[9] == FUNC_DOWNLINK_BEACON) {
                tag_rx_timesync(rx_buf, frame_len, rx_hw_ts);
            }
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
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
            int i;
            for (i = 0; i < DL_MAX_TRACKED_ANCHORS; i++) {
                if (!s_trackers[i].is_active) continue;
                const AnchorClockSync *s = &s_trackers[i].sync;
                uint32_t age_ms = (s_trackers[i].last_seen_ms == 0) ? 0
                            : (HAL_GetTick() - s_trackers[i].last_seen_ms);
                pos += sprintf(out + pos, " R%d:F=%+07ld:Q=%02u:A=%04u",
                    s_trackers[i].rank,
                    (long)((s->factor - 1.0) * 1e6),
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