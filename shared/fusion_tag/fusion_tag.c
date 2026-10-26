/*! ----------------------------------------------------------------------------
 *  @file    fusion_tag.c
 *  @brief   BD8NCF-style tag — multi-anchor tracking + pairwise TDOA solve
 *
 *  Each super-cycle (100 ms = 2 TDMA rounds):
 *    1. Continuous RX for one super-cycle — track all visible anchors' beacons
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

/* Diagnostic output throttling.
 *
 * The debug UART is a BLOCKING resource: HAL_UART_Transmit() spins until
 * the last byte has left the shift register, so every printed line stalls
 * the RX loop for (bytes x 10 bits) / baud.  With 8 anchors the tag
 * receives 160 beacons/s (8 anchors x 20 rounds/s) and the DW3000 RX
 * buffer is ONE frame deep — a print that lasts longer than the 6.25 ms
 * slot spacing overruns it and the beacon is lost.  At 115200 baud the
 * two per-frame lines (RX_TS + CFO, ~90 bytes) blocked the loop for 8 ms
 * per frame, i.e. the tag was dropping beacons by construction.
 *
 * The per-frame diagnostics are therefore sampled.  The per-solve summary
 * (TSUM) and the product line (P,SEQ,...) stay at full rate: they
 * describe the solution and are what the bench actually needs. */
#define TAG_DIAG_RX_EVERY      8u   /* RX_TS / CFO: 1 beacon in 8       */
#define TAG_DIAG_DETAIL_EVERY  4u   /* TDOA_DBG / TSUM_CYC: 1 solve in 4 */

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
    uint16_t           cycle_id;         /* TDMA cycle id carried in the beacon (P0-4) */
    uint8_t            slot;             /* TDMA slot index (255=unknown) */
    uint64_t           last_rx_ts;       /* Raw RX local timestamp for TDOA */
    uint32_t           clock_reject_count; /* clock updates refused as outliers */

    /* ── P1-2: multi-cycle average of the TDOA against the Root ─────────
     * The ABSOLUTE slot-compensated TOA cannot be averaged across cycles:
     * it contains the round origin, which advances by one round (50 ms) in
     * the tag's clock, so averaging it would smear the TDOA by the tag's
     * clock error (50 ppm x 0.4 s = 20 ns = 0.6 m!).  The DIFFERENCE
     * against the Root's beacon of the SAME round is comparable across
     * rounds — the origin cancels — and that difference is exactly what
     * the TDOA is.  So the average is taken on the difference. */
    int64_t            tdoa_now;         /* this cycle's TDOA against the Root   */
    uint8_t            tdoa_valid;       /* tdoa_now holds a value                */
    int64_t            tdoa_avg;         /* multi-cycle average of tdoa_now       */
    uint8_t            tdoa_n;           /* samples in that average               */

    /* ── P1-1: grid self-check ─────────────────────────────────────────
     * The beacon advertises the instant it was actually transmitted.  The
     * tag must receive it shortly after — the DIFFERENCE between the
     * arrival and the advertised instant must stay constant.  A change
     * means the anchor left the grid (its beacon moved) or the clock
     * model broke.  The absolute value of the difference is meaningless
     * (it contains the unknown tag<->anchor clock offset), so the check
     * is made against a baseline frame. */
    uint64_t           sc_rx0;           /* baseline RX instant                  */
    uint64_t           sc_tx0;           /* baseline advertised TX instant       */
    uint8_t            sc_valid;
    uint8_t            sc_n;             /* frames since the baseline            */
    uint32_t           desync_count;     /* consecutive self-check failures      */
    uint8_t            desynced;         /* the anchor left the grid             */
} TagAnchorTracker;

/* P1-1 self-check tuning -------------------------------------------------
 * The check compares the arrival-vs-advertised difference against a
 * baseline frame, so it measures how much the anchor's beacon MOVED.
 * 2000 DTU = 94 cm: a real grid jump is far larger, a static deployment
 * never comes close (the residual is the path change plus the factor
 * error over the baseline span). */
#define TAG_SELFCHECK_BASELINE   16u   /* frames between re-baselining (1.3 s) */
#define TAG_DESYNC_DTU           2000LL
#define TAG_DESYNC_LIMIT         3u    /* consecutive failures before exclusion */
#define TAG_TOA_AVG_N            8u    /* rounds in the TDOA average (0.4 s)     */
#define TAG_TRACKER_FRESH_MS     250u  /* drop an anchor after 5 silent rounds  */

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
 * Position report — the tag's UWB uplink (see DL_POS_REPORT_LEN)
 *
 * The tag solves the TDOA itself and ships the RESULT to the gateway, so
 * the report frame carries x/y/z/q and not raw timestamps; the gateway only
 * relays it to the host.  The report goes out once per TDMA round (20 Hz),
 * in the guard time of slot 0.
 *==========================================================================*/
static uint16_t s_tag_id        = 0;    /* CRC16 over the 96-bit STM32 UID   */
static bool     s_solve_valid   = false;
static int16_t  s_solve_x_cm    = 0;
static int16_t  s_solve_y_cm    = 0;
static int16_t  s_solve_z_cm    = 0;
static uint16_t s_solve_q       = TAG_REPORT_QUALITY_NONE;
static uint32_t s_solve_ms      = 0;
static uint8_t  s_report_seq    = 0;

/*============================================================================
 * tag_build_pos_report — THE position report, one format for both outputs
 *
 * P2-3: the tag used to send a 47-byte frame over its local UART and a
 * 22-byte frame over UWB.  The two shared the header but not the length,
 * and byte 2 meant different things (the UART frame carried the firmware
 * version, the UWB frame a sequence counter) — so a parser had to know
 * which link it was looking at, and the two could not be compared.
 *
 * Now ONE builder fills ONE 22-byte frame, and the very same bytes go out
 * over the air and over the wire.  (The local frame also used to send
 * z = 0 while the radio frame sent the real height.)
 *==========================================================================*/
static void tag_build_pos_report(uint8_t *pkt)
{
    int16_t  x, y, z;
    uint16_t q;

    /* The payload: the last solution, or the "no solution" marker. */
    x = s_solve_x_cm; y = s_solve_y_cm; z = s_solve_z_cm;
    q = s_solve_q;
    if (!s_solve_valid) {
        x = 0; y = 0; z = 0;
        q = TAG_REPORT_QUALITY_NONE;
    } else if ((HAL_GetTick() - s_solve_ms) > 500u) {
        /* The solution is stale — the tag is alive but no longer solving. */
        q = TAG_REPORT_QUALITY_NONE;
    }

    memset(pkt, 0, DL_POS_REPORT_LEN);
    pkt[0] = 0x41; pkt[1] = 0x88;
    pkt[2] = s_report_seq++;
    pkt[3] = 0xCA; pkt[4] = 0xDE;
    pkt[5] = 0xFF; pkt[6] = 0xFF;                 /* broadcast */
    pkt[7] = (uint8_t)(s_tag_id & 0xFF);
    pkt[8] = (uint8_t)(s_tag_id >> 8);
    pkt[9] = FUNC_DL_POS_REPORT;
    pkt[10] = (uint8_t)((uint16_t)x & 0xFF);
    pkt[11] = (uint8_t)(((uint16_t)x >> 8) & 0xFF);
    pkt[12] = (uint8_t)((uint16_t)y & 0xFF);
    pkt[13] = (uint8_t)(((uint16_t)y >> 8) & 0xFF);
    pkt[14] = (uint8_t)((uint16_t)z & 0xFF);
    pkt[15] = (uint8_t)(((uint16_t)z >> 8) & 0xFF);
    pkt[16] = (uint8_t)(q & 0xFF);
    pkt[17] = (uint8_t)((q >> 8) & 0xFF);
    pkt[18] = (uint8_t)(FW_VERSION_U16 & 0xFF);
    pkt[19] = (uint8_t)((FW_VERSION_U16 >> 8) & 0xFF);
    /* [20..21] pad */
}
static uint32_t s_n_report_tx   = 0;    /* reports actually transmitted      */
static uint32_t s_n_report_skip = 0;    /* reports dropped (too late)        */

/* P1-4: independent Root-rate estimator.
 *
 * The Root's clock rate is needed to express a slot duration in OUR clock
 * (the slot compensation).  Taking it from the clock-sync filter's factor
 * ties it to the outlier gate: a Root frame that the filter rejects does
 * not update the factor, so the rate freezes — and a frozen rate is
 * exactly what the slot compensation must not use.
 *
 * The rate is therefore measured here, on its own channel, straight from
 * the interval between two successive Root beacons:
 *
 *     rate = (rx_local[n] - rx_local[n-1]) / (tx_remote[n] - tx_remote[n-1])
 *
 * Both intervals are exact (the two frames' own TX and RX instants), so a
 * single cycle already yields the ratio; the median over the last few
 * cycles removes the jitter of a single frame.  This channel is updated on
 * EVERY Root frame — accepted or rejected by the clock filter. */
#define TAG_RATE_SAMPLES 5
static double   s_rate_buf[TAG_RATE_SAMPLES];
static uint8_t  s_rate_n    = 0;   /* valid entries in the ring           */
static uint8_t  s_rate_idx  = 0;   /* next slot to write                    */
static uint64_t s_rate_rx_prev = 0;
static uint64_t s_rate_tx_prev = 0;
static uint8_t  s_rate_have_prev = 0;

/* P1-2: the Root's slot-compensated TOA of the cycle currently in flight.
 * The Root transmits in slot 0, so its beacon of a cycle always arrives
 * before the other anchors' beacons of the same cycle — which is what
 * makes the per-anchor TDOA against the Root computable in the RX path. */
static int64_t  s_root_toa_eq    = 0;
static uint16_t s_root_toa_cycle = 0;
static uint8_t  s_root_toa_valid = 0;

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
/* P1-4: how many anchors with a KNOWN slot make the network "ready" for a
 * position.  Three is the geometric minimum — two TDOAs for two unknowns,
 * exactly determined and without any redundancy, so a single noisy
 * measurement moves the answer.  Four is what a usable network has.  Lower
 * it to 3 for a deliberately small deployment. */
#define TAG_NET_READY_ANCHORS 4

static void tag_solve_and_report(void)
{
    int i;
    uint32_t now_ms = HAL_GetTick();

    /* Solve counter for the diagnostic decimation (TAG_DIAG_DETAIL_EVERY).
     * The solve itself runs at 20 Hz; the detailed TDOA dump is 1 in 4. */
    static uint32_t s_solve_diag_ctr;
    uint32_t solve_n = s_solve_diag_ctr++;
    bool detail = ((solve_n % TAG_DIAG_DETAIL_EVERY) == 0);

    /* Find fresh active trackers (within FRESH_MS_MAX ms) */
    int n_active = 0;
    int n_desync = 0;
    int active_idx[DL_MAX_TRACKED_ANCHORS];
    for (i = 0; i < DL_MAX_TRACKED_ANCHORS; i++) {
        if (!s_trackers[i].is_active) continue;
        /* P1-1: an anchor that failed the grid self-check is EXCLUDED from
         * the solve — its beacon no longer sits on the grid, so its slot
         * compensation would shift the whole solution. */
        if (s_trackers[i].desynced) { n_desync++; continue; }
        uint32_t age_ms = (s_trackers[i].last_seen_ms == 0) ? 0
                          : (now_ms - s_trackers[i].last_seen_ms);
        if (age_ms > TAG_TRACKER_FRESH_MS) continue;  /* measurement too stale */
        active_idx[n_active++] = i;
    }
    if (n_active < 3) {
        { char dbg[72]; int pos = 0;
          pos += sprintf(dbg + pos, "TSUM n_act=%d SKIP need 3 anchors", n_active);
          if (n_desync) pos += sprintf(dbg + pos, " desync=%d", n_desync);
          pos += sprintf(dbg + pos, "\r\n");
          test_run_info((unsigned char *)dbg); }
        return;
    }

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
     * Slot-based TDOA: toa_equiv = rx - slot × 6.25ms
     * This normalizes all beacons to slot=0 (Root's TX time).
     * No anchor clock sync needed: Tag's local clock is the common reference.
     *
     * DTU bookkeeping (P0-4): one DW3000 device time unit is 15.65 ps, so
     * 1 µs = 63,898 DTU and one 6.25 ms TDMA slot = 6250 × 63898 =
     * 399,362,500 DTU.  The old constant (192,000) belonged to a 38.4
     * DTU/µs clock and was 3328× too small: the slot compensation could
     * never cancel the 0..43.75 ms spread between the slots, every TDOA was
     * rejected and the solver collapsed onto the centroid of the anchors. */
    #define DTU_PER_SLOT  ((int64_t)DL_SLOT_DTU_US * (int64_t)UUS_TO_DWT_TIME)

    /* Acceptance window for a TDOA difference.  Light covers 1 m in 3.3356 ns
     * = 213.20 DTU (1 DTU = 15.65 ps), so the window has to cover the
     * largest legitimate path difference of the site.  The dominant error
     * source (a beacon from a different TDMA round, 50 ms = 3.19 M DTU) is
     * removed by the cycle-id pairing below — and by this window as a second
     * line of defence: with the old (1000x too large) constant the window
     * was ±100 km and let cross-round frames through.
     *
     * P2-4: the unit was the truncated 213 and the cap 100 m — both too
     * tight for a site whose anchors are more than 100 m apart.  The value
     * now comes from shared_defines.h (213.20 DTU/m, kept at 1/1000
     * resolution) and the cap is 150 m of path difference, which is still
     * two orders of magnitude below the cross-round error. */
    #define TDOA_DTU_PER_M_MILLI  213200LL   /* 213.20 DTU per metre × 1000 */
    #define TDOA_MAX_PATH_M        150LL     /* ±150 m of path difference    */
    #define TDOA_MAX_DTU           ((TDOA_MAX_PATH_M * TDOA_DTU_PER_M_MILLI) / 1000LL)

    /* ── Slot spacing is defined in the ROOT's clock, not in ours ──────────
     * The anchors place their beacons on the Root's grid: anchor k transmits
     * at  G + slot_k × 6.25 ms  where G and the 6.25 ms are the ROOT's clock.
     * Our own crystal differs from the Root's by up to ±20 ppm, so 6.25 ms is
     * 6.25 ms × f in our clock, where f is the rate ratio (our clock)/(Root's
     * clock).  Subtracting the nominal 6.25 ms instead would leave a
     * per-anchor error of slot × 6.25 ms × 20 ppm = up to 875 ns ≈ 26 cm at
     * slot 7 — far above the 0.67 ns the 20 cm target needs.  f is measured from the
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

    /* ── P0-4: pairing rule ───────────────────────────────────────────────
     * The slot-compensated TDOA
     *     toa = rx_local - slot × 6.25 ms
     * cancels the round origin only when both beacons belong to the same
     * round — one round of skew is 50 ms = 3.19 M DTU and would poison every
     * pair it enters.
     *
     * The Root-reference path (the normal case) does NOT need a cycle
     * filter on the anchor SET: the per-anchor tdoa_avg was built in the
     * RX path out of pairs of the SAME round (there an anchor is only
     * paired with the Root's TOA of an identical cycle_id), so every
     * sample inside the average is a round-consistent difference.
     *
     * Filtering the set by the cycle of each anchor's LAST frame used to
     * throw away every anchor whose last frame is one round old.  At
     * 20 Hz that is roughly half of them at any moment — and worse, it
     * could filter the ROOT out, which pushed the solve onto the
     * absolute-TOA path where rounds ARE mixed and the geometry is
     * meaningless. */
    int i2, j2, k2;
    uint16_t best_cycle_all = 0;
    int best_cnt_all = 0;

    /* Is the Root (slot 0) among the slot-known anchors?  Its beacon
     * defines the round origin, so it is the reference of the good path. */
    int root_idx = -1;
    for (i2 = 0; i2 < n_sel; i2++) {
        if (s_trackers[sel_idx[i2]].rank == s_root_rank) {
            root_idx = sel_idx[i2];
            break;
        }
    }

    /* The legacy paths (no Root in the set) still need the cycle filter:
     * there the absolute TOAs are differenced, and only beacons of the
     * same round may be differenced against each other. */
    uint16_t best_cycle_sel = 0;
    int best_cnt_sel = 0;
    if (root_idx < 0) {
        for (i2 = 0; i2 < n_sel; i2++) {
            uint16_t c = s_trackers[sel_idx[i2]].cycle_id;
            int cnt = 0;
            for (j2 = 0; j2 < n_sel; j2++)
                if (s_trackers[sel_idx[j2]].cycle_id == c) cnt++;
            if (cnt > best_cnt_sel) { best_cnt_sel = cnt; best_cycle_sel = c; }
        }
        k2 = 0;
        for (i2 = 0; i2 < n_sel; i2++) {
            int idx = sel_idx[i2];
            if (s_trackers[idx].cycle_id == best_cycle_sel) {
                sel_idx[k2++] = idx;
            } else {
                s_n_cycle_mismatch++;
            }
        }
        n_sel = k2;
    }

    for (i2 = 0; i2 < n_active; i2++) {
        uint16_t c = s_trackers[active_idx[i2]].cycle_id;
        int cnt = 0;
        for (j2 = 0; j2 < n_active; j2++)
            if (s_trackers[active_idx[j2]].cycle_id == c) cnt++;
        if (cnt > best_cnt_all) { best_cnt_all = cnt; best_cycle_all = c; }
    }

    int ref_idx;
    int n_valid;
    bool use_slot_compensation = (n_sel >= 3);
    /* P1-2: were the TDOA values taken from the multi-cycle average? */
    bool use_avg = false;

    if (use_slot_compensation) {
        /* ── Slot-compensated TDOA ────────────────────────────────────────
         * toa_equiv = rx - slot × 6.25 ms (normalizes all to slot=0)
         * Works when anchors are in Root's TDMA table. */

        /* Reference = the Root when it is in the set (its beacon defines
         * the round origin); otherwise the anchor with the lowest slot. */
        bool ref_is_root = (root_idx >= 0);
        if (ref_is_root) {
            ref_idx = root_idx;
        } else {
            ref_idx = sel_idx[0];
            for (i = 1; i < n_sel; i++) {
                if (s_trackers[sel_idx[i]].slot < s_trackers[ref_idx].slot) {
                    ref_idx = sel_idx[i];
                }
            }
        }

        uint8_t ref_slot = s_trackers[ref_idx].slot;

        if (ref_is_root) {
            /* Every value is a difference against the Root, so the Root is
             * the origin and the solver sees the same TDOAs as before.
             *
             * The value used is the multi-round average (P1-2): each of its
             * samples was paired with the Root's TOA of the SAME round in
             * the RX path, so the average is round-consistent by
             * construction and needs no cycle filter — while it also
             * removes the per-frame RX jitter, which is the dominant noise
             * term at a 6.25 ms slot spacing. */
            use_avg = true;
            toa_dtu[0] = 0;
            anchors[0] = s_trackers[ref_idx].pos;
            valid_idx[0] = ref_idx;

            n_valid = 1;
            for (i = 0; i < n_sel; i++) {
                int idx = sel_idx[i];
                if (idx == ref_idx) continue;
                if (!s_trackers[idx].tdoa_valid) continue;

                int64_t tdoa_dtu = s_trackers[idx].tdoa_avg;
                if (tdoa_dtu > TDOA_MAX_DTU || tdoa_dtu < -TDOA_MAX_DTU) continue;
                s_n_pair++;

                toa_dtu[n_valid] = (uint64_t)(int64_t)tdoa_dtu;
                anchors[n_valid] = s_trackers[idx].pos;
                valid_idx[n_valid] = idx;
                n_valid++;
            }
        } else {
            /* The reference is not the Root: the absolute TOAs have to be
             * used (no averaging possible). */
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
    /* Debug: print raw TDOA values (1 solve in TAG_DIAG_DETAIL_EVERY).
     * The buffer is 288 bytes: the worst case is 8 anchors x (" R" + 5-digit
     * id + "=" + 14-digit signed DTU difference) plus a 40-byte header, i.e.
     * ~220 bytes — the old 128-byte buffer could overflow it. */
    if (detail) {
      char dbg[288]; int pos = 0;
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
    { char dbg[128]; int pos = 0;
      pos += sprintf(dbg + pos, "TSUM n_act=%d n_sel=%d n=%d pair=%u mism=%u cyc=%u ref=R%d",
          n_active, n_sel, n_valid, (unsigned)s_n_pair,
          (unsigned)s_n_cycle_mismatch,
          (unsigned)s_trackers[ref_idx].cycle_id,
          s_trackers[ref_idx].rank);
      if (n_desync) pos += sprintf(dbg + pos, " desync=%d", n_desync);
      pos += sprintf(dbg + pos, " avg=%d\r\n", use_avg ? 1 : 0);
      test_run_info((unsigned char *)dbg); }

    /* Per-anchor cycle id of the selected set — the fastest way to see
     * which anchor is one round behind.  Sampled 1 solve in 4. */
    if (detail) {
      char dbg[128]; int pos = 0;
      pos += sprintf(dbg + pos, "TSUM_CYC");
      for (i = 0; i < n_sel && pos < 100; i++) {
          int idx = sel_idx[i];
          pos += sprintf(dbg + pos, " R%u:c%u",
              s_trackers[idx].rank, (unsigned)s_trackers[idx].cycle_id);
      }
      pos += sprintf(dbg + pos, "\r\n");
      test_run_info((unsigned char *)dbg); }

    if (n_valid < 3) {
        { char dbg[64];
          sprintf(dbg, "TSUM n=%d SKIP need 3 anchors\r\n", n_valid);
          test_run_info((unsigned char *)dbg); }
        return;
    }

    /* P1-4: the network is only "ready" when enough anchors are on the
     * grid with a KNOWN slot.  Until then the solve would collapse onto
     * the centroid of the anchors — that looks like a position on the map
     * but is not one, and it hides the real problem (the network is not
     * formed).  So say what is missing instead of publishing a number.
     *
     * The UWB report is NOT stopped: the tag keeps sending its 0xE3 frame
     * with the "no quality" marker (65535), so it stays visible on the
     * host as an online device that has no position yet. */
    if (n_sel < TAG_NET_READY_ANCHORS) {
        int wait_slot = 255;
        for (i2 = 0; i2 < n_sel; i2++) {
            int sl = s_trackers[sel_idx[i2]].slot;
            if (sl < wait_slot) wait_slot = sl;   /* the lowest slot = the Root */
        }
        { char dbg[64];
          sprintf(dbg, "WAIT NET n=%d slot=%d\r\n", n_sel, wait_slot);
          test_run_info((unsigned char *)dbg); }
        return;
    }

    float x_cm = 0, y_cm = 0, z_cm = 0, rms = 0;
    int ret = tdoa_solve(anchors, toa_dtu, (uint8_t)n_valid,
                         &x_cm, &y_cm, &z_cm, &rms);
    if (ret != 0) return;

    /* Hand the solution to the UWB uplink: the report goes out once per
     * TDMA round, in the guard time of slot 0. */
    s_solve_x_cm  = (int16_t)x_cm;
    s_solve_y_cm  = (int16_t)y_cm;
    s_solve_z_cm  = (int16_t)z_cm;
    s_solve_q     = (uint16_t)(rms / 100.0f * 1000.0f);
    s_solve_valid = true;
    s_solve_ms    = HAL_GetTick();

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
    /* P2-3: the local UART carries the SAME 22-byte report as the radio.
     * One format, one parser — and the height is no longer dropped. */
    {
        uint8_t tx[DL_POS_REPORT_LEN];
        tag_build_pos_report(tx);
        HAL_UART_Transmit(&huart2, tx, DL_POS_REPORT_LEN, 10);
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
    /* Find an empty slot.
     *
     * P2-8: the old test was `!is_active && rank == 0`.  Once a slot had
     * been used its rank was non-zero forever, so it could never be
     * reused — the "find empty" loop always ran to the end and every new
     * anchor went straight to the eviction path.  With 8 anchors and 8
     * tracker slots that only bites at the ninth anchor, but the eviction
     * path then picked the tracker with the LOWEST quality, and the Root's
     * tracker is a perfectly valid candidate: the Root owns the grid the
     * other anchors are placed on and the TDOA solve is referenced to its
     * beacon, so evicting it collapses the whole estimate.
     */
    for (i = 0; i < DL_MAX_TRACKED_ANCHORS; i++) {
        if (!s_trackers[i].is_active) {
            s_trackers[i].rank = rank;
            s_trackers[i].slot = 255;      /* unknown until beacon received */
            s_trackers[i].last_rx_ts = 0;
            s_trackers[i].cycle_id = 0;    /* P0-4: TDMA cycle id (0 = unknown) */
            s_trackers[i].last_seen_ms = 0;
            s_trackers[i].quality = 0.0f;
            s_trackers[i].tdoa_valid = 0;
            s_trackers[i].desynced = 0;
            s_trackers[i].desync_count = 0;
            clock_sync_init(&s_trackers[i].sync, 1.0, 0.0, 0.0, 0.0, 0.0);
            return i;
        }
    }
    /* Every slot is taken: evict the WEAKEST anchor — but never the Root.
     * The Root sits in slot 0 of the TDMA grid and is the reference of the
     * slot-compensated TDOA; the reference anchor is chosen by quality
     * (see tag_solve_and_report), so the Root has to stay in the table. */
    int worst = -1;
    float worst_q = 0.0f;
    for (i = 0; i < DL_MAX_TRACKED_ANCHORS; i++) {
        if (s_trackers[i].slot == 0) continue;   /* the Root — keep it */
        if (worst < 0 || s_trackers[i].quality < worst_q) {
            worst_q = s_trackers[i].quality;
            worst = i;
        }
    }
    if (worst < 0) worst = 0;   /* only the Root is present — cannot happen */
    s_trackers[worst].rank = rank;
    s_trackers[worst].slot = 255;
    s_trackers[worst].last_rx_ts = 0;
    s_trackers[worst].cycle_id = 0;      /* P0-4 */
    s_trackers[worst].last_seen_ms = 0;
    s_trackers[worst].quality = 0.0f;
    s_trackers[worst].tdoa_now   = 0;
    s_trackers[worst].tdoa_valid = 0;
    s_trackers[worst].tdoa_avg   = 0;
    s_trackers[worst].tdoa_n     = 0;
    s_trackers[worst].sc_valid   = 0;
    s_trackers[worst].sc_n       = 0;
    s_trackers[worst].desync_count = 0;
    s_trackers[worst].desynced   = 0;
    clock_sync_init(&s_trackers[worst].sync, 1.0, 0.0, 0.0, 0.0, 0.0);
    return worst;
}

/*============================================================================
 * Local clock — 40-bit extended to 64 bit
 *
 * The DW3000's timestamps are a 40-bit counter that wraps every 17.21 s.
 * The tag compares the RX instants of DIFFERENT anchors with each other
 * (that is the whole TDOA), and the TDOA window is only a few tens of
 * microseconds wide — so as soon as one anchor's beacon is received before
 * the wrap and another's after it, the raw 40-bit difference is 2^40 instead
 * of the real few hundred DTU, and the pair is thrown away.  The same
 * happens in the clock-sync rate window.
 *
 * All local instants are therefore kept on an extended 64-bit timeline
 * that never wraps: the raw stamp is combined with the epoch of SYS_TIME.
 *==========================================================================*/
static uint64_t s_tag_sys_time_ext = 0;

/* Current SYS_TIME, extended to 64 bit.
 *
 * The epoch (the number of wraps so far) is carried over from the previous
 * call and only the low 40 bits are replaced by the new reading — otherwise
 * the "extended" clock falls back into the raw 40-bit domain one call after
 * every wrap, and the stamps of the different anchors stop being
 * comparable exactly when they have to be. */
static uint64_t tag_now_sys_ext(void)
{
    uint64_t raw = get_sys_timestamp_u64() & 0xFFFFFFFFFFULL;
    uint64_t ext = s_tag_sys_time_ext;
    if (ext != 0) {
        uint64_t last_raw = ext & 0xFFFFFFFFFFULL;
        uint64_t epoch    = ext & ~0xFFFFFFFFFFULL;
        if (raw < last_raw && (last_raw - raw) > (1ULL << 39)) {
            epoch += (1ULL << 40);      /* the counter wrapped */
        }
        ext = epoch | raw;
    } else {
        ext = raw;
    }
    s_tag_sys_time_ext = ext;
    return ext;
}

/* Local RX timestamp, extended to the 64-bit timeline.
 *
 * The RX latch is read FIRST and SYS_TIME afterwards, so SYS_TIME is
 * guaranteed to be >= the RX instant; the epoch comes from SYS_TIME and an
 * RX instant that still belongs to the previous epoch is pulled back by one
 * wrap.  The 512-DTU tolerance covers the truncation of SYS_TIME, whose low
 * 8 bits are always zero (it is read as a 32-bit register and shifted up
 * by 8). */
static uint64_t tag_rx_ts_ext(void)
{
    uint64_t raw = get_rx_timestamp_u64() & 0xFFFFFFFFFFULL;
    uint64_t now = tag_now_sys_ext();
    uint64_t ext = (now & ~0xFFFFFFFFFFULL) | raw;
    if (ext > now + 512ULL && ext >= (1ULL << 40)) ext -= (1ULL << 40);
    return ext;
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
     * If all anchors report identical rx_local, RX timestamp latch is broken.
     *
     * Sampled (TAG_DIAG_RX_EVERY): this runs on EVERY received beacon
     * (160/s with 8 anchors) and the print blocks the RX loop.  The diff
     * itself is still updated on every frame, so a stall of the RX latch
     * shows up in the sampled line as well. */
    {
        static uint64_t s_last_rx;
        static uint32_t s_rx_diag_ctr;
        char dbg[96];
        if ((s_rx_diag_ctr++ % TAG_DIAG_RX_EVERY) == 0) {
            sprintf(dbg, "RX_TS R=%u tx=%llu rx=%llu diff=%lld\r\n",
                src_rank,
                (unsigned long long)tx_remote,
                (unsigned long long)rx_local,
                (long long)(rx_local - s_last_rx));
            test_run_info((unsigned char *)dbg);
        }
        s_last_rx = rx_local;
    }

    int tracker_slot = find_tracker_slot(src_rank);
    if (tracker_slot < 0) tracker_slot = alloc_tracker_slot(src_rank);

    /* Track which TDMA cycle this RX belongs to.  The authoritative value is
     * the cycle id carried in the beacon (P0-4) — every anchor of the same
     * grid cycle stamps the same id, so the Tag can pair beacons that truly
     * belong to the same cycle.  It is always present in a valid frame
     * (DL_TS_CYCLE_IDX + 2 <= DL_TIMESYNC_LEN), so no local fallback
     * counter is needed (P1-11). */
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
        static uint32_t s_cfo_diag_ctr;
        char dbg2[64];
        if ((s_cfo_diag_ctr++ % TAG_DIAG_RX_EVERY) == 0) {
            sprintf(dbg2, "CFO cfo_raw=%d factor_meas=%.9f\r\n",
                cfo_raw, 1.0 + (double)cfo_raw / 67108864.0);
            if (cfo_raw != s_last_cfo || s_last_cfo == 0) {
                test_run_info((unsigned char *)dbg2);
                s_last_cfo = cfo_raw;
            }
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

        /* P1-4: the Root's clock rate on its own channel (see
         * s_rate_buf).  Updated on every Root frame, outlier or not. */
        {
            uint64_t d_rx = rx_local - s_rate_rx_prev;
            uint64_t d_tx = (tx_remote - s_rate_tx_prev) & 0xFFFFFFFFFFULL;
            if (s_rate_have_prev && d_tx > 0u && d_rx > 0u
                && d_tx < (1ULL << 30) && d_rx < (1ULL << 30)) {
                double r = (double)d_rx / (double)d_tx;
                /* A round is 50 ms; anything outside 1 % is not a clock
                 * ratio but a lost or duplicated frame. */
                if (r > 0.99 && r < 1.01) {
                    s_rate_buf[s_rate_idx] = r;
                    s_rate_idx = (uint8_t)((s_rate_idx + 1u) % TAG_RATE_SAMPLES);
                    if (s_rate_n < TAG_RATE_SAMPLES) s_rate_n++;
                }
            }
            s_rate_rx_prev = rx_local;
            s_rate_tx_prev = tx_remote;
            s_rate_have_prev = 1;
        }
        if (s_rate_n >= 3u) {
            /* Median of the last cycles (insertion sort, n <= 5). */
            double tmp[TAG_RATE_SAMPLES];
            uint8_t i, j;
            for (i = 0; i < s_rate_n; i++) tmp[i] = s_rate_buf[i];
            for (i = 1; i < s_rate_n; i++) {
                double v = tmp[i];
                for (j = i; j > 0u && tmp[j - 1u] > v; j--) tmp[j] = tmp[j - 1u];
                tmp[j] = v;
            }
            s_root_rate = tmp[s_rate_n / 2u];
            s_root_rate_valid = 1;
        } else if (s_rate_n > 0u) {
            s_root_rate = s_rate_buf[0];
            s_root_rate_valid = 1;
        } else {
            /* Nothing measured yet — fall back to the clock filter's factor
             * (which needs a few updates before it is trustworthy). */
            s_root_rate = s_trackers[tracker_slot].sync.factor;
            s_root_rate_valid = (s_trackers[tracker_slot].sync.update_count >= 8)
                                ? 1 : 0;
        }
    }

    /* ── P1-2: TDOA against the Root, averaged over several cycles ───────
     * toa_equiv = rx_local - slot x 6.25 ms removes the TDMA slot from the
     * arrival instant.  Against the Root's beacon of the SAME cycle the
     * cycle origin cancels as well, so the difference is the pure TDOA —
     * and, unlike the absolute TOA, it is comparable across cycles and
     * can therefore be averaged (which removes the RX timestamp jitter
     * and the per-frame grid jitter, the dominant error terms).
     *
     * The Root's own beacon is stored separately; the other anchors pair
     * with it as long as the cycle id matches. */
    {
        TagAnchorTracker *t = &s_trackers[tracker_slot];
        bool is_root = (frame_len >= DL_TS_LEVEL_IDX + 1
                        && rx_buf[DL_TS_LEVEL_IDX] == 0);
        if (t->slot != 255 && s_root_rate_valid) {
            int64_t slot_eff = (int64_t)((double)DTU_PER_SLOT * s_root_rate);
            int64_t toa_eq   = (int64_t)rx_local - (int64_t)t->slot * slot_eff;
            if (is_root) {
                s_root_toa_eq    = toa_eq;
                s_root_toa_cycle = t->cycle_id;
                s_root_toa_valid = 1;
            } else if (s_root_toa_valid && s_root_toa_cycle == t->cycle_id) {
                t->tdoa_now   = toa_eq - s_root_toa_eq;
                t->tdoa_valid = 1;
                if (t->tdoa_n == 0) {
                    t->tdoa_avg = t->tdoa_now;
                    t->tdoa_n   = 1;
                } else {
                    uint32_t w = (t->tdoa_n < TAG_TOA_AVG_N) ? t->tdoa_n
                                                             : TAG_TOA_AVG_N;
                    t->tdoa_avg += (t->tdoa_now - t->tdoa_avg) / (int64_t)w;
                    t->tdoa_n++;
                }
            }
        }
    }

    /* ── P1-1: grid self-check ───────────────────────────────────────────
     * The beacon advertises the instant it actually left the antenna.  The
     * tag receives it a fraction of a microsecond later — the DIFFERENCE
     * between the arrival and the advertised instant must stay constant.
     * Its absolute value contains the unknown tag<->anchor clock offset
     * and is therefore meaningless, but a CHANGE of it means the anchor's
     * beacon moved (the anchor left the grid) or the clock model broke.
     *
     * The baseline is re-taken every TAG_SELFCHECK_BASELINE frames, so
     * the factor error (which multiplies the baseline span) stays small.
     * The check only runs once the clock model has settled. */
    {
        TagAnchorTracker *t = &s_trackers[tracker_slot];
        if (t->sync.update_count >= 8 && t->sync.quality > 0.5f) {
            if (!t->sc_valid || t->sc_n >= TAG_SELFCHECK_BASELINE) {
                t->sc_rx0 = rx_local;
                t->sc_tx0 = tx_remote;
                t->sc_valid = 1;
                t->sc_n = 0;
            } else {
                t->sc_n++;
                double f = t->sync.factor;
                uint64_t d_tx = (tx_remote - t->sc_tx0) & 0xFFFFFFFFFFULL;
                int64_t  d_res = (int64_t)((double)(rx_local - t->sc_rx0)
                               - f * (double)d_tx);
                if (d_res > TAG_DESYNC_DTU || d_res < -TAG_DESYNC_DTU) {
                    if (t->desync_count < TAG_DESYNC_LIMIT) t->desync_count++;
                    if (t->desync_count >= TAG_DESYNC_LIMIT && !t->desynced) {
                        t->desynced = 1;
                        { char dbg[72];
                          sprintf(dbg, "TAG_DESYNC R=%u d=%lld dtu (%.1f cm) EXCLUDED\r\n",
                                  t->rank, (long long)d_res,
                                  (double)d_res * CM_PER_DTU_F);
                          test_run_info((unsigned char *)dbg); }
                    }
                } else {
                    if (t->desynced) {
                        t->desynced = 0;
                        { char dbg[56];
                          sprintf(dbg, "TAG_DESYNC R=%u RECOVERED\r\n", t->rank);
                          test_run_info((unsigned char *)dbg); }
                    }
                    t->desync_count = 0;
                }
            }
        }
    }
}

/*============================================================================
 * Tag id — CRC16 over the full 96-bit STM32 UID, exactly like the anchors'
 * rank.  A constant id would collapse every tag in the room into a single
 * device in the host list.
 *==========================================================================*/
static uint16_t tag_compute_id(void)
{
    uint32_t w0 = HAL_GetUIDw0();
    uint32_t w1 = HAL_GetUIDw1();
    uint32_t w2 = HAL_GetUIDw2();
    uint8_t  idb[12];
    uint16_t crc = 0xFFFFu;
    int i, b;

    for (i = 0; i < 4; i++) idb[i]     = (uint8_t)((w0 >> (i * 8)) & 0xFF);
    for (i = 0; i < 4; i++) idb[4 + i] = (uint8_t)((w1 >> (i * 8)) & 0xFF);
    for (i = 0; i < 4; i++) idb[8 + i] = (uint8_t)((w2 >> (i * 8)) & 0xFF);

    for (i = 0; i < 12; i++) {
        crc ^= (uint16_t)((uint16_t)idb[i] << 8);
        for (b = 0; b < 8; b++) {
            if (crc & 0x8000u) crc = (uint16_t)((crc << 1) ^ 0x1021u);
            else               crc = (uint16_t)(crc << 1);
        }
    }
    if (crc == 0) crc = 1;  /* 0 stays reserved for the gateway */
    return crc;
}

/*============================================================================
 * Report the solved position to the gateway over UWB.
 *
 * Called right after the ROOT's beacon (slot 0) was received: root_rx is
 * the instant the beacon arrived at the tag, i.e. the round origin in the
 * tag's own clock.  The frame is transmitted with a DELAYED TX at
 *
 *     root_rx + TAG_REPORT_DELAY_US + ALOHA jitter
 *
 * which lies inside slot 0's guard time (< 6.25 ms) — the one interval of
 * the round in which no anchor transmits.  The jitter is an ALOHA offset so
 * that several tags do not always land on the same microsecond.
 *
 * The TX instant does NOT influence the accuracy of the reported position:
 * the tag has already solved the TDOA itself and ships the result.  The
 * delayed TX exists only to place the frame inside the guard window.
 *==========================================================================*/
static void tag_tx_pos_report(uint64_t root_rx)
{
    uint8_t  pkt[DL_POS_REPORT_LEN];
    uint64_t now, target, slot1;
    uint32_t delay_us;

    /* ALOHA offset — an LFSR seeded from the UID, so that two tags in the
     * room do not always report in the same microsecond. */
    static uint32_t s_jitter = 0;
    if (s_jitter == 0) {
        s_jitter = HAL_GetUIDw0() ^ HAL_GetUIDw1() ^ HAL_GetUIDw2();
        if (s_jitter == 0) s_jitter = 0xACE1u;
    }
    s_jitter = s_jitter * 1664525u + 1013904223u;
    delay_us = TAG_REPORT_DELAY_US + (s_jitter % TAG_REPORT_JITTER_US);

    now    = tag_now_sys_ext();
    target = root_rx + (uint64_t)delay_us * UUS_TO_DWT_TIME;

    /* Slot 1's beacon arrives at root_rx + 6.25 ms.  A report after that
     * would land ON an anchor's transmission: it would spoil that anchor's
     * reception AND corrupt the tag's own TDOA for that anchor.  Losing one
     * report is far cheaper than corrupting a measurement, so the report is
     * simply skipped. */
    slot1 = root_rx + (uint64_t)DL_SLOT_DTU_US * UUS_TO_DWT_TIME;
    if (now + (200ULL * UUS_TO_DWT_TIME) > slot1) {
        s_n_report_skip++;
        return;
    }

    /* Not enough time left to program the delayed TX (the SPI writes plus
     * the start command take ~300 us) — transmit right away instead, which
     * still lies inside the guard window (checked above). */
    if (target < now + (400ULL * UUS_TO_DWT_TIME)) {
        target = 0;
    }

    /* P2-3: the same builder as the local UART path — one format. */
    tag_build_pos_report(pkt);

    /* Quiet the radio before the TX, exactly like the anchors do. */
    port_DisableEXT_IRQ();
    dwt_forcetrxoff();
    port_rf_switch_tx();
    dwt_writetxdata(DL_POS_REPORT_LEN, pkt, 0);
    dwt_writetxfctrl(DL_POS_REPORT_LEN, 0, 0);

    bool tx_started = false;
    if (target != 0) {
        /* DX_TIME holds the device time bits [39:8] — mask to 40 bits first
         * (the device time counter is 40 bit) and then shift. */
        dwt_setdelayedtrxtime((uint32_t)((target & 0xFFFFFFFFFFULL) >> 8));
        if (dwt_starttx(DWT_START_TX_DELAYED) == DWT_SUCCESS) {
            tx_started = true;
        } else {
            /* The delayed TX was refused (late programming / HPDWARN) — fall
             * back to an immediate TX, but only while the guard window is
             * still open; a report that lands on slot 1's beacon would
             * corrupt a measurement, which is worse than a lost report. */
            uint64_t n2 = tag_now_sys_ext();
            if (n2 + (200ULL * UUS_TO_DWT_TIME) < slot1) {
                dwt_starttx(DWT_START_TX_IMMEDIATE);
                tx_started = true;
            } else {
                s_n_report_skip++;
            }
        }
    } else {
        dwt_starttx(DWT_START_TX_IMMEDIATE);
        tx_started = true;
    }

    if (tx_started) {
        /* Wait for the frame to leave the antenna.  The wait is bounded: the
         * receiver has to come back even if the transmitter misbehaves. */
        uint32_t dl = HAL_GetTick() + 10u;
        while ((dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK) == 0) {
            if (HAL_GetTick() > dl) break;
        }
        dwt_write32bitreg(SYS_STATUS_ID,
                          SYS_STATUS_TXFRS_BIT_MASK | SYS_STATUS_HPDWARN_BIT_MASK);
        s_n_report_tx++;
    }

    port_rf_switch_rx();
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    port_EnableEXT_IRQ();
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

    /* Our own 16-bit id (CRC16 over the 96-bit UID) — the gateway uses it
     * as the source address of the position report. */
    s_tag_id = tag_compute_id();

    /* Init tracker slots */
    int i;
    for (i = 0; i < DL_MAX_TRACKED_ANCHORS; i++) {
        s_trackers[i].rank = 0;
        s_trackers[i].is_active = 0;
        s_trackers[i].last_seen_ms = 0;
        s_trackers[i].quality = 0;
        s_trackers[i].sync.initialized = false;
    }

    /* Solve cadence: one solve per TDMA round (DL_SOLVE_INTERVAL_MS = 50 ms
     * → 20 Hz).
     *
     * The deadline lives OUTSIDE the RX-window loop on purpose.  It used to
     * be re-armed with every window; since the window and the solve
     * interval had the same length, the timer could only ever fire ONCE
     * per window — the "20 Hz solve" was in fact 12.5 Hz (one solve per
     * 80 ms window).  An absolute deadline keeps the cadence at 20 Hz and
     * does not drift with the duration of the solve itself. */
    uint32_t last_solve_ms = HAL_GetTick();

    while (1) {
        /* Phase 1: Continuous RX for one super-cycle */
        dwt_forcetrxoff();
        port_rf_switch_rx();
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        uint32_t rx_start = HAL_GetTick();
        while ((HAL_GetTick() - rx_start) < DL_RX_WINDOW_MS) {
            Sleep(1);
            uint32_t now = HAL_GetTick();

            /* 20 Hz solve: fixed 50 ms interval on an absolute grid */
            if ((int32_t)(now - last_solve_ms) >= DL_SOLVE_INTERVAL_MS) {
                last_solve_ms += DL_SOLVE_INTERVAL_MS;
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
             * latch still holds the timestamp for the received frame.
             * tag_rx_ts_ext() puts it on the extended 64-bit local timeline
             * (the raw 40-bit stamp wraps every 17.21 s). */
            uint64_t rx_hw_ts = tag_rx_ts_ext();
            if (!match_header(rx_buf, match_timesync, 10, 1, 0, 1)) {
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
                continue;
            }
            if (rx_buf[9] == FUNC_DOWNLINK_BEACON) {
                tag_rx_timesync(rx_buf, frame_len, rx_hw_ts);

                /* The Root's beacon (slot 0, level 0) defines the round
                 * origin.  The tag reports its position in the guard time
                 * that follows it — the one interval of the round in which
                 * no anchor is transmitting, whatever the anchor count is. */
                if (frame_len >= DL_TIMESYNC_LEN
                    && rx_buf[DL_TS_SLOT_IDX] == 0
                    && rx_buf[DL_TS_LEVEL_IDX] == 0) {
                    tag_tx_pos_report(rx_hw_ts);
                }
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
            /* 288 bytes: 8 trackers x " R65535:F=+123456:Q=100:A=9999"
             * (~30 bytes each) plus the header is ~246 bytes — the old
             * 128-byte buffer overflowed with 5+ active anchors. */
            char out[288]; int pos = 0;
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
            pos += sprintf(out + pos, " RPT tx=%u skip=%u id=%04x",
                (unsigned)s_n_report_tx,
                (unsigned)s_n_report_skip,
                (unsigned)s_tag_id);
            pos += sprintf(out + pos, "\r\n");
            test_run_info((unsigned char *)out);
        }
    }
    return 0;
}

#endif /* TEST_FUSION_TAG */