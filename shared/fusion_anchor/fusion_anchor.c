/*! ----------------------------------------------------------------------------
 *  @file    fusion_anchor.c
 *  @brief   BD8NCF-style per-anchor TimeSync broadcaster (Phase 1.1)
 *
 *  Every anchor runs the same firmware.  Each anchor:
 *    - Periodically broadcasts a compact 26-byte TimeSync frame (~100 ms cadence)
 *      carrying its predicted TX timestamp, cs_level, parent_EUI64, position.
 *    - Listens between broadcasts for other anchors' TimeSync frames and
 *      updates its per-source Kalman clock-sync state (see clock_sync.h).
 *    - Accepts COORD_WRITE from the Gateway via UWB unicast to set its
 *      position.  Position is then broadcast in every TimeSync.
 *    - Keeps all OTA hooks intact for in-field firmware updates.
 *
 *  Phase 1.2 will introduce full cascading-sync election logic and Phase 1.3
 *  Observer feedback.  For Phase 1.1 every anchor assumes cs_level = 1
 *  (child of an implicit "root") — sufficient for first bring-up and test.
 *
 * @attention
 * Copyright 2015-2026 (c) UWB project authors.
 * --------------------------------------------------------------------------- */
#include <stdio.h>
#include <string.h>
#include <main.h>
#include <deca_device_api.h>
#include <deca_regs.h>
#include <deca_spi.h>
#include <port.h>
#include <shared_defines.h>
#include <shared_functions.h>
#include <example_selection.h>
#include "auto_anchor.h"
#include "gateway_protocol.h"
#include "firmware_version.h"
#include "../gw_ota/gw_ota.h"
#include "downlink_protocol.h"
#include "clock_sync.h"

#if ENABLE_GATEWAY
#include "gateway_cmd.h"
#else
static int gateway_flash_save_coord(int16_t x, int16_t y, int16_t z)
{
    (void)x; (void)y; (void)z; return 0;
}
#endif

#if defined(TEST_FUSION_ANCHOR)

extern void test_run_info(unsigned char *data);
#define DBG_PRINT(msg) test_run_info((unsigned char *)(msg))

#define APP_NAME "ANCHOR V1.0"   /* BD8NCF migration begins */

/*============================================================================
 * Compile-time configuration
 *============================================================================*/
#define ANCHOR_RX_POLL_MS         78       /* Listen window inside the 80 ms TDMA cycle */
#define ANCHOR_PERIOD_MS          DL_TIMESYNC_PERIOD_MS   /* 80 ms = 16 slots × 5 ms */
#define ANCHOR_MAX_NEIGHBORS      DL_MAX_TRACKED_ANCHORS
#define ANCHOR_SLOT_DTU_US        DL_SLOT_DTU_US           /* 5 ms per TDMA slot */
#define ANCHOR_MAX_SLOTS          16       /* max simultaneous anchors in slot table */

/* Lead time before the scheduled TX instant at which the delayed-TX is armed.
 * 2 ms — comfortably above the jitter of the listen-window wake-up (Sleep(1)
 * granularity) so that the wake-up never lands "too late to arm" and never
 * pushes the transmission into the next cycle. */
#define ANCHOR_TX_LEAD_US         2000u
#define ANCHOR_TX_LEAD_DTU        ((uint64_t)ANCHOR_TX_LEAD_US * UUS_TO_DWT_TIME)

/* Antenna delay — must match Tag (same DW3000 hardware) */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

/* DW3000 channel config — same as Tag (channel 9, 128 preamble, 6.8Mbps) */
extern dwt_txconfig_t txconfig_options;
static dwt_config_t config = {
    UWB_CHANNEL, DWT_PLEN_128, DWT_PAC8,
    9, 9, 1, DWT_BR_6M8, DWT_PHRMODE_STD, DWT_PHRRATE_STD,
    (129 + 8 - 8), DWT_STS_MODE_OFF, DWT_STS_LEN_64, DWT_PDOA_M0
};

/* BD8NCF frame header match (same byte layout, but func=0xEE reused) */
static const uint8_t match_timesync[] = {0x41, 0x88, 0,
                                         0xCA, 0xDE, 0xFF, 0xFF,
                                         0,    0,    0xEE};

/* DS-TWR correction frame header (unicast: dst = the child's rank) */
static const uint8_t match_twr_corr[] = {0x41, 0x88, 0,
                                          0xCA, 0xDE, 0, 0,
                                          0,    0,    FUNC_TWR_CORR};

/*============================================================================
 * Anchor state machine for hierarchical clock sync
 *============================================================================*/
typedef enum {
    STATE_SCAN = 0,   /* Boot: listening only, collecting peer info (5 seconds) */
    STATE_ELECTING,   /* 5s scan ended: comparing ranks, determining role */
    STATE_SYNC,       /* Normal operation: synced to parent */
    STATE_ORPHAN      /* No parent heard for 30s: re-entering SCAN */
} AnchorRunState;

/*============================================================================
 * Manual network startup state (controlled by Gateway command)
 *============================================================================*/
typedef enum {
    NET_IDLE = 0,         /* Power-on: silent, waiting for NETWORK_START */
    NET_STARTING,          /* NETWORK_START received: waiting child delay */
    NET_WAIT_CONFIRM,      /* Sent ALOHA, waiting Root beacon to confirm slot in table */
    NET_RUNNING            /* Root confirmed slot: normal TDMA operation */
} NetworkState;

/*============================================================================
 * Per-anchor runtime state (BD8NCF)
 *============================================================================*/
typedef struct {
    uint16_t           rank;             /* Anchor rank (0..15) — same as SRC addr */
    uint8_t            cs_level;         /* 0 = root, 1+ = child */
    uint64_t           parent_eui64;     /* EUI64 of anchor we sync to (0 if root) */
    uint64_t           my_eui64;         /* This anchor's EUI64 */
    dl_anchor_entry_t  my_pos;           /* My position, broadcast in TimeSync */
    uint64_t           prev_tx_ts;       /* Last actual TX timestamp (40-bit, latched) */
    uint64_t           prev_tx_ts_ext;   /* Extended prev_tx_ts for interval calc (no wrap) */
    uint64_t           prev_scheduled_tx; /* Planned TX time for scheduler (for next period calc) */
    uint32_t           seq;              /* Monotonic seq counter */
    uint32_t           tx_count;
    uint32_t           rx_count;

    /* Hierarchical sync state */
    AnchorRunState     run_state;        /* SCAN / ELECTING / SYNC / ORPHAN */
    uint16_t           parent_rank;      /* Rank of anchor we sync to (0 = no parent) */
    uint32_t           scan_start_ms;    /* When SCAN began (for 5s timeout) */
    uint32_t           last_parent_rx_ms;/* Last time we heard our parent */
    uint32_t           electing_start_ms; /* When ELECTING began (for timeout) */

    /* Per-source clock-sync state: when we RX another anchor's TimeSync, we
     * track its clock offset relative to ours via a Kalman filter. */
    AnchorClockSync    neighbors[ANCHOR_MAX_NEIGHBORS];

    /* Last heard timestamp per neighbor rank — for stale detection */
    uint32_t           last_heard_ms[ANCHOR_MAX_NEIGHBORS];

    /* TDMA TX slot scheduling: each slot holds one rank, anchors TX in rank order */
    uint16_t           slot_owners[DL_MAX_TRACKED_ANCHORS];  /* rank who owns each slot */
    uint8_t            my_slot_idx;    /* this anchor's TX slot (255 = none/root) */
    uint32_t           last_schedule_rx_ms; /* when we last got a slot schedule */
} AnchorState;

static AnchorState g_anchor;

/* TDMA reference state: updated when we receive Root's beacon.
 * Used by non-root anchors to schedule their TX relative to Root's TX time.
 *
 * P0-3: the grid is anchored on the ROOT's clock, but every anchor schedules
 * its own delayed TX in ITS OWN DW3000 SYS_TIME.  The two clocks are NOT the
 * same (each DW3000's SYS_TIME starts at 0 on power-up, so the offset between
 * two anchors equals their power-up skew — potentially seconds!).  Therefore
 * the Root's grid instants MUST be converted into the local clock before they
 * are used as a delayed-TX target, using the clock-sync model we maintain
 * towards the Root (factor_R / offset_R).
 */
static struct {
    uint64_t root_tx_ts;       /* Root's TX timestamp, EXTENDED to 64 bit      */
    uint64_t root_tx_raw;      /* last raw 40-bit value (wrap detection)      */
    uint64_t root_local_ts;    /* our SYS_TIME when that Root frame arrived    */
    uint64_t base_origin;      /* cycle origin in OUR clock (Root ts mapped)   */
    uint64_t base_local_time;  /* our SYS_TIME when that beacon was received   */
    uint32_t root_rx_ms;       /* SysTick when we received that beacon         */
    uint8_t  root_slot;        /* Root's slot index (always 0)                 */
    bool     have_root_ref;    /* true once we've received a Root beacon       */
    uint16_t root_cycle_id;    /* cycle counter carried by the last Root beacon*/
} g_tdma_ref;

/* Grid base for the Root: the Root's own first TX timestamp after entering
 * NET_RUNNING.  Fixed, so the cycle index (now - base) / period is stable. */
static uint64_t s_root_grid_base = 0;

/* Last grid instant (local clock) at which we actually transmitted.
 * Used to guarantee at most ONE transmission per TDMA cycle. */
static uint64_t s_last_tx_target = 0;

/* Extended (64-bit) local SYS_TIME — the DW3000 counter wraps every ~17 s. */
static uint64_t s_sys_time_ext = 0;

/* Participant roster received with NETWORK_START (P1-1). */
static uint16_t s_roster[NET_MAX_PARTICIPANTS];
static uint8_t  s_roster_n = 0;

/*============================================================================
 * DS-TWR state
 *
 * The child's beacon is placed on the Root's grid by mapping the Root's TX
 * instant into the child's clock with the sync model
 *     local = factor * root + offset_dtu.
 * The offset estimate absorbs the Root→Child propagation delay (a one-way
 * observation cannot separate clock offset from path length), so the child's
 * beacon lands one propagation delay LATE on the grid.  That delay is a
 * constant per-anchor bias in the Tag's slot-compensated TDOA — 30 ns
 * (≈ 1 m) at 10 m — which no amount of clock filtering can remove.
 *
 * The round trip removes it:
 *   forward leg  Root TX at T1 → child RX at r1
 *   reverse leg  child TX at T2 → Root RX at r2
 *     (r2 - T1) = prop_R->X + factor_root * (T2 - r1) + prop_X->R
 * The Root solves 2*prop and unicasts the correction (FUNC_TWR_CORR); the
 * child shifts its slot instant back onto the grid.
 *==========================================================================*/

/* The instant WE received the Root frame we used as clock reference, and the
 * cycle id that frame carried.  Reported in every beacon so the Root can
 * close the round trip. */
static uint64_t s_ref_rx_ts   = 0;    /* our clock */
static uint16_t s_ref_cycle   = 0;

/* Correction (DTU) received from the Root: how much our beacon is late on
 * the grid.  Subtracted from our slot instant. */
static uint32_t s_twr_corr_dtu    = 0;
static uint8_t  s_twr_corr_valid  = 0;

/* Root side: one link state per tracked neighbour. */
typedef struct {
    uint64_t child_tx;      /* the child's TX instant (the child's clock)      */
    uint64_t child_rx_ref;  /* the RX instant the child reported (its clock)   */
    uint16_t ref_cycle;     /* the Root cycle id that RX instant refers to    */
    uint8_t  have;          /* a usable measurement was received              */
    uint32_t corr_dtu;      /* smoothed propagation delay (DTU, CHILD's clock) */
    uint8_t  corr_valid;    /* corr_dtu holds a measurement                   */
    uint32_t corr_count;    /* number of round trips folded into corr_dtu      */
} TwrLinkState;

static TwrLinkState s_twr_link[ANCHOR_MAX_NEIGHBORS];
static uint8_t      s_twr_tx_cursor = 0;   /* rotating child for corrections */
static uint32_t     s_twr_last_tx_ms = 0;  /* when the last correction went out */

/*============================================================================
 * Device TX delay calibration (K_dev)
 *
 * The DW3000 does not put the preamble on the air at the programmed DX_TIME:
 * there is a fixed pipeline delay (digital TX start + PA ramp) between the
 * programmed instant and the antenna.  Call it δ_dev.  It is measured here as
 *     δ = (actual TX timestamp) − (programmed instant)
 * and averaged over the last frames.
 *
 * Why it matters (P0-5):
 *   - In the DS-TWR round trip, using the nominal TX instants biases the
 *     solved propagation delay by (δ_root + f·δ_child)/2.  That term is
 *     PER CHILD, so it does not cancel in the Tag's TDOA — it stays in.
 *   - In the Tag's time-of-arrival,  toa = δ_anchor + path,  so every
 *     anchor contributes its own δ as a constant TDOA bias (≈ 30 cm per ns).
 *
 * Both disappear once the TX instant is corrected with the calibrated δ.
 * Note that the programmed instant is truncated to the DX_TIME granularity
 * of 256 DTU (4 ns); because the TDMA period (80 ms = 5,111,840,000 DTU) is
 * an exact multiple of 256 DTU, every grid target shares the same truncation
 * offset — so it is a constant and is absorbed by the calibration.
 *==========================================================================*/
#define TX_DELAY_SAMPLES  16u
#define TX_DELAY_MIN      4u      /* frames needed before the average is used */
static uint64_t s_tx_delay_buf[TX_DELAY_SAMPLES];
static uint8_t  s_tx_delay_idx   = 0;
static uint8_t  s_tx_delay_n     = 0;
static uint32_t s_tx_delay_dtu   = 0;     /* averaged δ in DTU                */
static uint8_t  s_tx_delay_valid = 0;

/* Record one (actual − programmed) TX delay sample and refresh the average. */
static void tx_delay_note(uint64_t delta_dtu)
{
    s_tx_delay_buf[s_tx_delay_idx] = delta_dtu;
    s_tx_delay_idx = (uint8_t)((s_tx_delay_idx + 1u) % TX_DELAY_SAMPLES);
    if (s_tx_delay_n < TX_DELAY_SAMPLES) s_tx_delay_n++;

    uint64_t sum = 0;
    uint8_t i;
    for (i = 0; i < s_tx_delay_n; i++) sum += s_tx_delay_buf[i];
    s_tx_delay_dtu = (uint32_t)(sum / s_tx_delay_n);
    if (s_tx_delay_n >= TX_DELAY_MIN) s_tx_delay_valid = 1;
}

/*============================================================================
 * P1-9: the ONE place where the anchor leaves the grid.
 *
 * Losing the slot silently is the worst failure mode of the TDMA: the
 * anchor simply stops transmitting on the grid and the Tag sees a missing
 * anchor with no explanation.  Every reset of my_slot_idx therefore goes
 * through this function, which logs the reason.
 *==========================================================================*/
static void anchor_clear_slot(const char *reason)
{
    if (g_anchor.my_slot_idx != 255) {
        char dbg[72];
        sprintf(dbg, "SLOT_CLEAR %s (was %u)\r\n",
                reason, (unsigned)g_anchor.my_slot_idx);
        DBG_PRINT(dbg);
    }
    g_anchor.my_slot_idx = 255;
}

/*============================================================================
 * Root side: the ACTUAL TX instant of the last few cycles, keyed by cycle id.
 *
 * The DS-TWR round trip needs the Root's real TX instant (T1).  Using the
 * nominal grid instant instead leaks the grid placement error and δ_root
 * straight into the solved propagation delay (both of them ~ the whole
 * TDOA budget).  The history is short because the reference frame is at most
 * a couple of cycles old.
 *==========================================================================*/
#define ROOT_TX_HIST 8u
static uint64_t s_root_tx_ts[ROOT_TX_HIST];
static uint16_t s_root_tx_cycle[ROOT_TX_HIST];
static uint8_t  s_root_tx_idx = 0;

/* Look up the Root's ACTUAL TX instant for a cycle id (0 = not in history). */
static uint64_t root_tx_ts_for_cycle(uint16_t cycle)
{
    int i;
    for (i = 0; i < ROOT_TX_HIST; i++) {
        /* newest first */
        uint8_t k = (uint8_t)((s_root_tx_idx + 1u + i) % ROOT_TX_HIST);
        if (s_root_tx_ts[k] != 0 && s_root_tx_cycle[k] == cycle) return s_root_tx_ts[k];
    }
    return 0;
}

/* Manual network state: controlled by Gateway's NETWORK_START command */
static NetworkState s_net_state = NET_IDLE;
static uint16_t     s_net_root_rank = 0;     /* designated root rank */
static uint8_t      s_net_slot_idx = 255;    /* this anchor's assigned slot */
static uint32_t     s_net_start_ms = 0;      /* when NETWORK_START was received */
static uint32_t     s_aloha_sent_ms = 0;     /* timestamp of last ALOHA TX (0 = none/cooldown expired) */
static uint32_t     s_confirm_wait_ms = 0;   /* when NET_WAIT_CONFIRM started */
static uint8_t      s_aloha_mode = 0;        /* 1 = in ALOHA fallback (timeout), allows TX even if slot=255 */
static uint32_t     s_last_start_ms = 0;     /* when NETWORK_START was last applied (dedup) */

/*============================================================================
 * Helpers
 *============================================================================*/
static uint64_t pack_eui64_u64(uint8_t *dst, uint64_t eui)
{
    /* EUI64 little-endian (LSB first) */
    int i;
    for (i = 0; i < 8; i++) {
        dst[i] = (uint8_t)((eui >> (i * 8)) & 0xFF);
    }
    return eui;
}

static uint64_t unpack_eui64_u64(const uint8_t *src)
{
    uint64_t eui = 0;
    int i;
    for (i = 0; i < 8; i++) {
        eui |= ((uint64_t)src[i]) << (i * 8);
    }
    return eui;
}

/* Derive 32-bit UID from STM32 96-bit unique ID */
static uint32_t compute_uid32(void)
{
    uint32_t id0 = HAL_GetUIDw0();
    uint32_t id1 = HAL_GetUIDw1();
    uint32_t id2 = HAL_GetUIDw2();
    return id0 ^ id1 ^ id2;
}

/* Get rank from UID — CRC16 over the full 96-bit STM32 unique ID.
 *
 * P1-4: the old scheme was `uid % 1000`, i.e. only 999 distinct ranks for an
 * unlimited number of anchors.  Two anchors in the room could therefore end up
 * with the SAME 802.15.4 short address, and the operator would see them
 * collapsed into one device in the host list.  A CRC16 over the whole UID
 * gives 65535 distinct ranks with a ~2^-16 collision probability.
 * Rank 0 stays reserved for the gateway / legacy master. */
static uint16_t compute_rank_from_uid(void)
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
    if (crc == 0) crc = 1;  /* Reserve 0 for the gateway / legacy master */
    return crc;
}

/* Low 64 bit of the 96-bit STM32 UID, little-endian.
 * P1-4: the rank is only 16 bit, so two anchors can collide.  The gateway
 * compares this fingerprint to detect "same rank, different anchor" and warn
 * the operator instead of silently showing one device twice. */
static uint64_t uid_fingerprint64(void)
{
    uint32_t w0 = HAL_GetUIDw0();
    uint32_t w1 = HAL_GetUIDw1();
    uint64_t fp = 0;
    int i;
    for (i = 0; i < 4; i++) fp |= ((uint64_t)((w0 >> (i * 8)) & 0xFF)) << (i * 8);
    for (i = 0; i < 4; i++) fp |= ((uint64_t)((w1 >> (i * 8)) & 0xFF)) << (32 + i * 8);
    return fp;
}

/* Compare rx buffer against pattern, masking SN/DST/SRC.
 * Returns 1 if header matches (with masked fields ignored), 0 otherwise. */
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

/* Stub: rank → coordinate write — full table lives in master/maintainer.
 * Phase 1.1: each anchor only writes its own row. */
static dl_anchor_entry_t s_anchor_table[DL_MAX_ANCHORS];  /* indexed by rank via linear search */

void dl_set_anchor_coord(uint16_t rank, int16_t x_cm, int16_t y_cm, int16_t z_cm)
{
    int i;
    /* Find existing slot or empty slot */
    for (i = 0; i < (int)DL_MAX_ANCHORS; i++) {
        if (s_anchor_table[i].id == rank || s_anchor_table[i].id == 0) {
            s_anchor_table[i].id   = rank;
            s_anchor_table[i].x_cm = x_cm;
            s_anchor_table[i].y_cm = y_cm;
            s_anchor_table[i].z_cm = z_cm;
            return;
        }
    }
}

void dl_get_anchor_coord(uint16_t rank, dl_anchor_entry_t *out)
{
    int i;
    for (i = 0; i < (int)DL_MAX_ANCHORS; i++) {
        if (s_anchor_table[i].id == rank) {
            *out = s_anchor_table[i];
            return;
        }
    }
    out->id = rank; out->x_cm = 0; out->y_cm = 0; out->z_cm = 0;
}

static int find_or_alloc_neighbor_slot(uint16_t rank)
{
    int i;
    for (i = 0; i < ANCHOR_MAX_NEIGHBORS; i++) {
        /* Allocate on first uninitialized slot */
        if (!g_anchor.neighbors[i].initialized) {
            clock_sync_init(&g_anchor.neighbors[i], 1.0, 0.0,
                            0.0, 0.0, 0.0);  /* defaults */
            g_anchor.neighbors[i].neighbor_rank = rank;
            g_anchor.last_heard_ms[i] = HAL_GetTick();
            return i;
        }
    }
    /* No free slot — overwrite oldest (highest last_heard_ms) */
    int oldest = 0;
    uint32_t oldest_ms = g_anchor.last_heard_ms[0];
    for (i = 1; i < ANCHOR_MAX_NEIGHBORS; i++) {
        if (g_anchor.last_heard_ms[i] > oldest_ms) {
            oldest_ms = g_anchor.last_heard_ms[i];
            oldest = i;
        }
    }
    clock_sync_init(&g_anchor.neighbors[oldest], 1.0, 0.0, 0.0, 0.0, 0.0);
    g_anchor.neighbors[oldest].neighbor_rank = rank;
    g_anchor.last_heard_ms[oldest] = HAL_GetTick();
    return oldest;
}

static int find_neighbor_slot_by_rank(uint16_t rank)
{
    int i;
    for (i = 0; i < ANCHOR_MAX_NEIGHBORS; i++) {
        if (g_anchor.neighbors[i].initialized
            && g_anchor.neighbors[i].neighbor_rank == rank) {
            return i;
        }
    }
    return -1;
}

/* TDMA slot scheduling helpers ------------------------------------------------*/

/* Parse the 8-byte TX slot table from a received Timesync frame.
 * Populates slot_owners[] with the ranks in slot order.
 * Returns the number of active slots. */
static uint8_t parse_slot_table(const uint8_t *buf)
{
    uint8_t count = 0;
    int i;
    for (i = 0; i < ANCHOR_MAX_SLOTS; i++) {
        uint16_t r = (uint16_t)buf[i * 2] | ((uint16_t)buf[i * 2 + 1] << 8);
        g_anchor.slot_owners[i] = r;
        if (r != 0) count++;
    }
    g_anchor.last_schedule_rx_ms = HAL_GetTick();
    return count;
}

/* Find which slot index this anchor's rank occupies.
 * Returns 255 if not found (no slot assigned). */
static uint8_t find_my_slot(uint16_t my_rank)
{
    int i;
    for (i = 0; i < ANCHOR_MAX_SLOTS; i++) {
        if (g_anchor.slot_owners[i] == my_rank) return (uint8_t)i;
    }
    return 255;
}

/* Root only: collect all discovered anchor ranks (from neighbors[]) and fill
 * slot_owners[] in ascending rank order. Self (root rank) goes in slot 0.
 * Call this whenever a new neighbor is discovered or the topology changes. */
static void tdma_build_table(void)
{
    /* Collect up to ANCHOR_MAX_SLOTS unique ranks */
    uint16_t ranks[ANCHOR_MAX_SLOTS];
    int count = 0;
    int i;

    /* Root (cs_level=0) always occupies slot 0 — do not sort root away.
     * This prevents a lower-ranked child anchor from displacing root. */
    if (g_anchor.cs_level == 0) {
        ranks[count++] = g_anchor.rank;  /* slot 0 = root */
    }

    /* Add all discovered neighbors (skip self) */
    for (i = 0; i < ANCHOR_MAX_NEIGHBORS && count < ANCHOR_MAX_SLOTS; i++) {
        if (!g_anchor.neighbors[i].initialized) continue;
        uint16_t r = g_anchor.neighbors[i].neighbor_rank;
        if (r == 0 || r == g_anchor.rank) continue;
        /* Avoid duplicates */
        int j; bool dup = false;
        for (j = 0; j < count; j++) { if (ranks[j] == r) { dup = true; break; } }
        if (!dup) ranks[count++] = r;
    }

    /* Sort non-root ranks ascending (slot 0 already taken by root) */
    int sort_start = (g_anchor.cs_level == 0) ? 1 : 0;
    for (i = sort_start; i < count - 1; i++) {
        int j;
        for (j = i + 1; j < count; j++) {
            if (ranks[j] < ranks[i]) {
                uint16_t tmp = ranks[i]; ranks[i] = ranks[j]; ranks[j] = tmp;
            }
        }
    }

    /* Fill slot_owners with sorted ranks, zero-fill rest */
    for (i = 0; i < ANCHOR_MAX_SLOTS; i++) {
        g_anchor.slot_owners[i] = (i < count) ? ranks[i] : 0;
    }

    /* Update our own slot index — but only if Root has confirmed us.
     * During NET_WAIT_CONFIRM the Child is NOT yet in Root's slot_owners,
     * so find_my_slot() returns 255.
     * P1-2: in that case my_slot_idx stays 255 (the pre-computed bootstrap
     * slot is NOT used) — the Tag then skips this anchor instead of pairing
     * it against a slot instant it does not own. */
    {
        uint8_t real_slot = find_my_slot(g_anchor.rank);
        if (real_slot != 255) {
            g_anchor.my_slot_idx = real_slot;
        }
    }
}

/* Child only: check if our rank appears anywhere in Root's slot_owners table.
 * This confirms that Root has heard our ALOHA beacon and registered us. */
static bool is_in_root_slot_table(void)
{
    int i;
    for (i = 0; i < ANCHOR_MAX_SLOTS; i++) {
        if (g_anchor.slot_owners[i] == g_anchor.rank)
            return true;
    }
    return false;
}

/*============================================================================
 * TDMA grid (P0-3) — ONE single time source for the whole scheduler
 *
 * The old code decided "is it my slot?" from HAL_GetTick() % 50 ms while the
 * actual transmission was scheduled on the DW3000 SYS_TIME axis (root_tx_ts +
 * slot × 5 ms).  Those two clocks are unrelated, so the gate and the radio
 * disagreed: an anchor could pass the gate and then have its delayed TX land
 * outside its slot (or be silently dropped), which in the worst case meant an
 * entire cycle without any beacon from that anchor.  The Tag then paired
 * beacons from two different cycles and produced garbage TDOA.
 *
 * Everything below is derived from the DW3000 SYS_TIME counter:
 *   • the grid origin is the Root's beacon TX timestamp (the Root defines the
 *     cycle),
 *   • our slot instant is  origin + my_slot × 5 ms,
 *   • the Root's clock is mapped into OUR clock with the clock-sync model we
 *     maintain towards the Root (each DW3000's SYS_TIME starts at 0 on
 *     power-up, so the raw Root timestamp is NOT usable as a local target).
 *==========================================================================*/

/* Current SYS_TIME, extended to 64 bit (the 40-bit counter wraps every ~17 s). */
static uint64_t now_sys_ext(void)
{
    uint64_t raw = get_sys_timestamp_u64() & 0xFFFFFFFFFFULL;
    if (s_sys_time_ext != 0 && raw < (s_sys_time_ext & 0xFFFFFFFFFFULL)
        && (s_sys_time_ext - raw) > (1ULL << 39)) {
        raw += (1ULL << 40);
    }
    s_sys_time_ext = raw;
    return raw;
}

/* Clock-sync model of this anchor towards the designated Root (if any). */
static const AnchorClockSync *root_sync(void)
{
    int i;
    if (s_net_root_rank == 0) return NULL;
    for (i = 0; i < ANCHOR_MAX_NEIGHBORS; i++) {
        if (g_anchor.neighbors[i].initialized
            && g_anchor.neighbors[i].neighbor_rank == s_net_root_rank) {
            return &g_anchor.neighbors[i];
        }
    }
    return NULL;
}

/* Mutable variant of root_sync() — used to book TX-side events (GTX freeze,
 * delayed-TX fallback) on the link towards the Root. */
static AnchorClockSync *root_sync_mut(void)
{
    int i;
    if (s_net_root_rank == 0) return NULL;
    for (i = 0; i < ANCHOR_MAX_NEIGHBORS; i++) {
        if (g_anchor.neighbors[i].initialized
            && g_anchor.neighbors[i].neighbor_rank == s_net_root_rank) {
            return &g_anchor.neighbors[i];
        }
    }
    return NULL;
}

/* Map an instant expressed in the ROOT's clock into this anchor's clock.
 * The sync model is  local_rx ≈ factor * remote_tx + offset_dtu, i.e.
 *   local_clock = factor * remote_clock + offset_dtu.
 * Without a converged model we fall back to identity (assume identical
 * clocks) — the grid still works, only the absolute slot accuracy degrades.
 *
 * NOTE: this form is only used for DIAGNOSTICS.  It multiplies the factor
 * error with the absolute timestamp and is therefore far too coarse to
 * place a beacon on the grid — the grid origin is taken from our own RX
 * instant of the Root's beacon instead (see anchor_rx_timesync()). */
static uint64_t root_clock_to_local(uint64_t root_ts)
{
    const AnchorClockSync *s = root_sync();
    if (!s || s->update_count < 3) return root_ts;
    double v = s->factor * (double)root_ts + (double)s->offset_dtu;
    if (v < 0.0) v = 0.0;
    return (uint64_t)v;
}

/* Compute the exact instant of our next slot in the Root's grid.
 *
 *   *target_out : the instant (OUR clock) at which we must transmit
 *   *cycle_out  : TDMA cycle counter to stamp into the frame
 *
 * Returns false when no grid is known yet (no Root beacon heard).
 */
static bool tdma_grid_target(uint64_t *target_out, uint16_t *cycle_out)
{
    uint64_t period = (uint64_t)DL_TIMESYNC_PERIOD_UUS * UUS_TO_DWT_TIME;

    if (g_anchor.cs_level == 0) {
        /* ── Root: the grid origin is our own first beacon of this network ── */
        if (s_root_grid_base == 0) return false;
        uint64_t now = now_sys_ext();
        if (now < s_root_grid_base) return false;
        uint64_t k    = (now - s_root_grid_base) / period;
        uint64_t tgt = s_root_grid_base + k * period;
        uint16_t cyc = (uint16_t)(k & 0xFFFFu);
        if (tgt + ANCHOR_TX_LEAD_DTU < now) {   /* this cycle's instant is gone */
            tgt += period;
            cyc = (uint16_t)(cyc + 1u);
        }
        *target_out = tgt;
        *cycle_out  = cyc;
        return true;
    }

    /* ── Child: origin = the Root's beacon TX instant, mapped to our clock ── */
    if (!g_tdma_ref.have_root_ref) return false;
    if (g_anchor.my_slot_idx == 255) return false;   /* not admitted yet */

    /* The sync model is  local = factor * root + offset_dtu, i.e. `factor`
     * is OUR clock rate relative to the Root's.  An interval measured in
     * the Root's clock is therefore factor × longer in ours.  The old code
     * divided instead of multiplying — invisible while the factor estimate
     * was stuck at 1.0, but at 200 ppm it displaced the slot instant by
     * slot × 5 ms × 200 ppm ≈ 15 µs (≈ 5 m). */
    double f = 1.0;
    const AnchorClockSync *s = root_sync();
    if (s && s->update_count >= 3) f = s->factor;
    if (!(f > 0.5 && f < 2.0)) f = 1.0;
    uint64_t period_local = (uint64_t)((double)period * f);
    if (period_local == 0) period_local = period;

    uint64_t now = now_sys_ext();
    if (now < g_tdma_ref.base_local_time) return false;
    /* Cycles elapsed since the reference beacon (the reference is refreshed
     * on every Root beacon, so this stays within ±1 cycle). */
    uint64_t k    = (now - g_tdma_ref.base_local_time) / period_local;
    uint64_t slot_off = (uint64_t)((double)((uint64_t)g_anchor.my_slot_idx
                                             * ANCHOR_SLOT_DTU_US * UUS_TO_DWT_TIME) * f);
    uint64_t tgt  = g_tdma_ref.base_origin + k * period_local + slot_off;

    /* DS-TWR: the Root→Child propagation delay is already removed from
     * base_origin (it is the instant the Root's beacon was TRANSMITTED, not
     * the instant it arrived), so no further shift is needed here.  The
     * correction s_twr_corr_dtu is applied where base_origin is built. */

    uint16_t cyc  = (uint16_t)(g_tdma_ref.root_cycle_id + (uint16_t)k);
    if (tgt + ANCHOR_TX_LEAD_DTU < now) {
        tgt += period_local;
        cyc = (uint16_t)(cyc + 1u);
    }
    *target_out = tgt;
    *cycle_out  = cyc;
    return true;
}

/* Check if it is safe to transmit right now based on the TDMA schedule.
 *
 * P0-3: the gate and the transmission now share the SAME time source — the
 * function hands the caller the exact instant to transmit at, and it also
 * guarantees at most ONE transmission per TDMA cycle (never two in the same
 * slot, and never a whole cycle skipped because the gate and the radio
 * disagreed).
 *
 * Returns true if TX is permitted; *target_out / *cycle_out are then valid.
 */
static bool tdma_may_tx(uint64_t *target_out, uint16_t *cycle_out)
{
    /* Manual network: NET_IDLE / NET_STARTING / NET_WAIT_CONFIRM → silent.
     * (NET_WAIT_CONFIRM uses the off-grid ALOHA bootstrap instead, which the
     * main loop sends explicitly — see below.) */
    if (s_net_state == NET_IDLE) return false;
    if (s_net_state == NET_STARTING) return false;
    if (s_net_state == NET_WAIT_CONFIRM) return false;

    /* ── ALOHA fallback (after NET_CONFIRM_TIMEOUT) ───────────────────────
     * s_aloha_mode=1 means the Root never confirmed our slot (RF link issue).
     * In this mode we transmit off-grid every ~500 ms to re-establish contact.
     * Off-grid beacons carry slot = 255, so the Tag excludes them from the
     * TDOA solve (P1-2) — they can never corrupt the geometry. */
    if (s_aloha_mode) {
        if (g_anchor.my_slot_idx != 255) s_aloha_mode = 0;   /* Root confirmed us */
        return false;   /* handled by the ALOHA path in the main loop */
    }

    /* ── Normal TDMA operation (NET_RUNNING, slot confirmed) ─────────────
     * The Root's schedule is stale (> 5 s without a Root beacon) — we keep
     * our slot but stop transmitting rather than guessing a slot instant. */
    uint32_t age_ms = HAL_GetTick() - g_anchor.last_schedule_rx_ms;
    if (age_ms > 5000) {
        if (g_anchor.cs_level != 0) {
            /* A child that has not heard the Root for 5 s cannot know its
             * slot instant any more: it leaves the grid instead of
             * transmitting at a guessed instant (a mis-timed beacon
             * poisons the TDOA of every other anchor in that cycle). */
            anchor_clear_slot("ROOT_SCHEDULE_STALE");
            return false;
        }
        /* The Root owns slot 0 and is the clock source of the whole
         * network — it keeps beaconing even when no child has been heard
         * for a while, otherwise a one-way link silences the network. */
    }

    if (!tdma_grid_target(target_out, cycle_out)) return false;

    /* At most one transmission per TDMA cycle: the grid instant of the cycle
     * we already used must not be reused. */
    uint64_t period = (uint64_t)DL_TIMESYNC_PERIOD_UUS * UUS_TO_DWT_TIME;
    if (s_last_tx_target != 0 && *target_out <= s_last_tx_target + period / 2u) {
        return false;   /* this cycle was already transmitted */
    }
    return true;
}

/*============================================================================
 * TimeSync TX — one beacon per TDMA cycle, at our slot instant
 *
 * scheduled_tx : the exact instant (our DW3000 clock) to transmit at.
 *                0 → transmit immediately (bootstrap / ALOHA).
 * cycle_id     : TDMA cycle counter to stamp into the frame (P0-4).
 *============================================================================*/
static void anchor_tx_timesync(uint64_t scheduled_tx, uint16_t cycle_id)
{
    uint8_t pkt[96];
    memset(pkt, 0, sizeof(pkt));   /* zero-initialize: clears TS_UPPER zone (buf[15..17]) */

    /* IEEE 802.15.4 header */
    pkt[0] = 0x41; pkt[1] = 0x88;
    pkt[2] = (uint8_t)(g_anchor.seq & 0xFF);
    pkt[3] = 0xCA; pkt[4] = 0xDE;
    pkt[5] = 0xFF; pkt[6] = 0xFF;
    pkt[7] = (uint8_t)(g_anchor.rank & 0xFF);
    pkt[8] = (uint8_t)((g_anchor.rank >> 8) & 0xFF);
    pkt[9] = FUNC_DOWNLINK_BEACON;

    /* ── P0-2: THIS frame's TX instant, not the previous frame's ──────────
     * The frame used to carry prev_tx_ts_ext while every receiver paired it
     * with THIS frame's rx_local — a whole TDMA period (80 ms) plus the
     * propagation delay leaked into the clock offset estimate, so the filter
     * learned a wrong quantity and the child's grid placement inherited the
     * Root's per-cycle TX jitter.
     *
     * For a delayed (on-grid) frame the TX instant IS the programmed target
     * of dwt_setdelayedtrxtime() — known exactly before the frame is built
     * and accurate to the 8-DTU (125 ps) hardware granularity — so the
     * PRECISE flag is set.
     * For an immediate frame (ALOHA / bootstrap) the real TX instant can only
     * be read after the transmission, so we put the current SYS_TIME (an
     * estimate, off by the SPI write + starttx latency ≈ 30 µs) and CLEAR the
     * PRECISE flag: receivers must not use imprecise timestamps for the
     * clock model or for the grid origin. */
    if (scheduled_tx != 0 && s_tx_delay_valid) {
        /* The advertised TX instant is the programmed instant PLUS the
         * calibrated device TX delay — that is the instant the preamble
         * actually reaches the antenna, which is the only instant every
         * receiver can compare against (clock model, DS-TWR round trip,
         * Tag's TDOA).  Without the +δ the anchor's own TX latency shows up
         * as a constant TDOA bias of δ (≈ 30 cm per ns). */
        pack_40bit_ts(&pkt[DL_TS_TX_TS_IDX],
                      scheduled_tx + (uint64_t)s_tx_delay_dtu);
        pkt[DL_TS_FLAGS_IDX] |= DL_TS_FLAG_TX_TS_PRECISE;
    } else if (scheduled_tx != 0) {
        /* Calibration not yet available (first frames after boot): advertise
         * the programmed instant but do NOT claim it is precise — a δ-biased
         * stamp must not enter the clock model. */
        pack_40bit_ts(&pkt[DL_TS_TX_TS_IDX], scheduled_tx);
    } else {
        pack_40bit_ts(&pkt[DL_TS_TX_TS_IDX], now_sys_ext());
    }
    /* P0-1: cs_level and slot are two DIFFERENT bytes now.  They used to
     * share byte 18, so writing the slot destroyed the level and every
     * receiver saw a corrupted hierarchy. */
    pkt[DL_TS_LEVEL_IDX] = g_anchor.cs_level;
    pkt[DL_TS_SLOT_IDX]  = g_anchor.my_slot_idx;   /* 255 = not admitted */
    pack_eui64_u64(&pkt[DL_TS_PARENT_EUI_IDX], g_anchor.parent_eui64);

    /* Root broadcasts TX slot table (16 slots × 2 bytes = 32 bytes).
     * Layout: uint16_t rank per slot, 0 = empty. All anchors listen to the
     * same table so they can compute their own slot index. Non-root anchors
     * store this table to find their slot but do not re-broadcast it. */
    {
        int si;
        for (si = 0; si < ANCHOR_MAX_SLOTS; si++) {
            uint16_t r = g_anchor.slot_owners[si];
            pkt[DL_TS_SLOT_TABLE_IDX + si * 2]     = (uint8_t)(r & 0xFF);
            pkt[DL_TS_SLOT_TABLE_IDX + si * 2 + 1] = (uint8_t)(r >> 8);
        }
    }

    /* Append anchor coordinates for Tag to use in TDOA solve */
    uint16_t x = (uint16_t)g_anchor.my_pos.x_cm;
    uint16_t y = (uint16_t)g_anchor.my_pos.y_cm;
    uint16_t z = (uint16_t)g_anchor.my_pos.z_cm;
    pkt[DL_TS_COORD_IDX + 0] = (uint8_t)(x & 0xFF);
    pkt[DL_TS_COORD_IDX + 1] = (uint8_t)(x >> 8);
    pkt[DL_TS_COORD_IDX + 2] = (uint8_t)(y & 0xFF);
    pkt[DL_TS_COORD_IDX + 3] = (uint8_t)(y >> 8);
    pkt[DL_TS_COORD_IDX + 4] = (uint8_t)(z & 0xFF);
    pkt[DL_TS_COORD_IDX + 5] = (uint8_t)(z >> 8);
    pkt[DL_TS_COORD_IDX + 6] = 0xCC;  /* DW3210 RX pad */
    pkt[DL_TS_COORD_IDX + 7] = 0xDD;

    /* P0-4: TDMA cycle counter.  The Tag only pairs beacons that carry the
     * same cycle id — one cycle of skew is 80 ms, far beyond the TDOA
     * acceptance window. */
    pkt[DL_TS_CYCLE_IDX]     = (uint8_t)(cycle_id & 0xFF);
    pkt[DL_TS_CYCLE_IDX + 1] = (uint8_t)(cycle_id >> 8);

    /* ── DS-TWR: the clock reference this beacon was built on ─────────────
     * rx_ref_ts40 = the instant WE received the reference frame (our clock),
     * rx_ref_cycle = the cycle id that frame carried.
     * The Root closes the round trip with
     *     (r2 - T1) = prop_R->X + f * (T2 - r1) + prop_X->R
     * (r1 = rx_ref_ts40, T2 = our TX instant above, T1 = the Root's TX
     * instant of the frame with cycle id rx_ref_cycle, r2 = the Root's RX
     * instant) and solves the Root→Child propagation delay.  Without it the
     * child's beacon sits one propagation delay away from the Root's grid,
     * which shows up as a systematic per-anchor bias in the Tag's TDOA. */
    pack_40bit_ts(&pkt[DL_TS_RX_REF_TS_IDX], s_ref_rx_ts);
    pkt[DL_TS_RX_REF_CYCLE_IDX]     = (uint8_t)(s_ref_cycle & 0xFF);
    pkt[DL_TS_RX_REF_CYCLE_IDX + 1] = (uint8_t)(s_ref_cycle >> 8);

    /* P1-1: presence bitmap over the participant roster.  Only the Root
     * fills it in: bit N is set when the Root has heard participant N. */
    {
        uint16_t mask = 0;
        if (g_anchor.cs_level == 0) {
            int i;
            for (i = 0; i < s_roster_n; i++) {
                uint16_t r = s_roster[i];
                bool heard = (r == g_anchor.rank);
                if (!heard) {
                    int j;
                    for (j = 0; j < ANCHOR_MAX_NEIGHBORS; j++) {
                        if (g_anchor.neighbors[j].initialized
                            && g_anchor.neighbors[j].neighbor_rank == r) {
                            heard = true;
                            break;
                        }
                    }
                }
                if (heard) mask |= (uint16_t)(1u << i);
            }
        }
        pkt[DL_TS_PRESENT_IDX]     = (uint8_t)(mask & 0xFF);
        pkt[DL_TS_PRESENT_IDX + 1] = (uint8_t)(mask >> 8);
    }

    /* Quiet radio before TX: mask IRQ + force IDLE to avoid EXTI canceling DTX */
    port_DisableEXT_IRQ();
    dwt_forcetrxoff();
    dwt_writetxdata(DL_TIMESYNC_LEN, pkt, 0);
    dwt_writetxfctrl(DL_TIMESYNC_LEN, 0, 0);

    uint64_t period_dtu = (uint64_t)DL_TIMESYNC_PERIOD_UUS * UUS_TO_DWT_TIME;

    /* SPI jitter test: dwt_starttx() command time, measured with SYS_TIME hi32.
     * Separated for IMMEDIATE vs DELAYED modes (different jitter characteristics).
     * Also separately track fallback events and freeze events. */
    static uint32_t jitter_imm_max, jitter_imm_sum, jitter_imm_cnt;
    static uint32_t jitter_dly_max, jitter_dly_sum, jitter_dly_cnt;
    static uint32_t jitter_fallback_cnt;
    static uint32_t freeze_cnt;

    uint32_t ts_before, ts_after, delta, jitter_dtu;
    uint8_t  tx_freeze = 0, tx_dly_fail = 0;

    if (scheduled_tx == 0) {
        /* Frame 1 (bootstrap) or ALOHA fallback: immediate TX */
        ts_before = dwt_readsystimestamphi32();
        dwt_starttx(DWT_START_TX_IMMEDIATE);
        ts_after = dwt_readsystimestamphi32();
        delta = (ts_after >= ts_before) ? (ts_after - ts_before) : 0;
        jitter_dtu = delta * 256;
        if (delta == 0) { freeze_cnt++; tx_freeze = 1; }
        jitter_imm_sum += jitter_dtu;
        jitter_imm_cnt++;
        if (jitter_dtu > jitter_imm_max) jitter_imm_max = jitter_dtu;
    } else {
        /* Delayed TX at the exact grid instant.
         *
         * P0-2: dwt_setdelayedtrxtime() takes the 32-bit DX_TIME register,
         * which holds device time bits [39:8].  scheduled_tx is a 64-bit
         * extended timestamp, so shifting it down by 8 and truncating to
         * 32 bits silently threw away the upper 8 bits — after one 40-bit
         * wrap (17.2 s) the programmed instant was off by 2^32 DTU = 67 ms,
         * which is larger than the whole TDMA slot.  Mask to 40 bits first
         * (the device time counter is 40 bit), then shift. */
        dwt_setdelayedtrxtime((uint32_t)((scheduled_tx & 0xFFFFFFFFFFULL) >> 8));
        ts_before = dwt_readsystimestamphi32();
        int ret = dwt_starttx(DWT_START_TX_DELAYED);
        ts_after = dwt_readsystimestamphi32();
        delta = (ts_after >= ts_before) ? (ts_after - ts_before) : 0;
        jitter_dtu = delta * 256;
        if (delta == 0) { freeze_cnt++; tx_freeze = 1; }
        jitter_dly_sum += jitter_dtu;
        jitter_dly_cnt++;
        if (jitter_dtu > jitter_dly_max) jitter_dly_max = jitter_dtu;
        if (ret != DWT_SUCCESS) {
            /* Fallback to immediate — the cycle is lost, but the NEXT cycle
             * still uses the grid (the grid itself is not affected). */
            jitter_fallback_cnt++;
            tx_dly_fail = 1;
            dwt_starttx(DWT_START_TX_IMMEDIATE);
        }
    }

    /* Book the TX-side events on the link towards the Root (P1-6: these
     * counters used to be dead fields). */
    clock_sync_note_tx_event(root_sync_mut(), tx_freeze, tx_dly_fail);
    /* Remember the grid instant we used (one TX per cycle, P0-3) */
    if (scheduled_tx != 0) s_last_tx_target = scheduled_tx;
    g_anchor.prev_scheduled_tx = scheduled_tx;

    /* Wait for TX complete (fixed ~143 μs frame duration — not jitter) */
    uint32_t tx_dl = HAL_GetTick() + DL_TIMESYNC_PERIOD_MS + 20u;
    while ((dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK) == 0) {
        if (HAL_GetTick() > tx_dl) break;
    }

    /* [P1] Read TX timestamp BEFORE clearing TXFRS */
    uint64_t ts_raw = get_tx_timestamp_u64();

    /* Clear both TXFRS and HPDWARN */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK | SYS_STATUS_HPDWARN_BIT_MASK);
    port_EnableEXT_IRQ();

    /* Extended-timestamp update with 40-bit wrap detection. */
    if (g_anchor.prev_tx_ts_ext != 0 &&
        ts_raw < g_anchor.prev_tx_ts_ext &&
        (g_anchor.prev_tx_ts_ext - ts_raw) > (1ULL << 39)) {
        ts_raw += (1ULL << 40);
    }
    g_anchor.prev_tx_ts_ext = ts_raw;

    /* ── K_dev: measure the device TX delay ───────────────────────────────
     * ts_raw is the instant the preamble actually left the antenna,
     * scheduled_tx the instant we programmed.  Both are on the same 64-bit
     * extended timeline ✓.  The difference is the fixed pipeline delay
     * (plus the constant DX_TIME truncation offset, see the note above). */
    if (scheduled_tx != 0 && ts_raw > scheduled_tx) {
        tx_delay_note(ts_raw - scheduled_tx);
    }

    /* Root side: remember the ACTUAL TX instant of this cycle — the DS-TWR
     * round trip uses it as T1 (the nominal grid instant would leak the
     * grid placement error into the solved propagation delay). */
    if (g_anchor.cs_level == 0) {
        s_root_tx_ts[s_root_tx_idx]    = ts_raw;
        s_root_tx_cycle[s_root_tx_idx] = cycle_id;
        s_root_tx_idx = (uint8_t)((s_root_tx_idx + 1u) % ROOT_TX_HIST);
    }

    /* Grid accuracy check: how far did the ACTUAL TX instant land from the
     * programmed grid instant?  A CONSTANT offset is common-mode — every
     * anchor has it, so it cancels in the TDOA difference.  A varying one is
     * per-anchor and does not cancel, so the spread is what matters. */
    if (scheduled_tx != 0) {
        static int64_t s_tx_min, s_tx_max;
        static uint32_t s_tx_n;
        int64_t d = (int64_t)(ts_raw - scheduled_tx);
        if (s_tx_n == 0) { s_tx_min = s_tx_max = d; }
        else { if (d < s_tx_min) s_tx_min = d; if (d > s_tx_max) s_tx_max = d; }
        s_tx_n++;
        if ((s_tx_n % 25) == 0) {
            char dbg[72];
            sprintf(dbg, "TX_GRID err=%ld dtu (%.2f cm) spread=%ld dtu\r\n",
                    (long)d, (double)d * CM_PER_DTU_F, (long)(s_tx_max - s_tx_min));
            DBG_PRINT(dbg);
        }
    }

    /* The Root defines the TDMA grid: its very first beacon of this network
     * becomes the cycle origin every child schedules against. */
    if (g_anchor.cs_level == 0 && s_root_grid_base == 0) {
        /* Put the raw 40-bit TX timestamp onto the same 64-bit timeline as
         * now_sys_ext().  Without this an anchor that has been running for
         * more than one SYS_TIME wrap (17.2 s) would see the grid origin
         * jump by 2^40 and the cycle counter start at a random value. */
        uint64_t base = (s_sys_time_ext & ~0xFFFFFFFFFFULL) | (ts_raw & 0xFFFFFFFFFFULL);
        if (s_sys_time_ext != 0 && base > s_sys_time_ext) base -= (1ULL << 40);
        s_root_grid_base = base;
        { char dbg[48]; sprintf(dbg, "GRID_BASE set ts=%llu\r\n",
            (unsigned long long)ts_raw); DBG_PRINT(dbg); }
    }

    /* Print jitter stats every 200 frames (guard against div-by-zero) */
    if ((g_anchor.seq % 200) == 0 && (jitter_imm_cnt > 0 || jitter_dly_cnt > 0)) {
        char dbg[128];
        int len = 0;
        if (jitter_imm_cnt > 0) {
            len = sprintf(dbg, "TX_JITTER IMM:avg=%lu max=%lu DTU\r\n",
                (unsigned long)(jitter_imm_sum / jitter_imm_cnt),
                (unsigned long)jitter_imm_max);
        }
        if (jitter_dly_cnt > 0) {
            len += sprintf(dbg + len, "TX_JITTER DLY:avg=%lu max=%lu DTU\r\n",
                (unsigned long)(jitter_dly_sum / jitter_dly_cnt),
                (unsigned long)jitter_dly_max);
        }
        if (jitter_fallback_cnt > 0) {
            len += sprintf(dbg + len, "TX_JITTER FALLBACKS=%lu\r\n",
                (unsigned long)jitter_fallback_cnt);
        }
        if (freeze_cnt > 0) {
            len += sprintf(dbg + len, "TX_JITTER FREEZE=%lu\r\n",
                (unsigned long)freeze_cnt);
        }
        DBG_PRINT(dbg);
        /* Reset */
        jitter_imm_sum = jitter_imm_cnt = jitter_imm_max = 0;
        jitter_dly_sum = jitter_dly_cnt = jitter_dly_max = 0;
        jitter_fallback_cnt = 0;
        freeze_cnt = 0;
    }

    g_anchor.seq++;
    g_anchor.tx_count++;
}

/*============================================================================
 * DS-TWR correction frame — Root → Child (unicast)
 *
 * Carries the one-way propagation delay Root→Child that the Root measured
 * from the round trip.  The child subtracts it from its own slot instant, so
 * its beacon lands exactly on the Root's grid instead of one propagation
 * delay late.  Without this every anchor contributes a systematic, distance
 * dependent bias to the Tag's TDOA solution.
 *
 * Frame (17 B):
 *   [0..1]  frame type 0x41 0x88
 *   [2]     sequence
 *   [3..4]  source EUI64 tag 0xCA 0xDE
 *   [5..6]  destination rank (unicast)
 *   [7..8]  source rank (Root)
 *   [9]     FUNC_TWR_CORR
 *   [10..11]target rank (uint16 LE, redundant with dst, for validation)
 *   [12..15]propagation delay, DTU (uint32 LE, in the CHILD's clock)
 *   [16]    quality / status
 *============================================================================*/
static void anchor_tx_twr_corr(uint16_t dst_rank, uint32_t corr_dtu, uint8_t quality)
{
    uint8_t pkt[DL_TWR_CORR_LEN];
    memset(pkt, 0, sizeof(pkt));

    pkt[0] = 0x41; pkt[1] = 0x88;
    pkt[2] = (uint8_t)(g_anchor.seq & 0xFF);
    pkt[3] = 0xCA; pkt[4] = 0xDE;
    pkt[5] = (uint8_t)(dst_rank & 0xFF);
    pkt[6] = (uint8_t)(dst_rank >> 8);
    pkt[7] = (uint8_t)(g_anchor.rank & 0xFF);
    pkt[8] = (uint8_t)(g_anchor.rank >> 8);
    pkt[9] = FUNC_TWR_CORR;
    pkt[DL_TWR_CORR_RANK_IDX]     = (uint8_t)(dst_rank & 0xFF);
    pkt[DL_TWR_CORR_RANK_IDX + 1] = (uint8_t)(dst_rank >> 8);
    pkt[DL_TWR_CORR_PROP_IDX]     = (uint8_t)(corr_dtu & 0xFF);
    pkt[DL_TWR_CORR_PROP_IDX + 1] = (uint8_t)((corr_dtu >> 8) & 0xFF);
    pkt[DL_TWR_CORR_PROP_IDX + 2] = (uint8_t)((corr_dtu >> 16) & 0xFF);
    pkt[DL_TWR_CORR_PROP_IDX + 3] = (uint8_t)((corr_dtu >> 24) & 0xFF);
    pkt[DL_TWR_CORR_QUAL_IDX] = quality;

    port_DisableEXT_IRQ();
    dwt_forcetrxoff();
    dwt_writetxdata(DL_TWR_CORR_LEN, pkt, 0);
    dwt_writetxfctrl(DL_TWR_CORR_LEN, 0, 0);
    dwt_starttx(DWT_START_TX_IMMEDIATE);
    { uint32_t dl = HAL_GetTick() + 10u;
      while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {
        if (HAL_GetTick() > dl) break;
      } }
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK | SYS_STATUS_HPDWARN_BIT_MASK);
    port_EnableEXT_IRQ();

    g_anchor.seq++;
    g_anchor.tx_count++;
}

/* Root side: send the pending correction, one child per call, round-robin.
 * Called right after the Root's own grid beacon (slot 0) so the frame lands
 * in the slot-0 window and never collides with a child's slot. */
static void anchor_twr_corr_poll(void)
{
    if (g_anchor.cs_level != 0) return;              /* Root only */
    if (s_root_grid_base == 0) return;

    uint32_t now_ms = HAL_GetTick();
    if ((now_ms - s_twr_last_tx_ms) < 60u) return;   /* at most one per cycle */

    int i;
    for (i = 0; i < ANCHOR_MAX_NEIGHBORS; i++) {
        int j = (s_twr_tx_cursor + i) % ANCHOR_MAX_NEIGHBORS;
        TwrLinkState *L = &s_twr_link[j];
        if (!L->have || !L->corr_valid) continue;
        if (!g_anchor.neighbors[j].initialized) continue;

        anchor_tx_twr_corr(g_anchor.neighbors[j].neighbor_rank,
                           L->corr_dtu, 0x01);
        s_twr_tx_cursor = (uint8_t)((j + 1) % ANCHOR_MAX_NEIGHBORS);
        s_twr_last_tx_ms = now_ms;
        { char dbg[56];
          sprintf(dbg, "TWR_CORR TX R=%u %lu dtu (%.1f cm)\r\n",
                  (unsigned)g_anchor.neighbors[j].neighbor_rank,
                  (unsigned long)L->corr_dtu,
                  (double)L->corr_dtu * CM_PER_DTU_F);
          DBG_PRINT(dbg); }
        return;
    }
}

/*============================================================================
 * DS-TWR correction received from the Root (child side)
 *============================================================================*/
static void anchor_handle_twr_corr(const uint8_t *rx_buf, uint32_t frame_len)
{
    if (frame_len < DL_TWR_CORR_LEN) return;

    uint16_t dst = (uint16_t)rx_buf[5] | ((uint16_t)rx_buf[6] << 8);
    if (dst != 0xFFFF && dst != g_anchor.rank) return;   /* not for us */

    uint16_t tgt = (uint16_t)(rx_buf[DL_TWR_CORR_RANK_IDX]
                   | ((uint16_t)rx_buf[DL_TWR_CORR_RANK_IDX + 1] << 8));
    if (tgt != g_anchor.rank) return;                     /* stale/other child */

    uint32_t corr = (uint32_t)rx_buf[DL_TWR_CORR_PROP_IDX]
                  | ((uint32_t)rx_buf[DL_TWR_CORR_PROP_IDX + 1] << 8)
                  | ((uint32_t)rx_buf[DL_TWR_CORR_PROP_IDX + 2] << 16)
                  | ((uint32_t)rx_buf[DL_TWR_CORR_PROP_IDX + 3] << 24);

    /* Sanity: 0.2 m .. 100 m, in DTU (1 m = 213.20 DTU) */
    if (corr < DL_TWR_CORR_MIN_DTU || corr > DL_TWR_CORR_MAX_DTU) {
        char dbg[48];
        sprintf(dbg, "TWR_CORR bad prop=%lu dtu\r\n", (unsigned long)corr);
        DBG_PRINT(dbg);
        return;
    }

    /* The delay is already expressed in OUR clock units — the Root divides
     * the solved path by f_root before sending it. */
    uint32_t prop = corr;

    if (!s_twr_corr_valid) {
        s_twr_corr_dtu = prop;
        s_twr_corr_valid = 1;
    } else {
        /* IIR smoothing, same filter the Root uses on its own measurement. */
        s_twr_corr_dtu = (s_twr_corr_dtu * 3u + prop) / 4u;
    }
    { char dbg[56];
      sprintf(dbg, "TWR_CORR RX prop=%lu dtu (%.1f cm)\r\n",
              (unsigned long)prop, (double)prop * CM_PER_DTU_F);
      DBG_PRINT(dbg); }
}

/*============================================================================
 * Handle NETWORK_START command from Gateway
 *============================================================================*/
static void anchor_handle_network_start(const uint8_t *rx_buf, uint32_t frame_len)
{
    uint16_t designated_root = (uint16_t)rx_buf[DL_TS_NET_ROOT_IDX]
                            | ((uint16_t)rx_buf[DL_TS_NET_ROOT_IDX + 1] << 8);
    uint8_t flags = rx_buf[DL_TS_NET_FLAGS_IDX];

    /* The gateway broadcasts NETWORK_START in a burst (30 × ~10 ms).
     * Re-applying it on every frame would reset the neighbour table and the
     * child's slot timer over and over, so the child would never reach
     * NET_WAIT_CONFIRM.  Treat repeats of the SAME root within the dedup
     * window as the same command: keep the current state (the caller still
     * re-sends the ACK).  A genuine re-run (same root, later than the
     * window, or a different root) falls through and does a full reset. */
    {
        uint32_t now_ms = HAL_GetTick();
        if (s_net_state != NET_IDLE && s_net_root_rank == designated_root &&
            (int32_t)(now_ms - s_last_start_ms) < NET_START_DEDUP_MS) {
            s_last_start_ms = now_ms;
            return;
        }
        s_last_start_ms = now_ms;
    }

    /* Force reset: clear neighbor table and reset state */
    if (flags & NET_FLAG_FORCE_RESET) {
        s_net_state = NET_IDLE;
        s_aloha_mode = 0;
        s_aloha_sent_ms = 0;
        g_anchor.run_state = STATE_SCAN;
        anchor_clear_slot("NET_FORCE_RESET");
        int ni;
        for (ni = 0; ni < ANCHOR_MAX_NEIGHBORS; ni++) {
            g_anchor.neighbors[ni].initialized = false;
            g_anchor.last_heard_ms[ni] = 0;
        }
        for (ni = 0; ni < ANCHOR_MAX_SLOTS; ni++) {
            g_anchor.slot_owners[ni] = 0;
        }
    }

    /* P1-1: participant roster.  Every anchor receives the SAME ordered list,
     * so bit N of the Root's presence bitmap always means the same anchor. */
    s_roster_n = 0;
    {
        uint8_t n = rx_buf[DL_TS_NET_NPART_IDX];
        int i;
        if (n > NET_MAX_PARTICIPANTS) n = NET_MAX_PARTICIPANTS;
        /* Only trust the roster entries that are actually present in the frame */
        if ((uint32_t)(DL_TS_NET_ROSTER_IDX + (int)n * 2) > frame_len) {
            n = (uint8_t)((frame_len > DL_TS_NET_ROSTER_IDX)
                          ? (frame_len - DL_TS_NET_ROSTER_IDX) / 2u : 0u);
        }
        for (i = 0; i < (int)n; i++) {
            int off = DL_TS_NET_ROSTER_IDX + i * 2;
            s_roster[i] = (uint16_t)(rx_buf[off] | ((uint16_t)rx_buf[off + 1] << 8));
        }
        s_roster_n = n;
    }

    s_net_root_rank = designated_root;

    /* Restart the TDMA grid from scratch (P0-3): the Root's beacon defines
     * the cycle origin, so a fresh network needs a fresh origin. */
    s_root_grid_base = 0;
    s_last_tx_target = 0;
    g_tdma_ref.have_root_ref = false;
    g_tdma_ref.root_cycle_id = 0;
    g_tdma_ref.root_tx_ts = 0;
    g_tdma_ref.root_local_ts = 0;

    if (g_anchor.rank == designated_root) {
        /* I am the Root — start immediately */
        g_anchor.cs_level = 0;
        g_anchor.parent_rank = 0;
        g_anchor.my_slot_idx = 0;
        s_net_slot_idx = 0;
        s_net_state = NET_RUNNING;
        s_aloha_mode = 0;
        s_aloha_sent_ms = 0;
        g_anchor.run_state = STATE_SYNC;
        tdma_build_table();
        { char dbg[64]; sprintf(dbg, "NET_START→ROOT(slot=0) n=%u\r\n",
            (unsigned)s_roster_n); DBG_PRINT(dbg); }
    } else {
        /* I am a Child — wait before TX so Root can go first */
        g_anchor.cs_level = 1;
        g_anchor.parent_rank = designated_root;
        /* Compute a tentative slot from the 16-bit rank distance to the root.
         * This is only a BOOTSTRAP slot — the Root's table is authoritative
         * and overrides it as soon as this anchor is admitted.
         *
         * P1-2: the tentative slot is reported in the NETWORK_ACK but NOT
         * used for scheduling.  Until the Root confirms us our beacon carries
         * slot = 255, which makes the Tag exclude us from the TDOA solve
         * instead of pairing us against the wrong slot instant. */
        uint32_t diff = (g_anchor.rank > designated_root)
                        ? (uint32_t)(g_anchor.rank - designated_root)
                        : (uint32_t)(g_anchor.rank + 65536u - designated_root);
        s_net_slot_idx = (uint8_t)((diff % (NET_MAX_SLOTS - 1u)) + 1u);  /* slot 1..15 */
        if (s_net_slot_idx >= NET_MAX_SLOTS) s_net_slot_idx = NET_MAX_SLOTS - 1;
        anchor_clear_slot("NOT_ADMITTED_YET");   /* P1-2 */
        s_net_state = NET_STARTING;
        s_net_start_ms = HAL_GetTick();
        s_aloha_sent_ms = 0;
        s_aloha_mode = 0;  /* reset on fresh NETWORK_START */
        g_anchor.run_state = STATE_SYNC;
        tdma_build_table();
        { char dbg[64]; sprintf(dbg, "NET_START→CHILD(slot=%u) parent=R%u\r\n",
            s_net_slot_idx, designated_root); DBG_PRINT(dbg); }
    }
}

/*============================================================================
 * Send NETWORK_ACK to Gateway (unicast to rank 0)
 *============================================================================*/
static void anchor_send_network_ack(void)
{
    uint8_t ack[FUNC_NETWORK_ACK_LEN];
    memset(ack, 0, sizeof(ack));

    ack[0] = 0x41; ack[1] = 0x88;
    ack[2] = (uint8_t)(g_anchor.seq & 0xFF);
    ack[3] = 0xCA; ack[4] = 0xDE;
    /* dest = Gateway short address (GW_MASTER_ADDR) */
    ack[5] = (uint8_t)(GW_MASTER_ADDR & 0xFF);
    ack[6] = (uint8_t)(GW_MASTER_ADDR >> 8);
    /* src = my rank */
    ack[7] = (uint8_t)(g_anchor.rank & 0xFF);
    ack[8] = (uint8_t)(g_anchor.rank >> 8);
    ack[9] = FUNC_NETWORK_ACK;
    /* root_rank confirmed */
    ack[DL_TS_NETACK_ROOT_IDX]   = (uint8_t)(s_net_root_rank & 0xFF);
    ack[DL_TS_NETACK_ROOT_IDX+1] = (uint8_t)(s_net_root_rank >> 8);
    /* cs_level and slot */
    ack[DL_TS_NETACK_LEVEL_IDX] = g_anchor.cs_level;
    ack[DL_TS_NETACK_SLOT_IDX]  = s_net_slot_idx;
    /* neighbor count */
    { uint8_t nn = 0; int ni;
      for (ni = 0; ni < ANCHOR_MAX_NEIGHBORS; ni++) {
          if (g_anchor.neighbors[ni].initialized) nn++;
      }
      ack[DL_TS_NETACK_NNEIGH_IDX] = nn;
    }

    port_DisableEXT_IRQ();
    dwt_forcetrxoff();
    dwt_writetxdata(FUNC_NETWORK_ACK_LEN, ack, 0);
    dwt_writetxfctrl(FUNC_NETWORK_ACK_LEN, 0, 0);
    dwt_starttx(DWT_START_TX_IMMEDIATE);

    { uint32_t dl = HAL_GetTick() + 10u;
      while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {
          if (HAL_GetTick() > dl) break;
      }
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK | SYS_STATUS_HPDWARN_BIT_MASK);
      port_EnableEXT_IRQ(); }

    { char dbg[32]; sprintf(dbg, "NET_ACK TX\r\n"); DBG_PRINT(dbg); }
}

/*============================================================================
 * Anchor Discovery TX — broadcast while not yet in a network
 *
 * Without this frame an anchor that has not been networked is completely
 * silent: the gateway never hears from it, the host never lists it, and the
 * operator has nothing to pick as Root  →  hard deadlock on manual network
 * formation.  Every anchor announces itself here with FUNC_ANCHOR_DISCOVERY
 * (0xEC) at NET_DISCOVERY_PERIOD_MS intervals while s_net_state == NET_IDLE.
 *
 * The frame is unicast-free (broadcast dst 0xFFFF) and carries the anchor
 * rank as SRC so the gateway can register it directly.
 *============================================================================*/
static void anchor_tx_discovery(void)
{
    uint8_t dsc[FUNC_ANCHOR_DISCOVERY_LEN];
    static uint8_t disc_seq = 0;

    memset(dsc, 0, sizeof(dsc));

    dsc[0] = 0x41; dsc[1] = 0x88;
    dsc[2] = disc_seq++;
    dsc[3] = 0xCA; dsc[4] = 0xDE;
    dsc[5] = 0xFF; dsc[6] = 0xFF;          /* broadcast */
    dsc[7] = (uint8_t)(g_anchor.rank & 0xFF);
    dsc[8] = (uint8_t)((g_anchor.rank >> 8) & 0xFF);
    dsc[9] = FUNC_ANCHOR_DISCOVERY;

    dsc[DL_TS_DISC_RANK_IDX]     = (uint8_t)(g_anchor.rank & 0xFF);
    dsc[DL_TS_DISC_RANK_IDX + 1] = (uint8_t)((g_anchor.rank >> 8) & 0xFF);
    dsc[DL_TS_DISC_FW_IDX]       = (uint8_t)(FW_VERSION_U16 & 0xFF);
    dsc[DL_TS_DISC_FW_IDX + 1]   = (uint8_t)((FW_VERSION_U16 >> 8) & 0xFF);
    dsc[DL_TS_DISC_STATE_IDX]    = (uint8_t)s_net_state;
    /* P1-4: UID fingerprint — lets the gateway detect two different anchors
     * that ended up with the same 16-bit rank. */
    pack_eui64_u64(&dsc[DL_TS_DISC_UID_IDX], uid_fingerprint64());

    port_DisableEXT_IRQ();
    dwt_forcetrxoff();
    dwt_writetxdata(FUNC_ANCHOR_DISCOVERY_LEN, dsc, 0);
    dwt_writetxfctrl(FUNC_ANCHOR_DISCOVERY_LEN, 0, 0);
    dwt_starttx(DWT_START_TX_IMMEDIATE);

    { uint32_t dl = HAL_GetTick() + 10u;
      while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {
          if (HAL_GetTick() > dl) break;
      }
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK | SYS_STATUS_HPDWARN_BIT_MASK);
      port_EnableEXT_IRQ(); }
}

/*============================================================================
 * TimeSync RX — listen for neighbor anchors' TimeSync frames
 *============================================================================*/
static void anchor_rx_timesync(uint8_t *rx_buf, uint32_t frame_len)
{
    /* [FIX4] Capture rx_local BEFORE parsing the buffer — minimizes ISR delay.
     * get_rx_timestamp_u64() reads the DW3000 hardware register directly,
     * not the buffer, so this is safe to do as soon as the frame is received. */
    uint64_t rx_local = get_rx_timestamp_u64();

    if (frame_len < DL_TIMESYNC_LEN) return;

    uint16_t src_rank = (uint16_t)rx_buf[7] | ((uint16_t)rx_buf[8] << 8);
    uint8_t func = rx_buf[9];

    if (src_rank == 0 || src_rank == g_anchor.rank) return;  /* skip self and duplicates */
    /* Note: ANCHOR_MAX_NEIGHBORS limits how many neighbors we track simultaneously
     * (LRU eviction if > 8), but does NOT restrict which rank values are accepted.
     * Rank 633/643 > 254 is perfectly valid and must be processed. */

    /* P1 fix: unpack extended 64-bit timestamp to avoid 40-bit wrap ambiguity */
    uint64_t tx_remote  = unpack_40bit_ts(&rx_buf[DL_TS_TX_TS_IDX]);

    /* P0-2: only a frame whose TX timestamp is EXACT may be used for the
     * clock model or as the grid origin.  An immediate frame carries an
     * estimate that is off by the SPI-write + starttx latency (≈ 30 µs),
     * which is 1000× the propagation delay we are trying to measure. */
    uint8_t  ts_flags   = rx_buf[DL_TS_FLAGS_IDX];
    bool     ts_precise = (ts_flags & DL_TS_FLAG_TX_TS_PRECISE) ? true : false;

    /* Update / allocate Kalman slot */
    int prev_slot = find_neighbor_slot_by_rank(src_rank);
    int slot = (prev_slot >= 0) ? prev_slot : find_or_alloc_neighbor_slot(src_rank);
    /* Root always rebuilds slot table on every RX to keep it current */
    if (g_anchor.cs_level == 0) {
        tdma_build_table();
    }

    /* ── NET_WAIT_CONFIRM: Child waiting for Root to confirm our slot.
     * Every time we receive a Root beacon (src = designated_root), check if
     * our rank now appears in the slot_owners table. If yes → we are confirmed.
     * If timeout expires → give up and fall back to ALOHA (NET_RUNNING). */
    if (s_net_state == NET_WAIT_CONFIRM) {
        if (src_rank == s_net_root_rank) {
            if (is_in_root_slot_table()) {
                s_net_state = NET_RUNNING;
                s_aloha_sent_ms = 0;
                s_aloha_mode = 0;  /* confirmed: normal TDMA, no ALOHA */
                { char dbg[32]; sprintf(dbg, "NET_CONFIRMED→RUNNING\r\n"); DBG_PRINT(dbg); }
            } else if ((HAL_GetTick() - s_confirm_wait_ms) > NET_CONFIRM_TIMEOUT_MS) {
                /* Timeout: Root never heard our ALOHA. Fall back to ALOHA mode.
                 * Mark slot invalid so tdma_may_tx() uses its stale-slot ALOHA path. */
                s_net_state = NET_RUNNING;
                anchor_clear_slot("CONFIRM_TIMEOUT");
                s_aloha_sent_ms = 0;
                s_aloha_mode = 1;  /* enter ALOHA fallback — TX allowed even if slot=255 */
                { char dbg[48]; sprintf(dbg, "NET_CONFIRM_TIMEOUT→ALOHA\r\n"); DBG_PRINT(dbg); }
            }
        }
    }

    /* ── Hierarchical election ─────────────────────────────────────────
     * In SYNC state: hear lower cs_level (e.g. level 0 root) → adopt level+1.
     * In SCAN/ELECTING: hear lower rank → immediately become child (level 1).
     *   This handles the case where all anchors start at cs_level=1 and the
     *   first one to discover a lower-rank peer wins as child.
     * parent_rank is the sync parent identifier. */
    uint8_t sender_level = rx_buf[DL_TS_CS_LEVEL_IDX];
    AnchorRunState prev_state = g_anchor.run_state;
    if (sender_level < g_anchor.cs_level) {
        g_anchor.cs_level = sender_level + 1;
        g_anchor.parent_rank = src_rank;
        g_anchor.last_parent_rx_ms = HAL_GetTick();
        g_anchor.run_state = STATE_SYNC;
    } else if (prev_state != STATE_SYNC && src_rank < g_anchor.rank) {
        /* In SCAN/ELECTING: heard a lower-ranked anchor → become its child.
         * This catches the case where peer hasn't reached SYNC yet (cs_level=1). */
        g_anchor.cs_level = 1;
        g_anchor.parent_rank = src_rank;
        g_anchor.last_parent_rx_ms = HAL_GetTick();
        g_anchor.run_state = STATE_SYNC;
    }
    /* Print ELECT only on the transition into SYNC (not every frame) */
    if (prev_state != STATE_SYNC && g_anchor.run_state == STATE_SYNC) {
        char dbg[48];
        sprintf(dbg, "ELECT[L=%u P=%u]\r\n",
                (unsigned)g_anchor.cs_level, (unsigned)g_anchor.parent_rank);
        DBG_PRINT(dbg);
    }
    /* Track parent alive heartbeat */
    if (src_rank == g_anchor.parent_rank) {
        g_anchor.last_parent_rx_ms = HAL_GetTick();
    }

    /* Parse TX slot table if present.
     *
     * ONLY a non-root anchor adopts the received table.  The Root's table is
     * authoritative and was just rebuilt from its own neighbour list above
     * (tdma_build_table) — which now includes the anchor that just sent this
     * frame.  Letting the Root overwrite its table with the copy echoed by a
     * child would erase that very child again (the child's copy is one
     * broadcast old), and the child could then never be confirmed. */
    if (frame_len >= DL_TS_SLOT_TABLE_IDX + ANCHOR_MAX_SLOTS * 2u) {
        /* Adopt the table ONLY from a Root frame (sender_level == 0).
         * A child echoes the table it last received, which is one broadcast
         * old — adopting that would roll our slot assignment backwards. */
        if (g_anchor.cs_level != 0 && sender_level == 0) {
            uint8_t cnt = parse_slot_table(&rx_buf[DL_TS_SLOT_TABLE_IDX]);
            (void)cnt;   /* unused but available for future diagnostics */
            /* Update our assigned slot index from the received table — but
             * only if the table actually contains us.  Until the Root has
             * admitted us, find_my_slot() returns 255 and we must keep the
             * pre-computed slot, otherwise TX would be blocked for 3 s. */
            {
                uint8_t real_slot = find_my_slot(g_anchor.rank);
                if (real_slot != 255) {
                    g_anchor.my_slot_idx = real_slot;
                    s_net_slot_idx = real_slot;   /* authoritative slot (P1-2) */
                }
            }
        } else if (g_anchor.cs_level == 0) {
            /* Root: keep our own table, but refresh the schedule heartbeat. */
            g_anchor.last_schedule_rx_ms = HAL_GetTick();
        }

        /* ALOHA fallback recovery: the Root may admit us long after the
         * 3 s confirmation window expired (e.g. the first ALOHA was lost).
         * Now that the fresh table is parsed, check again and switch back to
         * normal TDMA if we are in it. */
        if (s_aloha_mode && s_net_state == NET_RUNNING
            && src_rank == s_net_root_rank && is_in_root_slot_table()) {
            s_aloha_mode = 0;
            s_aloha_sent_ms = 0;
            {
                uint8_t real_slot = find_my_slot(g_anchor.rank);
                if (real_slot != 255) {
                    g_anchor.my_slot_idx = real_slot;
                    s_net_slot_idx = real_slot;
                }
            }
            { char dbg[48]; sprintf(dbg, "ALOHA→TDMA (slot=%u)\r\n",
                (unsigned)g_anchor.my_slot_idx); DBG_PRINT(dbg); }
        }
    }

    /* TDMA reference update: if this is a Root (cs_level=0) beacon,
     * capture its TX timestamp so child anchors can schedule delayed TX.
     *
     * P0-3: the Root's timestamp lives in the ROOT's clock domain, ours lives
     * in ours — the two are NOT the same (each DW3000's SYS_TIME starts at 0
     * on power-up).  The Root's cycle origin is therefore mapped into our
     * clock with the sync model we maintain towards the Root before it is
     * used as a scheduling base. */
    if (sender_level == 0 && ts_precise) {
        uint64_t now_local = now_sys_ext();

        /* Extend the Root's 40-bit timestamp to 64 bit (it wraps every
         * ~17.2 s).
         *
         * P1-5: the extension used to be derived from the PREVIOUS Root
         * frame (self-referential), so an outage longer than one wrap left
         * the high part wrong by a multiple of 2^40 and it could never
         * recover.  The wrap count is now taken from OUR clock: the two
         * crystals run at the same rate (the clock-sync factor is within
         * a few ppm), so the number of wraps the Root's counter went
         * through during an outage is known from the elapsed local time.
         * The absolute offset between the two clocks is unknown — and
         * irrelevant: only the RELATIVE progression is used. */
        uint64_t raw = tx_remote & 0xFFFFFFFFFFULL;
        uint64_t root_ext;
        if (!g_tdma_ref.have_root_ref) {
            /* First Root frame: the Root's counter starts at 0 when it is
             * powered on, so its epoch is simply the raw value. */
            root_ext = raw;
        } else {
            /* Estimate the Root's counter from OUR clock: both crystals
             * run at the same rate (within a few ppm), so the Root's
             * counter advanced by ~elapsed since the previous frame.  The
             * unknown constant offset between the two clocks cancels out,
             * only the progression is used. */
            uint64_t elapsed = now_local - g_tdma_ref.root_local_ts;
            uint64_t expect = g_tdma_ref.root_tx_ts + elapsed;
            root_ext = (expect & ~0xFFFFFFFFFFULL) | raw;
            /* Pick the wrap that puts the value closest to the estimate. */
            if (root_ext + (1ULL << 39) < expect) {
                root_ext += (1ULL << 40);
            } else if (root_ext > expect + (1ULL << 39)) {
                root_ext -= (1ULL << 40);
            }
        }
        g_tdma_ref.root_tx_ts = root_ext;
        g_tdma_ref.root_local_ts = now_local;
        g_tdma_ref.root_rx_ms = HAL_GetTick();
        g_tdma_ref.root_slot = 0;
        g_tdma_ref.have_root_ref = true;

        /* Cycle counter carried by the Root (P0-4) */
        if (frame_len >= DL_TS_CYCLE_IDX + 2) {
            g_tdma_ref.root_cycle_id = (uint16_t)(rx_buf[DL_TS_CYCLE_IDX]
                                    | ((uint16_t)rx_buf[DL_TS_CYCLE_IDX + 1] << 8));
        }

        /* Grid reference: the Root's beacon instant, expressed in our clock,
         * plus the local time at which we received it.
         *
         * The origin is the instant WE received the Root's beacon, minus the
         * Root→Child propagation delay (which the DS-TWR round trip measures
         * and the Root unicasts back to us).  It is deliberately NOT computed
         * as factor * root_ts + offset: that form multiplies the factor
         * error by the absolute timestamp (1 ppm at 1e12 DTU = 15 µs = 4.7 m)
         * and it also carries the propagation delay inside the offset.
         *
         * Using our own RX instant keeps the factor error acting only on the
         * elapsed time since this reference (< one TDMA cycle), i.e. the grid
         * placement error is bounded by 40 ms × Δf ≈ 40 ns at 1 ppm. */
        g_tdma_ref.base_origin = rx_local;
        if (s_twr_corr_valid) {
            g_tdma_ref.base_origin -= (uint64_t)s_twr_corr_dtu;
        }
        /* Subtract our OWN device TX delay as well.
         *
         * The grid instant is the instant the Root's beacon left the
         * antenna; our own beacon has to leave the antenna at the SAME
         * instant, otherwise every anchor is late by its own δ (≈ 30 cm per
         * ns) and that bias lands straight in the Tag's TDOA.  Our
         * programmed instant is δ_child earlier than the antenna instant,
         * so the programmed instant has to be pulled back by δ_child —
         * the advertised TX stamp then adds δ̂_child back and lands on the
         * Root's antenna instant (up to the calibration residual). */
        if (s_tx_delay_valid) {
            g_tdma_ref.base_origin -= (uint64_t)s_tx_delay_dtu;
        }
        g_tdma_ref.base_local_time = now_local;

        /* DS-TWR: remember the instant we received this frame and the cycle
         * id it carried — the Root needs both to close the round trip with
         * our next beacon. */
        s_ref_rx_ts = rx_local;
        s_ref_cycle = g_tdma_ref.root_cycle_id;

        /* Diagnostic: how far the clock-offset model is from our own RX
         * instant.  The difference is (mostly) the Root→Child propagation
         * delay that the offset estimate absorbed — exactly what the DS-TWR
         * correction removes. */
        {
            static uint32_t s_last_diag_ms;
            uint32_t now_ms = HAL_GetTick();
            if (now_ms - s_last_diag_ms > 2000u) {
                s_last_diag_ms = now_ms;
                int64_t err = (int64_t)(root_clock_to_local(root_ext)
                                        - g_tdma_ref.base_origin);
                char dbg[64];
                sprintf(dbg, "GRID_REF err=%ld dtu (%.1f cm) corr=%lu\r\n",
                        (long)err, (double)err * CM_PER_DTU_F,
                        (unsigned long)(s_twr_corr_valid ? s_twr_corr_dtu : 0));
                DBG_PRINT(dbg);
            }
        }
    }

    /* Root diagnostic: print neighbor table and slot table on every RX */
    if (g_anchor.cs_level == 0) {
        char dbg[128]; int pos = 0;
        int n_neigh = 0; int ni;
        for (ni = 0; ni < ANCHOR_MAX_NEIGHBORS; ni++) {
            if (g_anchor.neighbors[ni].initialized) n_neigh++;
        }
        pos += sprintf(dbg + pos, "NEIGHBORS count=%d", n_neigh);
        for (ni = 0; ni < ANCHOR_MAX_NEIGHBORS; ni++) {
            if (!g_anchor.neighbors[ni].initialized) continue;
            pos += sprintf(dbg + pos, " [%d]R%u",
                ni, (unsigned)g_anchor.neighbors[ni].neighbor_rank);
        }
        pos += sprintf(dbg + pos, " TABLE:");
        int si;
        for (si = 0; si < ANCHOR_MAX_SLOTS; si++) {
            if (g_anchor.slot_owners[si] == 0) continue;
            pos += sprintf(dbg + pos, " sl%d=R%u", si, (unsigned)g_anchor.slot_owners[si]);
        }
        pos += sprintf(dbg + pos, "\r\n");
        DBG_PRINT(dbg);
    }

    /* Delta-range check on RAW timestamps (no wrap correction here).
     * clock_sync_update() handles 40-bit wrap internally using its own
     * wrap-safe delta computation (stores raw, corrects inline).
     * Here we only guard against duplicate / absurdly small intervals.
     * NOTE: if delta_tx wraps (raw value goes negative when cast to int64_t),
     * clock_sync_update will reject it as outlier — this is the correct
     * graceful degradation for a rare multi-wrap gap. */
    uint64_t prev_tx = g_anchor.neighbors[slot].last_remote_tx_ts;
    if (prev_tx != 0) {
        int64_t raw_delta = (int64_t)(tx_remote - prev_tx);
        /* Reject duplicate / absurdly small intervals (including wrapped negatives) */
        if (raw_delta < (int64_t)(10 * 1000 * UUS_TO_DWT_TIME)) {
            /* Slide baseline forward so next beacon can form a valid delta */
            g_anchor.neighbors[slot].last_remote_tx_ts = tx_remote;
            g_anchor.neighbors[slot].last_local_rx_ts  = rx_local;
            g_anchor.last_heard_ms[slot] = HAL_GetTick();
            return;
        }
        /* Reject absurdly large gaps (> 3 s) — could be multi-wrap gap or
         * out-of-order beacon; slide baseline to resync from here. */
        if (raw_delta > (int64_t)(3000ULL * 1000 * UUS_TO_DWT_TIME)) {
            /* Resync: just update baseline and wait for next frame */
            g_anchor.neighbors[slot].last_remote_tx_ts = tx_remote;
            g_anchor.neighbors[slot].last_local_rx_ts  = rx_local;
            g_anchor.last_heard_ms[slot] = HAL_GetTick();
            return;
        }
    }

    int16_t cfo_raw = dwt_readclockoffset();

    /* P0-2: an imprecise TX timestamp would put a ~30 µs error into the
     * clock offset — a thousand times the propagation delay we want to
     * resolve.  Such frames are still used for the neighbour table, but not
     * for the clock model. */
    if (!ts_precise) {
        g_anchor.last_heard_ms[slot] = HAL_GetTick();
        g_anchor.rx_count++;
        return;
    }

    /* Pass RAW tx_remote — clock_sync_update() does wrap-safe delta internally.
     * rx_local is captured at the top of this function (FIX4).
     * now_ms = HAL_GetTick(): the filter derives its own dt internally. */
    bool ok = clock_sync_update(&g_anchor.neighbors[slot], cfo_raw,
                      rx_local, tx_remote, HAL_GetTick());
    g_anchor.last_heard_ms[slot] = HAL_GetTick();
    g_anchor.rx_count++;

    {
        char dbg[48];
        const AnchorClockSync *s = &g_anchor.neighbors[slot];
        sprintf(dbg, "RX[R=%03u] Q=%02u%% F=%.9f (%+.1f ppm)\r\n",
            src_rank,
            (unsigned int)(s->quality * 100),
            s->factor,
            (double)(s->factor - 1.0) * 1e6);
        DBG_PRINT(dbg);
    }

    /* ── DS-TWR: close the round trip (Root side) ─────────────────────────
     * The child's beacon carries
     *   T2      = its TX instant (this frame's precise timestamp)
     *   r1      = the instant it received the Root frame it used as clock
     *             reference  (rx_ref_ts40)
     *   refc    = the cycle id of that Root frame (rx_ref_cycle)
     * and we know
     *   r2      = our own RX instant (rx_local)
     *   T1      = our ACTUAL TX instant of the frame with cycle id refc
     *             (recorded right after that transmission — the nominal
     *              grid instant would leak the grid error into the path)
     *
     *   (r2 - T1) = prop_R->X + f_root * (T2 - r1) + prop_X->R
     *   => 2 * prop = (r2 - T1) - f_root * (T2 - r1)
     *
     * f_root is our model of the child's clock rate relative to ours
     * (the clock_sync factor of this neighbour).
     *
     * Which errors enter this equation:
     *   - The CHILD's grid placement error does NOT enter: T2 is the TX
     *     instant the child actually advertised (programmed + its calibrated
     *     TX delay), so the round trip measures the physical round trip, not
     *     the child's idea of the grid.  (What does enter is the residual of
     *     the child's TX-delay calibration — see K_dev above.)
     *   - The ROOT's grid placement error DOES enter through T1, so T1 must
     *     be the Root's ACTUAL TX instant, not the nominal grid instant —
     *     otherwise the grid error (and δ_root) leaks straight into the
     *     solved propagation delay.  That is what root_tx_ts_for_cycle()
     *     provides.
     *
     * All four timestamps are 40-bit on the wire; the differences are small
     * compared to 2^40, so a plain subtraction with wrap handling is exact. */
    if (g_anchor.cs_level == 0 && s_root_grid_base != 0 && rx_local >= s_root_grid_base
        && sender_level >= 1 && slot >= 0 && slot < ANCHOR_MAX_NEIGHBORS) {
        uint64_t t2 = tx_remote & 0xFFFFFFFFFFULL;              /* child clock */
        uint64_t r1 = unpack_40bit_ts(&rx_buf[DL_TS_RX_REF_TS_IDX]); /* child clock */
        uint16_t refc = (uint16_t)(rx_buf[DL_TS_RX_REF_CYCLE_IDX]
                        | ((uint16_t)rx_buf[DL_TS_RX_REF_CYCLE_IDX + 1] << 8));
        uint64_t period = (uint64_t)DL_TIMESYNC_PERIOD_UUS * UUS_TO_DWT_TIME;

        /* The Root's cycle id is 16 bit and wraps every 65536 cycles
         * (~87 min).  Reconstruct the absolute cycle of the reference frame
         * from our own current cycle: the reference is at most a couple of
         * cycles old (the child refreshes it on every Root beacon). */
        uint64_t cur_k = (rx_local - s_root_grid_base) / period;
        uint32_t delta = (uint32_t)((cur_k - (uint64_t)refc) & 0xFFFFu);
        if (delta <= 3) {
            uint64_t k_ref = cur_k - delta;

            /* T1 = the Root's ACTUAL TX instant of that cycle (P0-5).  The
             * nominal grid instant is only a fallback; a measurement that
             * needs it is dropped, because the grid error would end up in
             * the propagation delay. */
            uint64_t t1 = root_tx_ts_for_cycle(refc);
            bool t1_exact = (t1 != 0);
            if (!t1_exact) t1 = s_root_grid_base + k_ref * period;

            if (r1 != 0 && t1 != 0 && t1_exact) {
                /* The child's turnaround, in the child's clock.  Both stamps
                 * are 40 bit, so the difference is modular: the 40-bit wrap
                 * (every ~17 s) is handled by the mask.  The turnaround is
                 * < 1 s, so a value above 2^39 means a bogus reference. */
                uint64_t turn = (t2 - r1) & 0xFFFFFFFFFFULL;
                double f_root = g_anchor.neighbors[slot].factor;

                if (turn <= (1ULL << 39) && f_root > 0.5 && f_root < 2.0
                    && rx_local > t1) {
                    double rt = (double)(rx_local - t1) - f_root * (double)turn;
                    double p_root = rt * 0.5;      /* one-way delay, root clock */
                    /* Plausible path: 0.2 m .. 100 m
                     * (1 m = 213.20 DTU -> 0.2 m = 42.6 DTU, 100 m = 21,320 DTU) */
                    if (p_root > (double)DL_TWR_CORR_MIN_DTU
                        && p_root < (double)DL_TWR_CORR_MAX_DTU) {
                        /* Convert to the child's clock units: a duration D_root
                         * corresponds to D_root / f_root in the child's clock. */
                        double p_child = p_root / f_root;
                        TwrLinkState *L = &s_twr_link[slot];
                        if (!L->corr_valid) {
                            L->corr_dtu = (uint32_t)p_child;
                            L->corr_valid = 1;
                            L->corr_count = 1;
                        } else {
                            /* IIR smoothing: the path length changes slowly,
                             * the measurement is noisy (±cm from multipath). */
                            L->corr_dtu = (L->corr_dtu * 3u
                                           + (uint32_t)p_child) / 4u;
                            L->corr_count++;
                        }
                        L->child_tx = t2;
                        L->child_rx_ref = r1;
                        L->ref_cycle = refc;
                        L->have = 1;
                        { char dbg[64];
                          sprintf(dbg, "TWR R=%u prop=%ucm\r\n",
                                  (unsigned)src_rank,
                                  (unsigned)((uint32_t)(p_root * CM_PER_DTU_F + 0.5)));
                          DBG_PRINT(dbg); }
                    }
                }
            }
        }
    }
}

/*============================================================================
 * Coordinator frames (OTA + COORD_WRITE)
 *============================================================================*/
static void anchor_handle_coord_write(uint16_t src_uid, const uint8_t *payload, uint32_t len)
{
    if (len < 8) return;
    int16_t x_cm = (int16_t)((uint16_t)payload[0] | ((uint16_t)payload[1] << 8));
    int16_t y_cm = (int16_t)((uint16_t)payload[2] | ((uint16_t)payload[3] << 8));
    int16_t z_cm = (int16_t)((uint16_t)payload[4] | ((uint16_t)payload[5] << 8));
    (void)payload[6]; (void)payload[7];   /* pad */

    dl_set_anchor_coord(g_anchor.rank, x_cm, y_cm, z_cm);
    g_anchor.my_pos = (dl_anchor_entry_t){
        .id = g_anchor.rank, .x_cm = x_cm, .y_cm = y_cm, .z_cm = z_cm
    };
    gateway_flash_save_coord(x_cm, y_cm, z_cm);

    /* ACK to gateway */
    uint8_t ack[13];
    ack[0] = 0x41; ack[1] = 0x88; ack[2] = 0;
    ack[3] = 0xCA; ack[4] = 0xDE;
    ack[5] = (uint8_t)(src_uid & 0xFF); ack[6] = (uint8_t)(src_uid >> 8);
    ack[7] = (uint8_t)(g_anchor.rank & 0xFF);
    ack[8] = (uint8_t)(g_anchor.rank >> 8);
    ack[9] = FUNC_COORD_ACK;
    ack[10] = 0xCC; ack[11] = 0xDD; ack[12] = 0xEE;

    port_DisableEXT_IRQ();
    dwt_forcetrxoff();
    dwt_writetxdata(sizeof(ack), ack, 0);
    dwt_writetxfctrl(sizeof(ack), 0, 0);
    dwt_starttx(DWT_START_TX_IMMEDIATE);
    { uint32_t dl = HAL_GetTick() + 10u;
      while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK))
        { if (HAL_GetTick() > dl) break; } }
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK | SYS_STATUS_HPDWARN_BIT_MASK);
    port_EnableEXT_IRQ();

    DBG_PRINT("+COORD_OK\r\n");
}

/*============================================================================
 * Frame dispatch (RX → handlers)
 *============================================================================*/

/* FIND LED blink state — file scope so anchor_process_rx can arm it */
static uint32_t s_find_blink_until_ms;  /* 0 = no blink; non-zero = stop time */
static uint32_t s_find_blink_next_ms;   /* next toggle time during blink */

static void anchor_process_rx(uint8_t *rx_buf, uint32_t frame_len)
{
    uint16_t src = (uint16_t)rx_buf[7] | ((uint16_t)rx_buf[8] << 8);
    uint8_t  func = rx_buf[9];

    /* OTA frames take priority and abort normal RX */
    if (func == FUNC_OTA_DATA && frame_len >= GW_HDR_LEN + 4) {
        gw_ota_handle_data(rx_buf, frame_len, g_anchor.rank);
        return;
    }
    if (func == FUNC_OTA_END)  { gw_ota_finish(g_anchor.rank); return; }
    if (func == FUNC_OTA_ABORT) { g_anchor.rank == 0 ? (void)0 : (void)0; /* stub */
                                  gw_ota_abort(); return; }
    if (func == FUNC_PREP_CMD && src == GW_MASTER_ADDR) {
        g_anchor.rank == 0 ? (void)0 : (void)0;
        return;
    }

    /* NETWORK_START from Gateway (rank 0) — variable length (roster), bypass
     * anchor_rx_timesync() length check (DL_TIMESYNC_LEN=72). */
    if (func == FUNC_NETWORK_START && src == GW_MASTER_ADDR) {
        anchor_handle_network_start(rx_buf, frame_len);
        anchor_send_network_ack();
        return;
    }

    if (func == FUNC_DOWNLINK_BEACON) {
        anchor_rx_timesync(rx_buf, frame_len);
        return;
    }

    /* DS-TWR correction from the Root (unicast to one child) */
    if (func == FUNC_TWR_CORR) {
        anchor_handle_twr_corr(rx_buf, frame_len);
        return;
    }

    if (func == FUNC_FIND_REQ) {
        uint16_t dst = (uint16_t)rx_buf[5] | ((uint16_t)rx_buf[6] << 8);
        if (dst != 0xFFFF && dst != g_anchor.rank) return;
        uint8_t on_off = rx_buf[GW_HDR_LEN];
        /* Trigger 3-second rapid flicker (50ms interval) when on_off=1 */
        if (on_off) {
            s_find_blink_until_ms = HAL_GetTick() + 3000;
            s_find_blink_next_ms  = HAL_GetTick() + 50;
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
        } else {
            s_find_blink_until_ms = 0;
            s_find_blink_next_ms  = 0;
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
        }

        uint8_t ack[13];
        ack[0] = 0x41; ack[1] = 0x88; ack[2] = 0;
        ack[3] = 0xCA; ack[4] = 0xDE;
        ack[5] = (uint8_t)(src & 0xFF); ack[6] = (uint8_t)(src >> 8);
        ack[7] = (uint8_t)(g_anchor.rank & 0xFF);
        ack[8] = (uint8_t)(g_anchor.rank >> 8);
        ack[9] = FUNC_FIND_ACK;
        ack[10] = on_off;
        ack[11] = 0xCC; ack[12] = 0xDD;

        port_DisableEXT_IRQ();
        dwt_forcetrxoff();
        dwt_writetxdata(sizeof(ack), ack, 0);
        dwt_writetxfctrl(sizeof(ack), 0, 0);
        dwt_starttx(DWT_START_TX_IMMEDIATE);
        { uint32_t dl = HAL_GetTick() + 10u;
          while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK))
            { if (HAL_GetTick() > dl) break; } }
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK | SYS_STATUS_HPDWARN_BIT_MASK);
        port_EnableEXT_IRQ();
        { char dbg[24]; sprintf(dbg, "FIND_ACK TX\r\n"); DBG_PRINT(dbg); }
        return;
    }

    if (func == FUNC_COORD_WRITE) {
        uint16_t dst = (uint16_t)rx_buf[5] | ((uint16_t)rx_buf[6] << 8);
        if (dst != 0xFFFF && dst != g_anchor.rank) return;
        anchor_handle_coord_write(src,
                                  &rx_buf[GW_HDR_LEN],
                                  frame_len - GW_HDR_LEN);
        return;
    }

    /* Unknown func — only reached if none of above matched */
    {
        char dbg[24];
        sprintf(dbg, "ARX_FUNC[R=%03u]f=0x%02X\r\n",
            g_anchor.rank, func);
        DBG_PRINT(dbg);
    }
}

/*============================================================================
 * Single unified anchor loop (BD8NCF — no master/slave distinction)
 *============================================================================*/
static void anchor_loop(void)
{
    uint32_t status;
    uint32_t frame_len;
    uint8_t  rx_buf[128];
    static uint32_t s_rx_err_ctr;
    static uint32_t s_rx_ok_ctr;
    static uint32_t s_rx_ignored_ctr;
    static uint32_t s_last_rtc_print_ms;

    /* Clock measurement complete — entering normal loop */
    {
        char dbg[48];
        sprintf(dbg, "CLK DONE\r\n");
        DBG_PRINT(dbg);
    }

    while (1) {
        uint32_t now_ms = HAL_GetTick();

        /* ── RTC clock accuracy check every 10s ──────────────────────────── */
        if ((int32_t)(now_ms - s_last_rtc_print_ms) >= 10000) {
            s_last_rtc_print_ms += 10000;
            uint32_t tr = RTC->TR;
            uint32_t dr = RTC->DR;
            uint8_t hour   = (uint8_t)((tr & (RTC_TR_HT | RTC_TR_HU)) >> RTC_TR_HU_Pos);
            uint8_t minute = (uint8_t)((tr & (RTC_TR_MNT | RTC_TR_MNU)) >> RTC_TR_MNU_Pos);
            uint8_t second = (uint8_t)((tr & (RTC_TR_ST | RTC_TR_SU)) >> RTC_TR_SU_Pos);
            uint8_t year   = (uint8_t)((dr & (RTC_DR_YT | RTC_DR_YU)) >> RTC_DR_YU_Pos);
            uint8_t month  = (uint8_t)((dr & (RTC_DR_MT | RTC_DR_MU)) >> RTC_DR_MU_Pos);
            uint8_t day    = (uint8_t)((dr & (RTC_DR_DT | RTC_DR_DU)) >> RTC_DR_DU_Pos);
            char dbg[48];
            sprintf(dbg, "RTC %02x:%02x:%02x 20%02x/%02x/%02x\r\n",
                    hour, minute, second, year, month, day);
            DBG_PRINT(dbg);
        }

        /* FIND LED flicker: 3s rapid blink (50ms interval) then OFF */
        if (s_find_blink_until_ms != 0) {
            if ((int32_t)(now_ms - s_find_blink_until_ms) >= 0) {
                s_find_blink_until_ms = 0;
                HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
            } else if ((int32_t)(now_ms - s_find_blink_next_ms) >= 0) {
                s_find_blink_next_ms += 50;
                HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
            }
        }

        /* ── Hierarchical state machine (manual: triggered by NETWORK_START) ─────── */
        switch (g_anchor.run_state) {
            case STATE_SCAN: {
                /* Manual mode: SCAN is only entered on FORCE_RESET.
                 * Stay here silently until NETWORK_START received (which sets run_state=STATE_SYNC). */
                break;
            }
            case STATE_ELECTING: {
                /* In manual mode, this state is bypassed — NETWORK_START sets run_state=STATE_SYNC.
                 * If we somehow end up here, transition to SYNC immediately. */
                g_anchor.run_state = STATE_SYNC;
                DBG_PRINT("ELECT→SYNC (manual)\r\n");
                break;
            }
            case STATE_SYNC: {
                /* Check if our parent has gone silent for > 30s → orphan */
                if (g_anchor.parent_rank != 0 &&
                    (now_ms - g_anchor.last_parent_rx_ms) >= DL_PARENT_LOST_TIMEOUT_MS) {
                    g_anchor.run_state = STATE_ORPHAN;
                    DBG_PRINT("PARENT_LOST→ORPHAN\r\n");
                }
                break;
            }
            case STATE_ORPHAN: {
                /* No parent heard for 30s — re-enter SCAN and reset network */
                if ((now_ms - g_anchor.last_parent_rx_ms) >= DL_PARENT_LOST_TIMEOUT_MS) {
                    g_anchor.run_state = STATE_SCAN;
                    g_anchor.scan_start_ms = now_ms;
                    g_anchor.parent_rank = 0;
                    g_anchor.cs_level = 1;
                    anchor_clear_slot("ORPHAN");
                    s_net_state = NET_IDLE;  /* wait for new NETWORK_START */
                    DBG_PRINT("ORPHAN→SCAN\r\n");
                }
                break;
            }
            default:
                break;
        }

        /* ── NET state machine ──────────────────────────────────────────────── */
        /* NET_STARTING: Child waits NET_CHILD_TX_DELAY_MS before TX */
        if (s_net_state == NET_STARTING) {
            if ((HAL_GetTick() - s_net_start_ms) >= NET_CHILD_TX_DELAY_MS) {
                /* Transition to NET_WAIT_CONFIRM: Child will send one ALOHA TX
                 * (via tdma_may_tx) then wait for Root beacon to confirm slot. */
                s_net_state = NET_WAIT_CONFIRM;
                s_confirm_wait_ms = HAL_GetTick();
                s_aloha_sent_ms = 0;
                { char dbg[32]; sprintf(dbg, "NET_STARTING→WAIT_CONFIRM\r\n"); DBG_PRINT(dbg); }
            }
        }

        /* ── TX: only in NET_RUNNING state ─────────────────────────────────
         * P0-3: the slot instant is computed on the DW3000 SYS_TIME axis
         * (the same axis the delayed TX is programmed on), so the gate and
         * the radio can no longer disagree. */
        uint64_t tx_target = 0;
        uint16_t tx_cycle  = 0;
        bool     tx_armed  = false;
        if (s_net_state == NET_RUNNING
            && (g_anchor.run_state == STATE_SYNC || g_anchor.run_state == STATE_ELECTING)
            && !gw_ota_is_active()
            && !s_aloha_mode
            && tdma_may_tx(&tx_target, &tx_cycle)) {
            tx_armed = true;
        }

        /* ── NET_WAIT_CONFIRM TX: ALOHA to announce to Root ───────────────
         * Retried every 500 ms until the Root confirms our slot (or the
         * 3 s confirmation window expires and we fall back to ALOHA mode).
         * A single lost ALOHA would otherwise cost a full 3 s stall.
         *
         * P1-2: the ALOHA beacon goes out with slot = 255 (we are not in the
         * Root's table yet), so the Tag excludes us from the TDOA solve
         * instead of pairing us against a slot instant we do not own. */
        if (s_net_state == NET_WAIT_CONFIRM
            && (HAL_GetTick() - s_net_start_ms) >= (NET_CHILD_TX_DELAY_MS + 50)
            && (s_aloha_sent_ms == 0 ||
                (HAL_GetTick() - s_aloha_sent_ms) >= NET_ALOHA_RETRY_MS)) {
            /* ALOHA bootstrap — must go out immediately, not on the TDMA
             * schedule (the schedule may already be in the past). */
            anchor_tx_timesync(0, 0);
            s_aloha_sent_ms = HAL_GetTick();
        }

        /* ── ALOHA fallback TX (Root never confirmed our slot) ─────────────
         * Off-grid beacon every 500 ms so the Root can hear us again.
         * slot = 255 keeps us out of the Tag's TDOA solve. */
        if (s_net_state == NET_RUNNING && s_aloha_mode && !gw_ota_is_active()
            && (s_aloha_sent_ms == 0 ||
                (HAL_GetTick() - s_aloha_sent_ms) >= NET_ALOHA_RETRY_MS)) {
            anchor_tx_timesync(0, 0);
            s_aloha_sent_ms = HAL_GetTick();
        }

        /* ── Root bootstrap beacon (P0-3) ───────────────────────────────────
         * The Root's FIRST beacon defines the cycle origin of the whole
         * network: every child schedules its own slot against that instant
         * (s_root_grid_base).  Without a bootstrap the Root would have no
         * grid instant to schedule against — tdma_grid_target() returns
         * false while the base is zero — so the Root would never transmit
         * at all and the whole network would stay silent.
         * The bootstrap goes out immediately; the first grid beacon follows
         * one period later. */
        if (s_net_state == NET_RUNNING && g_anchor.cs_level == 0
            && s_root_grid_base == 0 && !s_aloha_mode && !gw_ota_is_active()) {
            anchor_tx_timesync(0, 0);
        }

        /* ── Discovery beacon ──────────────────────────────────────────────
         * Keeps the anchor visible to the gateway/host so the operator can
         * select it (or another anchor) as Root.
         * P2-4: while the network is running the discovery frame is kept as a
         * LOW RATE heartbeat (2 s) — otherwise the anchor would vanish from
         * the host list as soon as it joined the network, because the 0xEE
         * beacon alone does not reach the gateway's device table on every
         * network.
         *
         * It is skipped when a grid transmission is armed: the immediate TX
         * of the discovery frame would push the delayed grid beacon past its
         * instant and cost the anchor a whole TDMA cycle. */
        if (!gw_ota_is_active() && !tx_armed) {
            static uint32_t s_discovery_ms = 0;
            uint32_t dnow = HAL_GetTick();
            uint32_t dperiod = (s_net_state == NET_IDLE)
                             ? NET_DISCOVERY_PERIOD_MS
                             : NET_RUNNING_DISCOVERY_MS;
            if (s_discovery_ms == 0 || (dnow - s_discovery_ms) >= dperiod) {
                s_discovery_ms = dnow;
                anchor_tx_discovery();
            }
        }

        /* Phase 2: listen window — grid-locked (P0-3).
         *
         * When a slot instant is armed, the window ends exactly at that
         * instant (minus the lead time) so the delayed TX fires on the grid
         * and the anchor is listening for the WHOLE cycle in between.  The
         * window length is therefore the cycle length minus our own slot, not
         * a fixed constant — the Root's beacon always falls inside it. */
        dwt_forcetrxoff();
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        uint64_t wake_at = now_sys_ext()
                         + (uint64_t)ANCHOR_RX_POLL_MS * 1000u * UUS_TO_DWT_TIME;
        if (tx_armed && tx_target > ANCHOR_TX_LEAD_DTU) {
            uint64_t t = tx_target - ANCHOR_TX_LEAD_DTU;
            if (t < wake_at) wake_at = t;
        } else if (!tx_armed && s_last_tx_target != 0) {
            /* No slot for this cycle — keep listening until the next grid
             * instant so we do not leave a hole in the cycle. */
            uint64_t period = (uint64_t)DL_TIMESYNC_PERIOD_UUS * UUS_TO_DWT_TIME;
            uint64_t t = s_last_tx_target + period;
            if (t > ANCHOR_TX_LEAD_DTU && (t - ANCHOR_TX_LEAD_DTU) < wake_at) {
                wake_at = t - ANCHOR_TX_LEAD_DTU;
            }
        }

        while (now_sys_ext() < wake_at) {
            Sleep(1);
            status = dwt_read32bitreg(SYS_STATUS_ID);
            if (status & SYS_STATUS_ALL_RX_ERR) {
                s_rx_err_ctr++;
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
                continue;
            }
            if (!(status & SYS_STATUS_RXFCG_BIT_MASK)) continue;
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
            if (frame_len == 0 || frame_len > sizeof(rx_buf)) {
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
                continue;
            }
            dwt_readrxdata(rx_buf, frame_len, 0);
            /* Dispatch by function code.
             * NOTE: 0xEE frames additionally require the full TimeSync header
             * pattern (dst=0xFFFF broadcast).  All other function codes are
             * dispatched purely by func byte — keep this switch exhaustive so
             * new protocols are not silently dropped. */
            {
                uint8_t func = rx_buf[9];
                int handled = 0;
                switch (func) {
                    case FUNC_DOWNLINK_BEACON:
                        handled = match_header(rx_buf, match_timesync, 10, 1, 0, 1);
                        break;
                    case FUNC_NETWORK_START:   /* GW → anchors: manual net formation */
                    case FUNC_TWR_CORR:        /* Root → child: path correction */
                        handled = match_header(rx_buf, match_twr_corr, 10, 1, 1, 1);
                        break;
                    case FUNC_FIND_REQ:
                    case FUNC_COORD_WRITE:
                    case FUNC_OTA_DATA:
                    case FUNC_OTA_END:
                    case FUNC_OTA_ABORT:
                        handled = 1;
                        break;
                    default:
                        break;
                }
                if (handled) {
                    s_rx_ok_ctr++;
                    anchor_process_rx(rx_buf, frame_len);
                } else {
                    s_rx_ignored_ctr++;
                }
            }
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }

        /* ── Grid-locked transmission ──────────────────────────────────────
         * The delayed TX is armed for the exact grid instant computed above.
         * If we woke up too late to arm it in time, the instant is dropped
         * (the anchor simply misses this cycle) — sending a beacon at the
         * wrong instant would poison the TDOA of every other anchor in this
         * cycle, so silence is strictly better than a late beacon. */
        if (tx_armed) {
            uint64_t now = now_sys_ext();
            if (now + ANCHOR_TX_LEAD_DTU < tx_target) {
                /* not yet due — wait for the instant (should not happen) */
                while (now_sys_ext() + ANCHOR_TX_LEAD_DTU < tx_target) { Sleep(1); }
            }
            if (now_sys_ext() + ANCHOR_TX_LEAD_DTU <= tx_target) {
                anchor_tx_timesync(tx_target, tx_cycle);
                /* DS-TWR: the Root ships the measured path correction to one
                 * child per cycle.  We are in slot 0 right now, so the extra
                 * frame stays inside our own slot window and cannot collide
                 * with a child's beacon. */
                anchor_twr_corr_poll();
            } else {
                { char dbg[48]; sprintf(dbg, "TX_SKIP late cycle=%u\r\n",
                    (unsigned)tx_cycle); DBG_PRINT(dbg); }
            }
        }

#ifndef GATEWAY_ROLE
        if (gw_ota_poll()) {
            /* OTA timed out — resume ranging */
        }
#endif

        /* SYNC summary: every 20 frames (~1 s at 20Hz) */
        static uint32_t s_sync_ctr;
        if (++s_sync_ctr >= 20) {
            s_sync_ctr = 0;

            /* Anchor state diagnostic: run_state, cs_level, my_slot, num_neighbors */
            { char dbg[64];
              int n_neigh = 0;
              int ni;
              for (ni = 0; ni < ANCHOR_MAX_NEIGHBORS; ni++) {
                  if (g_anchor.neighbors[ni].initialized) n_neigh++;
              }
              sprintf(dbg, "A_STATE R=%u st=%u lv=%u sl=%u n_neigh=%d tdma_ref=%u\r\n",
                  (unsigned)g_anchor.rank,
                  (unsigned)g_anchor.run_state,
                  (unsigned)g_anchor.cs_level,
                  (unsigned)g_anchor.my_slot_idx,
                  n_neigh,
                  g_tdma_ref.have_root_ref ? 1 : 0);
              DBG_PRINT(dbg);
            }

            int i;
            static uint32_t s_prev_upd[ANCHOR_MAX_NEIGHBORS];
            static uint32_t s_prev_heard[ANCHOR_MAX_NEIGHBORS];
            for (i = 0; i < ANCHOR_MAX_NEIGHBORS; i++) {
                if (!g_anchor.neighbors[i].initialized) continue;
                const AnchorClockSync *s = &g_anchor.neighbors[i];
                uint32_t now_ms = HAL_GetTick();
                uint32_t elapsed_ms = (g_anchor.last_heard_ms[i] == 0)
                    ? 0 : (now_ms - g_anchor.last_heard_ms[i]);
                /* Loss over the last summary interval: how many beacons we
                 * should have heard vs. how many the filter actually used.
                 * (The old code compared the CUMULATIVE update_count with a
                 * per-interval expectation — always a bogus number.) */
                uint32_t d_upd = (s->update_count >= s_prev_upd[i])
                              ? (s->update_count - s_prev_upd[i]) : 0;
                uint32_t d_ms  = (now_ms >= s_prev_heard[i])
                              ? (now_ms - s_prev_heard[i]) : 0;
                uint32_t expected = (d_ms / DL_TIMESYNC_PERIOD_MS) + 1;
                uint32_t loss = (expected > d_upd) ? (expected - d_upd) : 0;
                uint32_t loss_pct = (expected > 0) ? (loss * 100u / expected) : 0;
                s_prev_upd[i]   = s->update_count;
                s_prev_heard[i] = now_ms;
                double factor_ppm = (s->factor - 1.0) * 1e6;
                char line[160];
                sprintf(line, "SYNC[R=%u,L=%u→R=%u] F=%.10f(%.3fppm) Q=%03u%% upd=%05lu elaps=%05lums loss=%lu/%lu(%u%%) rx=%lu/%lu frz=%lu dly=%lu out=%lu\r\n",
                    (unsigned)g_anchor.rank,
                    (unsigned)g_anchor.cs_level,
                    g_anchor.neighbors[i].neighbor_rank,
                    s->factor,
                    (double)factor_ppm,
                    (unsigned int)(s->quality * 100),
                    (unsigned long)s->update_count,
                    (unsigned long)elapsed_ms,
                    (unsigned long)loss,
                    (unsigned long)expected,
                    (unsigned)loss_pct,
                    (unsigned long)s->rx_ok,
                    (unsigned long)s->rx_total,
                    (unsigned long)s->freeze_count,
                    (unsigned long)s->dly_fail_count,
                    (unsigned long)s->outlier_count);
                DBG_PRINT(line);
            }
        }

        /* RX summary: every 1000 frames (~50 s at 20Hz) */
        static uint32_t s_rx_ctr;
        if (++s_rx_ctr >= 1000) {
            s_rx_ctr = 0;
            char line2[64];
            sprintf(line2, "RX  err=%lu ign=%lu ok=%lu\r\n",
                (unsigned long)s_rx_err_ctr,
                (unsigned long)s_rx_ignored_ctr,
                (unsigned long)s_rx_ok_ctr);
            DBG_PRINT(line2);
        }
    }
}

/* Stub: AT+RATE command from gateway — not implemented in Phase 1.1
 * (fixed 100 ms period via DL_TIMESYNC_PERIOD_MS). */
void dl_set_beacon_interval(uint16_t ms)
{
    (void)ms;
}

/*============================================================================
 * Anchor init — one-time setup before anchor_loop()
 *============================================================================*/
static int anchor_init(void)
{
    uint32_t uid32 = compute_uid32();
    g_anchor.rank = compute_rank_from_uid();  /* CRC16 over the full 96-bit UID */
    g_anchor.cs_level = 1;                  /* Initial: assume child, root will be elected */
    g_anchor.parent_eui64 = 0;
    g_anchor.my_eui64 = (uint64_t)uid32;
    g_anchor.seq = 0;
    g_anchor.tx_count = 0;
    g_anchor.rx_count = 0;
    g_anchor.prev_tx_ts = 0;
    g_anchor.prev_tx_ts_ext = 0;
    g_anchor.prev_scheduled_tx = 0;

    /* Hierarchical sync: start in SCAN state */
    g_anchor.run_state = STATE_SCAN;
    g_anchor.scan_start_ms = HAL_GetTick();
    g_anchor.parent_rank = 0;
    g_anchor.last_parent_rx_ms = HAL_GetTick();  /* avoid immediate PARENT_LOST on first SYNC entry */
    g_anchor.electing_start_ms = 0;  /* Will be set when entering ELECTING */
    {
        int i;
        for (i = 0; i < ANCHOR_MAX_SLOTS; i++) g_anchor.slot_owners[i] = 0;
    }
    g_anchor.my_slot_idx = 255;       /* 255 = no slot assigned yet */
    g_anchor.last_schedule_rx_ms = 0;

    /* TDMA reference state: updated when we receive Root's beacon */
    g_tdma_ref.have_root_ref = false;
    g_tdma_ref.root_slot = 0;
    g_tdma_ref.root_tx_ts = 0;
    g_tdma_ref.root_local_ts = 0;
    g_tdma_ref.root_rx_ms = 0;

    /* Manual network state: start in NET_IDLE (silent, waiting for Gateway command) */
    s_net_state = NET_IDLE;
    s_net_root_rank = 0;
    s_net_slot_idx = 255;
    s_net_start_ms = 0;

    g_anchor.my_pos = (dl_anchor_entry_t){
        .id = g_anchor.rank, .x_cm = 0, .y_cm = 0, .z_cm = 0
    };

    /* Clear neighbor slots */
    int i;
    for (i = 0; i < ANCHOR_MAX_NEIGHBORS; i++) {
        g_anchor.last_heard_ms[i] = 0;
        g_anchor.neighbors[i].initialized = false;
    }

    /* Announce level and rank + UID details for debugging */
    {
        uint32_t uid0 = HAL_GetUIDw0();
        uint32_t uid1 = HAL_GetUIDw1();
        uint32_t uid2 = HAL_GetUIDw2();
        uint32_t uid32 = uid0 ^ uid1 ^ uid2;
        char dbg[96];
        sprintf(dbg, "BOOT RANK=%u NET_IDLE\r\n",
                (unsigned)g_anchor.rank);
        DBG_PRINT(dbg);
    }

    return 0;
}

/*============================================================================
 * Force PLL calibration and switch to IDLE_PLL state.
 * DW3000 stays in IDLE_RC after reset because OTP bias_tune is not burned.
 * Non-fatal: TX/RX timestamps use FOSC and work without PLL lock.
 *============================================================================*/
static int ensure_pll_locked(void)
{
    uint32_t sys_status = dwt_read32bitreg(SYS_STATUS_ID);
    uint32_t clk_ctrl = dwt_read32bitreg(CLK_CTRL_ID);

    if ((clk_ctrl & 0x3) == 1 && ((sys_status >> 1) & 1)) {
        return 0;
    }

    /* Manual PLL calibration — may fail if OTP bias_tune not burned */
    dwt_write16bitoffsetreg(CLK_CTRL_ID, 0, 0x0003);
    dwt_modify32bitoffsetreg(SEQ_CTRL_ID, 0,
        (uint32_t)(~SEQ_CTRL_AINIT2IDLE_BIT_MASK),
        SEQ_CTRL_FORCE2INIT_BIT_MASK);
    Sleep(5);

    dwt_write16bitoffsetreg(PLL_CFG_ID, 0, 0x0F3C);
    dwt_write8bitoffsetreg(LDO_RLOAD_ID, 1, 0x14);
    dwt_write8bitoffsetreg(SYS_STATUS_ID, 0, 0x02);
    Sleep(1);
    dwt_write8bitoffsetreg(PLL_CAL_ID, 0, 0x81);

    dwt_modify32bitoffsetreg(SEQ_CTRL_ID, 0,
        (uint32_t)(~SEQ_CTRL_FORCE2INIT_BIT_MASK), 0);
    dwt_or8bitoffsetreg(SEQ_CTRL_ID, 0x01, 0x01);
    Sleep(5);
    dwt_write16bitoffsetreg(CLK_CTRL_ID, 0, 0x0000);
    Sleep(5);

    int i;
    for (i = 0; i < 200; i++) {
        sys_status = dwt_read32bitreg(SYS_STATUS_ID);
        clk_ctrl = dwt_read32bitreg(CLK_CTRL_ID);
        if ((sys_status & 0x02) && ((clk_ctrl & 0x3) == 1)) {
            return 0;
        }
        Sleep(1);
    }

    return -1;
}

/*============================================================================
 * Public entry — called from main.c
 *============================================================================*/
int fusion_anchor(void)
{
    DBG_PRINT("ANCHOR START\r\n");

    reset_DWIC();
    Sleep(50);

    /* Release PA6/PA7 — DW3210 controls RF switch via 0x70014 AUTO mode */
    port_rf_switch_disable();
    dwt_write32bitoffsetreg(0x70014, 0, 0x1C000000);

    while (!dwt_checkidlerc()) { }
    if (dwt_check_dev_id() != DWT_SUCCESS) {
        DBG_PRINT("DEV ERR\r\n");
        return -1;
    }
    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
        DBG_PRINT("INIT ERR\r\n");
        return -1;
    }

    dwt_configure(&config);

    /* Note: manual PLL calibration (ensure_pll_locked) requires OTP-burned bias_tune
     * which this chip lacks. dwt_initialise() already handles bring-up.
     * TX/RX timestamps work in IDLE_RC (FOSC clock) regardless of PLL lock. */

    dwt_configuretxrf(&txconfig_options);
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_setleds(DWT_LEDS_ENABLE);

    if (anchor_init() != 0) {
        DBG_PRINT("INIT FAIL\r\n");
        return -1;
    }

    anchor_loop();
    return 0;
}

#endif /* TEST_FUSION_ANCHOR */
