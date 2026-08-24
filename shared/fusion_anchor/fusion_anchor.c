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
#define ANCHOR_RX_POLL_MS         95      /* Listen window within 100 ms cycle */
#define ANCHOR_PERIOD_MS          DL_TIMESYNC_PERIOD_MS   /* 100 ms */
#define ANCHOR_MAX_NEIGHBORS      DL_MAX_TRACKED_ANCHORS

/* BD8NCF frame header match (same byte layout, but func=0xEE reused) */
static const uint8_t match_timesync[] = {0x41, 0x88, 0,
                                         0xCA, 0xDE, 0xFF, 0xFF,
                                         0,    0,    0xEE};

/*============================================================================
 * Per-anchor runtime state (BD8NCF)
 *============================================================================*/
typedef struct {
    uint16_t           rank;             /* Anchor rank (0..15) — same as SRC addr */
    uint8_t            cs_level;         /* 0 = root, 1+ = child */
    uint64_t           parent_eui64;     /* EUI64 of anchor we sync to (0 if root) */
    uint64_t           my_eui64;         /* This anchor's EUI64 */
    dl_anchor_entry_t  my_pos;           /* My position, broadcast in TimeSync */
    uint64_t           prev_tx_ts;       /* Last actual TX timestamp (40-bit ext) */
    uint32_t           seq;              /* Monotonic seq counter */
    uint32_t           tx_count;
    uint32_t           rx_count;

    /* Per-source clock-sync state: when we RX another anchor's TimeSync, we
     * track its clock offset relative to ours via a Kalman filter. */
    AnchorClockSync    neighbors[ANCHOR_MAX_NEIGHBORS];

    /* Last heard timestamp per neighbor rank — for stale detection */
    uint32_t           last_heard_ms[ANCHOR_MAX_NEIGHBORS];
} AnchorState;

static AnchorState g_anchor;

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
static dl_anchor_entry_t s_anchor_table[DL_MAX_TRACKED_ANCHORS + 1];

void dl_set_anchor_coord(uint8_t rank, int16_t x_cm, int16_t y_cm, int16_t z_cm)
{
    if (rank < (DL_MAX_TRACKED_ANCHORS + 1)) {
        s_anchor_table[rank].id   = rank;
        s_anchor_table[rank].x_cm = x_cm;
        s_anchor_table[rank].y_cm = y_cm;
        s_anchor_table[rank].z_cm = z_cm;
    }
}

void dl_get_anchor_coord(uint8_t rank, dl_anchor_entry_t *out)
{
    if (rank < (DL_MAX_TRACKED_ANCHORS + 1)) {
        *out = s_anchor_table[rank];
    } else {
        out->id = rank; out->x_cm = 0; out->y_cm = 0; out->z_cm = 0;
    }
}

static int find_or_alloc_neighbor_slot(uint16_t rank)
{
    int i;
    for (i = 0; i < ANCHOR_MAX_NEIGHBORS; i++) {
        if (g_anchor.rank == rank + 1) continue;  /* skip self */
        if (g_anchor.last_heard_ms[i] != 0 && !g_anchor.neighbors[i].initialized) {
            /* reclaim stale slot */
        }
        /* For Phase 1.1 we just match by initialised state */
        if (!g_anchor.neighbors[i].initialized) {
            clock_sync_init(&g_anchor.neighbors[i], 1.0, 0.0,
                            0.0, 0.0, 0.0);  /* defaults */
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
    return oldest;
}

static int find_neighbor_slot_by_rank(uint16_t rank)
{
    /* For Phase 1.1 we use rank as the slot index directly (slot N = neighbor
     * with rank N+1).  Simplifies bookkeeping until Phase 1.4's 15-slot table. */
    if (rank == 0 || rank > ANCHOR_MAX_NEIGHBORS) return -1;
    int slot = rank - 1;
    if (slot >= ANCHOR_MAX_NEIGHBORS) return -1;
    if (!g_anchor.neighbors[slot].initialized) return -1;
    return slot;
}

/*============================================================================
 * TimeSync TX — broadcast every 100 ms
 *============================================================================*/
static void anchor_tx_timesync(void)
{
    uint8_t pkt[DL_TIMESYNC_LEN + 2];   /* +2 for DW3210 pad */
    uint64_t predicted_tx;

    /* Predicted TX time: prev + 100 ms (if first TX, use immediate) */
    if (g_anchor.tx_count == 0) {
        predicted_tx = 0;
    } else {
        predicted_tx = g_anchor.prev_tx_ts +
                        (uint64_t)DL_TIMESYNC_PERIOD_UUS * UUS_TO_DWT_TIME;
    }

    /* IEEE 802.15.4 header */
    pkt[0] = 0x41; pkt[1] = 0x88;        /* FC: data, short addr, PAN comp */
    pkt[2] = (uint8_t)(g_anchor.seq & 0xFF);
    pkt[3] = 0xCA; pkt[4] = 0xDE;
    pkt[5] = 0xFF; pkt[6] = 0xFF;        /* broadcast */
    pkt[7] = (uint8_t)(g_anchor.rank & 0xFF);
    pkt[8] = (uint8_t)((g_anchor.rank >> 8) & 0xFF);
    pkt[9] = FUNC_DOWNLINK_BEACON;

    pack_40bit_ts(&pkt[DL_TS_TX_TS_IDX], predicted_tx);
    pkt[DL_TS_CS_LEVEL_IDX] = g_anchor.cs_level;
    pack_eui64_u64(&pkt[DL_TS_PARENT_EUI_IDX], g_anchor.parent_eui64);
    pkt[DL_TS_RESERVED_IDX] = (uint8_t)(g_anchor.my_pos.id & 0x0F);

    /* Pad to absorb DW3210 RX last-2-byte corruption */
    pkt[DL_TIMESYNC_LEN + 0] = 0xCC;
    pkt[DL_TIMESYNC_LEN + 1] = 0xDD;

    dwt_forcetrxoff();
    port_rf_switch_tx();
    dwt_writetxdata(sizeof(pkt), pkt, 0);
    dwt_writetxfctrl(sizeof(pkt), 0, 0);

    if (predicted_tx != 0) {
        /* Use delayed TX for precise cadence */
        uint32_t tx_hi = (uint32_t)(predicted_tx >> 8);
        dwt_setdelayedtrxtime(tx_hi);
        if (dwt_starttx(DWT_START_TX_DLY_TS) != DWT_SUCCESS) {
            dwt_starttx(DWT_START_TX_IMMEDIATE);
        }
    } else {
        dwt_starttx(DWT_START_TX_IMMEDIATE);
    }

    /* Wait for TX complete (10 ms timeout) */
    uint32_t tx_dl = HAL_GetTick() + 10u;
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {
        if (HAL_GetTick() > tx_dl) break;
    }
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

    g_anchor.prev_tx_ts = get_tx_timestamp_u64();
    g_anchor.seq++;
    g_anchor.tx_count++;
}

/*============================================================================
 * TimeSync RX — listen for neighbor anchors' TimeSync frames
 *============================================================================*/
static void anchor_rx_timesync(uint8_t *rx_buf, uint32_t frame_len)
{
    if (frame_len < DL_TIMESYNC_LEN) return;

    uint16_t src_rank = (uint16_t)rx_buf[7] | ((uint16_t)rx_buf[8] << 8);
    if (src_rank == 0 || src_rank == g_anchor.rank) return;  /* skip master / self */
    if (src_rank > ANCHOR_MAX_NEIGHBORS) return;

    uint8_t  src_level = rx_buf[DL_TS_CS_LEVEL_IDX];
    uint64_t src_parent = unpack_eui64_u64(&rx_buf[DL_TS_PARENT_EUI_IDX]);
    uint64_t tx_remote  = unpack_40bit_ts(&rx_buf[DL_TS_TX_TS_IDX]);
    uint64_t rx_local   = get_rx_timestamp_u64();

    /* Reject obviously bogus timestamps (cross-cycle wrap) */
    if ((int64_t)(rx_local - tx_remote) < 0) return;
    if ((rx_local - tx_remote) > (uint64_t)(200 * 1000 * UUS_TO_DWT_TIME)) return;

    /* Update / allocate Kalman slot */
    int slot = find_neighbor_slot_by_rank(src_rank);
    if (slot < 0) slot = find_or_alloc_neighbor_slot(src_rank);

    int16_t cfo_raw = dwt_readclockoffset();
    uint32_t dt_ms = (g_anchor.last_heard_ms[slot] == 0)
                       ? 100
                       : HAL_GetTick() - g_anchor.last_heard_ms[slot];

    bool ok = clock_sync_update(&g_anchor.neighbors[slot], cfo_raw,
                      rx_local, tx_remote, dt_ms);
    g_anchor.last_heard_ms[slot] = HAL_GetTick();
    g_anchor.rx_count++;

    /* Diagnostics: print sync state after each update
     * Format: ARX[R=rank] CFO=±xxxxx F=xxxxxxx O=xxxxxxxx Q=xx N=xx U=xx
     *   F = factor * 1e6 (e.g. 1000189 = 1.000189)
     *   O = offset_dtu (int64 hex)
     *   Q = quality * 100 (e.g. 85 = 0.85)
     *   U = outlier count */
    {
        char dbg[80];
        int pos = 0;
        const AnchorClockSync *s = &g_anchor.neighbors[slot];
        pos += sprintf(dbg + pos, "ARX[R=%u]", src_rank);
        pos += sprintf(dbg + pos, " CFO=%+dd", cfo_raw);
        pos += sprintf(dbg + pos, " F=%07lu", (unsigned long)(s->factor * 1e6));
        pos += sprintf(dbg + pos, " O=%08lX", (unsigned long)s->offset_dtu);
        pos += sprintf(dbg + pos, " Q=%02u", (unsigned int)(s->quality * 100));
        pos += sprintf(dbg + pos, " N=%02u", s->outlier_count);
        pos += sprintf(dbg + pos, " U=%02u", s->update_count);
        pos += sprintf(dbg + pos, " %s\r\n", ok ? "OK" : "OUTL");
        DBG_PRINT(dbg);
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
        .id = (uint8_t)g_anchor.rank, .x_cm = x_cm, .y_cm = y_cm, .z_cm = z_cm
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
    dwt_forcetrxoff(); port_rf_switch_tx();
    dwt_writetxdata(sizeof(ack), ack, 0);
    dwt_writetxfctrl(sizeof(ack), 0, 0);
    dwt_starttx(DWT_START_TX_IMMEDIATE);
    { uint32_t dl = HAL_GetTick() + 10u;
      while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK))
        { if (HAL_GetTick() > dl) break; } }
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    port_EnableEXT_IRQ();
    port_rf_switch_rx();

    DBG_PRINT("+COORD_OK\r\n");
}

/*============================================================================
 * Frame dispatch (RX → handlers)
 *============================================================================*/
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

    if (func == FUNC_DOWNLINK_BEACON) {
        anchor_rx_timesync(rx_buf, frame_len);
        return;
    }

    if (func == FUNC_COORD_WRITE) {
        anchor_handle_coord_write(src,
                                  &rx_buf[GW_HDR_LEN],
                                  frame_len - GW_HDR_LEN);
        return;
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

    while (1) {
        /* Phase 1: broadcast TimeSync */
        if (!gw_ota_is_active()) {
            anchor_tx_timesync();
        }

        /* Phase 2: listen window (~95 ms) */
        dwt_forcetrxoff();
        port_rf_switch_rx();
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        uint32_t rx_start = HAL_GetTick();
        while ((HAL_GetTick() - rx_start) < ANCHOR_RX_POLL_MS) {
            Sleep(1);
            status = dwt_read32bitreg(SYS_STATUS_ID);
            if (status & SYS_STATUS_ALL_RX_ERR) {
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
            if (!match_header(rx_buf, match_timesync, 10, 1, 0, 1)) {
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
                continue;
            }
            anchor_process_rx(rx_buf, frame_len);
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }

#ifndef GATEWAY_ROLE
        if (gw_ota_poll()) {
            /* OTA timed out — resume ranging */
        }
#endif

        /* Periodic sync-state summary every ~5 s (50 × 100 ms cycles) */
        static uint32_t s_summary_ctr;
        if (++s_summary_ctr >= 50) {
            s_summary_ctr = 0;
            char summary[128];
            int pos = 0;
            pos += sprintf(summary + pos, "ASUM[R=%u] ", g_anchor.rank);
            int ni = 0;
            for (int i = 0; i < ANCHOR_MAX_NEIGHBORS; i++) {
                if (!g_anchor.neighbors[i].initialized) continue;
                const AnchorClockSync *s = &g_anchor.neighbors[i];
                uint32_t age_ms = (g_anchor.last_heard_ms[i] == 0) ? 0
                            : (HAL_GetTick() - g_anchor.last_heard_ms[i]);
                pos += sprintf(summary + pos, "N%d:F=%07lu:Q=%02u:A=%04u ",
                    ni,
                    (unsigned long)(s->factor * 1e6),
                    (unsigned int)(s->quality * 100),
                    age_ms);
                ni++;
            }
            pos += sprintf(summary + pos, "\r\n");
            DBG_PRINT(summary);
        }
    }
}

/*============================================================================
 * Anchor init — one-time setup before anchor_loop()
 *============================================================================*/
static int anchor_init(void)
{
    /* Determine rank from my_uid32 (Phase 1.1: simple mapping).
     * Phase 1.2 will introduce real election.  For now, lowest UID seen
     * during 5-second scan becomes cs_level=0; others cs_level=1.
     * Initial value: rank = uid32 % 16, cs_level = 1 (assume child). */
    uint32_t uid32 = compute_uid32();
    g_anchor.rank = (uint16_t)(uid32 % 16);
    if (g_anchor.rank == 0) g_anchor.rank = 1;  /* Reserve 0 for now (legacy) */
    g_anchor.cs_level = DL_CS_LEVEL_UNINIT;
    g_anchor.parent_eui64 = 0;
    g_anchor.my_eui64 = (uint64_t)uid32;
    g_anchor.seq = 0;
    g_anchor.tx_count = 0;
    g_anchor.rx_count = 0;
    g_anchor.prev_tx_ts = 0;

    g_anchor.my_pos = (dl_anchor_entry_t){
        .id = (uint8_t)g_anchor.rank, .x_cm = 0, .y_cm = 0, .z_cm = 0
    };

    /* Clear neighbor slots */
    int i;
    for (i = 0; i < ANCHOR_MAX_NEIGHBORS; i++) {
        g_anchor.last_heard_ms[i] = 0;
        g_anchor.neighbors[i].initialized = false;
    }

    /* Default cs_level=1 (will be re-elected in Phase 1.2).  For Phase 1.1
     * we accept that the lowest-UID anchor's TimeSync carries cs_level=0
     * only if explicitly configured via AT+COORD hierarchy message (TODO). */
    g_anchor.cs_level = 1;
    g_anchor.parent_eui64 = 0;

    return 0;
}

/*============================================================================
 * Public entry — called from main.c
 *============================================================================*/
int fusion_anchor(void)
{
    DBG_PRINT(APP_NAME "\r\n");
    DBG_PRINT("IDLE OK\r\n");

    if (anchor_init() != 0) {
        DBG_PRINT("INIT FAIL\r\n");
        return -1;
    }
    DBG_PRINT("INIT OK\r\n");
    DBG_PRINT("CONFIG OK\r\n");
    DBG_PRINT("INIT DONE\r\n");

    anchor_loop();
    return 0;
}

#endif /* TEST_FUSION_ANCHOR */