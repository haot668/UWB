/*! ----------------------------------------------------------------------------
 *  @file    fusion_anchor.c
 *  @brief   Downlink TDOA Anchor — supports Master (rank 0) and Slave (rank 1+)
 *
 *           Master Anchor (RANK == 0):
 *             1. RX TDOA Blink  → record timestamp
 *             2. TX Beacon      → periodic heartbeat + network formation
 *             3. Gateway UART   → poll AT commands
 *
 *           Slave Anchor (RANK >= 1):
 *             1. RX Beacon      → heartbeat + trigger TDMA blink
 *             2. TX Blink       → TDOA reference in TDMA slot
 *             3. Gateway UART   → poll AT commands (if ENABLE_GATEWAY)
 *
 * @attention
 * Copyright 2015-2020 (c) Decawave Ltd, Dublin, Ireland.
 * All rights reserved.
 */
#include <stdio.h>
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
#if ENABLE_GATEWAY
#include "gateway_cmd.h"
#else
/* Stub: gateway_flash_save_coord lives in gateway_cmd.c (Gateway only).
 * Master/Slave projects call it from COORD_WRITE handler — provide no-op. */
static int gateway_flash_save_coord(int16_t x, int16_t y, int16_t z)
{
    (void)x; (void)y; (void)z;
    return 0;
}
#endif
#ifdef FUSION_DOWNLINK_MODE
#include "downlink_protocol.h"
#endif

#if defined(AUTO_ANCHOR_ENABLE)
#define DISCOVERY_ENABLED 1
#else
#define DISCOVERY_ENABLED 0
#endif

#if defined(TEST_FUSION_ANCHOR)

extern void test_run_info(unsigned char *data);

/* Debug output — set to 0 to silence election/coord/OTA diagnostic prints */
#define DEBUG_ENABLE  1
#if DEBUG_ENABLE
#define DBG_PRINT(msg) test_run_info((unsigned char *)(msg))
#else
#define DBG_PRINT(msg) do {} while(0)
#endif

/* Auto-start TDOA on power-up (no上位机 needed).
 * 1 = anchors enter TDOA mode immediately after election
 * 0 = anchors wait for AT+START from上位机 (config mode) */
#define AUTO_TDOA_START  1

/* Build version string from firmware_version.h macros */
#define _VER_STR(s) #s
#define VER_STR(s)  _VER_STR(s)
#if ENABLE_GATEWAY
#define APP_NAME "FUSION GATEWAY v" VER_STR(FW_VERSION_MAJOR) "." VER_STR(FW_VERSION_MINOR)
#else
#define APP_NAME "FUSION ANCHOR v" VER_STR(FW_VERSION_MAJOR) "." VER_STR(FW_VERSION_MINOR)
#endif

/*============================================================================
 * Common configuration — 6.8 Mbps, PRF 64M, preamble code 9
 * Channel selected via UWB_CHANNEL macro in example_selection.h
 *============================================================================*/
static dwt_config_t config = {
    UWB_CHANNEL,      /* Channel number (5 or 9). */
    DWT_PLEN_128,    /* Preamble length. */
    DWT_PAC8,        /* Preamble acquisition chunk size. */
    9,               /* TX preamble code. */
    9,               /* RX preamble code. */
    1,               /* 0 = standard 8 symbol SFD, 1 = non-standard 8 symbol */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    DWT_PHRRATE_STD, /* PHY header rate. */
    (129 + 8 - 8),   /* SFD timeout */
    DWT_STS_MODE_OFF,/* STS disabled */
    DWT_STS_LEN_64,  /* STS length */
    DWT_PDOA_M0      /* PDOA mode off */
};

#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

/*============================================================================
 * Frame headers for matching (seq/addr bytes zeroed)
 *============================================================================*/
static uint8_t match_blink[]  = {0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0, 0, 0xE2};
#define HDR_LEN 10u
#define SN_IDX  2u

/*============================================================================
 * Auto-discovery match patterns (FC[2],SEQ,PAN[2],DEST[2],SRC[2],FUNC)
 *============================================================================*/
static uint8_t match_beacon[]    = {0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0, 0, 0xE7};
static uint8_t match_claim[]     = {0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0, 0, 0xE8};
static uint8_t match_join_ack[]  = {0x41, 0x88, 0, 0xCA, 0xDE, 0, 0, 0, 0, 0xEA};

#if ENABLE_GATEWAY
/* Gateway UWB response frames (addressed to gateway, SRC wildcarded).
 * NOTE: currently unused — gateway matches by func code, not address. */
static uint8_t match_gw_find_ack[]  = {0x41, 0x88, 0, 0xCA, 0xDE, 0x01, 0x80, 0, 0, 0xF3};
static uint8_t match_gw_coord_ack[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0x01, 0x80, 0, 0, 0xF5};
#endif

#ifdef FUSION_DOWNLINK_MODE
/* Downlink: beacon table + default positions */
static dl_anchor_entry_t g_dl_beacon_table[DL_MAX_ANCHORS];
static uint8_t  g_dl_num_anchors = 1;
static const int16_t g_dl_default_positions[][3] = {
    {DL_A0_X_CM, DL_A0_Y_CM, DL_A0_Z_CM},
    {DL_A1_X_CM, DL_A1_Y_CM, DL_A1_Z_CM},
    {DL_A2_X_CM, DL_A2_Y_CM, DL_A2_Z_CM},
    {DL_A3_X_CM, DL_A3_Y_CM, DL_A3_Z_CM},
    {DL_A4_X_CM, DL_A4_Y_CM, DL_A4_Z_CM},
};
static uint8_t match_dl_beacon[]  = {0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0, 0, 0xEE};
static uint8_t match_tag_blink[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0, 0, 0xE2};

/* Beacon interval for AT+RATE (runtime configurable) */
static uint16_t g_dl_beacon_interval_ms = 100;  /* default 10 Hz */

void dl_set_beacon_interval(uint16_t ms) { g_dl_beacon_interval_ms = ms; }
uint16_t dl_get_beacon_interval(void) { return g_dl_beacon_interval_ms; }

/* Update anchor coordinate in beacon table (called from gateway AT+COORD) */
void dl_set_anchor_coord(uint8_t rank, int16_t x_cm, int16_t y_cm, int16_t z_cm)
{
    if (rank < DL_MAX_ANCHORS) {
        g_dl_beacon_table[rank].id   = rank;
        g_dl_beacon_table[rank].x_cm = x_cm;
        g_dl_beacon_table[rank].y_cm = y_cm;
        g_dl_beacon_table[rank].z_cm = z_cm;
    }
}
#endif

/* Auto-discovery TX frames */
static uint8_t tx_claim[] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0x01, 0x00, 0xE8,
    0, 0, 0, 0,  0, 0
};
#undef  CLAIM_LEN
#define CLAIM_LEN 16u

static uint8_t tx_join_req[] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 0x01, 0x00, 0, 0, 0xE9,
    0, 0, 0, 0,  0, 0
};
#undef  JOIN_REQ_LEN
#define JOIN_REQ_LEN 16u

static uint8_t tx_beacon[] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0x01, 0x00, 0xE7,
    0,       /* GROUP */
    0,       /* COUNT */
    0, 0, 0, 0,   /* MASTER_UID[4] */
    0, 0     /* FCS[2] */
};
#undef  BEACON_LEN
#define BEACON_LEN 18u

/*============================================================================
 * RX buffer
 *============================================================================*/
/* Must fit OTA_DATA (261B max) + largest beacon (83B) */
#define RX_BUF_LEN  280u
static uint8_t rx_buffer[RX_BUF_LEN];

/*============================================================================
 * Timestamps and tracking variables
 *============================================================================*/
static uint64_t blink_rx_ts;
static uint8_t  last_blink_seq = 0;

#define ANCHOR_ID_VAL  g_my_addr    /* backward-compat alias */
static uint16_t g_my_addr = 0x0000u;  /* runtime: Master=0, Slave=rank (set after auto-discovery) */
static uint32_t g_my_uid32 = 0;       /* STM32 unique ID — for Master conflict detection */

extern dwt_txconfig_t txconfig_options;
extern dwt_txconfig_t txconfig_options_ch9;
#if UWB_CHANNEL == 9
#define TXCONFIG (&txconfig_options_ch9)
#else
#define TXCONFIG (&txconfig_options)
#endif

/*============================================================================
 * Helper: UART hex output
 *============================================================================*/
static const char hex[] = "0123456789ABCDEF";

/*============================================================================
 * Find blink — LED1 fast flash (5 Hz, 100ms on / 100ms off) for 5 seconds
 *============================================================================*/
uint32_t g_find_blink_until = 0;
static int    g_config_mode = 1;  /* 1=config(no beacon), 0=TDOA active */
/* gw_tdoa_active declared in gateway_cmd.h */
static int g_led_heartbeat = 0;  /* 1=Master (heartbeat blink), 0=Slave (LED OFF) */
static uint8_t g_node_count = 1; /* Master + joined slaves, shared by net-formation & main loop */

/* Rank dedup: remember the UID of each joined slave (index = rank-1) so a
 * slave that re-joins (e.g. after a brief master drop) is re-assigned its
 * ORIGINAL rank instead of consuming a fresh one and causing rank drift. */
static uint32_t g_joined_uids[16];
static uint8_t  g_joined_count = 0;

static int joined_rank_of(uint32_t uid)
{
    int i;
    for (i = 0; i < (int)g_joined_count; i++)
        if (g_joined_uids[i] == uid)
            return i + 1;   /* rank = index + 1 */
    return -1;
}

static void find_blink_poll(void)
{
    uint32_t now = HAL_GetTick();
    /* FIND active: fast blink at 10 Hz (50ms on / 50ms off) for 5 seconds.
     * Overrides normal LED pattern — FIND_REQ sets g_find_blink_until. */
    if (g_find_blink_until != 0 && now < g_find_blink_until) {
        uint32_t t = now % 100u;
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin,
            (t < 50u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        return;
    }
    /* V0.7: g_led_heartbeat selects normal LED pattern.
     *   1 = Master: slow heartbeat (500ms on / 500ms off)
     *   0 = Slave:  LED1 OFF (only blinks during FIND) */
    if (g_led_heartbeat) {
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin,
            ((now % 1000u) < 500u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
    }
}

/*============================================================================
 * Helper: match a received frame header
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

#if DISCOVERY_ENABLED

/*============================================================================
 * auto_discovery — LISTEN → ELECT → JOIN → RUN state machine
 *
 * Returns:
 *   0 on success (disc->my_rank set, 0=master, 1..15=slave)
 *   1 on error   (disc->state == AUTO_STATE_ERROR)
 *============================================================================*/
static int auto_discovery(auto_disc_t *disc)
{
    uint32_t status_reg, frame_len;

    /* Read STM32 unique ID (96-bit @ 0x1FFF7590).
     * XOR all three 32-bit words for better distribution — two chips from
     * the same batch may share similar lower-32 but differ in upper bits. */
    disc->my_uid32 = *(uint32_t *)0x1FFF7590UL      /* X-coordinate on wafer */
                   ^ *(uint32_t *)0x1FFF7594UL      /* Y-coordinate */
                   ^ *(uint32_t *)0x1FFF7598UL;     /* Lot number */
    disc->state = AUTO_STATE_LISTEN;
    disc->my_rank = 0xFFu;  /* unassigned */
    disc->group_id = 0;
    disc->node_count = 0;
    disc->best_uid = disc->my_uid32;
    disc->candidates_seen = 0;
    disc->master_addr[0] = 0x01;
    disc->master_addr[1] = 0x00;

    /* Initialize RF switch in RX mode for discovery listening */
    port_rf_switch_init();
    port_rf_switch_rx();

    /*=== Phase: LISTEN — listen for existing master beacon (2–5 sec) ===
     * Variable timeout per device based on UID: lower UID → shorter LISTEN
     * → finishes first → enters ELECT first → becomes Master.  Higher UID
     * devices listen longer and hear the new Master's beacon.
     *
     * Even with identical stagger, two devices will time out at different
     * times — the one that finishes first becomes Master, the other hears
     * its beacon during its remaining LISTEN window. */
    {
        int attempt;
        /* P2: staggered LISTEN (3-8s).  Lower UID → shorter LISTEN → enters
         * ELECT first → becomes Master → beacon reaches others still in LISTEN.
         * Range is a trade-off: too narrow = split-brain, too wide = slow boot. */
        int listen_attempts = 30 + (int)(disc->my_uid32 % 51u);  /* 30–80 = 3–8 sec */
        {
            char _dbg[40];
            snprintf(_dbg, sizeof(_dbg), "[EL] LISTEN uid=0x%08lX wait=%d\r\n",
                     disc->my_uid32, listen_attempts);
            DBG_PRINT(_dbg);
        }
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        /* RX enabled */

        for (attempt = 0; attempt < listen_attempts; attempt++)
        {
            Sleep(100);  /* ~100ms poll interval, total 20*100ms = 2 sec */
            status_reg = dwt_read32bitreg(SYS_STATUS_ID);
            if (status_reg & SYS_STATUS_ALL_RX_ERR)
            {
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
                continue;
            }
            if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK))
                continue;

            /* Frame received */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
            if (frame_len > sizeof(rx_buffer))
            {
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
                continue;
            }

            dwt_readrxdata(rx_buffer, frame_len, 0);

            /* Beacon (0xE7) → Master already exists, join it immediately */
            if (match_header(rx_buffer, match_beacon, HDR_LEN, 1, 0, 1))
            {
                disc->group_id = rx_buffer[AUTO_BCN_GROUP_IDX];
                disc->node_count = rx_buffer[AUTO_BCN_COUNT_IDX];
                disc->master_addr[0] = rx_buffer[7];  /* SRC low byte */
                disc->master_addr[1] = rx_buffer[8];  /* SRC high byte */
                disc->best_uid = unpack_uid(&rx_buffer[AUTO_UID_IDX]);
                disc->state = AUTO_STATE_JOIN;
                goto phase_join;
            }

            /* Claim (0xE8) → another device is in ELECT.  Extend LISTEN
             * to wait for the beacon that will follow: ELECT takes ~1.5s
             * plus ~0.5s until first beacon = need ≥2s more listening.
             * Cap at 80 polls (8s) to prevent pathological extension. */
            if (match_header(rx_buffer, match_claim, HDR_LEN, 1, 0, 1))
            {
                int needed = attempt + 20;
                if (needed > listen_attempts && listen_attempts < 80) {
                    listen_attempts = (needed < 80) ? needed : 80;
                }
            }

            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            continue;
        }


    }

    /*=== Phase: ELECT — no master found, run election ===
     * P0-2: Up to 3 re-election attempts.  Each attempt:
     *   1. Listen-only 200ms (conflict detection window)
     *   2. Random delay 1-200ms (UID-based desync)
     *   3. 3-round claim+listen cycle
     * If no candidates are seen after a full cycle, a collision likely
     * occurred — retry with a different random delay.  If candidates
     * are seen, the election is valid and the lowest UID wins. */
    {
        int elect_retry;

        for (elect_retry = 0; elect_retry < 3; elect_retry++)
        {
            disc->state = AUTO_STATE_ELECT;
            disc->candidates_seen = 0;
            disc->best_uid = disc->my_uid32;
            {
                char _dbg[48];
                snprintf(_dbg, sizeof(_dbg), "[EL] ELECT attempt %d uid=0x%08lX\r\n",
                         elect_retry, disc->my_uid32);
                DBG_PRINT(_dbg);
            }

        /* --- P2: Conflict detection window — listen 500ms before claiming.
             * If another device is already in ELECT, we hear its claim and yield.
             * 5 polls × 100ms = 500ms silent listen (was 200ms, too short). ---*/
            {
                int cw;
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
                for (cw = 0; cw < 5; cw++)
                {
                    Sleep(100);
                    status_reg = dwt_read32bitreg(SYS_STATUS_ID);
                    if (status_reg & SYS_STATUS_ALL_RX_ERR)
                    {
                        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                        dwt_rxenable(DWT_START_RX_IMMEDIATE);
                        continue;
                    }
                    if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK))
                        continue;
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
                    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
                    if (frame_len <= sizeof(rx_buffer))
                    {
                        dwt_readrxdata(rx_buffer, frame_len, 0);
                        if (match_header(rx_buffer, match_claim, HDR_LEN, 1, 0, 1))
                        {
                            uint32_t other_uid = unpack_uid(&rx_buffer[AUTO_UID_IDX]);
                            disc->candidates_seen++;
                            if (other_uid < disc->best_uid)
                                disc->best_uid = other_uid;
                        }
                        else if (match_header(rx_buffer, match_beacon, HDR_LEN, 1, 0, 1))
                        {
                            disc->group_id = rx_buffer[AUTO_BCN_GROUP_IDX];
                            disc->node_count = rx_buffer[AUTO_BCN_COUNT_IDX];
                            disc->best_uid = unpack_uid(&rx_buffer[AUTO_UID_IDX]);
                            disc->state = AUTO_STATE_JOIN;
                            dwt_rxenable(DWT_START_RX_IMMEDIATE);
                            goto phase_join;
                        }
                    }
                    dwt_rxenable(DWT_START_RX_IMMEDIATE);
                }
            }

            /* Already found a better candidate during conflict window → yield */
            if (disc->candidates_seen > 0 && disc->best_uid < disc->my_uid32)
                break;

            /*--- P0-1: Wide random delay (1-200ms) using full 32-bit UID.
             * Two devices with UIDs differing only in high bytes still get
             * different delays.  Range: uid % 199 + 1 = 1..200ms. ---*/
            Sleep((disc->my_uid32 % 199u) + 1u);

            /* 3-round claim: TX→listen→TX→listen→TX→listen */
            {
                int round;
                for (round = 0; round < 3; round++)
                {
                    /* TX ELECTION_CLAIM */
                    dwt_forcetrxoff();
                    port_rf_switch_tx();
                    tx_claim[2] = (uint8_t)round;
                    tx_claim[7] = (uint8_t)(disc->my_uid32 & 0xFFu);
                    tx_claim[8] = (uint8_t)((disc->my_uid32 >> 8) & 0xFFu);
                    pack_uid(&tx_claim[AUTO_UID_IDX], disc->my_uid32);
                    dwt_writetxdata(CLAIM_LEN, tx_claim, 0);
                    dwt_writetxfctrl(CLAIM_LEN, 0, 0);
                    if (dwt_starttx(DWT_START_TX_IMMEDIATE) == DWT_SUCCESS)
                    {
                        { uint32_t tx_dl = HAL_GetTick() + 10u; while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) { if (HAL_GetTick() > tx_dl) break; } }
                        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
                    }
                    port_rf_switch_rx();

                    /* Listen for other claims — 5 polls * 100ms = 500ms per round */
                    {
                        int claim_attempt;
                        dwt_rxenable(DWT_START_RX_IMMEDIATE);

                        for (claim_attempt = 0; claim_attempt < 5; claim_attempt++)
                        {
                            Sleep(100);
                            status_reg = dwt_read32bitreg(SYS_STATUS_ID);
                            if (status_reg & SYS_STATUS_ALL_RX_ERR)
                            {
                                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                                dwt_rxenable(DWT_START_RX_IMMEDIATE);
                                continue;
                            }
                            if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK))
                                continue;

                            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
                            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
                            if (frame_len > sizeof(rx_buffer))
                            {
                                dwt_rxenable(DWT_START_RX_IMMEDIATE);
                                continue;
                            }

                            dwt_readrxdata(rx_buffer, frame_len, 0);
                            if (match_header(rx_buffer, match_claim, HDR_LEN, 1, 0, 1))
                            {
                                uint32_t other_uid = unpack_uid(&rx_buffer[AUTO_UID_IDX]);
                                disc->candidates_seen++;
                                if (other_uid < disc->best_uid)
                                    disc->best_uid = other_uid;
                            }
                            else if (match_header(rx_buffer, match_beacon, HDR_LEN, 1, 0, 1))
                            {
                                /* Master already exists — yield election */
                                disc->group_id = rx_buffer[AUTO_BCN_GROUP_IDX];
                                disc->node_count = rx_buffer[AUTO_BCN_COUNT_IDX];
                                disc->best_uid = unpack_uid(&rx_buffer[AUTO_UID_IDX]);
                                disc->state = AUTO_STATE_JOIN;
                                dwt_rxenable(DWT_START_RX_IMMEDIATE);
                                goto phase_join;
                            }
                            dwt_rxenable(DWT_START_RX_IMMEDIATE);
                        }
                    }

                    /* If we already see a better candidate, yield early */
                    if (disc->candidates_seen > 0 && disc->best_uid < disc->my_uid32)
                        break;
                }
            }

            /*--- P0-2: If we saw other candidates or a beacon, the election
             * was valid — stop retrying.  If nobody was seen (possible RF
             * collision), retry with a different random delay. ---*/
            if (disc->candidates_seen > 0)
                break;


        }
    }

    /* Decide: lowest UID wins */
    if (disc->my_uid32 <= disc->best_uid)
    {
        /* P2 Grace Period: won election, but listen 2s for conflicting
         * Master beacons before declaring victory.  Two devices can both
         * win ELECT if they don't hear each other's claims (half-duplex
         * collision).  If we hear a beacon from a Master with a LOWER UID,
         * we yield — reboot to re-discover as Slave. */
        {
            int gp;
            DBG_PRINT("[EL] WIN MASTER — grace period 2s...\r\n");
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            for (gp = 0; gp < 20; gp++)
            {
                Sleep(100);
                status_reg = dwt_read32bitreg(SYS_STATUS_ID);
                if (status_reg & SYS_STATUS_ALL_RX_ERR) {
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                    dwt_rxenable(DWT_START_RX_IMMEDIATE);
                    continue;
                }
                if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK))
                    continue;
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
                frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
                if (frame_len <= sizeof(rx_buffer)) {
                    dwt_readrxdata(rx_buffer, frame_len, 0);
                    if (match_header(rx_buffer, match_beacon, HDR_LEN, 1, 0, 1)) {
                        uint32_t other_uid = unpack_uid(&rx_buffer[AUTO_UID_IDX]);
                        if (other_uid < disc->my_uid32) {
                            DBG_PRINT("[EL] CONFLICT — lower UID Master seen → reboot\r\n");
                            NVIC_SystemReset();
                            while(1){}
                        }
                    }
                }
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
            }
        }
        /* I am master */
        disc->my_rank = 0;
        disc->node_count = 1;  /* only me for now */
        disc->master_addr[0] = 0x01;
        disc->master_addr[1] = 0x00;

        disc->state = AUTO_STATE_RUN;
        DBG_PRINT("[EL] WIN MASTER confirmed\r\n");
        return 0;
    }

    {
        char _dbg[48];
        snprintf(_dbg, sizeof(_dbg), "[EL] LOSE → JOIN best=0x%08lX\r\n", disc->best_uid);
        DBG_PRINT(_dbg);
    }
    disc->state = AUTO_STATE_JOIN;

phase_join:
    /*=== Phase: JOIN — request rank from master ===*/
    {
        int retry;

        /* Up to 3 join attempts; fall back to LISTEN on total failure */
        for (retry = 0; retry < 3; retry++)
        {
            int join_attempt;

            /* Random delay before each attempt to avoid collisions */
            Sleep(((disc->my_uid32 >> (retry * 3)) & 0xFFu) % 100u);

            /* TX JOIN_REQUEST */
            dwt_forcetrxoff();  /* force DW3000 out of RX before TX */
            port_rf_switch_tx();
            tx_join_req[2] = (uint8_t)retry;
            tx_join_req[7] = disc->my_uid32 & 0xFFu;
            tx_join_req[8] = (disc->my_uid32 >> 8) & 0xFFu;
            pack_uid(&tx_join_req[AUTO_UID_IDX], disc->my_uid32);
            dwt_writetxdata(JOIN_REQ_LEN, tx_join_req, 0);
            dwt_writetxfctrl(JOIN_REQ_LEN, 0, 0);
            if (dwt_starttx(DWT_START_TX_IMMEDIATE) == DWT_SUCCESS)
            {
                { uint32_t tx_dl = HAL_GetTick() + 10u; while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) { if (HAL_GetTick() > tx_dl) break; } }
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
            }
            /* Switch to RX BEFORE UART to minimize RX dead window */
            port_rf_switch_rx();

            /* Wait for JOIN_ACK — Sleep(1) polling to avoid single-buffer
             * overwrite by a later beacon (1 ms poll vs 100 ms before).
             * 2000 iterations × 1 ms = 2 sec timeout. */
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            /* join request sent */

            for (join_attempt = 0; join_attempt < 2000; join_attempt++)
            {
                Sleep(1);
                status_reg = dwt_read32bitreg(SYS_STATUS_ID);
                if (status_reg & SYS_STATUS_ALL_RX_ERR)
                {
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                    dwt_rxenable(DWT_START_RX_IMMEDIATE);
                    continue;
                }
                if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK))
                    continue;

                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
                frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
                if (frame_len > sizeof(rx_buffer))
                {
                    dwt_rxenable(DWT_START_RX_IMMEDIATE);
                    continue;
                }

                dwt_readrxdata(rx_buffer, frame_len, 0);

                /* Check if this is a JOIN_ACK — ignore DST+SRC (UID verified separately) */
                if (!match_header(rx_buffer, match_join_ack, HDR_LEN, 1, 1, 1))
                {
                    dwt_rxenable(DWT_START_RX_IMMEDIATE);
                    continue;
                }

                /* Verify the UID matches ours */
                {
                    uint32_t ack_uid = unpack_uid(&rx_buffer[AUTO_UID_IDX]);
                    if (ack_uid != disc->my_uid32)
                    {
                        dwt_rxenable(DWT_START_RX_IMMEDIATE);
                        continue;
                    }

                    disc->my_rank = rx_buffer[AUTO_ACK_RANK_IDX];
                    disc->group_id = rx_buffer[AUTO_ACK_NODE_IDX];

                    {
                        char _dbg[40];
                        snprintf(_dbg, sizeof(_dbg), "[EL] JOIN OK rank=%d\r\n",
                                 (int)disc->my_rank);
                        DBG_PRINT(_dbg);
                    }
                    disc->state = AUTO_STATE_RUN;
                    return 0;
                }
            }


        }

        /* All join retries failed — restart discovery from scratch */

        disc->state = AUTO_STATE_ERROR;
        return 1;
    }
}

#else /* !DISCOVERY_ENABLED — static rank from example_selection.h */

static int auto_discovery(auto_disc_t *disc)
{
#ifndef FUSION_ANCHOR_RANK
#define FUSION_ANCHOR_RANK 1
#endif
    disc->my_rank = (uint8_t)FUSION_ANCHOR_RANK;
    disc->node_count = DL_NUM_ANCHORS;
    disc->master_addr[0] = 0x01;
    disc->master_addr[1] = 0x00;
    disc->state = AUTO_STATE_RUN;
    return 0;
}

#endif /* DISCOVERY_ENABLED */

/*============================================================================
 * Common init — performs DW3000 init sequence, returns 0 on success
 *============================================================================*/
static int anchor_init(void)
{
    int attempt;

    test_run_info((unsigned char *)APP_NAME);

    /* DW3000 PLL may fail to lock on warm reset (deca_device.c:1523-1534).
     * Three-tier retry: (1) extended PLL timeout in driver (50×20us=1ms),
     * (2) soft PLL re-lock + PGF re-cal without cold reset, (3) full cold reset. */
    for (attempt = 0; attempt < 3; attempt++)
    {
        int timeout;

        port_set_dw_ic_spi_fastrate();
        reset_DWIC();
        Sleep(100);

        timeout = 1000;
        while (!dwt_checkidlerc() && --timeout > 0) { Sleep(1); }
        if (timeout <= 0)
        {

            continue;  /* retry with another reset */
        }


        if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
        {

            continue;
        }


        if (dwt_configure(&config) == DWT_SUCCESS)
        {

            break;  /* success */
        }

        /* --- PLL or PGF failed. Tier-2: soft re-try without cold reset ---
         * Registers are already written by dwt_configure. Re-trigger PLL
         * calibration with extended timeout (5ms), then re-run PGF cal. */

        {
            uint8_t pll_cnt, pll_ok = 0;

            /* Re-write PLL delay, clear lock bit, kick calibration */
            dwt_write8bitoffsetreg(PLL_CAL_ID, 0, RF_PLL_CFG_LD);
            dwt_write8bitoffsetreg(SYS_STATUS_ID, 0, SYS_STATUS_CP_LOCK_BIT_MASK);
            dwt_setdwstate(DWT_DW_IDLE);

            /* Wait up to 5ms for PLL lock (250 × 20us) */
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

                if (dwt_pgf_cal(1) == DWT_SUCCESS)
                {

                    break;  /* soft retry succeeded */
                }

            }
            else
            {

            }
        }
        /* Tier-3: full cold reset on next iteration */
    }

    if (attempt >= 3)
    {
        test_run_info((unsigned char *)"DWF");
        return -1;
    }

    /* Disable hardware frame filtering (FFEN=0) — accept all frames */
    {
        uint32_t sys_cfg = dwt_read32bitreg(SYS_CFG_ID);
        sys_cfg &= ~((uint32_t)0x1u);  /* SYS_CFG bit 0 = FFEN */
        sys_cfg |= SYS_CFG_RXAUTR_BIT_MASK;
        dwt_write32bitreg(SYS_CFG_ID, sys_cfg);
    }

    /* Enable CIA diagnostics — required for dwt_readclockoffset() and
     * dwt_readcarrierintegrator() to return valid clock offset data.
     * Per DW3000 API Guide §5.6.9, must be called after dwt_configure(). */
    dwt_configciadiag(DW_CIA_DIAG_LOG_ALL);

    /* Enable DW3000 GPIO clocks (required for IRQ etc.) */
    dwt_enablegpioclocks();

    /* Boost TX power to maximum for debugging (was 0xfefefefe) */
    {
        dwt_txconfig_t max_tx = { 0x34, 0x1F1F1F1F, 0x0 };
        dwt_configuretxrf(&max_tx);
    }
    /* (Original TXCONFIG would be applied here — replaced by max power above) */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_setfinegraintxseq(0);
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
    test_run_info((unsigned char *)"IOK\r\n");
    return 0;
}

/*============================================================================
 * MAIN ANCHOR — rank 0: Full protocol with TDMA collector
 *============================================================================*/
static void main_anchor_loop(void)
{
    uint16_t beacon_cycle = 0;
    static uint8_t  is_first_beacon = 1;
    static uint64_t prev_tx_ts_dtu = 0;  /* last actual TX timestamp for DLY_TS */
    test_run_info((unsigned char *)"RDY\r\n");

    port_rf_switch_init();

#ifndef GATEWAY_ROLE
    /* P2: Confirm successful boot — clear bootloader's STATE=BOOTING flag.
     * Must be called AFTER all init (DW3000, UART, etc.) and BEFORE main loop.
     * If app crashed before this point, bootloader will re-copy from staging.
     * Gateway doesn't have a bootloader → skip. */
    gw_ota_confirm_boot();
#endif

    while (1)
    {
        uint32_t status_reg, frame_len;

        /*=== Phase 1: RX for ~40ms (1ms polling to prevent DW3000 buffer overflow) ===*/
        {
            int _q;
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            for (_q = 0; _q < 40; _q++) {
                Sleep(1);  /* 1ms polling — drains DW3000 HW buffer quickly */
                find_blink_poll();
                status_reg = dwt_read32bitreg(SYS_STATUS_ID);
                if (status_reg & SYS_STATUS_ALL_RX_ERR) {
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                    dwt_rxenable(DWT_START_RX_IMMEDIATE);
                    continue;
                }
                if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK)) continue;
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
                frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
                if (frame_len > sizeof(rx_buffer)) {
                    dwt_rxenable(DWT_START_RX_IMMEDIATE);
                    continue;
                }
                dwt_readrxdata(rx_buffer, frame_len, 0);

#ifndef GATEWAY_ROLE
                /*=== OTA fast-path: handle BEFORE re-entering RX ===
                 * DW3000 is in IDLE (EXTI handler called dwt_forcetrxoff()
                 * when this frame arrived).  By processing OTA frames before
                 * dwt_rxenable(), we keep DW3000 in IDLE during the entire
                 * flash-write + ACK sequence — eliminating the trap window
                 * that caused chunk loss.
                 *
                 * ota_send_ack() re-enables RX after the ACK TX burst. */
                if (rx_buffer[9] == FUNC_OTA_DATA && frame_len >= GW_HDR_LEN + 4)
                {
                    gw_ota_handle_data(rx_buffer, frame_len, ANCHOR_ID_VAL);
                    continue;
                }
                if (rx_buffer[9] == FUNC_OTA_END)
                {
                    gw_ota_finish(ANCHOR_ID_VAL);
                    /* gw_ota_finish reboots on success; if it returns,
                     * re-enter RX so this 470ms phase isn't deaf. */
                    dwt_rxenable(DWT_START_RX_IMMEDIATE);
                    continue;
                }
                if (rx_buffer[9] == FUNC_OTA_ABORT)
                {
                    g_config_mode = 0;
                    gw_ota_abort();
                    dwt_rxenable(DWT_START_RX_IMMEDIATE);
                    continue;
                }
#endif
                /* IRQ handler's dwt_forcetrxoff() forced DW3000 to IDLE.
                 * Re-enter RX for non-OTA frames — OTA frames were handled
                 * above with DW3000 still in IDLE (no trap window). */
                dwt_rxenable(DWT_START_RX_IMMEDIATE);

                if (match_header(rx_buffer, match_blink, HDR_LEN, 1, 0, 1))
                {
                    last_blink_seq = rx_buffer[SN_IDX];
                    blink_rx_ts = get_rx_timestamp_u64();
#if ENABLE_GATEWAY
                    {
                        char uid[8];
                        uint16_t tag_addr = (uint16_t)(rx_buffer[7] | (rx_buffer[8] << 8));
                        uid[0] = hex[(tag_addr >> 12) & 0xF];
                        uid[1] = hex[(tag_addr >>  8) & 0xF];
                        uid[2] = hex[(tag_addr >>  4) & 0xF];
                        uid[3] = hex[ tag_addr        & 0xF];
                        uid[4] = '\0';
                        gateway_device_seen(uid, "TAG", 0);
                    }
#endif
                }
                /* Master conflict detection: if another Master is beaconing
                 * with a lower UID than ours, we must yield.  This handles
                 * the split-brain scenario where two anchors both win ELECT
                 * (e.g., due to temporary RF isolation during election).
                 * Lowest UID always wins — the higher-UID Master reboots
                 * and re-runs discovery, joining as Slave. */
                else if (match_header(rx_buffer, match_beacon, HDR_LEN, 1, 0, 1))
                {
                    uint32_t bcn_uid = unpack_uid(&rx_buffer[AUTO_UID_IDX]);
                    if (bcn_uid < g_my_uid32) {

                        HAL_Delay(100);
                        NVIC_SystemReset();
                        while (1) { }
                    }
                }
                else if (rx_buffer[9] == FUNC_ELECTION_CLAIM)
                {
                    port_DisableEXT_IRQ();
                    dwt_forcetrxoff();
                    port_rf_switch_tx();
                    tx_beacon[2] = 0;
                    tx_beacon[AUTO_BCN_GROUP_IDX] = 0;
                    tx_beacon[AUTO_BCN_COUNT_IDX] = g_node_count;
                    pack_uid(&tx_beacon[AUTO_UID_IDX], g_my_uid32);
                    dwt_writetxdata(BEACON_LEN, tx_beacon, 0);
                    dwt_writetxfctrl(BEACON_LEN, 0, 0);
                    if (dwt_starttx(DWT_START_TX_IMMEDIATE) == DWT_SUCCESS) {
                        { uint32_t tx_dl = HAL_GetTick() + 10u; while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) { if (HAL_GetTick() > tx_dl) break; } }
                        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
                    }
                    port_rf_switch_rx();
                    dwt_rxenable(DWT_START_RX_IMMEDIATE);
                    EXTI->PR1 = (1u << 12);
                    port_EnableEXT_IRQ();
                }
                else if (rx_buffer[9] == FUNC_JOIN_REQUEST && frame_len >= JOIN_REQ_LEN - 2)
                {
                    uint32_t req_uid = unpack_uid(&rx_buffer[AUTO_UID_IDX]);
                    uint8_t req_src_lo = rx_buffer[7];
                    uint8_t req_src_hi = rx_buffer[8];
                    int existing = joined_rank_of(req_uid);
                    uint8_t new_rank;
                    int assign = 0;
                    if (existing >= 0) {
                        new_rank = (uint8_t)existing;   /* re-join: keep original rank */
                        assign = 1;
                    } else if (g_node_count < 16) {
                        new_rank = g_node_count;
                        g_joined_uids[g_joined_count++] = req_uid;
                        g_node_count++;
                        assign = 1;
                    }
                    if (assign) {
                        port_DisableEXT_IRQ();
                        dwt_forcetrxoff();
                        port_rf_switch_tx();
                        {
                            uint8_t ack[] = {
                                0x41, 0x88, 0, 0xCA, 0xDE,
                                req_src_lo, req_src_hi,
                                0x01, 0x00, 0xEA,
                                0, 0, 0, 0, new_rank, 0, 0
                            };
                            pack_uid(&ack[10], req_uid);
                            dwt_writetxdata(sizeof(ack), ack, 0);
                            dwt_writetxfctrl(sizeof(ack), 0, 0);
                            if (dwt_starttx(DWT_START_TX_IMMEDIATE) == DWT_SUCCESS) {
                                { uint32_t tx_dl = HAL_GetTick() + 10u; while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) { if (HAL_GetTick() > tx_dl) break; } }
                                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
                            }
                        }
                        port_rf_switch_rx();
                        dwt_rxenable(DWT_START_RX_IMMEDIATE);
                        EXTI->PR1 = (1u << 12);
                        port_EnableEXT_IRQ();
                    }
                }
                /* FIND_REQ (0xF2) */
                else if (rx_buffer[9] == 0xF2 && (frame_len == 11 || frame_len == 13)
                         && ((uint16_t)(rx_buffer[5] | (rx_buffer[6] << 8)) == ANCHOR_ID_VAL
                             || (uint16_t)(rx_buffer[5] | (rx_buffer[6] << 8)) == 0xFFFF))
                {
                    g_find_blink_until = HAL_GetTick() + 5000;
                    /* ACK back to Gateway — 13B padding avoids DW3210
                     * last-2B RX corruption on the Gateway side. */
                    { uint8_t ack[13]; int _r;
                      ack[0]=0x41; ack[1]=0x88; ack[2]=0;
                      ack[3]=0xCA; ack[4]=0xDE; ack[5]=rx_buffer[7]; ack[6]=rx_buffer[8];
                      ack[7]=rx_buffer[5]; ack[8]=rx_buffer[6]; ack[9]=0xF3;
                      ack[10]=0xCC; ack[11]=0xDD; ack[12]=0xEE;
                      port_DisableEXT_IRQ();
                      for(_r=0;_r<3;_r++){
                        dwt_forcetrxoff(); port_rf_switch_tx();
                        dwt_writetxdata(sizeof(ack),ack,0); dwt_writetxfctrl(sizeof(ack),0,0);
                        if(dwt_starttx(DWT_START_TX_IMMEDIATE)==DWT_SUCCESS){
                          { uint32_t tx_dl = HAL_GetTick() + 10u; while(!(dwt_read32bitreg(SYS_STATUS_ID)&SYS_STATUS_TXFRS_BIT_MASK)){ if(HAL_GetTick() > tx_dl) break; } }
                          dwt_write32bitreg(SYS_STATUS_ID,SYS_STATUS_TXFRS_BIT_MASK);}
                        port_rf_switch_rx(); if(_r<2) Sleep(5);
                      }
                      dwt_rxenable(DWT_START_RX_IMMEDIATE);
                      EXTI->PR1 = (1u << 12);
                      port_EnableEXT_IRQ(); }
                    /* The immediate ACK TX above overwrote the DW3000 TX_TIME
                     * (the DLY_TS reference for the next beacon).  Force the next
                     * beacon to use Immediate TX so TX_TIME + prev_tx_ts_dtu are
                     * re-synced; otherwise the beacon cadence and the embedded
                     * predicted TX timestamp drift → slaves/tag lose the downlink. */
                    is_first_beacon = 1;
                }
                /* COORD_WRITE (0xF4) */
                else if (rx_buffer[9] == 0xF4 && frame_len >= GW_HDR_LEN + 6)
                {
                    uint8_t  tr = (uint8_t)(rx_buffer[5] | (rx_buffer[6] << 8));
                    int16_t cx = (int16_t)(rx_buffer[GW_HDR_LEN]   | (rx_buffer[GW_HDR_LEN+1] << 8));
                    int16_t cy = (int16_t)(rx_buffer[GW_HDR_LEN+2] | (rx_buffer[GW_HDR_LEN+3] << 8));
                    int16_t cz = (int16_t)(rx_buffer[GW_HDR_LEN+4] | (rx_buffer[GW_HDR_LEN+5] << 8));
                    dl_set_anchor_coord(tr, cx, cy, cz);
                    {
                        char _dbg[64];
                        snprintf(_dbg, sizeof(_dbg),
                                 "[COORD] rank=%d → (%d.%02d,%d.%02d,%d.%02d)\r\n",
                                 (int)tr, (int)(cx/100), (int)((cx<0?-cx:cx)%100),
                                 (int)(cy/100), (int)((cy<0?-cy:cy)%100),
                                 (int)(cz/100), (int)((cz<0?-cz:cz)%100));
                        DBG_PRINT(_dbg);
                    }
                    /* ACK back to Gateway */
                    { uint8_t ack[13]; int _r;
                      ack[0]=0x41; ack[1]=0x88; ack[2]=0;
                      ack[3]=0xCA; ack[4]=0xDE; ack[5]=rx_buffer[7]; ack[6]=rx_buffer[8];
                      ack[7]=rx_buffer[5]; ack[8]=rx_buffer[6]; ack[9]=FUNC_COORD_ACK;
                      ack[10]=0xCC; ack[11]=0xDD; ack[12]=0xEE;
                      port_DisableEXT_IRQ();
                      for(_r=0;_r<3;_r++){
                        dwt_forcetrxoff(); port_rf_switch_tx();
                        dwt_writetxdata(sizeof(ack),ack,0); dwt_writetxfctrl(sizeof(ack),0,0);
                        if(dwt_starttx(DWT_START_TX_IMMEDIATE)==DWT_SUCCESS){
                          { uint32_t tx_dl = HAL_GetTick() + 10u; while(!(dwt_read32bitreg(SYS_STATUS_ID)&SYS_STATUS_TXFRS_BIT_MASK)){ if(HAL_GetTick() > tx_dl) break; } }
                          dwt_write32bitreg(SYS_STATUS_ID,SYS_STATUS_TXFRS_BIT_MASK);}
                        port_rf_switch_rx(); if(_r<2) Sleep(5);
                      }
                      dwt_rxenable(DWT_START_RX_IMMEDIATE);
                      EXTI->PR1 = (1u << 12);
                      port_EnableEXT_IRQ(); }
                    /* Immediate ACK TX overwrote TX_TIME — re-sync DLY_TS base
                     * (same as FIND/START; COORD can arrive during active TDOA). */
                    is_first_beacon = 1;
                }
                /* PREP_CMD (0xF0) — unified prepare: enter cmd mode, stop ranging.
                 * Payload: total_size(4B LE) — firmware image size in bytes.
                 * If frame has the 4B payload, initialise OTA state machine
                 * (erase staging, set total_size).  Backward-compatible with
                 * old 3B {'F',0xCC,0xDD} payload — just enters config mode. */
                else if (rx_buffer[9] == FUNC_PREP_CMD)
                {
                    test_run_info((unsigned char *)"PREP CMD\r\n");
                    g_config_mode = 1;
                    /* ACK first — sent BEFORE flash erase so Gateway gets
                     * instant response.  Erase takes ~660ms which would
                     * exceed Gateway's 10ms-per-burst polling window. */
                    { uint8_t ack[13]; int _r;
                      ack[0]=0x41; ack[1]=0x88; ack[2]=0;
                      ack[3]=0xCA; ack[4]=0xDE; ack[5]=rx_buffer[7]; ack[6]=rx_buffer[8];
                      ack[7]=ANCHOR_ID_VAL&0xFF; ack[8]=(ANCHOR_ID_VAL>>8)&0xFF;
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
                      port_EnableEXT_IRQ(); }
#ifndef GATEWAY_ROLE
                    /* Erase staging AFTER ACK: Gateway now knows we're ready.
                     * Gateway Block-0 uses 3000ms timeout to cover our erase. */
                    if (frame_len >= GW_HDR_LEN + 4) {
                        uint32_t ota_sz = (uint32_t)(rx_buffer[GW_HDR_LEN])
                                        | ((uint32_t)(rx_buffer[GW_HDR_LEN + 1]) << 8)
                                        | ((uint32_t)(rx_buffer[GW_HDR_LEN + 2]) << 16)
                                        | ((uint32_t)(rx_buffer[GW_HDR_LEN + 3]) << 24);
                        gw_ota_handle_prepare(ota_sz);
                    } else {
                        test_run_info((unsigned char *)"NOPAY\r\n");
                    }
#endif
                }
                /* START_TDOA / STOP_TDOA / NETWORK_CFG — all reply with ACK */
                else if (rx_buffer[9] == 0xF8) {
                    DBG_PRINT("[TDOA] START → active\r\n");
                    g_config_mode = 0;
                    /* ACK back to Gateway — 13B to avoid DW3210 last-2B RX bug */
                    { uint8_t ack[13]; int _r;
                      ack[0]=0x41; ack[1]=0x88; ack[2]=0;
                      ack[3]=0xCA; ack[4]=0xDE; ack[5]=rx_buffer[7]; ack[6]=rx_buffer[8];
                      ack[7]=rx_buffer[5]; ack[8]=rx_buffer[6]; ack[9]=FUNC_START_ACK;
                      ack[10]=0xCC; ack[11]=0xDD; ack[12]=0xEE;
                      port_DisableEXT_IRQ();
                      for(_r=0;_r<3;_r++){
                        dwt_forcetrxoff(); port_rf_switch_tx();
                        dwt_writetxdata(sizeof(ack),ack,0); dwt_writetxfctrl(sizeof(ack),0,0);
                        if(dwt_starttx(DWT_START_TX_IMMEDIATE)==DWT_SUCCESS){
                          { uint32_t tx_dl = HAL_GetTick() + 10u; while(!(dwt_read32bitreg(SYS_STATUS_ID)&SYS_STATUS_TXFRS_BIT_MASK)){ if(HAL_GetTick() > tx_dl) break; } }
                          dwt_write32bitreg(SYS_STATUS_ID,SYS_STATUS_TXFRS_BIT_MASK);}
                        port_rf_switch_rx(); if(_r<2) Sleep(5);
                      }
                      dwt_rxenable(DWT_START_RX_IMMEDIATE);
                      EXTI->PR1 = (1u << 12);
                      port_EnableEXT_IRQ(); }
                    /* Transition to TDOA: the immediate ACK TX overwrote TX_TIME.
                     * Re-sync the DLY_TS reference so the first TDOA beacon has a
                     * clean timing base (see FIND handler note). */
                    is_first_beacon = 1;
                }
                else if (rx_buffer[9] == 0xFA) {
                    DBG_PRINT("[TDOA] STOP → config\r\n");
                    g_config_mode = 1;
                    /* ACK — 13B to avoid DW3210 last-2B RX bug */
                    { uint8_t ack[13]; int _r;
                      ack[0]=0x41; ack[1]=0x88; ack[2]=0;
                      ack[3]=0xCA; ack[4]=0xDE; ack[5]=rx_buffer[7]; ack[6]=rx_buffer[8];
                      ack[7]=rx_buffer[5]; ack[8]=rx_buffer[6]; ack[9]=FUNC_STOP_ACK;
                      ack[10]=0xCC; ack[11]=0xDD; ack[12]=0xEE;
                      port_DisableEXT_IRQ();
                      for(_r=0;_r<3;_r++){
                        dwt_forcetrxoff(); port_rf_switch_tx();
                        dwt_writetxdata(sizeof(ack),ack,0); dwt_writetxfctrl(sizeof(ack),0,0);
                        if(dwt_starttx(DWT_START_TX_IMMEDIATE)==DWT_SUCCESS){
                          { uint32_t tx_dl = HAL_GetTick() + 10u; while(!(dwt_read32bitreg(SYS_STATUS_ID)&SYS_STATUS_TXFRS_BIT_MASK)){ if(HAL_GetTick() > tx_dl) break; } }
                          dwt_write32bitreg(SYS_STATUS_ID,SYS_STATUS_TXFRS_BIT_MASK);}
                        port_rf_switch_rx(); if(_r<2) Sleep(5);
                      }
                      dwt_rxenable(DWT_START_RX_IMMEDIATE);
                      EXTI->PR1 = (1u << 12);
                      port_EnableEXT_IRQ(); }
                }
                else if (rx_buffer[9] == 0xF9 && frame_len >= GW_HDR_LEN + 2) {
                    uint16_t mask = (uint16_t)(rx_buffer[GW_HDR_LEN] | (rx_buffer[GW_HDR_LEN+1] << 8));
                    int i, cnt=0, dst=0; dl_anchor_entry_t tmp[DL_MAX_ANCHORS];
                    for(i=0;i<DL_MAX_ANCHORS;i++)tmp[i]=g_dl_beacon_table[i];
                    for(i=0;i<DL_MAX_ANCHORS;i++){if(mask&(1u<<i)){
                      if(dst<DL_MAX_ANCHORS){g_dl_beacon_table[dst]=tmp[i];g_dl_beacon_table[dst].id=(uint8_t)dst;dst++;}
                      cnt++;}}
                    g_dl_num_anchors=(uint8_t)cnt;
                    /* ACK — 14B to avoid DW3210 last-2B RX bug */
                    { uint8_t ack[14]; int _r;
                      ack[0]=0x41; ack[1]=0x88; ack[2]=0;
                      ack[3]=0xCA; ack[4]=0xDE; ack[5]=rx_buffer[7]; ack[6]=rx_buffer[8];
                      ack[7]=rx_buffer[5]; ack[8]=rx_buffer[6]; ack[9]=FUNC_NETWORK_ACK;
                      ack[10]=rx_buffer[GW_HDR_LEN]; ack[11]=rx_buffer[GW_HDR_LEN+1];
                      ack[12]=0xCC; ack[13]=0xDD;
                      port_DisableEXT_IRQ();
                      for(_r=0;_r<3;_r++){
                        dwt_forcetrxoff(); port_rf_switch_tx();
                        dwt_writetxdata(sizeof(ack),ack,0); dwt_writetxfctrl(sizeof(ack),0,0);
                        if(dwt_starttx(DWT_START_TX_IMMEDIATE)==DWT_SUCCESS){
                          { uint32_t tx_dl = HAL_GetTick() + 10u; while(!(dwt_read32bitreg(SYS_STATUS_ID)&SYS_STATUS_TXFRS_BIT_MASK)){ if(HAL_GetTick() > tx_dl) break; } }
                          dwt_write32bitreg(SYS_STATUS_ID,SYS_STATUS_TXFRS_BIT_MASK);}
                        port_rf_switch_rx(); if(_r<2) Sleep(5);
                      }
                      dwt_rxenable(DWT_START_RX_IMMEDIATE);
                      EXTI->PR1 = (1u << 12);
                      port_EnableEXT_IRQ(); }
                }
#ifndef GATEWAY_ROLE  /* Master/Slave only — Gateway forwards, doesn't receive */
                /* OTA_PREPARE (0xE5) — stay on Ch9, stop ranging during OTA */
                else if (rx_buffer[9] == FUNC_OTA_PREPARE)
                {
                    uint8_t ack[13]; int _r;
                    g_config_mode = 1;  /* Stop_Ranging: pause TDMA, free air for OTA */
                    ack[0]=0x41; ack[1]=0x88; ack[2]=0;
                    ack[3]=0xCA; ack[4]=0xDE; ack[5]=rx_buffer[7]; ack[6]=rx_buffer[8];
                    ack[7]=ANCHOR_ID_VAL&0xFF; ack[8]=(ANCHOR_ID_VAL>>8)&0xFF;
                    ack[9]=FUNC_OTA_PREP_ACK; ack[10]=0xCC; ack[11]=0xDD; ack[12]=0xEE;
                    port_DisableEXT_IRQ();
                    for(_r=0;_r<3;_r++){
                        dwt_forcetrxoff(); port_rf_switch_tx();
                        dwt_writetxdata(sizeof(ack),ack,0); dwt_writetxfctrl(sizeof(ack),0,0);
                        if(dwt_starttx(DWT_START_TX_IMMEDIATE)==DWT_SUCCESS){
                          { uint32_t tx_dl = HAL_GetTick() + 10u; while(!(dwt_read32bitreg(SYS_STATUS_ID)&SYS_STATUS_TXFRS_BIT_MASK)){ if(HAL_GetTick() > tx_dl) break; } }
                          dwt_write32bitreg(SYS_STATUS_ID,SYS_STATUS_TXFRS_BIT_MASK);}
                        port_rf_switch_rx(); if(_r<2) Sleep(5);
                    }
                    dwt_rxenable(DWT_START_RX_IMMEDIATE);
                    EXTI->PR1 = (1u << 12);
                    port_EnableEXT_IRQ();
                }
#endif
#if ENABLE_GATEWAY
                else if (rx_buffer[9] == FUNC_DOWNLINK_BLINK
                         && frame_len >= DL_BLINK_LEN)
                {
                    uint16_t rank16 = (uint16_t)(rx_buffer[7])
                                    | ((uint16_t)(rx_buffer[8]) << 8);
                    char uid[8];
                    uid[0] = hex[(rank16 >> 12) & 0xF]; uid[1] = hex[(rank16 >>  8) & 0xF];
                    uid[2] = hex[(rank16 >>  4) & 0xF]; uid[3] = hex[ rank16        & 0xF];
                    uid[4] = '\0';
                    gateway_device_seen(uid, "ANCHOR", (int)rank16);
                    /* Extract firmware version from SEQ byte (DL blink embeds FW_VERSION_BYTE) */
                    gateway_device_version(uid, fw_byte_to_u16(rx_buffer[2]));
                }
                else if ((rx_buffer[9] == FUNC_DL_POS_REPORT || rx_buffer[9] == 0xF3
                       || rx_buffer[9] == 0xF5 || rx_buffer[9] == 0xF7)
                         && frame_len >= GW_HDR_LEN) {
                    gateway_handle_uwb_rx(rx_buffer, frame_len);
                }
#endif
                /* Fallback: re-enable RX unconditionally after every frame.
                 * During OTA, the conditional dwt_rxenable() above skips RX so
                 * OTA_DATA handler's ota_send_ack() avoids the trap window.
                 * But non-OTA frames (tag blinks, etc.) arrive during OTA and
                 * their handlers don't always re-enable RX — this fallback
                 * guarantees DW3000 is never left deaf. */
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
            }
        }

        /* OTA timeout poll — L1 rollback: if no data for 5s, resume ranging */
#ifndef GATEWAY_ROLE
        if (gw_ota_poll()) {
            g_config_mode = 0;  /* OTA timed out - resume TDOA */
        }
#endif

        /*=== Phase 2: TX Downlink Beacon ===
         * Beacon every cycle (~500ms) regardless of config/TDOA mode.
         * Fast beaconing during config mode is critical: devices in LISTEN
         * only have 2–5s to hear a beacon; at 2s intervals (old behavior)
         * they might only get 1–2 chances, making split-brain likely. */
#ifndef GATEWAY_ROLE
        if (!gw_ota_is_active())
#else
        if (1)
#endif
        {
#ifdef FUSION_DOWNLINK_MODE
            uint16_t bcn_len = (uint16_t)DL_BEACON_LEN((int)g_dl_num_anchors);
            uint8_t  dl_bcn[DL_BEACON_LEN(DL_MAX_ANCHORS)];
            uint64_t tx_ts_dtu;
            int i;

            /* Predict TX timestamp: last actual TX + 500ms period.
             * First beacon uses Immediate TX (no previous reference). */
            if (is_first_beacon) {
                tx_ts_dtu = 0;  /* placeholder */
            } else {
                tx_ts_dtu = prev_tx_ts_dtu + ((uint64_t)DL_CYCLE_PERIOD_UUS * UUS_TO_DWT_TIME);
            }

            dl_bcn[0] = 0x41; dl_bcn[1] = 0x88;
            dl_bcn[2] = FW_VERSION_BYTE;  /* SEQ byte carries firmware version */
            dl_bcn[3] = 0xCA; dl_bcn[4] = 0xDE;
            dl_bcn[5] = 0xFF; dl_bcn[6] = 0xFF;
            dl_bcn[7] = ANCHOR_ID_VAL & 0xFF;
            dl_bcn[8] = (ANCHOR_ID_VAL >> 8) & 0xFF;
            dl_bcn[9] = FUNC_DOWNLINK_BEACON;
            /* Byte 15: bit7=TDOA_ACTIVE, bit6-0=num_anchors */
            dl_bcn[DL_BCN_NUM_IDX]   = g_dl_num_anchors
                                     | (g_config_mode ? 0 : DL_BCN_TDOA_ACTIVE);
            dl_bcn[DL_BCN_CYCLE_IDX] = (uint8_t)beacon_cycle;
            pack_40bit_ts(&dl_bcn[DL_BCN_TX_TS_IDX], tx_ts_dtu);
            for (i = 0; i < (int)g_dl_num_anchors && i < DL_MAX_ANCHORS; i++) {
                pack_anchor_entry(
                    &dl_bcn[DL_BCN_TABLE_IDX + i * DL_ANCHOR_ENTRY_SIZE],
                    &g_dl_beacon_table[i]);
            }
            /* Pad last 2 bytes — absorbed by DW3210 RX corruption */
            dl_bcn[bcn_len - 2] = 0xCC;
            dl_bcn[bcn_len - 1] = 0xDD;

            dwt_forcetrxoff();
            port_rf_switch_tx();
            dwt_writetxdata(bcn_len, dl_bcn, 0);
            dwt_writetxfctrl(bcn_len, 0, 0);

            if (is_first_beacon || g_config_mode) {
                /* Config mode or first beacon: no need for precise DLY_TS timing.
                 * Config beacons are spaced ~2s apart (every 4th cycle). */
                dwt_starttx(DWT_START_TX_IMMEDIATE);
            } else {
                /* TDOA mode: DLY_TS for precise 500ms beacon interval.
                 * Constant offset = 500ms in DLY_TS units (DTU >> 8). */
                uint32_t period_hi = (uint32_t)(((uint64_t)DL_CYCLE_PERIOD_UUS * UUS_TO_DWT_TIME) >> 8);
                dwt_setdelayedtrxtime(period_hi);
                if (dwt_starttx(DWT_START_TX_DLY_TS) != DWT_SUCCESS) {
                    dwt_starttx(DWT_START_TX_IMMEDIATE);
                }
            }

            { uint32_t tx_dl = HAL_GetTick() + 10u; while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) { if (HAL_GetTick() > tx_dl) break; } }
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

            /* Read actual TX time for next DLY_TS prediction */
            {
                uint64_t actual_tx = get_tx_timestamp_u64();
                prev_tx_ts_dtu = actual_tx;
                is_first_beacon = 0;
            }

            port_rf_switch_rx();
#endif

            beacon_cycle++;
            if (beacon_cycle >= 10 && !g_config_mode) {
                beacon_cycle = 0;
                dwt_forcetrxoff();
                port_rf_switch_tx();
                tx_beacon[2] = 0;
                tx_beacon[AUTO_BCN_GROUP_IDX] = 0;
                tx_beacon[AUTO_BCN_COUNT_IDX] = g_node_count;
                pack_uid(&tx_beacon[AUTO_UID_IDX], g_my_uid32);
                dwt_writetxdata(BEACON_LEN, tx_beacon, 0);
                dwt_writetxfctrl(BEACON_LEN, 0, 0);
                if (dwt_starttx(DWT_START_TX_IMMEDIATE) == DWT_SUCCESS) {
                    { uint32_t tx_dl = HAL_GetTick() + 10u; while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) { if (HAL_GetTick() > tx_dl) break; } }
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
                    /* Sync DLY_TS reference: auto-discovery TX overwrites TX_TIME_LO */
                    prev_tx_ts_dtu = get_tx_timestamp_u64();
                }
                port_rf_switch_rx();
            }

#if ENABLE_GATEWAY
            gateway_poll_uart();
#endif
        }
    }
}

/*============================================================================
 * SLAVE ANCHOR — rank 1+:  receives beacon + sends TDMA blink
 *
 *   Each cycle:
 *     1. RX downlink beacon (0xF0) from master → heartbeat
 *     2. TX downlink blink (0xF1) in TDMA slot (based on beacon RX time)
 *============================================================================*/
static void master_network_formation(void);  /* forward decl — defined below */

static void slave_anchor_loop(uint8_t my_rank)
{
    int master_miss_count = 0;
    static int consecutive_lost = 0;
    /* Config-mode heartbeat blink counter: send blink every 8 beacon cycles
     * (~4s) so the Gateway can see the Slave even before TDOA starts.
     * Wide slot (50ms/rank) prevents collisions between multiple slaves. */
    static int heartbeat_ctr = 8;  /* start at 8 → send first blink immediately */
    /* Master-loss detection: count of consecutive 45ms RX windows with no
     * beacon (0xE7 or 0xF0).  40 windows ≈ 1.8s of silence.
     * Must exceed the ~1s gap between 0xE7 formation beacons (~22 windows)
     * so a slave that just JOINed during master_network_formation doesn't
     * falsely trip re_discover before the Master reaches the 0xF0 loop. */
    #define MASTER_LOST_THRESHOLD  40

slave_entry:
    test_run_info((unsigned char *)"SRDY\r\n");

    port_rf_switch_init();

#ifndef GATEWAY_ROLE
    /* P2: Confirm successful boot — clear bootloader's STATE=BOOTING flag.
     * Gateway doesn't have a bootloader → skip. */
    gw_ota_confirm_boot();
#endif

    while (1)
    {
        uint32_t status_reg, frame_len;
        int got_beacon = 0;

        /*=== Phase 1: RX for ~45ms (1ms fast polling for DW3000 double buffer) ===*/
        {
            int _p;
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            for (_p = 0; _p < 45; _p++)  /* 45×1ms = 45ms RX window */
            {
                Sleep(1);
                find_blink_poll();  /* LED fast blink during FIND */
                status_reg = dwt_read32bitreg(SYS_STATUS_ID);
                if (status_reg & SYS_STATUS_ALL_RX_ERR) {
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                    dwt_rxenable(DWT_START_RX_IMMEDIATE);
                    continue;
                }
                if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK)) continue;
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
                frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
                if (frame_len > sizeof(rx_buffer)) {
                    dwt_rxenable(DWT_START_RX_IMMEDIATE);
                    continue;
                }
                dwt_readrxdata(rx_buffer, frame_len, 0);

#ifndef GATEWAY_ROLE
                /*=== OTA fast-path: handle BEFORE re-entering RX ===
                 * Same pattern as Master path — keep DW3000 in IDLE
                 * during flash write + ACK, eliminating trap window. */
                if (rx_buffer[9] == FUNC_OTA_DATA && frame_len >= GW_HDR_LEN + 4)
                {
                    gw_ota_handle_data(rx_buffer, frame_len, (uint16_t)my_rank);
                    continue;
                }
                if (rx_buffer[9] == FUNC_OTA_END)
                {
                    gw_ota_finish((uint16_t)my_rank);
                    dwt_rxenable(DWT_START_RX_IMMEDIATE);
                    continue;
                }
                if (rx_buffer[9] == FUNC_OTA_ABORT)
                {
                    g_config_mode = 0;
                    gw_ota_abort();
                    dwt_rxenable(DWT_START_RX_IMMEDIATE);
                    continue;
                }
                if (rx_buffer[9] == FUNC_PREP_CMD
                    && frame_len <= GW_HDR_LEN + 12  /* PREP_CMD=16B(incl FCS), DL beacon=47+B(incl FCS) — distinguish! */
                    && ((uint16_t)(rx_buffer[5] | (rx_buffer[6] << 8)) == (uint16_t)my_rank
                        || (uint16_t)(rx_buffer[5] | (rx_buffer[6] << 8)) == 0xFFFFu))
                {
                    g_config_mode = 1;
                    /* ACK first — sent BEFORE flash erase so Gateway gets
                     * instant response. Erase takes ~660ms which would
                     * exceed Gateway's 10ms-per-burst polling window. */
                    { uint8_t ack[13]; int _r;
                      ack[0]=0x41; ack[1]=0x88; ack[2]=0;
                      ack[3]=0xCA; ack[4]=0xDE; ack[5]=rx_buffer[7]; ack[6]=rx_buffer[8];
                      ack[7]=my_rank&0xFF; ack[8]=(my_rank>>8)&0xFF;
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
                      port_EnableEXT_IRQ(); }
                    /* Erase staging AFTER ACK */
                    if (frame_len >= GW_HDR_LEN + 4) {
                        uint32_t ota_sz = (uint32_t)(rx_buffer[GW_HDR_LEN])
                                        | ((uint32_t)(rx_buffer[GW_HDR_LEN + 1]) << 8)
                                        | ((uint32_t)(rx_buffer[GW_HDR_LEN + 2]) << 16)
                                        | ((uint32_t)(rx_buffer[GW_HDR_LEN + 3]) << 24);
                        gw_ota_handle_prepare(ota_sz);
                    }
                    continue;
                }
#endif
                /* Re-enable RX for non-OTA frames (OTA handlers already did it) */
                dwt_rxenable(DWT_START_RX_IMMEDIATE);

                /* Auto-discovery beacon (0xE7) → master heartbeat */
                if (match_header(rx_buffer, match_beacon, HDR_LEN, 1, 0, 1))
                {
                    got_beacon = 1;         /* 0xE7 also counts as master presence */
                    master_miss_count = 0;
                    consecutive_lost = 0;
                }
#ifdef FUSION_DOWNLINK_MODE
                /* Downlink beacon (0xF0) → heartbeat + TDMA blink via DLY_RS.
                 * During OTA: heartbeat only — skip blink TX to avoid killing RX. */
                /* Downlink beacon (0xF0) — broadcast from Master.  DST=0xFFFF.
                 * zero_dst=0 (no longer ignored): lets unicast PREP_CMD (0xF0
                 * from Gateway to a specific rank) fall through to the
                 * #ifndef GATEWAY_ROLE handler below. */
                else if (match_header(rx_buffer, match_dl_beacon, HDR_LEN, 1, 0, 1))
                {
                    uint64_t bcn_rx_ts = get_rx_timestamp_u64();
                    uint32_t slot_offset_hi;
                    uint8_t dl_blink[DL_BLINK_LEN];
                    int dly_ok;

                    master_miss_count = 0;
                    consecutive_lost = 0;
                    got_beacon = 1;
                    /* P6: Sync local g_config_mode with Master's TDOA_ACTIVE flag.
                     * g_config_mode=0 enables master-loss detection and other
                     * TDOA-mode guards.  The Master beacon is the single source
                     * of truth for TDOA state. */
                    g_config_mode = (rx_buffer[DL_BCN_NUM_IDX] & DL_BCN_TDOA_ACTIVE) ? 0 : 1;

                    uint8_t tdoa_active = (rx_buffer[DL_BCN_NUM_IDX] & DL_BCN_TDOA_ACTIVE);
                    int send_blink = 0;
                    int is_heartbeat = 0;
#ifndef GATEWAY_ROLE
                    if (!gw_ota_is_active()) {
#else
                    if (1) {
#endif
                        if (tdoa_active) {
                            /* TDOA mode: send precise TDMA blink every cycle */
                            send_blink = 1;
                            heartbeat_ctr = 0;
                        } else if (++heartbeat_ctr >= 40) {
                            /* Config mode: heartbeat blink every 4 cycles (~2s).
                             * Wide slot (50ms/rank) prevents collisions.
                             * Gateway uses this to detect slave online. */
                            heartbeat_ctr = 0;
                            send_blink = 1;
                            is_heartbeat = 1;
                        }
                    }
                    if (send_blink) {
                    /* Build blink frame */
                    dl_blink[0] = 0x41; dl_blink[1] = 0x88;
                    dl_blink[2] = FW_VERSION_BYTE;  /* SEQ byte carries firmware version */
                    dl_blink[3] = 0xCA; dl_blink[4] = 0xDE;
                    dl_blink[5] = 0xFF; dl_blink[6] = 0xFF;
                    dl_blink[7] = my_rank & 0xFF;
                    dl_blink[8] = (my_rank >> 8) & 0xFF;
                    dl_blink[9] = FUNC_DOWNLINK_BLINK;
                    /* TX timestamp at bytes 10-14 (verified correct on DW3210) */
                    {
                        uint32_t slot_us = DL_SLOT_DURATION_UUS;
                        uint64_t blink_tx_ts = bcn_rx_ts
                            + (uint64_t)my_rank * slot_us * UUS_TO_DWT_TIME;
                        pack_40bit_ts(&dl_blink[DL_BLINK_TX_TS_IDX], blink_tx_ts);
                    }
                    /* Rank is encoded in SRC (bytes 7-8, verified reliable).
                     * Pad last 2 bytes to absorb DW3210 RX corruption. */
                    dl_blink[DL_BLINK_LEN - 2] = 0;
                    dl_blink[DL_BLINK_LEN - 1] = 0;

                    {
                        uint32_t slot_us = DL_SLOT_DURATION_UUS;
                        slot_offset_hi = (uint32_t)(
                            ((uint64_t)my_rank * slot_us * UUS_TO_DWT_TIME) >> 8);
                    }

                    /* Now stop RX, switch, and TX */
                    dwt_forcetrxoff();
                    port_rf_switch_tx();
                    dwt_writetxdata(DL_BLINK_LEN, dl_blink, 0);
                    dwt_writetxfctrl(DL_BLINK_LEN, 0, 0);
                    dwt_setdelayedtrxtime(slot_offset_hi);
                    dly_ok = (dwt_starttx(DWT_START_TX_DLY_RS) == DWT_SUCCESS);

                    if (dly_ok) {
                        /* Heartbeat blink uses a wide slot (50ms×rank) — its
                         * delayed TX fires far past the normal 10ms wait, and
                         * dwt_rxenable() below would cancel a still-pending
                         * DLY_RS, silently dropping the heartbeat and making
                         * slaves read as offline in config mode.  Wait long
                         * enough for the delayed TX to actually fire. */
                        { uint32_t tx_dl = HAL_GetTick() + (is_heartbeat ? (my_rank * 3u + 10u) : 10u); while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) { if (HAL_GetTick() > tx_dl) break; } }
                        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
                    } else {
                        /* DLY_RS failed (target passed) → fallback Immediate */
                        if (dwt_starttx(DWT_START_TX_IMMEDIATE) == DWT_SUCCESS) {
                            { uint32_t tx_dl = HAL_GetTick() + 10u; while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) { if (HAL_GetTick() > tx_dl) break; } }
                            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
                        }
                    }
                    port_rf_switch_rx();
                    }

                    /* CRITICAL: re-enable RX even if blink was skipped (OTA).
                     * Without this, DW3000 stays in IDLE for the rest of the 500ms window. */
                    dwt_rxenable(DWT_START_RX_IMMEDIATE);
                }
                /* Tag blink (0xE2) — uplink TDOA reference */
                else if (match_header(rx_buffer, match_tag_blink, HDR_LEN, 1, 0, 1))
                {
                    uint64_t tag_toa = get_rx_timestamp_u64();
                    (void)tag_toa;
                }
#endif
                /* FIND_REQ (0xF2) from Gateway — fast-blink LED1 for 5 seconds */
                else if (rx_buffer[9] == FUNC_FIND_REQ
                         && frame_len >= GW_HDR_LEN + 1
                         && (uint16_t)(rx_buffer[5] | (rx_buffer[6] << 8)) == my_rank)
                {
                    uint8_t on_off = rx_buffer[GW_HDR_LEN];
                    if (on_off)
                        g_find_blink_until = HAL_GetTick() + 5000;  /* 5s fast blink */
                    else {
                        g_find_blink_until = 0;
                        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
                    }
                    /* Direct DW3000 TX — 3× FIND_ACK burst to Gateway.
                     * Pad to 13B to avoid DW3210 last-2B RX corruption bug.
                     * Disable EXTI IRQ during TX: otherwise TX-completion IRQ
                     * fires dwt_forcetrxoff() which clears TXFRS → while() hangs
                     * → watchdog reset → device reboots. */
                    { uint8_t ack[13]; int _r;
                      ack[0]=0x41; ack[1]=0x88; ack[2]=0;
                      ack[3]=0xCA; ack[4]=0xDE; ack[5]=rx_buffer[7]; ack[6]=rx_buffer[8];
                      ack[7]=my_rank & 0xFF; ack[8]=(my_rank >> 8) & 0xFF;
                      ack[9]=FUNC_FIND_ACK; ack[10]=on_off;
                      ack[11]=0xCC; ack[12]=0xDD;  /* pad: avoid DW3210 last-2B bug */
                      port_DisableEXT_IRQ();
                      for(_r=0;_r<3;_r++){
                        dwt_forcetrxoff(); port_rf_switch_tx();
                        dwt_writetxdata(sizeof(ack),ack,0); dwt_writetxfctrl(sizeof(ack),0,0);
                        if(dwt_starttx(DWT_START_TX_IMMEDIATE)==DWT_SUCCESS){
                          { uint32_t tx_dl = HAL_GetTick() + 10u; while(!(dwt_read32bitreg(SYS_STATUS_ID)&SYS_STATUS_TXFRS_BIT_MASK)){ if(HAL_GetTick() > tx_dl) break; } }
                          dwt_write32bitreg(SYS_STATUS_ID,SYS_STATUS_TXFRS_BIT_MASK);}
                        port_rf_switch_rx(); if(_r<2) Sleep(5);
                      }
                      dwt_rxenable(DWT_START_RX_IMMEDIATE);
                      EXTI->PR1 = (1u << 12);
                      port_EnableEXT_IRQ(); }
                }
                /* COORD_WRITE (0xF4) from Gateway — save anchor coordinates */
                else if (rx_buffer[9] == FUNC_COORD_WRITE
                         && frame_len >= GW_HDR_LEN + 6
                         && (uint16_t)(rx_buffer[5] | (rx_buffer[6] << 8)) == my_rank)
                {
                    int16_t cx, cy, cz;
                    cx = (int16_t)(rx_buffer[GW_HDR_LEN]     | (rx_buffer[GW_HDR_LEN+1] << 8));
                    cy = (int16_t)(rx_buffer[GW_HDR_LEN+2]   | (rx_buffer[GW_HDR_LEN+3] << 8));
                    cz = (int16_t)(rx_buffer[GW_HDR_LEN+4]   | (rx_buffer[GW_HDR_LEN+5] << 8));
                    gateway_flash_save_coord(cx, cy, cz);
                    {
                        char _dbg[64];
                        snprintf(_dbg, sizeof(_dbg),
                                 "[COORD] my rank=%d → (%d.%02d,%d.%02d,%d.%02d)\r\n",
                                 (int)my_rank, (int)(cx/100), (int)((cx<0?-cx:cx)%100),
                                 (int)(cy/100), (int)((cy<0?-cy:cy)%100),
                                 (int)(cz/100), (int)((cz<0?-cz:cz)%100));
                        DBG_PRINT(_dbg);
                    }
                    /* ACK back to Gateway */
                    { uint8_t ack[13]; int _r;
                      ack[0]=0x41; ack[1]=0x88; ack[2]=0;
                      ack[3]=0xCA; ack[4]=0xDE; ack[5]=rx_buffer[7]; ack[6]=rx_buffer[8];
                      ack[7]=my_rank & 0xFF; ack[8]=(my_rank >> 8) & 0xFF;
                      ack[9]=FUNC_COORD_ACK; ack[10]=0xCC; ack[11]=0xDD; ack[12]=0xEE;
                      port_DisableEXT_IRQ();
                      for(_r=0;_r<3;_r++){
                        dwt_forcetrxoff(); port_rf_switch_tx();
                        dwt_writetxdata(sizeof(ack),ack,0); dwt_writetxfctrl(sizeof(ack),0,0);
                        if(dwt_starttx(DWT_START_TX_IMMEDIATE)==DWT_SUCCESS){
                          { uint32_t tx_dl = HAL_GetTick() + 10u; while(!(dwt_read32bitreg(SYS_STATUS_ID)&SYS_STATUS_TXFRS_BIT_MASK)){ if(HAL_GetTick() > tx_dl) break; } }
                          dwt_write32bitreg(SYS_STATUS_ID,SYS_STATUS_TXFRS_BIT_MASK);}
                        port_rf_switch_rx(); if(_r<2) Sleep(5);
                      }
                      dwt_rxenable(DWT_START_RX_IMMEDIATE);
                      EXTI->PR1 = (1u << 12);
                      port_EnableEXT_IRQ(); }
                }
                /* Re-enable RX after handling frame */
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
            }
        }

        /* OTA timeout poll — L1 rollback: if no data for 5s, resume ranging */
#ifndef GATEWAY_ROLE
        if (gw_ota_poll()) {
            g_config_mode = 0;  /* OTA timed out - resume TDOA */
        }
#endif

        /*=== Master heartbeat detection ===
         * Detect master loss in BOTH config and TDOA modes — the network
         * idles in config mode, so master power-off there must also trigger
         * failover.  Only skip during active OTA, when the Master
         * intentionally stops beaconing. */
#ifndef GATEWAY_ROLE
        if (!got_beacon && !gw_ota_is_active())
#else
        if (!got_beacon)
#endif
        {
            master_miss_count++;
            if (master_miss_count >= MASTER_LOST_THRESHOLD)
            {
                consecutive_lost++;

                if (consecutive_lost >= 3)
                {
                    test_run_info((unsigned char *)"FATAL: MULTI LOST");
                    NVIC_SystemReset();
                    while (1) { }
                }
                goto re_discover;
            }
        }
    }

re_discover:
    /* Master lost — re-run auto-discovery.  Must force IDLE before hw-reset
     * because DW3000 is still in RX (from heartbeat check).  RST-ing an
     * actively-receiving DW3000 can leave its SPI state-machine stuck. */
    {
        auto_disc_t disc;
        int total_attempts;

        for (total_attempts = 0; total_attempts < 6; total_attempts++)
        {
            int idle_timeout;

            /* re-init */

            /* Force DW3000 out of RX before touching RST pin */
            dwt_forcetrxoff();

            port_set_dw_ic_spi_fastrate();

            /* reset DW3000 */
            reset_DWIC();
            Sleep(100);  /* warm-reset needs longer recovery than cold-boot */

            /* Wait for IDLE with timeout (~1 s at 1 ms polling) */
            idle_timeout = 1000;
            while (!dwt_checkidlerc() && --idle_timeout > 0)
            {
                Sleep(1);
            }
            if (idle_timeout <= 0)
            {
                /* Warm-reset failed — DW3000 RST pin may not recover on all
                 * hardware revisions.  Fall back to full system reset which
                 * gives a clean cold-boot (power-on-reset) sequence. */

                NVIC_SystemReset();
                while (1) { }  /* unreachable */
            }


            if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
            {
                test_run_info((unsigned char *)"INITF");
                continue;
            }


            if (dwt_configure(&config))
            {

                continue;
            }


            dwt_configuretxrf(TXCONFIG);
            dwt_setrxantennadelay(RX_ANT_DLY);
            dwt_settxantennadelay(TX_ANT_DLY);
            dwt_setfinegraintxseq(0);
            dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

            if (auto_discovery(&disc) == 0)
                break;

            /* re-discovery retry */
        }

        if (total_attempts >= 6)
        {

            goto re_discover;
        }

        if (disc.my_rank == 0)
        {
            /* Promoted to Master: reset runtime address and LED role.
             * g_my_addr still holds the old slave rank — it must become
             * 0x0000 so the downlink beacon SRC and the Gateway's master
             * detection are correct. */
            g_my_addr = 0x0000u;
            g_led_heartbeat = 1;
            master_network_formation();
            main_anchor_loop();  /* never returns */
        }
        else
        {

            my_rank = disc.my_rank;
            master_miss_count = 0;
            goto slave_entry;
        }
    }
}

/*============================================================================
 * master_network_formation — post-discovery: beacon + listen for JOIN_REQ
 * Runs before main_anchor_loop to allow slaves to join.
 * Assigns dynamic ranks via g_node_count — each slave gets a unique rank.
 *============================================================================*/
static void master_network_formation(void)
{
    int round;
    uint32_t status_reg, frame_len;

    /* Reset node count — Master always starts formation with just itself.
     * Needed for re-discovery path where g_node_count may be stale. */
    g_node_count = 1;
    g_joined_count = 0;

    for (round = 0; round < 5; round++)
    {
        /* TX Beacon */
        dwt_forcetrxoff();
        port_rf_switch_tx();
        {
            uint8_t bcn[] = {
                0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0x01, 0x00, 0xE7,
                0,                        /* GROUP */
                g_node_count,             /* COUNT = current node count */
                0, 0, 0, 0,              /* MASTER_UID[4] */
                0, 0                      /* FCS[2] */
            };
            pack_uid(&bcn[AUTO_UID_IDX], g_my_uid32);
            dwt_writetxdata(sizeof(bcn), bcn, 0);
            dwt_writetxfctrl(sizeof(bcn), 0, 0);
            if (dwt_starttx(DWT_START_TX_IMMEDIATE) == DWT_SUCCESS)
            {
                { uint32_t tx_dl = HAL_GetTick() + 10u; while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) { if (HAL_GetTick() > tx_dl) break; } }
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
            }
        }
        /* Switch to RX BEFORE UART to minimize RX dead window */
        port_rf_switch_rx();

        /* Listen for JOIN_REQ — 20 polls * 50ms = 1 sec per round */
        {
            int attempt;
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            /* net-formation beacon sent */
            for (attempt = 0; attempt < 20; attempt++)
            {
                Sleep(50);
                status_reg = dwt_read32bitreg(SYS_STATUS_ID);
                if (status_reg & SYS_STATUS_ALL_RX_ERR)
                {
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                    dwt_rxenable(DWT_START_RX_IMMEDIATE);
                    continue;
                }
                if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK))
                    continue;

                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
                frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
                if (frame_len > sizeof(rx_buffer)) { dwt_rxenable(DWT_START_RX_IMMEDIATE); continue; }

                dwt_readrxdata(rx_buffer, frame_len, 0);
                if (rx_buffer[9] == FUNC_JOIN_REQUEST && frame_len >= JOIN_REQ_LEN - 2)
                {
                    uint32_t req_uid = unpack_uid(&rx_buffer[AUTO_UID_IDX]);
                    uint8_t req_src_lo = rx_buffer[7];
                    uint8_t req_src_hi = rx_buffer[8];
                    int existing = joined_rank_of(req_uid);
                    uint8_t new_rank;

                    /* Re-join: keep original rank.  New join: assign a fresh
                     * rank (max 15 slaves).  Skip if table full. */
                    if (existing >= 0) {
                        new_rank = (uint8_t)existing;
                    } else if (g_node_count < 16) {
                        new_rank = g_node_count;
                        g_joined_uids[g_joined_count++] = req_uid;
                        g_node_count++;
                    } else {
                        dwt_rxenable(DWT_START_RX_IMMEDIATE);
                        continue;
                    }

                    /* TX JOIN_ACK */
                    dwt_forcetrxoff();
                    port_rf_switch_tx();
                    {
                        uint8_t ack[] = {
                            0x41, 0x88, 0, 0xCA, 0xDE,
                            req_src_lo, req_src_hi,
                            0x01, 0x00,
                            0xEA,
                            0, 0, 0, 0,
                            new_rank,  /* dynamic rank: 1, 2, 3... */
                            0, 0
                        };
                        pack_uid(&ack[10], req_uid);
                        dwt_writetxdata(sizeof(ack), ack, 0);
                        dwt_writetxfctrl(sizeof(ack), 0, 0);
                        if (dwt_starttx(DWT_START_TX_IMMEDIATE) == DWT_SUCCESS)
                        {
                            { uint32_t tx_dl = HAL_GetTick() + 10u; while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) { if (HAL_GetTick() > tx_dl) break; } }
                            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
                        }
                    }
                    /* Switch to RX BEFORE UART */
                    port_rf_switch_rx();
                }
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
            }
        }
    }


}

/*============================================================================
 * GATEWAY SNIFFER — pure listener, no TX (except AT-command responses)
 *============================================================================*/
#if defined(ENABLE_GATEWAY) && defined(GATEWAY_ROLE)
/* Format uint16_t as 4-char hex string (e.g. 1 → "0001") */
char *fmt_uid4(char *buf, uint16_t val)  /* non-static — used by gateway_cmd.c */
{
    buf[0] = hex[(val >> 12) & 0xF];
    buf[1] = hex[(val >>  8) & 0xF];
    buf[2] = hex[(val >>  4) & 0xF];
    buf[3] = hex[ val        & 0xF];
    buf[4] = '\0';
    return buf;
}

static void gateway_sniffer_loop(void)
{
    uint32_t status_reg, frame_len;
    uint8_t  rx_buf[DL_BEACON_LEN(DL_MAX_ANCHORS) + 4];

    test_run_info((unsigned char *)"GATEWAY SNIFFER");

    /* Register Gateway itself ONCE at startup.  Must use host-visible
     * +ONLINE format (not gateway_device_seen which has debounce ≥2).
     * The host connects via UART and needs this to show Gateway online.
     * Previously removed as "phantom fix" — that was a mistake; the
     * Gateway MUST announce itself for the host to display it. */
    test_run_info((unsigned char *)"+ONLINE,GW01,GATEWAY,0");

    port_rf_switch_init();
    port_rf_switch_rx();
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    while (1)
    {
        int p;
        for (p = 0; p < 500; p++)  /* 500ms window × 1ms poll */
        {
            Sleep(1);
            status_reg = dwt_read32bitreg(SYS_STATUS_ID);
            if (status_reg & SYS_STATUS_ALL_RX_ERR) {
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
                continue;
            }
            if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK)) continue;

            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
            if (frame_len > sizeof(rx_buf) || frame_len < HDR_LEN) {
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
                continue;
            }
            dwt_readrxdata(rx_buf, frame_len, 0);

            /* Match known frame types.
             * FUNC_DOWNLINK_BEACON=0xEE, FUNC_DOWNLINK_BLINK=0xEF —
             * no collision with gateway protocol (0xF0-0xFE). */
            if (rx_buf[9] == FUNC_DOWNLINK_BEACON && frame_len >= DL_BEACON_LEN(2))
            {
                /* Register Master ONLY — slaves appear via their own blink frames.
                 * Beacon slave-loop was creating phantom devices (n=65 from stale
                 * auto-discovery state with only 1 physical anchor). */
                uint16_t master_addr = (uint16_t)(rx_buf[7])
                                     | ((uint16_t)(rx_buf[8]) << 8);
                char uid[8];
                fmt_uid4(uid, master_addr);
                gateway_device_seen(uid, "ANCHOR", 0);
                /* Extract firmware version from SEQ byte (DL beacon embeds FW_VERSION_BYTE) */
                gateway_device_version(uid, fw_byte_to_u16(rx_buf[2]));
            }
            else if (rx_buf[9] == FUNC_DOWNLINK_BLINK
                   && frame_len >= DL_BLINK_LEN)
            {
                /* Register individual slave by its actual source address */
                uint16_t rank16 = (uint16_t)(rx_buf[7])
                                | ((uint16_t)(rx_buf[8]) << 8);
                char uid[8];
                fmt_uid4(uid, rank16);
                gateway_device_seen(uid, "ANCHOR", (int)rank16);
                /* Extract firmware version from SEQ byte (DL blink embeds FW_VERSION_BYTE) */
                gateway_device_version(uid, fw_byte_to_u16(rx_buf[2]));
            }
            /* Auto-discovery beacon (0xE7, ≥18B) — Master uses temporary address
             * during election.  Do NOT register here: the regular beacon (0xEE)
             * handler registers the Master with its final address. */
            else if ((rx_buf[9] == FUNC_DL_POS_REPORT || rx_buf[9] == 0xF3
                   || rx_buf[9] == 0xF5 || rx_buf[9] == 0xF7)
                   && frame_len >= GW_HDR_LEN) {
                gateway_handle_uwb_rx(rx_buf, frame_len);
            }

            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }

        /* Poll AT commands after each RX window */
        gateway_poll_uart();
    }
}
#endif /* defined(ENABLE_GATEWAY) && defined(GATEWAY_ROLE) */

/*============================================================================
 * fusion_anchor — init → auto-discovery → run
 *============================================================================*/
int fusion_anchor(void)
{
    auto_disc_t disc;

    if (anchor_init() != 0)
    {
        test_run_info((unsigned char *)"INITF");
        while (1) { }
    }

#if defined(ENABLE_GATEWAY) && defined(GATEWAY_ROLE)
    /* Pure gateway sniffer — no auto-discovery, no anchor loop */
    gateway_sniffer_loop();
    return 0;
#endif

    /* Staggered startup: each anchor delays based on its unique ID.
     * Prevents all anchors from entering LISTEN simultaneously →
     * timing out together → all entering ELECT → split-brain.
     *
     * Uses full 32-bit UID modulo a prime (~8000) for wide spread:
     *   Delay = (uid % 7993) + 200ms  →  range 200–8193ms.
     *
     * With random UIDs, average stagger difference between any two
     * devices is ~4s — well over the 2s LISTEN phase.  The first
     * device becomes Master and beacons before others finish LISTEN. */
    {
        /* P0-1: Random backoff 1–3 seconds before LISTEN.
         * Uses full 32-bit UID modulo 2001 → 0–2000ms, plus 1000ms base.
         * Range: 1000–3000ms.  Combined with variable LISTEN timeout,
         * the probability of two devices entering ELECT simultaneously
         * is < 1 / (2001 × 31) ≈ 0.0016%. */
        uint32_t uid = *(uint32_t *)0x1FFF7590UL;
        uint32_t stagger_ms = (uid % 2001u) + 1000u;
        Sleep(stagger_ms);
    }


    while (auto_discovery(&disc) != 0)
    {
        /* Random stagger 200–700ms before retry.
         * Without this, two devices that both fail discovery (e.g., JOIN
         * timeout) would retry simultaneously and collide again. */
        uint32_t retry_delay = (*(uint32_t *)0x1FFF7590UL % 500u) + 200u;

        Sleep(retry_delay);
    }

    /* Unified firmware: set runtime address from discovered rank.
     * Master (rank=0) → addr 0x0000, Slave → addr = rank. */
    g_my_addr  = (disc.my_rank == 0) ? 0x0000u : (uint16_t)disc.my_rank;
    g_my_uid32 = disc.my_uid32;              /* XOR'd UID — must match beacon/claim packing */

    if (disc.my_rank == 0)
    {

        /* Register self as gateway anchor device */
#if ENABLE_GATEWAY
        gateway_device_seen("0000", "GATEWAY", 0);
#endif
#ifdef FUSION_DOWNLINK_MODE
        /* Initialize beacon table: master = index 0 */
        g_dl_beacon_table[0].id = 0;
        g_dl_beacon_table[0].x_cm = g_dl_default_positions[0][0];
        g_dl_beacon_table[0].y_cm = g_dl_default_positions[0][1];
        g_dl_beacon_table[0].z_cm = g_dl_default_positions[0][2];
        /* Slaves populate during master_network_formation */
        {
            int di;
            for (di = 1; di < (int)DL_NUM_ANCHORS && di < DL_MAX_ANCHORS; di++)
            {
                g_dl_beacon_table[di].id = (uint8_t)di;
                g_dl_beacon_table[di].x_cm = g_dl_default_positions[di][0];
                g_dl_beacon_table[di].y_cm = g_dl_default_positions[di][1];
                g_dl_beacon_table[di].z_cm = g_dl_default_positions[di][2];
            }
            g_dl_num_anchors = DL_NUM_ANCHORS;
            if (g_dl_num_anchors > DL_MAX_ANCHORS)
                g_dl_num_anchors = DL_MAX_ANCHORS;
        }
#endif
        /*=== Beacon burst: immediately announce new Master to any devices
         * still in LISTEN.  5 rapid beacons at 100ms intervals cover 500ms,
         * ensuring at least one beacon is heard by listening devices.
         * Without this burst, the first beacon comes ~500ms later in the
         * main loop and the listening device may have already timed out. ===*/
        {
            int bb;

            for (bb = 0; bb < 5; bb++)
            {
                port_DisableEXT_IRQ();
                dwt_forcetrxoff();
                port_rf_switch_tx();
                tx_beacon[2] = 0;
                tx_beacon[AUTO_BCN_GROUP_IDX] = 0;
                tx_beacon[AUTO_BCN_COUNT_IDX] = 1;
                pack_uid(&tx_beacon[AUTO_UID_IDX], g_my_uid32);
                dwt_writetxdata(BEACON_LEN, tx_beacon, 0);
                dwt_writetxfctrl(BEACON_LEN, 0, 0);
                if (dwt_starttx(DWT_START_TX_IMMEDIATE) == DWT_SUCCESS) {
                    uint32_t tx_dl = HAL_GetTick() + 10u;
                    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {
                        if (HAL_GetTick() > tx_dl) break;
                    }
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
                }
                port_rf_switch_rx();
                EXTI->PR1 = (1u << 12);
                port_EnableEXT_IRQ();
                if (bb < 4) Sleep(100);
            }
        }
        master_network_formation();
#if AUTO_TDOA_START
        /* Auto-enter TDOA mode: beacon carries TDOA_ACTIVE=1 → slaves
         * begin TDMA blinks immediately.  No上位机 AT+START needed. */
        g_config_mode = 0;
#else
        g_config_mode = 1;  /* wait for AT+START from上位机 */
#endif
        g_led_heartbeat = 1;  /* V0.7: Master heartbeat blink */
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

        main_anchor_loop();
    }
    else
    {

        /* Slave: start in CONFIG mode (1) regardless of AUTO_TDOA_START.
         * Local g_config_mode is synced from the Master beacon's TDOA_ACTIVE
         * flag inside slave_anchor_loop (rx_buffer[DL_BCN_NUM_IDX] bit7) once
         * TDOA actually begins.
         *
         * Starting at 0 (TDOA) here is a BUG: it enables master-loss detection
         * immediately, but the Master is still in master_network_formation()
         * sending 0xE7 auto-discovery beacons ~1s apart (NOT 0xEE).  With
         * MASTER_LOST_THRESHOLD=10 × 45ms ≈ 450ms, the slave trips re_discover
         * before the next 0xE7 arrives, cycling JOIN→re_discover forever and
         * never blinking → never appearing online in the host. */
        g_config_mode = 1;
        g_led_heartbeat = 0; /* V0.7: LED OFF, only FIND blinks */
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
        slave_anchor_loop(disc.my_rank);
    }

    return 0;
}

#endif /* TEST_FUSION_ANCHOR */
