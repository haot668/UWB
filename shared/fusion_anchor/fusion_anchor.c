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
#if ENABLE_GATEWAY
#include "gateway_cmd.h"
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

#ifndef ANCHOR_IS_MAIN
#ifdef FUSION_ANCHOR_RANK
#if FUSION_ANCHOR_RANK == 0
#define ANCHOR_IS_MAIN 1
#else
#define ANCHOR_IS_MAIN 0
#endif
#else
#define ANCHOR_IS_MAIN 1
#endif
#endif

#if ENABLE_GATEWAY
#define APP_NAME "FUSION GATEWAY v2.2"
#else
#define APP_NAME "FUSION ANCHOR v2.2"
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
/* Gateway UWB response frames (addressed to master, SRC wildcarded) */
static uint8_t match_gw_find_ack[]  = {0x41, 0x88, 0, 0xCA, 0xDE, 0x01, 0x00, 0, 0, 0xF3};
static uint8_t match_gw_coord_ack[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0x01, 0x00, 0, 0, 0xF5};
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
static uint8_t match_dl_beacon[]  = {0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0, 0, 0xF0};
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
#define RX_BUF_LEN 32u
static uint8_t rx_buffer[RX_BUF_LEN];

/*============================================================================
 * Timestamps and tracking variables
 *============================================================================*/
static uint64_t blink_rx_ts;
static uint8_t  last_blink_seq = 0;

#define ANCHOR_ID_VAL  0x0000u  /* Master = 0, Slave1 = 1, Slave2 = 2 */

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

    /* Read STM32 unique ID (96-bit @ 0x1FFF7590, use lower 32-bit) */
    disc->my_uid32 = *(uint32_t *)0x1FFF7590UL;
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

    /*=== Phase: LISTEN — listen for existing master beacon (2 sec) ===*/
    {
        int attempt;
        test_run_info((unsigned char *)"AUTO LISTEN");
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        /* RX enabled */

        for (attempt = 0; attempt < 20; attempt++)
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
            if (!match_header(rx_buffer, match_beacon, HDR_LEN, 1, 0, 1))
            {
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
                continue;
            }

            /* Beacon received → existing master present, join it */
            disc->group_id = rx_buffer[AUTO_BCN_GROUP_IDX];
            disc->node_count = rx_buffer[AUTO_BCN_COUNT_IDX];
            disc->master_addr[0] = rx_buffer[7];  /* SRC low byte */
            disc->master_addr[1] = rx_buffer[8];  /* SRC high byte */
            disc->best_uid = unpack_uid(&rx_buffer[AUTO_UID_IDX]);

            disc->state = AUTO_STATE_JOIN;  /* skip election, join existing master */
            goto phase_join;
        }

        test_run_info((unsigned char *)"LISTEN TIMEOUT");
    }

    /*=== Phase: ELECT — no master found, run election ===*/
    disc->state = AUTO_STATE_ELECT;
    {
        test_run_info((unsigned char *)"AUTO ELECT");

        /* Short random delay based on UID to desynchronize */
        Sleep((disc->my_uid32 & 0xFFu) % 50u);

        /* 3-round claim: TX→listen→TX→listen→TX→listen ensures mutual discovery */
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
                    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) { }
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
                }
                /* Switch to RX BEFORE UART to minimize RX dead window */
                port_rf_switch_rx();

                /* Listen for other claims — 5 polls * 100ms = 500ms per round */
                {
                    int claim_attempt;
                    dwt_rxenable(DWT_START_RX_IMMEDIATE);
                    /* claim sent */

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
                            /* claim received */
                            if (other_uid < disc->best_uid)
                                disc->best_uid = other_uid;
                        }
                        else if (match_header(rx_buffer, match_beacon, HDR_LEN, 1, 0, 1))
                        {
                            /* Master already exists — yield election */
                            disc->group_id = rx_buffer[AUTO_BCN_GROUP_IDX];
                            disc->node_count = rx_buffer[AUTO_BCN_COUNT_IDX];
                            disc->best_uid = unpack_uid(&rx_buffer[AUTO_UID_IDX]);
                            /* beacon-yield */
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
    }

    /* Decide: lowest UID wins */
    if (disc->my_uid32 <= disc->best_uid)
    {
        /* I am master */
        disc->my_rank = 0;
        disc->node_count = 1;  /* only me for now */
        disc->master_addr[0] = 0x01;
        disc->master_addr[1] = 0x00;
        test_run_info((unsigned char *)"ELECT WIN → MASTER");
        disc->state = AUTO_STATE_RUN;
        return 0;
    }

    test_run_info((unsigned char *)"ELECT LOSE → JOIN");
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
                while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) { }
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

                    disc->state = AUTO_STATE_RUN;
                    return 0;
                }
            }

            test_run_info((unsigned char *)"JOIN TIMEOUT");
        }

        /* All join retries failed — restart discovery from scratch */
        test_run_info((unsigned char *)"JOIN ALL FAIL → RELISTEN");
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
    int timeout;

    test_run_info((unsigned char *)APP_NAME);

    port_set_dw_ic_spi_fastrate();
    reset_DWIC();
    Sleep(100);

    timeout = 1000;
    while (!dwt_checkidlerc() && --timeout > 0) { Sleep(1); }
    if (timeout <= 0)
    {
        test_run_info((unsigned char *)"IDLE TIMEOUT");
        return -1;
    }
    test_run_info((unsigned char *)"IDLE OK");

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
    {
        test_run_info((unsigned char *)"INIT FAILED");
        return -1;
    }
    test_run_info((unsigned char *)"INIT OK");

    if (dwt_configure(&config))
    {
        test_run_info((unsigned char *)"CONFIG FAILED");
        return -1;
    }
    test_run_info((unsigned char *)"CONFIG OK");

    /* Enable CIA diagnostics — required for dwt_readclockoffset() and
     * dwt_readcarrierintegrator() to return valid clock offset data.
     * Per DW3000 API Guide §5.6.9, must be called after dwt_configure(). */
    dwt_configciadiag(DW_CIA_DIAG_LOG_ALL);

    /* Enable DW3000 GPIO clocks (required for IRQ etc.) */
    dwt_enablegpioclocks();

    /* Enable DW3000 auto RX re-enable (RXAUTR, bit 10 in SYS_CFG). */
    {
        uint32_t sys_cfg = dwt_read32bitreg(SYS_CFG_ID);
        sys_cfg |= SYS_CFG_RXAUTR_BIT_MASK;
        dwt_write32bitreg(SYS_CFG_ID, sys_cfg);
    }

    dwt_configuretxrf(TXCONFIG);
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_setfinegraintxseq(0);
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
    return 0;
}

/*============================================================================
 * MAIN ANCHOR — rank 0: Full protocol with TDMA collector
 *============================================================================*/
static void main_anchor_loop(void)
{
    uint8_t node_count = 1;
    uint16_t beacon_cycle = 0;
    static uint8_t  is_first_beacon = 1;
    static uint64_t prev_tx_ts_dtu = 0;  /* last actual TX timestamp for DLY_TS */

    test_run_info((unsigned char *)"ANCHOR READY");
    port_rf_switch_init();

    while (1)
    {
        uint32_t status_reg, frame_len;

        /*=== Phase 1: RX for ~490ms (leave 10ms margin for Delayed TX setup) ===*/
        {
            int _q;
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            for (_q = 0; _q < 5; _q++) {
                HAL_Delay(94);  /* 5×94ms = 470ms, leaving 30ms margin for DLY_TS */
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

                if (match_header(rx_buffer, match_blink, HDR_LEN, 1, 0, 1))
                {
                    last_blink_seq = rx_buffer[SN_IDX];
                    blink_rx_ts = get_rx_timestamp_u64();
                    {
                        char _s[24]; int _i;
                        _s[0]='B'; _s[1]=','; _s[2]='0'; _s[3]=',';
                        _s[4]=hex[(last_blink_seq>>4)&0xF]; _s[5]=hex[last_blink_seq&0xF];
                        _s[6]=',';
                        for (_i=0;_i<10;_i++) _s[7+_i]=hex[(blink_rx_ts>>(36-_i*4))&0xF];
                        _s[17]='\0';
                        test_run_info((unsigned char *)_s);
                    }
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
                else if (rx_buffer[9] == FUNC_ELECTION_CLAIM)
                {
                    dwt_forcetrxoff();
                    port_rf_switch_tx();
                    tx_beacon[2] = 0;
                    tx_beacon[AUTO_BCN_GROUP_IDX] = 0;
                    tx_beacon[AUTO_BCN_COUNT_IDX] = node_count;
                    pack_uid(&tx_beacon[AUTO_UID_IDX], *(uint32_t *)0x1FFF7590UL);
                    dwt_writetxdata(BEACON_LEN, tx_beacon, 0);
                    dwt_writetxfctrl(BEACON_LEN, 0, 0);
                    if (dwt_starttx(DWT_START_TX_IMMEDIATE) == DWT_SUCCESS) {
                        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) { }
                        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
                    }
                    port_rf_switch_rx();
                }
                else if (rx_buffer[9] == FUNC_JOIN_REQUEST && frame_len >= JOIN_REQ_LEN - 2)
                {
                    uint32_t req_uid = unpack_uid(&rx_buffer[AUTO_UID_IDX]);
                    uint8_t req_src_lo = rx_buffer[7];
                    uint8_t req_src_hi = rx_buffer[8];
                    uint8_t new_rank = node_count;
                    if (new_rank < 16) {
                        node_count++;
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
                                while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) { }
                                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
                            }
                        }
                        port_rf_switch_rx();
                    }
                }
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
                }
                else if ((rx_buffer[9] == FUNC_DL_POS_REPORT || rx_buffer[9] == 0xF3
                       || rx_buffer[9] == 0xF5 || rx_buffer[9] == 0xF7)
                         && frame_len >= GW_HDR_LEN) {
                    gateway_handle_uwb_rx(rx_buffer, frame_len);
                }
#endif
            }
        }

        /*=== Phase 2: TX Downlink Beacon — DWT_START_TX_DLY_TS (relative to last TX) ===*/
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
                tx_ts_dtu = prev_tx_ts_dtu + (500000ull * UUS_TO_DWT_TIME);
            }

            dl_bcn[0] = 0x41; dl_bcn[1] = 0x88;
            dl_bcn[2] = (uint8_t)beacon_cycle;
            dl_bcn[3] = 0xCA; dl_bcn[4] = 0xDE;
            dl_bcn[5] = 0xFF; dl_bcn[6] = 0xFF;
            dl_bcn[7] = ANCHOR_ID_VAL & 0xFF;
            dl_bcn[8] = (ANCHOR_ID_VAL >> 8) & 0xFF;
            dl_bcn[9] = FUNC_DOWNLINK_BEACON;
            dl_bcn[DL_BCN_NUM_IDX]   = g_dl_num_anchors;
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

            if (is_first_beacon) {
                /* First TX: no TX_TIME_LO reference yet → Immediate */
                dwt_starttx(DWT_START_TX_IMMEDIATE);
            } else {
                /* DLY_TS: TX at TX_TIME_LO + (500ms_DTU >> 8).
                 * Constant offset, independent of when this is issued. */
                uint32_t period_hi = (uint32_t)((500000ull * UUS_TO_DWT_TIME) >> 8);
                dwt_setdelayedtrxtime(period_hi);
                if (dwt_starttx(DWT_START_TX_DLY_TS) != DWT_SUCCESS) {
                    /* Target passed → fallback Immediate */
                    test_run_info((unsigned char *)"DLY FL\r\n");
                    dwt_starttx(DWT_START_TX_IMMEDIATE);
                }
            }

            while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) { }
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

            /* Read actual TX time + dump beacon payload for debug */
            {
                uint64_t actual_tx = get_tx_timestamp_u64();
                uint64_t predicted = is_first_beacon ? actual_tx
                    : (prev_tx_ts_dtu + (500000ull * UUS_TO_DWT_TIME));
                int64_t  err_dtu = (int64_t)(actual_tx - predicted);
                /* Print: M,<pred_hi>,<err_ns> */
                {
                    char _m[40]; int _p = 0;
                    _m[_p++]='M'; _m[_p++]=',';
                    _m[_p++]=hex[(predicted>>36)&0xF]; _m[_p++]=hex[(predicted>>32)&0xF];
                    _m[_p++]=',';
                    if (err_dtu < 0) { _m[_p++]='-'; err_dtu = -err_dtu; }
                    else _m[_p++]='+';
                    { int64_t ns = err_dtu / 64;
                      if (ns > 9999) ns = 9999;
                      _m[_p++]=hex[((int)ns>>12)&0xF]; _m[_p++]=hex[((int)ns>>8)&0xF];
                      _m[_p++]=hex[((int)ns>>4)&0xF]; _m[_p++]=hex[(int)ns&0xF]; }
                    _m[_p++]='\r'; _m[_p++]='\n'; _m[_p]='\0';
                    test_run_info((unsigned char *)_m);
                }
                prev_tx_ts_dtu = actual_tx;
                is_first_beacon = 0;
            }

            port_rf_switch_rx();
#endif

            beacon_cycle++;
            if (beacon_cycle >= 10) {
                beacon_cycle = 0;
                dwt_forcetrxoff();
                port_rf_switch_tx();
                tx_beacon[2] = 0;
                tx_beacon[AUTO_BCN_GROUP_IDX] = 0;
                tx_beacon[AUTO_BCN_COUNT_IDX] = node_count;
                pack_uid(&tx_beacon[AUTO_UID_IDX], *(uint32_t *)0x1FFF7590UL);
                dwt_writetxdata(BEACON_LEN, tx_beacon, 0);
                dwt_writetxfctrl(BEACON_LEN, 0, 0);
                if (dwt_starttx(DWT_START_TX_IMMEDIATE) == DWT_SUCCESS) {
                    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) { }
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
    #define MASTER_LOST_THRESHOLD  999

slave_entry:
    test_run_info((unsigned char *)"SLAVE READY");
    port_rf_switch_init();

    while (1)
    {
        uint32_t status_reg, frame_len;
        int got_beacon = 0;

        /*=== Phase 1: RX for ~490ms (1ms fast polling for DW3000 double buffer) ===*/
        {
            int _p;
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            for (_p = 0; _p < 500; _p++)  /* 500×1ms = 500ms RX window */
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
                if (frame_len > sizeof(rx_buffer)) {
                    dwt_rxenable(DWT_START_RX_IMMEDIATE);
                    continue;
                }
                dwt_readrxdata(rx_buffer, frame_len, 0);

                /* Auto-discovery beacon (0xE7) → master heartbeat */
                if (match_header(rx_buffer, match_beacon, HDR_LEN, 1, 0, 1))
                {
                    master_miss_count = 0;
                    consecutive_lost = 0;
                }
#ifdef FUSION_DOWNLINK_MODE
                /* Downlink beacon (0xF0) → heartbeat + TDMA blink via DLY_RS */
                else if (match_header(rx_buffer, match_dl_beacon, HDR_LEN, 1, 0, 1))
                {
                    uint64_t bcn_rx_ts = get_rx_timestamp_u64();
                    uint32_t slot_offset_hi;
                    uint8_t dl_blink[DL_BLINK_LEN];
                    int dly_ok;

                    master_miss_count = 0;
                    consecutive_lost = 0;
                    got_beacon = 1;

                    /* Build blink frame */
                    dl_blink[0] = 0x41; dl_blink[1] = 0x88;
                    dl_blink[2] = 0;
                    dl_blink[3] = 0xCA; dl_blink[4] = 0xDE;
                    dl_blink[5] = 0xFF; dl_blink[6] = 0xFF;
                    dl_blink[7] = my_rank & 0xFF;
                    dl_blink[8] = (my_rank >> 8) & 0xFF;
                    dl_blink[9] = FUNC_DOWNLINK_BLINK;
                    /* TX timestamp at bytes 10-14 (verified correct on DW3210) */
                    {
                        uint64_t blink_tx_ts = bcn_rx_ts
                            + (uint64_t)my_rank * DL_SLOT_DURATION_UUS * UUS_TO_DWT_TIME;
                        pack_40bit_ts(&dl_blink[DL_BLINK_TX_TS_IDX], blink_tx_ts);
                    }
                    /* Rank is encoded in SRC (bytes 7-8, verified reliable).
                     * Pad last 2 bytes to absorb DW3210 RX corruption. */
                    dl_blink[DL_BLINK_LEN - 2] = 0;
                    dl_blink[DL_BLINK_LEN - 1] = 0;

                    slot_offset_hi = (uint32_t)(
                        ((uint64_t)my_rank * DL_SLOT_DURATION_UUS * UUS_TO_DWT_TIME) >> 8);

                    /* Now stop RX, switch, and TX */
                    dwt_forcetrxoff();
                    port_rf_switch_tx();
                    dwt_writetxdata(DL_BLINK_LEN, dl_blink, 0);
                    dwt_writetxfctrl(DL_BLINK_LEN, 0, 0);
                    dwt_setdelayedtrxtime(slot_offset_hi);
                    dly_ok = (dwt_starttx(DWT_START_TX_DLY_RS) == DWT_SUCCESS);

                    if (dly_ok) {
                        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) { }
                        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
                        /* Clock sync: compare actual TX vs predicted
                         * + dump key blink bytes [7,8,9,10,14,15,16] */
                        {
                            uint64_t actual_tx  = get_tx_timestamp_u64();
                            uint64_t predicted = bcn_rx_ts
                                + (uint64_t)my_rank * DL_SLOT_DURATION_UUS * UUS_TO_DWT_TIME;
                            int64_t  err_dtu = (int64_t)(actual_tx - predicted);
                            char _t[64]; int _p = 0;
                            _t[_p++]='T'; _t[_p++]=',';
                            _t[_p++]=hex[my_rank&0xF]; _t[_p++]=',';
                            /* Blink bytes: SRC[7,8] FC[9] TS[10] TS[14] RANK[15,16] */
                            _t[_p++]='['; _t[_p++]=hex[(dl_blink[7]>>4)&0xF]; _t[_p++]=hex[dl_blink[7]&0xF];
                            _t[_p++]=' '; _t[_p++]=hex[(dl_blink[8]>>4)&0xF]; _t[_p++]=hex[dl_blink[8]&0xF];
                            _t[_p++]=' '; _t[_p++]=hex[(dl_blink[9]>>4)&0xF]; _t[_p++]=hex[dl_blink[9]&0xF];
                            _t[_p++]=' '; _t[_p++]=hex[(dl_blink[10]>>4)&0xF]; _t[_p++]=hex[dl_blink[10]&0xF];
                            _t[_p++]=' '; _t[_p++]=hex[(dl_blink[14]>>4)&0xF]; _t[_p++]=hex[dl_blink[14]&0xF];
                            _t[_p++]=' '; _t[_p++]=hex[(dl_blink[15]>>4)&0xF]; _t[_p++]=hex[dl_blink[15]&0xF];
                            _t[_p++]=' '; _t[_p++]=hex[(dl_blink[16]>>4)&0xF]; _t[_p++]=hex[dl_blink[16]&0xF];
                            _t[_p++]=']'; _t[_p++]=',';
                            { int64_t ns = err_dtu>=0 ? err_dtu/64 : -err_dtu/64;
                              if(ns>9999)ns=9999;
                              _t[_p++]=hex[(ns>>12)&0xF]; _t[_p++]=hex[(ns>>8)&0xF];
                              _t[_p++]=hex[(ns>>4)&0xF]; _t[_p++]=hex[ns&0xF]; }
                            _t[_p++]='\r'; _t[_p++]='\n'; _t[_p]='\0';
                            test_run_info((unsigned char *)_t);
                        }
                    } else {
                        /* DLY_RS failed (target passed) → fallback Immediate */
                        test_run_info((unsigned char *)"F\r\n");
                        if (dwt_starttx(DWT_START_TX_IMMEDIATE) == DWT_SUCCESS) {
                            while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) { }
                            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
                            test_run_info((unsigned char *)"t\r\n");
                        }
                    }
                    port_rf_switch_rx();
                }
                /* Tag blink (0xE2) — uplink TDOA reference */
                else if (match_header(rx_buffer, match_tag_blink, HDR_LEN, 1, 0, 1))
                {
                    uint64_t tag_toa = get_rx_timestamp_u64();
                    uint8_t   seq     = rx_buffer[SN_IDX];
                    {
                        char _s[24]; int _i;
                        _s[0]='B'; _s[1]=',';
                        _s[2]=hex[(my_rank>>4)&0xF]; _s[3]=hex[my_rank&0xF];
                        _s[4]=','; _s[5]=hex[(seq>>4)&0xF]; _s[6]=hex[seq&0xF];
                        _s[7]=',';
                        for (_i=0;_i<10;_i++) _s[8+_i]=hex[(tag_toa>>(36-_i*4))&0xF];
                        _s[18]='\0';
                        test_run_info((unsigned char *)_s);
                    }
                }
#endif
#if ENABLE_GATEWAY
                else if (rx_buffer[9] == FUNC_FIND_REQ
                         && frame_len >= GW_HDR_LEN + 1
                         && (uint16_t)(rx_buffer[5] | (rx_buffer[6] << 8)) == my_rank)
                {
                    uint8_t on_off = rx_buffer[GW_HDR_LEN];
                    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin,
                                      on_off ? GPIO_PIN_SET : GPIO_PIN_RESET);
                    gateway_send_uwb_ack_ex(GW_MASTER_ADDR, (uint16_t)my_rank,
                                            FUNC_FIND_ACK, &on_off, 1);
                }
                else if (rx_buffer[9] == FUNC_COORD_WRITE
                         && frame_len >= GW_HDR_LEN + 6
                         && (uint16_t)(rx_buffer[5] | (rx_buffer[6] << 8)) == my_rank)
                {
                    int16_t cx, cy, cz;
                    uint8_t ack_payload[6];
                    cx = (int16_t)(rx_buffer[GW_HDR_LEN]     | (rx_buffer[GW_HDR_LEN+1] << 8));
                    cy = (int16_t)(rx_buffer[GW_HDR_LEN+2]   | (rx_buffer[GW_HDR_LEN+3] << 8));
                    cz = (int16_t)(rx_buffer[GW_HDR_LEN+4]   | (rx_buffer[GW_HDR_LEN+5] << 8));
                    gateway_flash_save_coord(cx, cy, cz);
                    ack_payload[0] = cx & 0xFF;  ack_payload[1] = (cx >> 8) & 0xFF;
                    ack_payload[2] = cy & 0xFF;  ack_payload[3] = (cy >> 8) & 0xFF;
                    ack_payload[4] = cz & 0xFF;  ack_payload[5] = (cz >> 8) & 0xFF;
                    gateway_send_uwb_ack_ex(GW_MASTER_ADDR, (uint16_t)my_rank,
                                            FUNC_COORD_ACK, ack_payload, 6);
                }
#endif
                /* Re-enable RX after handling frame */
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
            }
        }

        /*=== Master heartbeat detection ===*/
        if (!got_beacon)
        {
            master_miss_count++;
            if (master_miss_count >= MASTER_LOST_THRESHOLD)
            {
                consecutive_lost++;
                test_run_info((unsigned char *)"MASTER LOST");
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
                test_run_info((unsigned char *)"IDLE TIMEOUT → SYS RST");
                NVIC_SystemReset();
                while (1) { }  /* unreachable */
            }
            test_run_info((unsigned char *)"IDLE OK");

            if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
            {
                test_run_info((unsigned char *)"INIT FAIL");
                continue;
            }
            test_run_info((unsigned char *)"INIT OK");

            if (dwt_configure(&config))
            {
                test_run_info((unsigned char *)"CONFIG FAIL");
                continue;
            }
            test_run_info((unsigned char *)"CONFIG OK");

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
            test_run_info((unsigned char *)"REDISC ALL FAIL");
            goto re_discover;
        }

        if (disc.my_rank == 0)
        {
            test_run_info((unsigned char *)"PROMOTED MASTER");
            master_network_formation();
            main_anchor_loop();  /* never returns */
        }
        else
        {
            test_run_info((unsigned char *)"SLAVE AGAIN");
            my_rank = disc.my_rank;
            master_miss_count = 0;
            goto slave_entry;
        }
    }
}

/*============================================================================
 * master_network_formation — post-discovery: beacon + listen for JOIN_REQ
 * Runs before main_anchor_loop to allow slaves to join.
 *============================================================================*/
static void master_network_formation(void)
{
    int round;
    uint32_t status_reg, frame_len;

    test_run_info((unsigned char *)"NET FORM");

    for (round = 0; round < 5; round++)
    {
        /* TX Beacon */
        dwt_forcetrxoff();
        port_rf_switch_tx();
        {
            uint8_t bcn[] = {
                0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0x01, 0x00, 0xE7,
                0,       /* GROUP */
                1,       /* COUNT = 1 (just me) */
                0, 0, 0, 0,   /* MASTER_UID[4] */
                0, 0     /* FCS[2] */
            };
            pack_uid(&bcn[AUTO_UID_IDX], *(uint32_t *)0x1FFF7590UL);
            dwt_writetxdata(sizeof(bcn), bcn, 0);
            dwt_writetxfctrl(sizeof(bcn), 0, 0);
            if (dwt_starttx(DWT_START_TX_IMMEDIATE) == DWT_SUCCESS)
            {
                while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) { }
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

                    /* join request received */

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
                            1,       /* rank = 1 (first slave) */
                            0, 0
                        };
                        pack_uid(&ack[10], req_uid);
                        dwt_writetxdata(sizeof(ack), ack, 0);
                        dwt_writetxfctrl(sizeof(ack), 0, 0);
                        if (dwt_starttx(DWT_START_TX_IMMEDIATE) == DWT_SUCCESS)
                        {
                            while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) { }
                            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
                        }
                    }
                    /* Switch to RX BEFORE UART */
                    port_rf_switch_rx();
                    /* ack sent */
                }
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
            }
        }
    }

    test_run_info((unsigned char *)"NET FORM DONE");
}

/*============================================================================
 * GATEWAY SNIFFER — pure listener, no TX (except AT-command responses)
 *============================================================================*/
#if defined(ENABLE_GATEWAY) && defined(GATEWAY_ROLE)
/* Format uint16_t as 4-char hex string (e.g. 1 → "0001") */
static char *fmt_uid4(char *buf, uint16_t val)
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
    /* Register gateway itself (unique UID, not a rank) */
    gateway_device_seen("GW01", "GATEWAY", 0);
    port_rf_switch_init();
    port_rf_switch_rx();
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    while (1)
    {
        int p, frame_cnt = 0;
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
            frame_cnt++;

            /* Match known frame types — consistent UID = 4-digit hex of rank/addr */
            if (rx_buf[9] == FUNC_DOWNLINK_BEACON && frame_len >= DL_BEACON_LEN(2))
            {
                uint8_t n = rx_buf[DL_BCN_NUM_IDX];
                uint16_t master_addr = (uint16_t)(rx_buf[7])
                                     | ((uint16_t)(rx_buf[8]) << 8);
                char uid[8]; uint8_t k;
                uid[4] = '\0';
                /* Log master */
                gateway_device_seen(fmt_uid4(uid, master_addr), "ANCHOR", 0);
                /* Log slaves from beacon table using their rank as UID */
                for (k = 1; k < n && k < DL_MAX_ANCHORS; k++) {
                    gateway_device_seen(fmt_uid4(uid, (uint16_t)k), "ANCHOR", k);
                }
            }
            else if (rx_buf[9] == FUNC_DOWNLINK_BLINK
                   && frame_len >= DL_BLINK_LEN)
            {
                uint16_t rank16 = (uint16_t)(rx_buf[7])
                                | ((uint16_t)(rx_buf[8]) << 8);
                char uid[8];
                uid[4] = '\0';
                gateway_device_seen(fmt_uid4(uid, rank16), "ANCHOR", (int)rank16);
            }
            else if (rx_buf[9] == 0xE7)
            {
                uint16_t master_addr = (uint16_t)(rx_buf[7])
                                     | ((uint16_t)(rx_buf[8]) << 8);
                char uid[8];
                uid[4] = '\0';
                gateway_device_seen(fmt_uid4(uid, master_addr), "ANCHOR", 0);
            }
            else if ((rx_buf[9] == FUNC_DL_POS_REPORT || rx_buf[9] == 0xF3
                   || rx_buf[9] == 0xF5 || rx_buf[9] == 0xF7)
                   && frame_len >= GW_HDR_LEN) {
                gateway_handle_uwb_rx(rx_buf, frame_len);
            }

            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }

        /* Debug: print frame count per window */
        if (frame_cnt > 0) {
            char _f[8];
            _f[0]='R'; _f[1]='X'; _f[2]=':';
            _f[3]=hex[(frame_cnt>>4)&0xF]; _f[4]=hex[frame_cnt&0xF];
            _f[5]='\r'; _f[6]='\n'; _f[7]='\0';
            test_run_info((unsigned char *)_f);
        }

        /* Poll AT commands after each RX window */
        gateway_poll_uart();
    }
}
#endif /* GATEWAY_ROLE */

/*============================================================================
 * fusion_anchor — init → auto-discovery → run
 *============================================================================*/
int fusion_anchor(void)
{
    auto_disc_t disc;

    if (anchor_init() != 0)
    {
        test_run_info((unsigned char *)"INIT FAIL");
        while (1) { }
    }

#if defined(ENABLE_GATEWAY) && defined(GATEWAY_ROLE)
    /* Pure gateway sniffer — no auto-discovery, no anchor loop */
    gateway_sniffer_loop();
    return 0;
#endif

    test_run_info((unsigned char *)"DISCOVERY");
    while (auto_discovery(&disc) != 0)
    {
        test_run_info((unsigned char *)"DISCOVERY FAIL — RETRY");
    }

    if (disc.my_rank == 0)
    {
        test_run_info((unsigned char *)"START MASTER");
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
        master_network_formation();
        main_anchor_loop();
    }
    else
    {
        test_run_info((unsigned char *)"START SLAVE");
        slave_anchor_loop(disc.my_rank);
    }

    return 0;
}

#endif /* TEST_FUSION_ANCHOR */
