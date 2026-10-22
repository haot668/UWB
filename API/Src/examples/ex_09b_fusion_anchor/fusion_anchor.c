/*! ----------------------------------------------------------------------------
 *  @file    fusion_anchor.c
 *  @brief   TDOA+TOF Fusion Anchor — supports Main (rank 0) and Slave (rank 1+)
 *
 *           Main Anchor (RANK == 0):
 *             1. RX TDOA Blink  → record timestamp + clock offset
 *             2. TX Clock Sync   → broadcast sync frame (delayed TX)
 *             3. RX TOF Poll     → record timestamp
 *             4. TX TOF Response → send with timestamps (delayed TX)
 *             5. RX TOF Final    → extract TOF distance
 *             6. RX Slave Reports→ TDMA collection from slave anchors
 *             7. UART output: F,SEQ,R=TS,D=DIST,C=CLK_OFF[,Sn:R=TS:C=CO]
 *
 *           Slave Anchor (RANK >= 1):
 *             1. RX TDOA Blink   → record timestamp
 *             2. RX Clock Sync    → record timestamp + clock offset vs main
 *             3. RX TOF Final     → extract TOF distance
 *             4. TX Slave Report  → send timestamps to main (TDMA slot)
 *             5. UART output: S,RANK,BLINK_TS,SYNC_TS,CLK_OFF,TOF_DIST
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
#include "downlink_protocol.h"
#include "gateway_cmd.h"

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

#define APP_NAME "FUSION ANCHOR v2.2"

/*============================================================================
 * Common configuration — Channel 5, 6.8 Mbps
 *============================================================================*/
static dwt_config_t config = {
    5,               /* Channel number. */
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
 * Timing constants (UWB microseconds)
 *============================================================================*/
#define BLINK_RX_TO_SYNC_TX_DLY_UUS  5000u  /* 5ms: Tag needs ~3ms to recover from TX+settle into RX */
#define POLL_RX_TO_RESP_TX_DLY_UUS   5000u
#define TDMA_BASE_DELAY_UUS          2000u
#define TDMA_SLOT_DURATION_UUS       5000u

/*============================================================================
 * Frame function codes
 *============================================================================*/
#define FUNC_TDOA_BLINK     0xE2u
#define FUNC_CLOCK_SYNC     0xAAu
#define FUNC_TOF_POLL       0xE3u
#define FUNC_TOF_RESP       0xE4u
#define FUNC_TOF_FINAL      0xE5u
#define FUNC_SLAVE_REPORT   0xE6u

/*============================================================================
 * Frame headers for matching (seq/addr bytes zeroed)
 *============================================================================*/
static uint8_t match_blink[]  = {0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0, 0, 0xE2};
static uint8_t match_poll[]   = {0x41, 0x88, 0, 0xCA, 0xDE, 0, 0, 0, 0, 0xE3};
static uint8_t match_final[]  = {0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0, 0, 0xE5};
static uint8_t match_sync[]   = {0x41, 0x88, 0, 0xCA, 0xDE, 0, 0, 0, 0, 0xAA};
#define HDR_LEN 10u
#define SN_IDX  2u

/*============================================================================
 * TX frame buffers
 *============================================================================*/
static uint8_t tx_sync[35] = {0x41, 0x88, 0, 0xCA, 0xDE, 0x01, 0x00, 0x01, 0x00, 0xAA, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
#define SYNC_LEN 35u
/* SYNC frame: [0-9]=IEEE hdr+FUNC, [10]=group, [11]=node, [12-16]=master_blink_rx_ts(40bit) */

static uint8_t tx_resp[] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 0, 0, 0, 0, 0xE4,
    0, 0, 0, 0, 0,  0, 0, 0, 0, 0
};
#define RESP_LEN 20u
#define RESP_POLL_RX_TS_IDX  10u
#define RESP_RESP_TX_TS_IDX  14u

static uint8_t tx_report[] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 0x01, 0x00, 0, 0, 0xE6,
    0, 0, 0, 0, 0,  0, 0, 0, 0, 0,  0, 0, 0, 0,  0, 0,  0, 0
};
#define REPORT_LEN 28u
/* SLAVE_REPORT: [10-14]=blink_rx_ts(40b), [15-19]=sync_rx_ts(40b), [20-23]=ferr(s32), [24-25]=tof_dist, [26-27]=rsvd */
#define REPORT_BLINK_TS_IDX  10u
#define REPORT_SYNC_TS_IDX   15u
#define REPORT_FERR_IDX      20u
#define REPORT_TOF_DIST_IDX  24u

/*============================================================================
 * Auto-discovery match patterns (FC[2],SEQ,PAN[2],DEST[2],SRC[2],FUNC)
 *============================================================================*/
static uint8_t match_beacon[]    = {0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0, 0, 0xE7};
static uint8_t match_claim[]     = {0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0, 0, 0xE8};
static uint8_t match_join_ack[]  = {0x41, 0x88, 0, 0xCA, 0xDE, 0, 0, 0, 0, 0xEA};

/* Gateway UWB response frames (addressed to master, SRC wildcarded) */
static uint8_t match_gw_find_ack[]  = {0x41, 0x88, 0, 0xCA, 0xDE, 0x01, 0x00, 0, 0, 0xF3};
static uint8_t match_gw_coord_ack[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0x01, 0x00, 0, 0, 0xF5};

/* Auto-discovery TX frames */
static uint8_t tx_claim[] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0x01, 0x00, 0xE8,
    0, 0, 0, 0,  0, 0
};
#define CLAIM_LEN 16u

static uint8_t tx_join_req[] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 0x01, 0x00, 0, 0, 0xE9,
    0, 0, 0, 0,  0, 0
};
#define JOIN_REQ_LEN 16u

static uint8_t tx_beacon[] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0x01, 0x00, 0xE7,
    0,       /* GROUP */
    0,       /* COUNT */
    0, 0, 0, 0,   /* MASTER_UID[4] */
    0, 0     /* FCS[2] */
};
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
static uint64_t poll_rx_ts;
static uint64_t sync_rx_ts;
static uint64_t resp_tx_ts;
static int      clock_offset = 0;
static uint8_t  last_blink_seq = 0;
static uint8_t  last_poll_seq = 0;
static uint16_t tof_distance_cm = 0;
static uint64_t prev_master_blink_rx_ts = 0;
static uint64_t prev_slave_blink_rx_ts = 0;
static int      prev_ferr_valid = 0;
static int32_t  slave_ferr = 0;

#define ANCHOR_ID_VAL  0x0001u
#define ANCHOR_GROUP   0u
#define ANCHOR_NODE    0u

extern dwt_txconfig_t txconfig_options;

/*============================================================================
 * Helper: UART hex output
 *============================================================================*/
static const char hex[] = "0123456789ABCDEF";

static void uart_fusion_line(uint8_t seq, uint64_t ts, uint16_t dist)
{
    char out[32];
    int i;
    out[0] = 'F'; out[1] = ',';
    out[2] = hex[(seq >> 4) & 0xF]; out[3] = hex[seq & 0xF];
    out[4] = ','; out[5] = 'R'; out[6] = '=';
    for (i = 0; i < 10; i++) out[7+i] = hex[(ts >> (36 - i*4)) & 0xF];
    out[17] = ','; out[18] = 'D'; out[19] = '=';
    out[20] = hex[(dist >> 12) & 0xF]; out[21] = hex[(dist >> 8) & 0xF];
    out[22] = hex[(dist >> 4) & 0xF]; out[23] = hex[dist & 0xF];
    out[24] = '\0';
    test_run_info((unsigned char *)out);
}

static void uart_slave_line(uint8_t rank, uint64_t brx, uint64_t srx, int32_t ferr, uint16_t dist)
{
    char out[55];
    int i;
    uint32_t ferr_u32 = (uint32_t)ferr;
    out[0] = 'S'; out[1] = ',';
    out[2] = hex[(rank >> 4) & 0xF]; out[3] = hex[rank & 0xF];
    out[4] = ','; out[5] = 'B'; out[6] = '=';
    for (i = 0; i < 10; i++) out[7+i] = hex[(brx >> (36 - i*4)) & 0xF];
    out[17] = ','; out[18] = 'Y'; out[19] = '=';
    for (i = 0; i < 10; i++) out[20+i] = hex[(srx >> (36 - i*4)) & 0xF];
    out[30] = ','; out[31] = 'F'; out[32] = '=';
    for (i = 0; i < 8; i++) out[33+i] = hex[(ferr_u32 >> (28 - i*4)) & 0xF];
    out[41] = ','; out[42] = 'D'; out[43] = '=';
    out[44] = hex[(dist >> 12) & 0xF]; out[45] = hex[(dist >> 8) & 0xF];
    out[46] = hex[(dist >> 4) & 0xF]; out[47] = hex[dist & 0xF];
    out[48] = '\0';
    test_run_info((unsigned char *)out);
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

/*============================================================================
 * Helper: wait for RX
 *============================================================================*/
static int wait_for_rx(uint32_t *status, uint32_t *flen)
{
    /* Sleep-based polling to avoid suppressing DW3000 SFD detection.
     * Master only calls this when a Tag is present (got_blink path). */
    while (1)
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
}

/*============================================================================
 * Helper: delayed TX start
 *============================================================================*/
static int do_delayed_tx(uint8_t *data, uint16_t len, uint8_t ranging)
{
    int ret;
    dwt_writetxdata(len, data, 0);
    dwt_writetxfctrl(len, 0, ranging ? 1 : 0);
    port_rf_switch_tx();
    ret = dwt_starttx(DWT_START_TX_DELAYED);
    if (ret == DWT_SUCCESS)
    {
        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) { }
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    }
    port_rf_switch_rx();
    return ret;
}

/*============================================================================
 * Helper: wait for RX with Sleep()-based timeout (does NOT use HAL_GetTick)
 * Returns 1 on frame received, 0 on timeout / error
 *============================================================================*/
static int wait_for_rx_timeout(uint32_t *status, uint32_t *flen, uint32_t timeout_ms)
{
    /* Sleep-based polling — one check per ms, same approach as auto_discovery
     * LISTEN phase.  Tight SPI polling (every ~6us) can suppress DW3000 SFD
     * detection; a 1 ms gap between register reads lets the receiver breathe. */
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
    return 0;  /* timeout */
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

    {
        char dbg[48];
        int x;
        const char *hx = "0123456789ABCDEF";
        dbg[0] = 'U'; dbg[1] = 'I'; dbg[2] = 'D'; dbg[3] = '=';
        for (x = 0; x < 8; x++)
            dbg[4+x] = hx[(disc->my_uid32 >> (28 - x*4)) & 0xF];
        dbg[12] = '\0';
        test_run_info((unsigned char *)dbg);
    }

    /* Initialize RF switch in RX mode for discovery listening */
    port_rf_switch_init();
    port_rf_switch_rx();

    /*=== Phase: LISTEN — listen for existing master beacon (2 sec) ===*/
    {
        int attempt;
        test_run_info((unsigned char *)"AUTO LISTEN");
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        test_run_info((unsigned char *)"RX ON");

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

            {
                char dbg[48];
                const char *hx = "0123456789ABCDEF";
                dbg[0] = 'B'; dbg[1] = 'E'; dbg[2] = 'A'; dbg[3] = 'C'; dbg[4] = 'O'; dbg[5] = 'N'; dbg[6] = ' ';
                dbg[7] = 'G'; dbg[8] = '='; dbg[9] = hx[disc->group_id & 0xF];
                dbg[10] = ' '; dbg[11] = 'C'; dbg[12] = '='; dbg[13] = hx[disc->node_count & 0xF];
                dbg[14] = '\0';
                test_run_info((unsigned char *)dbg);
            }

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
                    test_run_info((unsigned char *)"CLAIM TX");

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
                            test_run_info((unsigned char *)"CLAIM RX");
                            if (other_uid < disc->best_uid)
                                disc->best_uid = other_uid;
                        }
                        else if (match_header(rx_buffer, match_beacon, HDR_LEN, 1, 0, 1))
                        {
                            /* Master already exists — yield election */
                            disc->group_id = rx_buffer[AUTO_BCN_GROUP_IDX];
                            disc->node_count = rx_buffer[AUTO_BCN_COUNT_IDX];
                            disc->best_uid = unpack_uid(&rx_buffer[AUTO_UID_IDX]);
                            test_run_info((unsigned char *)"BCN-YIELD");
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
            test_run_info((unsigned char *)"JOIN REQ TX");

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
                        char dbg[32];
                        const char *hx = "0123456789ABCDEF";
                        dbg[0] = 'R'; dbg[1] = 'A'; dbg[2] = 'N'; dbg[3] = 'K'; dbg[4] = '=';
                        dbg[5] = hx[(disc->my_rank >> 4) & 0xF];
                        dbg[6] = hx[disc->my_rank & 0xF];
                        dbg[7] = '\0';
                        test_run_info((unsigned char *)dbg);
                    }

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
    disc->my_rank = (uint8_t)FUSION_ANCHOR_RANK;
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

    dwt_configuretxrf(&txconfig_options);
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
    uint8_t node_count = 1;   /* master counts as 1 */
    uint16_t beacon_cycle = 0;

    /* Init static frame fields */
    tx_sync[10] = ANCHOR_GROUP;
    tx_sync[11] = ANCHOR_NODE;
    tx_resp[7] = ANCHOR_ID_VAL & 0xFF;
    tx_resp[8] = (ANCHOR_ID_VAL >> 8) & 0xFF;

    test_run_info((unsigned char *)"ANCHOR READY");
    port_rf_switch_init();

    while (1)
    {
        uint32_t status_reg, frame_len;
        int got_blink = 0;

        /* Phase 1: RX TDOA Blink (500ms timeout → periodic beacon/join even without Tag).
         * Also handles CLAIM/JOIN_REQ from late-joining anchors to prevent dual-master. */
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        if (wait_for_rx_timeout(&status_reg, &frame_len, 500)
            && frame_len <= sizeof(rx_buffer))
        {
            dwt_readrxdata(rx_buffer, frame_len, 0);

            if (match_header(rx_buffer, match_blink, HDR_LEN, 1, 0, 1))
            {
                got_blink = 1;
                last_blink_seq = rx_buffer[SN_IDX];
                blink_rx_ts = get_rx_timestamp_u64();
                clock_offset = (int)dwt_readclockoffset();
            }
            else if (rx_buffer[9] == FUNC_ELECTION_CLAIM)
            {
                /* Another anchor is electing — assert master presence with immediate beacon */
                dwt_forcetrxoff();
                port_rf_switch_tx();
                tx_beacon[2] = 0;
                tx_beacon[AUTO_BCN_GROUP_IDX] = 0;
                tx_beacon[AUTO_BCN_COUNT_IDX] = node_count;
                pack_uid(&tx_beacon[AUTO_UID_IDX], *(uint32_t *)0x1FFF7590UL);
                dwt_writetxdata(BEACON_LEN, tx_beacon, 0);
                dwt_writetxfctrl(BEACON_LEN, 0, 0);
                if (dwt_starttx(DWT_START_TX_IMMEDIATE) == DWT_SUCCESS)
                {
                    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) { }
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
                }
                port_rf_switch_rx();
                test_run_info((unsigned char *)"CLAIM-BCN");
            }
            else if (rx_buffer[9] == FUNC_JOIN_REQUEST && frame_len >= JOIN_REQ_LEN - 2)
            {
                /* Late join request — send ACK immediately */
                uint32_t req_uid = unpack_uid(&rx_buffer[AUTO_UID_IDX]);
                uint8_t req_src_lo = rx_buffer[7];
                uint8_t req_src_hi = rx_buffer[8];
                uint8_t new_rank = node_count;
                if (new_rank < 16)
                {
                    node_count++;
                    dwt_forcetrxoff();
                    port_rf_switch_tx();
                    {
                        uint8_t ack[] = {
                            0x41, 0x88, 0, 0xCA, 0xDE,
                            req_src_lo, req_src_hi,
                            0x01, 0x00,
                            0xEA,
                            0, 0, 0, 0,
                            new_rank,
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
                    port_rf_switch_rx();
                    test_run_info((unsigned char *)"JOIN ACK TX");
                }
            }
            else if ((rx_buffer[9] == 0xF3 || rx_buffer[9] == 0xF5 || rx_buffer[9] == 0xF7)
                     && frame_len >= GW_HDR_LEN)
            {
                /* Gateway UWB response frame (FIND_ACK / COORD_ACK / OTA_ACK) */
                gateway_handle_uwb_rx(rx_buffer, frame_len);
            }
        }

        if (got_blink)
        {
            /* Phase 2: TX Clock Sync (delayed) */
            {
                uint32_t sync_tx_time;
                sync_tx_time = (uint32_t)((blink_rx_ts + (BLINK_RX_TO_SYNC_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8);
                dwt_setdelayedtrxtime(sync_tx_time);
                sync_rx_ts = (((uint64_t)(sync_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
                tx_sync[SN_IDX] = last_blink_seq;
                pack_40bit_ts(&tx_sync[12], blink_rx_ts);
                /* Append anchor coordinates for Tag TDOA solve (DL_TS_COORD_IDX=29) */
                {
                    dl_anchor_entry_t my_coord;
                    dl_get_anchor_coord((uint8_t)my_rank, &my_coord);
                    tx_sync[17] = (uint8_t)( my_coord.x_cm        & 0xFF);
                    tx_sync[18] = (uint8_t)((my_coord.x_cm >> 8)  & 0xFF);
                    tx_sync[19] = (uint8_t)( my_coord.y_cm        & 0xFF);
                    tx_sync[20] = (uint8_t)((my_coord.y_cm >> 8)  & 0xFF);
                    tx_sync[21] = (uint8_t)( my_coord.z_cm        & 0xFF);
                    tx_sync[22] = (uint8_t)((my_coord.z_cm >> 8)  & 0xFF);
                }
                if (do_delayed_tx(tx_sync, SYNC_LEN, 0) == DWT_SUCCESS)
                    test_run_info((unsigned char *)"SYNC TX");
                else
                {
                    test_run_info((unsigned char *)"SYNC TX FAIL");
                    continue;  /* skip Phases 3-6 — Tag won't send Poll without SYNC */
                }
            }

            /* Phase 3: RX TOF Poll (200ms timeout — Tag sends Poll ~2ms after SYNC) */
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            if (wait_for_rx_timeout(&status_reg, &frame_len, 200)
                && frame_len <= sizeof(rx_buffer))
            {
                uint8_t tag_id_lo, tag_id_hi;
                dwt_readrxdata(rx_buffer, frame_len, 0);
                tag_id_lo = rx_buffer[7];
                tag_id_hi = rx_buffer[8];
                if (match_header(rx_buffer, match_poll, HDR_LEN, 1, 1, 1))
                {
                    last_poll_seq = rx_buffer[SN_IDX];
                    test_run_info((unsigned char *)"POLL RX");
                    poll_rx_ts = get_rx_timestamp_u64();
                    /* Register tag with gateway */
                    {
                        char uid[8];
                        uint16_t tag_addr = (uint16_t)(tag_id_lo | (tag_id_hi << 8));
                        uid[0] = hex[(tag_addr >> 12) & 0xF];
                        uid[1] = hex[(tag_addr >>  8) & 0xF];
                        uid[2] = hex[(tag_addr >>  4) & 0xF];
                        uid[3] = hex[ tag_addr        & 0xF];
                        uid[4] = '\0';
                        gateway_device_seen(uid, "TAG", 0);
                    }

                    /* Phase 4: TX TOF Response (delayed) */
                    {
                        uint32_t resp_tx_time;
                        resp_tx_time = (uint32_t)((poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8);
                        dwt_setdelayedtrxtime(resp_tx_time);
                        resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
                        resp_msg_set_ts(&tx_resp[RESP_POLL_RX_TS_IDX], poll_rx_ts);
                        resp_msg_set_ts(&tx_resp[RESP_RESP_TX_TS_IDX], resp_tx_ts);
                        tx_resp[SN_IDX] = last_poll_seq;
                        tx_resp[5] = tag_id_lo;
                        tx_resp[6] = tag_id_hi;
                        if (do_delayed_tx(tx_resp, RESP_LEN, 1) == DWT_SUCCESS)
                        {
                            test_run_info((unsigned char *)"RESP TX");
                        }
                        else
                        {
                            /* Delayed TX failed (TX time likely in past) —
                             * fall back to immediate TX.  TX buffer was already
                             * written by do_delayed_tx, just restart immediately. */
                            dwt_forcetrxoff();
                            port_rf_switch_tx();
                            if (dwt_starttx(DWT_START_TX_IMMEDIATE) == DWT_SUCCESS)
                            {
                                while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) { }
                                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
                                test_run_info((unsigned char *)"RESP TX*");
                            }
                            else
                            {
                                test_run_info((unsigned char *)"RESP TX FAIL");
                            }
                            port_rf_switch_rx();
                        }
                    }

                    /* Phase 5: RX TOF Final (200ms timeout) */
                    dwt_rxenable(DWT_START_RX_IMMEDIATE);
                    if (wait_for_rx_timeout(&status_reg, &frame_len, 200)
                        && frame_len <= sizeof(rx_buffer))
                    {
                        dwt_readrxdata(rx_buffer, frame_len, 0);
                        if (match_header(rx_buffer, match_final, HDR_LEN, 1, 0, 1))
                        {
                            tof_distance_cm = (uint16_t)rx_buffer[10] | ((uint16_t)rx_buffer[11] << 8);
                            uart_fusion_line(last_poll_seq, blink_rx_ts, tof_distance_cm);

                            /* Phase 6: RX Slave Reports (TDMA).
                             * Only iterate known slaves (node_count-1), not all 15 slots.
                             * Short per-slot timeout — if a slave's report doesn't arrive
                             * within 3ms the master skips it and won't miss the next Blink. */
                            {
                                uint64_t final_rx_ts = get_rx_timestamp_u64();
                                int slave, slave_count;
                                slave_count = (int)node_count - 1;  /* exclude master */
                                for (slave = 1; slave <= slave_count; slave++)
                                {
                                    dwt_rxenable(DWT_START_RX_IMMEDIATE);
                                    if (!wait_for_rx_timeout(&status_reg, &frame_len, SLAVE_REPORT_SLOT_TIMEOUT_MS))
                                        continue;
                                    if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK))
                                        continue;
                                    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
                                    if (frame_len > sizeof(rx_buffer))
                                        continue;
                                    dwt_readrxdata(rx_buffer, frame_len, 0);
                                    if (match_header(rx_buffer, tx_report, HDR_LEN, 1, 0, 1))
                                    {
                                        uint64_t s_brx, s_srx;
                                        int32_t s_ferr;
                                        uint16_t s_dist;
                                        uint8_t s_rank = rx_buffer[7];
                                        s_brx  = unpack_40bit_ts(&rx_buffer[10]);
                                        s_srx  = unpack_40bit_ts(&rx_buffer[15]);
                                        s_ferr = *(int32_t*)&rx_buffer[20];
                                        s_dist = (uint16_t)rx_buffer[24] | ((uint16_t)rx_buffer[25] << 8);
                                        uart_slave_line(s_rank, s_brx, s_srx, s_ferr, s_dist);
                                        /* Register slave anchor with gateway */
                                        {
                                            char uid[8];
                                            uid[0] = hex[(s_rank >> 12) & 0xF];
                                            uid[1] = hex[(s_rank >>  8) & 0xF];
                                            uid[2] = hex[(s_rank >>  4) & 0xF];
                                            uid[3] = hex[ s_rank        & 0xF];
                                            uid[4] = '\0';
                                            gateway_device_seen(uid, "ANCHOR", s_rank);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        /*=== Phase 7: Periodic beacon (every 10 cycles). Join handled by Phase 1. ===*/
        {
            beacon_cycle++;
            if (beacon_cycle >= 10)
            {
                beacon_cycle = 0;
                dwt_forcetrxoff();
                port_rf_switch_tx();
                tx_beacon[2] = 0;
                tx_beacon[AUTO_BCN_GROUP_IDX] = 0;
                tx_beacon[AUTO_BCN_COUNT_IDX] = node_count;
                pack_uid(&tx_beacon[AUTO_UID_IDX],
                         *(uint32_t *)0x1FFF7590UL);
                dwt_writetxdata(BEACON_LEN, tx_beacon, 0);
                dwt_writetxfctrl(BEACON_LEN, 0, 0);
                if (dwt_starttx(DWT_START_TX_IMMEDIATE) == DWT_SUCCESS)
                {
                    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) { }
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
                }
                port_rf_switch_rx();
                test_run_info((unsigned char *)"BEACON TX");
            }

            /* Poll gateway AT commands (non-blocking, only if line available) */
            gateway_poll_uart();
        }
    }
}

/*============================================================================
 * SLAVE ANCHOR — rank 1+: RX Blink → RX Sync → RX Final → TX Report (TDMA)
 * All phases use timeout. Master loss detected via consecutive missed cycles
 * with beacon check. If master lost, re-discover (may become new master).
 *============================================================================*/
static void master_network_formation(void);  /* forward decl — defined below */

static void slave_anchor_loop(uint8_t my_rank)
{
    int master_miss_count = 0;
    static int consecutive_lost = 0;  /* persists across goto slave_entry */
    #define MASTER_LOST_THRESHOLD  3   /* ~5 seconds without master */

slave_entry:
    /* Init report frame static fields */
    tx_report[7] = my_rank & 0xFF;
    tx_report[8] = (my_rank >> 8) & 0xFF;
    tx_report[5] = ANCHOR_ID_VAL & 0xFF;
    tx_report[6] = (ANCHOR_ID_VAL >> 8) & 0xFF;

    test_run_info((unsigned char *)"SLAVE READY");
    port_rf_switch_init();

    while (1)
    {
        uint32_t status_reg, frame_len;
        int got_blink = 0, got_sync = 0, got_final = 0;

        /* Phase 1: RX TDOA Blink (500ms timeout) */
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        if (wait_for_rx_timeout(&status_reg, &frame_len, 500)
            && frame_len <= sizeof(rx_buffer))
        {
            dwt_readrxdata(rx_buffer, frame_len, 0);
            if (match_header(rx_buffer, match_blink, HDR_LEN, 1, 0, 1))
            {
                got_blink = 1;
                last_blink_seq = rx_buffer[SN_IDX];
                blink_rx_ts = get_rx_timestamp_u64();
            }
            else if (match_header(rx_buffer, match_beacon, HDR_LEN, 1, 0, 1))
            {
                /* Master beacon caught by Phase 1 — count as heartbeat */
                got_blink = 0;  /* no Tag, but master is alive */
                master_miss_count = 0;
                consecutive_lost = 0;
            }
            /* Gateway commands from master */
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
        }

        if (got_blink)
        {
            /* Phase 2: RX Clock Sync from Main Anchor (100ms timeout — should arrive ~300us after Blink) */
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            if (wait_for_rx_timeout(&status_reg, &frame_len, 100)
                && frame_len <= sizeof(rx_buffer))
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);
                if (match_header(rx_buffer, match_sync, HDR_LEN, 1, 0, 1))
                {
                    got_sync = 1;
                    sync_rx_ts = get_rx_timestamp_u64();
                    {
                        uint64_t master_blink_rx_ts = unpack_40bit_ts(&rx_buffer[12]);
                        int32_t ferr = 0;
                        if (prev_ferr_valid)
                        {
                            uint64_t num = master_blink_rx_ts - prev_master_blink_rx_ts;
                            uint64_t den = blink_rx_ts - prev_slave_blink_rx_ts;
                            if (den > 0)
                            {
                                ferr = (int32_t)(((int64_t)num * FERR_SCALE) / (int64_t)den - (int64_t)FERR_SCALE);
                            }
                        }
                        else
                        {
                            prev_ferr_valid = 1;
                        }
                        prev_master_blink_rx_ts = master_blink_rx_ts;
                        prev_slave_blink_rx_ts = blink_rx_ts;
                        slave_ferr = ferr;
                    }
                }
            }
        }

        if (got_sync)
        {
            /* Phase 3: RX TOF Final from Tag (500ms timeout) */
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            if (wait_for_rx_timeout(&status_reg, &frame_len, 500)
                && frame_len <= sizeof(rx_buffer))
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);
                if (match_header(rx_buffer, match_final, HDR_LEN, 1, 0, 1))
                {
                    got_final = 1;
                    tof_distance_cm = (uint16_t)rx_buffer[10] | ((uint16_t)rx_buffer[11] << 8);

                    /* Phase 4: TX Slave Report (TDMA delayed TX) */
                    {
                        uint32_t report_tx_time;
                        uint32_t tdma_delay;
                        tdma_delay = TDMA_BASE_DELAY_UUS + (my_rank - 1) * TDMA_SLOT_DURATION_UUS;
                        report_tx_time = (uint32_t)((get_rx_timestamp_u64() + tdma_delay * UUS_TO_DWT_TIME) >> 8);
                        dwt_setdelayedtrxtime(report_tx_time);

                        pack_40bit_ts(&tx_report[REPORT_BLINK_TS_IDX], blink_rx_ts);
                        pack_40bit_ts(&tx_report[REPORT_SYNC_TS_IDX], sync_rx_ts);
                        *(int32_t*)&tx_report[REPORT_FERR_IDX] = slave_ferr;
                        tx_report[REPORT_TOF_DIST_IDX] = (uint8_t)(tof_distance_cm & 0xFF);
                        tx_report[REPORT_TOF_DIST_IDX + 1] = (uint8_t)((tof_distance_cm >> 8) & 0xFF);
                        tx_report[SN_IDX] = last_blink_seq;

                        if (do_delayed_tx(tx_report, REPORT_LEN, 0) == DWT_SUCCESS)
                            test_run_info((unsigned char *)"RPT TX");
                        else
                            test_run_info((unsigned char *)"RPT TX FAIL");
                    }

                    /* Phase 5: UART Output */
                    uart_slave_line(my_rank, blink_rx_ts, sync_rx_ts, slave_ferr, tof_distance_cm);
                }
            }
        }

        /*=== Master heartbeat detection ===*/
        if (got_sync)
        {
            /* Full cycle completed — master is alive */
            master_miss_count = 0;
            consecutive_lost = 0;
        }
        else
        {
            /* No clock sync this cycle — check for beacon as secondary heartbeat.
             * Master sends beacon every ~1100ms; 1200ms timeout ensures the
             * slave's check window always overlaps at least one master cycle. */
            int heard_beacon = 0;
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            if (wait_for_rx_timeout(&status_reg, &frame_len, 1200)
                && frame_len <= sizeof(rx_buffer))
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);
                if (match_header(rx_buffer, match_beacon, HDR_LEN, 1, 0, 1))
                {
                    heard_beacon = 1;
                }
            }

            if (heard_beacon)
            {
                master_miss_count = 0;
                consecutive_lost = 0;
            }
            else
            {
                master_miss_count++;
                /* Heartbeat dot — one per missed cycle so we can see slave is alive */
                test_run_info((unsigned char *)".");
            }

            if (master_miss_count >= MASTER_LOST_THRESHOLD)
            {
                consecutive_lost++;
                test_run_info((unsigned char *)"MASTER LOST");
                if (consecutive_lost >= 3)
                {
                    test_run_info((unsigned char *)"FATAL: MULTI LOST → RST");
                    NVIC_SystemReset();
                    while (1) { }  /* unreachable */
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

            test_run_info((unsigned char *)"REINIT");

            /* Force DW3000 out of RX before touching RST pin */
            dwt_forcetrxoff();

            port_set_dw_ic_spi_fastrate();

            test_run_info((unsigned char *)"RST DW");
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

            dwt_configuretxrf(&txconfig_options);
            dwt_setrxantennadelay(RX_ANT_DLY);
            dwt_settxantennadelay(TX_ANT_DLY);
            dwt_setfinegraintxseq(0);
            dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

            if (auto_discovery(&disc) == 0)
                break;

            test_run_info((unsigned char *)"REDISC RETRY");
        }

        if (total_attempts >= 6)
        {
            test_run_info((unsigned char *)"REDISC ALL FAIL");
            goto re_discover;
        }

        if (disc.my_rank == 0)
        {
            test_run_info((unsigned char *)"PROMOTED MASTER");
            prev_ferr_valid = 0;
            master_network_formation();
            main_anchor_loop();  /* never returns */
        }
        else
        {
            test_run_info((unsigned char *)"SLAVE AGAIN");
            my_rank = disc.my_rank;
            master_miss_count = 0;
            prev_ferr_valid = 0;
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
            test_run_info((unsigned char *)"BCN TX");
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

                    test_run_info((unsigned char *)"JOIN RX");

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
                    test_run_info((unsigned char *)"ACK TX");
                }
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
            }
        }
    }

    test_run_info((unsigned char *)"NET FORM DONE");
}

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

    test_run_info((unsigned char *)"DISCOVERY");
    while (auto_discovery(&disc) != 0)
    {
        test_run_info((unsigned char *)"DISCOVERY FAIL — RETRY");
    }

    if (disc.my_rank == 0)
    {
        test_run_info((unsigned char *)"START MASTER");
        /* Register self as gateway anchor device */
        gateway_device_seen("0000", "ANCHOR", 0);
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
