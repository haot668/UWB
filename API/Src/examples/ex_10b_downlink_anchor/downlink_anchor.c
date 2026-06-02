/*! ----------------------------------------------------------------------------
 * @file    downlink_anchor.c
 * @brief   DOWNlink TDOA Anchor — Master broadcasts beacon + anchor table,
 *          Slaves transmit blinks in TDMA slots.
 *
 *          Master (rank 0):
 *            1. TX DOWNLINK_BEACON (0xF0) every 100ms — anchor positions table
 *            2. Short RX window for JOIN_REQUEST handling
 *            3. UART output: B,<seq>,N=<count>
 *
 *          Slave (rank 1+):
 *            1. RX DOWNLINK_BEACON → record rx timestamp, parse anchor table
 *            2. TX DOWNLINK_BLINK (0xF1) at beacon_rx_ts + rank * 1000us (delayed TX)
 *            3. Heartbeat: 6 missed beacons → re_discover
 *            4. UART output: b,<rank>,<seq>
 *
 *          Auto-discovery (0xE7-0xEA) reused from fusion anchor for
 *          master/slave election.
 *
 *          Hardware: STM32L412KBU6 + DW3210, USART1 output at 115200 8N1
 *
 * @attention
 */

#include <deca_device_api.h>
#include <deca_regs.h>
#include <deca_spi.h>
#include <port.h>
#include <shared_defines.h>
#include <shared_functions.h>
#include <example_selection.h>
#include "auto_anchor.h"
#include "downlink_protocol.h"

#if defined(TEST_DOWNLINK_ANCHOR)

extern void test_run_info(unsigned char *data);

#if defined(AUTO_ANCHOR_ENABLE)
#define DISCOVERY_ENABLED 1
#else
#define DISCOVERY_ENABLED 0
#endif

#define APP_NAME "DOWNLINK ANCHOR v1.0"

/*============================================================================
 * PHY config — Channel 5, 6.8 Mbps (same as tag for compatibility)
 *============================================================================*/
static dwt_config_t config = {
    5,               /* Channel number. */
    DWT_PLEN_128,    /* Preamble length. */
    DWT_PAC8,        /* Preamble acquisition chunk size. */
    9,               /* TX preamble code. */
    9,               /* RX preamble code. */
    1,               /* Non-standard 8 symbol SFD */
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

extern dwt_txconfig_t txconfig_options;

/*============================================================================
 * Match patterns (10-byte IEEE header: FC[2] SEQ PAN[2] DST[2] SRC[2] FUNC)
 *============================================================================*/
static uint8_t match_auto_beacon[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0, 0, 0xE7};
static uint8_t match_claim[]      = {0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0, 0, 0xE8};
static uint8_t match_join_ack[]   = {0x41, 0x88, 0, 0xCA, 0xDE, 0, 0, 0, 0, 0xEA};

/* DOWNLINK beacon match — used by slave to detect master heartbeat */
static uint8_t match_dl_beacon[]  = {0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0, 0, 0xF0};

#define HDR_LEN 10u
#define SN_IDX  2u

/*============================================================================
 * Auto-discovery TX frames (0xE7-0xEA)
 *============================================================================*/
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

static uint8_t tx_auto_beacon[] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0x01, 0x00, 0xE7,
    0,       /* GROUP */
    0,       /* COUNT */
    0, 0, 0, 0,   /* MASTER_UID[4] */
    0, 0     /* padding + FCS placeholder */
};
#define AUTO_BCN_LEN 18u

/*============================================================================
 * Anchor position table (compile-time from example_selection.h)
 *============================================================================*/
static const dl_anchor_entry_t anchor_positions[DL_NUM_ANCHORS] = {
    {0, DL_A0_X_CM, DL_A0_Y_CM, DL_A0_Z_CM},
    {1, DL_A1_X_CM, DL_A1_Y_CM, DL_A1_Z_CM},
    {2, DL_A2_X_CM, DL_A2_Y_CM, DL_A2_Z_CM},
    {3, DL_A3_X_CM, DL_A3_Y_CM, DL_A3_Z_CM},
    {4, DL_A4_X_CM, DL_A4_Y_CM, DL_A4_Z_CM},
};

/*============================================================================
 * RX buffer
 *============================================================================*/
#define RX_BUF_LEN 128u
static uint8_t rx_buffer[RX_BUF_LEN];

/*============================================================================
 * UART helpers
 *============================================================================*/
static const char hex_chars[] = "0123456789ABCDEF";

static void uart_beacon_line(uint8_t seq, uint8_t count)
{
    char out[24];
    out[0] = 'B'; out[1] = ',';
    out[2] = hex_chars[(seq >> 4) & 0xF];
    out[3] = hex_chars[seq & 0xF];
    out[4] = ','; out[5] = 'N'; out[6] = '=';
    out[7] = hex_chars[(count >> 4) & 0xF];
    out[8] = hex_chars[count & 0xF];
    out[9] = '\0';
    test_run_info((unsigned char *)out);
}

static void uart_blink_line(uint8_t rank, uint8_t seq)
{
    char out[16];
    out[0] = 'b'; out[1] = ',';
    out[2] = hex_chars[(rank >> 4) & 0xF];
    out[3] = hex_chars[rank & 0xF];
    out[4] = ',';
    out[5] = hex_chars[(seq >> 4) & 0xF];
    out[6] = hex_chars[seq & 0xF];
    out[7] = '\0';
    test_run_info((unsigned char *)out);
}

/*============================================================================
 * Helper: match frame header (zero out seq/dst/src for matching)
 *============================================================================*/
static int match_header(uint8_t *rx, uint8_t *pattern, uint8_t len,
                        uint8_t zero_sn, uint8_t zero_dst, uint8_t zero_src)
{
    uint8_t s_sn = rx[SN_IDX];
    uint8_t s_d0 = rx[5], s_d1 = rx[6];
    uint8_t s_s0 = rx[7], s_s1 = rx[8];
    if (zero_sn)  rx[SN_IDX] = 0;
    if (zero_dst) { rx[5] = 0; rx[6] = 0; }
    if (zero_src) { rx[7] = 0; rx[8] = 0; }
    int match = (memcmp(rx, pattern, len) == 0);
    if (zero_sn)  rx[SN_IDX] = s_sn;
    if (zero_dst) { rx[5] = s_d0; rx[6] = s_d1; }
    if (zero_src) { rx[7] = s_s0; rx[8] = s_s1; }
    return match;
}

/*============================================================================
 * Helper: wait for RX with Sleep-based timeout
 *============================================================================*/
static int wait_for_rx_timeout(uint32_t *status, uint32_t *flen, uint32_t timeout_ms)
{
    int max_loops = (int)timeout_ms * 500;
    int loop;
    for (loop = 0; loop < max_loops; loop++)
    {
        *status = dwt_read32bitreg(SYS_STATUS_ID);
        if (*status & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR))
            break;
    }
    if (loop >= max_loops)
        return 0;
    if (*status & SYS_STATUS_RXFCG_BIT_MASK)
    {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
        *flen = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
        return 1;
    }
    else
    {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        return 0;
    }
}

/*============================================================================
 * Helper: delayed TX
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

    disc->my_uid32 = *(uint32_t *)0x1FFF7590UL;
    disc->state = AUTO_STATE_LISTEN;
    disc->my_rank = 0xFFu;
    disc->group_id = 0;
    disc->node_count = 0;
    disc->best_uid = disc->my_uid32;
    disc->candidates_seen = 0;
    disc->master_addr[0] = 0x01;
    disc->master_addr[1] = 0x00;

    {
        char dbg[48];
        int x;
        dbg[0] = 'U'; dbg[1] = 'I'; dbg[2] = 'D'; dbg[3] = '=';
        for (x = 0; x < 8; x++)
            dbg[4+x] = hex_chars[(disc->my_uid32 >> (28 - x*4)) & 0xF];
        dbg[12] = '\0';
        test_run_info((unsigned char *)dbg);
    }

    port_rf_switch_init();
    port_rf_switch_rx();

    /*=== Phase: LISTEN (2 sec) ===*/
    {
        int attempt;
        test_run_info((unsigned char *)"AUTO LISTEN");
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        for (attempt = 0; attempt < 20; attempt++)
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

            /* Accept both auto-discovery beacons (0xE7) and DOWNLINK beacons (0xF0) */
            if (match_header(rx_buffer, match_auto_beacon, HDR_LEN, 1, 0, 1))
            {
                disc->group_id = rx_buffer[AUTO_BCN_GROUP_IDX];
                disc->node_count = rx_buffer[AUTO_BCN_COUNT_IDX];
                disc->master_addr[0] = rx_buffer[7];
                disc->master_addr[1] = rx_buffer[8];
                disc->best_uid = unpack_uid(&rx_buffer[AUTO_UID_IDX]);
            }
            else if (match_header(rx_buffer, match_dl_beacon, HDR_LEN, 1, 0, 1))
            {
                /* DOWNLINK beacon: master is active — extract info and join */
                disc->group_id = 0;
                disc->node_count = rx_buffer[DL_BCN_NUM_IDX];
                disc->master_addr[0] = rx_buffer[7];
                disc->master_addr[1] = rx_buffer[8];
                disc->best_uid = 0;  /* master wins any tiebreak */
            }
            else
            {
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
                continue;
            }

            {
                char dbg[32];
                dbg[0] = 'B'; dbg[1] = 'E'; dbg[2] = 'A'; dbg[3] = 'C'; dbg[4] = 'O'; dbg[5] = 'N';
                dbg[6] = ' '; dbg[7] = 'G'; dbg[8] = '=';
                dbg[9] = hex_chars[disc->group_id & 0xF];
                dbg[10] = ' '; dbg[11] = 'C'; dbg[12] = '=';
                dbg[13] = hex_chars[disc->node_count & 0xF];
                dbg[14] = '\0';
                test_run_info((unsigned char *)dbg);
            }

            disc->state = AUTO_STATE_JOIN;
            goto phase_join;
        }

        test_run_info((unsigned char *)"LISTEN TIMEOUT");
    }

    /*=== Phase: ELECT ===*/
    disc->state = AUTO_STATE_ELECT;
    {
        test_run_info((unsigned char *)"AUTO ELECT");

        Sleep((disc->my_uid32 & 0xFFu) % 50u);

        {
            int round;
            for (round = 0; round < 3; round++)
            {
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
                port_rf_switch_rx();

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
                        else if (match_header(rx_buffer, match_auto_beacon, HDR_LEN, 1, 0, 1))
                        {
                            disc->group_id = rx_buffer[AUTO_BCN_GROUP_IDX];
                            disc->node_count = rx_buffer[AUTO_BCN_COUNT_IDX];
                            disc->best_uid = unpack_uid(&rx_buffer[AUTO_UID_IDX]);
                            test_run_info((unsigned char *)"BCN-YIELD");
                            disc->state = AUTO_STATE_JOIN;
                            dwt_rxenable(DWT_START_RX_IMMEDIATE);
                            goto phase_join;
                        }
                        else if (match_header(rx_buffer, match_dl_beacon, HDR_LEN, 1, 0, 1))
                        {
                            /* DOWNLINK master already running — yield */
                            disc->group_id = 0;
                            disc->node_count = rx_buffer[DL_BCN_NUM_IDX];
                            disc->best_uid = 0;
                            test_run_info((unsigned char *)"DLBCN-YIELD");
                            disc->state = AUTO_STATE_JOIN;
                            dwt_rxenable(DWT_START_RX_IMMEDIATE);
                            goto phase_join;
                        }
                        dwt_rxenable(DWT_START_RX_IMMEDIATE);
                    }
                }

                if (disc->candidates_seen > 0 && disc->best_uid < disc->my_uid32)
                    break;
            }
        }
    }

    if (disc->my_uid32 <= disc->best_uid)
    {
        disc->my_rank = 0;
        disc->node_count = 1;
        disc->master_addr[0] = 0x01;
        disc->master_addr[1] = 0x00;
        test_run_info((unsigned char *)"ELECT WIN -> MASTER");
        disc->state = AUTO_STATE_RUN;
        return 0;
    }

    test_run_info((unsigned char *)"ELECT LOSE -> JOIN");
    disc->state = AUTO_STATE_JOIN;

phase_join:
    /*=== Phase: JOIN ===*/
    {
        int join_attempt;

        Sleep((disc->my_uid32 >> 8) % 100u);

        dwt_forcetrxoff();
        port_rf_switch_tx();
        tx_join_req[2] = 0;
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
        port_rf_switch_rx();

        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        test_run_info((unsigned char *)"JOIN REQ TX");

        for (join_attempt = 0; join_attempt < 20; join_attempt++)
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

            if (!match_header(rx_buffer, match_join_ack, HDR_LEN, 1, 1, 1))
            {
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
                continue;
            }

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
                    char dbg[16];
                    dbg[0] = 'R'; dbg[1] = 'A'; dbg[2] = 'N'; dbg[3] = 'K'; dbg[4] = '=';
                    dbg[5] = hex_chars[(disc->my_rank >> 4) & 0xF];
                    dbg[6] = hex_chars[disc->my_rank & 0xF];
                    dbg[7] = '\0';
                    test_run_info((unsigned char *)dbg);
                }

                disc->state = AUTO_STATE_RUN;
                return 0;
            }
        }

        test_run_info((unsigned char *)"JOIN TIMEOUT");
        disc->state = AUTO_STATE_ERROR;
        return 1;
    }
}

#else /* !DISCOVERY_ENABLED */

static int auto_discovery(auto_disc_t *disc)
{
    disc->my_rank = (uint8_t)FUSION_ANCHOR_RANK;
    disc->state = AUTO_STATE_RUN;
    return 0;
}

#endif /* DISCOVERY_ENABLED */

/*============================================================================
 * anchor_init — DW3000 init sequence
 *============================================================================*/
static int anchor_init(void)
{
    test_run_info((unsigned char *)APP_NAME);

    port_set_dw_ic_spi_fastrate();
    reset_DWIC();
    Sleep(10);

    while (!dwt_checkidlerc()) { }
    test_run_info((unsigned char *)"IDLE OK");

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
    {
        test_run_info((unsigned char *)"INIT FAILED");
        while (1) { }
    }
    test_run_info((unsigned char *)"INIT OK");

    if (dwt_configure(&config))
    {
        test_run_info((unsigned char *)"CONFIG FAILED");
        while (1) { }
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
 * master_network_formation — post-discovery beacon + join window
 *============================================================================*/
static void master_network_formation(void)
{
    int round;
    uint32_t status_reg, frame_len;

    test_run_info((unsigned char *)"NET FORM");

    for (round = 0; round < 5; round++)
    {
        dwt_forcetrxoff();
        port_rf_switch_tx();
        {
            uint8_t bcn[] = {
                0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0x01, 0x00, 0xE7,
                0,       /* GROUP */
                1,       /* COUNT = 1 (just me) */
                0, 0, 0, 0,   /* MASTER_UID[4] */
                0, 0     /* padding + FCS placeholder */
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
        port_rf_switch_rx();

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
 * downlink_master_loop — periodic beacon TX + JOIN handling
 *============================================================================*/
static void downlink_master_loop(uint8_t node_count)
{
    uint8_t seq = 0;

    test_run_info((unsigned char *)"MASTER READY");
    port_rf_switch_init();

    while (1)
    {
        uint32_t status_reg, frame_len;
        int i;

        /*=== TX DOWNLINK BEACON (0xF0) ===*/
        {
            uint16_t bcn_frame_len = (uint16_t)DL_BEACON_LEN(node_count);
            uint8_t beacon[DL_BEACON_LEN(DL_MAX_ANCHORS)];

            beacon[0] = 0x41; beacon[1] = 0x88;
            beacon[2] = seq;
            beacon[3] = 0xCA; beacon[4] = 0xDE;
            beacon[5] = 0xFF; beacon[6] = 0xFF;
            beacon[7] = 0x01; beacon[8] = 0x00;
            beacon[9] = FUNC_DOWNLINK_BEACON;
            beacon[10] = node_count;
            beacon[11] = seq;

            for (i = 0; i < (int)node_count && i < (int)DL_NUM_ANCHORS; i++)
            {
                pack_anchor_entry(&beacon[DL_BCN_TABLE_IDX + i * DL_ANCHOR_ENTRY_SIZE],
                                  &anchor_positions[i]);
            }

            dwt_forcetrxoff();
            port_rf_switch_tx();
            dwt_writetxdata(bcn_frame_len, beacon, 0);
            dwt_writetxfctrl(bcn_frame_len, 0, 0);
            if (dwt_starttx(DWT_START_TX_IMMEDIATE) == DWT_SUCCESS)
            {
                while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) { }
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
            }
            port_rf_switch_rx();
        }

        /*=== Short RX window for JOIN_REQUEST ===*/
        {
            int rx_attempts = 5;
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            while (rx_attempts-- > 0)
            {
                if (!wait_for_rx_timeout(&status_reg, &frame_len, 20))
                    continue;
                if (frame_len > sizeof(rx_buffer))
                    continue;

                dwt_readrxdata(rx_buffer, frame_len, 0);
                if (rx_buffer[9] == FUNC_JOIN_REQUEST && frame_len >= JOIN_REQ_LEN - 2)
                {
                    uint32_t req_uid = unpack_uid(&rx_buffer[AUTO_UID_IDX]);
                    uint8_t req_src_lo = rx_buffer[7];
                    uint8_t req_src_hi = rx_buffer[8];
                    uint8_t new_rank = node_count;

                    if (new_rank < 16 && new_rank < DL_NUM_ANCHORS)
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
                        test_run_info((unsigned char *)"JOIN ACK");
                    }
                }
            }
        }

        /*=== UART output ===*/
        uart_beacon_line(seq, node_count);

        seq++;

        /*=== Sleep for remainder of 100ms cycle ===*/
        Sleep(95);
    }
}

/*============================================================================
 * downlink_slave_loop — RX beacon → TX blink in TDMA slot
 *============================================================================*/
static void master_network_formation(void);
static void downlink_master_loop(uint8_t node_count);

static void downlink_slave_loop(uint8_t my_rank)
{
    int master_miss_count = 0;

slave_entry:
    test_run_info((unsigned char *)"SLAVE READY");
    port_rf_switch_init();

    while (1)
    {
        uint32_t status_reg, frame_len;
        uint64_t beacon_rx_ts;
        uint8_t  beacon_seq;
        int      got_beacon = 0;

        /*=== RX DOWNLINK BEACON (200ms timeout) ===*/
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        if (wait_for_rx_timeout(&status_reg, &frame_len, 200)
            && frame_len <= sizeof(rx_buffer))
        {
            dwt_readrxdata(rx_buffer, frame_len, 0);

            if (match_header(rx_buffer, match_dl_beacon, HDR_LEN, 1, 0, 1))
            {
                got_beacon = 1;
                beacon_seq = rx_buffer[DL_BCN_CYCLE_IDX];
                beacon_rx_ts = get_rx_timestamp_u64();
            }
        }

        if (got_beacon)
        {
            master_miss_count = 0;

            /*=== TX DOWNLINK BLINK at beacon_rx_ts + rank * 1000us ===*/
            {
                uint32_t blink_tx_time;
                uint8_t blink[] = {
                    0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0, 0, 0xF1,
                    0, 0
                };
                blink[7] = my_rank & 0xFF;
                blink[8] = (my_rank >> 8) & 0xFF;

                blink_tx_time = (uint32_t)(
                    (beacon_rx_ts + (uint32_t)my_rank * DL_SLOT_DURATION_UUS * UUS_TO_DWT_TIME) >> 8);
                dwt_setdelayedtrxtime(blink_tx_time);

                if (do_delayed_tx(blink, DL_BLINK_LEN, 0) == DWT_SUCCESS)
                {
                    uart_blink_line(my_rank, beacon_seq);
                }
                else
                {
                    test_run_info((unsigned char *)"BLINK TX FAIL");
                }
            }
        }
        else
        {
            /* No beacon this cycle — check secondary heartbeat via auto-disc beacon */
            int heard_auto_bcn = 0;
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            if (wait_for_rx_timeout(&status_reg, &frame_len, 50)
                && frame_len <= sizeof(rx_buffer))
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);
                if (match_header(rx_buffer, match_auto_beacon, HDR_LEN, 1, 0, 1)
                    || match_header(rx_buffer, match_dl_beacon, HDR_LEN, 1, 0, 1))
                {
                    heard_auto_bcn = 1;
                }
            }

            if (heard_auto_bcn)
            {
                master_miss_count = 0;
            }
            else
            {
                master_miss_count++;
                if (master_miss_count >= DL_MASTER_LOST_THRESHOLD)
                {
                    test_run_info((unsigned char *)"MASTER LOST");
                    goto re_discover;
                }
            }
        }
    }

re_discover:
    /* Master lost — re-init and re-run auto-discovery */
    {
        auto_disc_t disc;
        int retry;

        dwt_forcetrxoff();
        reset_DWIC();
        Sleep(100);
        while (!dwt_checkidlerc()) { }
        dwt_initialise(DWT_DW_INIT);
        dwt_configure(&config);
        dwt_configuretxrf(&txconfig_options);
        dwt_setrxantennadelay(RX_ANT_DLY);
        dwt_settxantennadelay(TX_ANT_DLY);

        for (retry = 0; retry < 3; retry++)
        {
            if (auto_discovery(&disc) == 0)
                break;
            test_run_info((unsigned char *)"REDISC RETRY");
            reset_DWIC();
            Sleep(100);
            while (!dwt_checkidlerc()) { }
            dwt_initialise(DWT_DW_INIT);
            dwt_configure(&config);
            dwt_configuretxrf(&txconfig_options);
            dwt_setrxantennadelay(RX_ANT_DLY);
            dwt_settxantennadelay(TX_ANT_DLY);
        }

        if (retry >= 3)
        {
            test_run_info((unsigned char *)"REDISC ALL FAIL");
            goto re_discover;
        }

        if (disc.my_rank == 0)
        {
            test_run_info((unsigned char *)"PROMOTED MASTER");
            master_network_formation();
            downlink_master_loop((uint8_t)disc.node_count);
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
 * downlink_anchor — init → auto-discovery → master or slave loop
 *============================================================================*/
int downlink_anchor(void)
{
    auto_disc_t disc;
    int retry;

    anchor_init();

    for (retry = 0; retry < 5; retry++)
    {
        if (auto_discovery(&disc) == 0)
            break;
        test_run_info((unsigned char *)"DISCOVERY RETRY");
        reset_DWIC();
        Sleep(100);
        while (!dwt_checkidlerc()) { }
        dwt_initialise(DWT_DW_INIT);
        dwt_configure(&config);
        dwt_configuretxrf(&txconfig_options);
        dwt_setrxantennadelay(RX_ANT_DLY);
        dwt_settxantennadelay(TX_ANT_DLY);
    }

    if (retry >= 5)
    {
        test_run_info((unsigned char *)"DISCOVERY FAILED");
        while (1) { }
    }

    if (disc.my_rank == 0)
    {
        master_network_formation();
        downlink_master_loop((uint8_t)disc.node_count);
    }
    else
    {
        downlink_slave_loop(disc.my_rank);
    }

    return 0;
}

#endif /* TEST_DOWNLINK_ANCHOR */
