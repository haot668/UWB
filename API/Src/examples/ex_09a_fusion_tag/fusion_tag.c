/*! ----------------------------------------------------------------------------
 *  @file    fusion_tag.c
 *  @brief   TDOA+TOF Fusion Tag — sends TDOA Blink + SS-TWR TOF ranging
 *
 *           Each cycle (~30ms, ≥20Hz):
 *             1. TX TDOA Blink (broadcast → all anchors record RX timestamp)
 *             2. RX Clock Sync from Main Anchor
 *             3. TX TOF Poll (unicast → Main Anchor)
 *             4. RX TOF Response (from Main Anchor)
 *             5. TX TOF Final (broadcast → contains TOF dist)
 *
 * @attention
 * Copyright 2015-2020 (c) Decawave Ltd, Dublin, Ireland.
 * All rights reserved.
 */
#include <deca_device_api.h>
#include <deca_regs.h>
#include <deca_spi.h>
#include <port.h>
#include <shared_defines.h>
#include <shared_functions.h>
#include <example_selection.h>

#if defined(TEST_FUSION_TAG)

extern void test_run_info(unsigned char *data);

#define APP_NAME "FUSION TAG v1.2"

/* Default communication configuration — Channel 5, 6.8 Mbps */
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

/* TDOA Blink: FC[2], SEQ, PAN[2], DEST[2], TAG_ID[2], FUNC=0xE2, FCS[2] = 12 bytes */
static uint8_t tx_blink[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0x01, 0x00, 0xE2, 0, 0};
#define BLINK_LEN 12

/* TOF Poll: FC[2], SEQ, PAN[2], DEST(ANCHOR)[2], SRC(TAG)[2], FUNC=0xE3, FCS[2] = 14 bytes */
static uint8_t tx_poll[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0x01, 0x00, 0x01, 0x00, 0xE3, 0, 0, 0, 0};
#define POLL_LEN 14

/* TOF Final: FC[2], SEQ, PAN[2], DEST(Bcast)[2], SRC(TAG)[2], FUNC=0xE5, TOF_DIST[2], FCS[2] = 16 bytes */
static uint8_t tx_final[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0x01, 0x00, 0xE5, 0, 0, 0, 0, 0, 0};
#define FINAL_LEN 16

/* Receive buffers */
static uint8_t rx_buf[32];

/* Sequence number and TAG ID */
static uint8_t frame_seq = 0;
#define TAG_ID      0x0001
#define ANCHOR_ID   0x0001

/* Timestamps for TOF distance calculation */
static uint64_t poll_tx_ts;
static uint64_t resp_rx_ts;

/* TOF distance result, in cm */
static uint16_t tof_distance_cm = 0;

extern dwt_txconfig_t txconfig_options;

/*============================================================================
 * wait_rx_tag — Sleep(1)-based RX wait with timeout, same pattern as anchor.
 *               Tight SPI polling suppresses DW3000 frame detection.
 *============================================================================*/
static int wait_rx_tag(uint32_t *status, uint32_t *flen, uint32_t timeout_ms)
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

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn fusion_tag()
 */
int fusion_tag(void)
{
    int timeout;

    test_run_info((unsigned char *)APP_NAME);

    /*--- Cold-boot init ---*/
    port_set_dw_ic_spi_fastrate();
    reset_DWIC();
    Sleep(100);

    timeout = 1000;
    while (!dwt_checkidlerc() && --timeout > 0) { Sleep(1); }
    if (timeout <= 0)
    {
        test_run_info((unsigned char *)"IDLE TIMEOUT");
        while (1) { }
    }
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

    test_run_info((unsigned char *)"INIT DONE");
    port_rf_switch_init();

    /* Tag cycle loop */
    while (1)
    {
        uint32_t status_reg;
        uint32_t frame_len;

        /* === Phase 1: TX TDOA Blink === */
        dwt_forcetrxoff();
        port_rf_switch_tx();

        tx_blink[2] = frame_seq;
        dwt_writetxdata(BLINK_LEN, tx_blink, 0);
        dwt_writetxfctrl(BLINK_LEN, 0, 0);
        if (dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS)
        {
            test_run_info((unsigned char *)"BLINK TX FAIL");
            port_rf_switch_rx();
            Sleep(10);
            continue;
        }
        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) { }
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
        test_run_info((unsigned char *)"BLINK OK");

        /* Recovery from tight SPI polling — give DW3000 time to settle.
         * SLEEP_TX_RECOVERY_MS balances two constraints:
         *  - Too short: DW3000 PLL not stable when SYNC arrives → SYNC TIMEOUT
         *  - Too long:  eats into the 3ms window before Master TXs SYNC */
        #define SLEEP_TX_RECOVERY_MS 2
        Sleep(SLEEP_TX_RECOVERY_MS);

        /* === Phase 2: RX Clock Sync (50ms timeout, arrives ~3ms after Blink) === */
        port_rf_switch_rx();
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        /* Let DW3000 analog front-end + PLL settle into RX before polling */
        Sleep(1);

        if (!wait_rx_tag(&status_reg, &frame_len, 50))
        {
            test_run_info((unsigned char *)"SYNC TIMEOUT");
            Sleep(10);
            continue;
        }
        if (frame_len <= sizeof(rx_buf))
            dwt_readrxdata(rx_buf, frame_len, 0);
        test_run_info((unsigned char *)"SYNC OK");

        /* === Phase 3: TX TOF Poll (ranging frame) === */
        dwt_forcetrxoff();
        port_rf_switch_tx();

        tx_poll[2] = frame_seq;
        tx_poll[5] = (ANCHOR_ID >> 0) & 0xFF;
        tx_poll[6] = (ANCHOR_ID >> 8) & 0xFF;
        tx_poll[7] = (TAG_ID >> 0) & 0xFF;
        tx_poll[8] = (TAG_ID >> 8) & 0xFF;

        dwt_writetxdata(POLL_LEN, tx_poll, 0);
        dwt_writetxfctrl(POLL_LEN, 0, 1);  /* ranging frame */
        if (dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS)
        {
            test_run_info((unsigned char *)"POLL TX FAIL");
            port_rf_switch_rx();
            Sleep(10);
            continue;
        }
        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) { }
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
        poll_tx_ts = get_tx_timestamp_u64();
        test_run_info((unsigned char *)"POLL OK");

        /* Brief recovery from tight SPI polling during TXFRS wait */
        Sleep(1);

        /* === Phase 4: RX TOF Response (50ms timeout) === */
        port_rf_switch_rx();
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        if (!wait_rx_tag(&status_reg, &frame_len, 50))
        {
            test_run_info((unsigned char *)"RESP TIMEOUT");
            Sleep(10);
            continue;
        }
        if (frame_len > sizeof(rx_buf))
        {
            test_run_info((unsigned char *)"RESP OVERFLOW");
            Sleep(10);
            continue;
        }
        dwt_readrxdata(rx_buf, frame_len, 0);
        resp_rx_ts = get_rx_timestamp_u64();

        /* Verify this is a RESP frame (FUNC=0xE4) */
        if (rx_buf[9] != 0xE4)
        {
            char dbg[32];
            dbg[0] = 'W'; dbg[1] = 'R'; dbg[2] = 'O'; dbg[3] = 'N'; dbg[4] = 'G';
            dbg[5] = ' '; dbg[6] = 'F'; dbg[7] = '=';
            dbg[8] = "0123456789ABCDEF"[(rx_buf[9] >> 4) & 0xF];
            dbg[9] = "0123456789ABCDEF"[rx_buf[9] & 0xF];
            dbg[10] = '\0';
            test_run_info((unsigned char *)dbg);
            Sleep(10);
            continue;
        }
        test_run_info((unsigned char *)"RESP OK");

        /* Compute TOF distance from response timestamps (SS-TWR) */
        {
            uint64_t poll_rx_ts32, resp_tx_ts32;
            uint64_t Tround, Treply;
            double Tprop, distance;

            /* Extract 32-bit timestamps from response message
             * Anchor sends RESP_LEN=20, payload offsets:
             *   [10-13] poll_rx_ts (4B, low 32-bit of 40-bit)
             *   [14-17] resp_tx_ts (4B, low 32-bit of 40-bit) */
            poll_rx_ts32 = (uint64_t)rx_buf[10]
                       | ((uint64_t)rx_buf[11] << 8)
                       | ((uint64_t)rx_buf[12] << 16)
                       | ((uint64_t)rx_buf[13] << 24);
            resp_tx_ts32 = (uint64_t)rx_buf[14]
                       | ((uint64_t)rx_buf[15] << 8)
                       | ((uint64_t)rx_buf[16] << 16)
                       | ((uint64_t)rx_buf[17] << 24);

            /* 32-bit counter wraps every ~67ms. Anchor's poll_rx→resp_tx
             * delay is ~500us, so at most one wrap. Detect and compensate. */
            if (resp_tx_ts32 < poll_rx_ts32)
                resp_tx_ts32 += (1ULL << 32);

            Tround = resp_rx_ts - poll_tx_ts;
            Treply = resp_tx_ts32 - poll_rx_ts32;
            Tprop = (double)((int64_t)(Tround - Treply)) / 2.0;

            distance = Tprop * (SPEED_OF_LIGHT / 499.2e6 / 128.0);
            tof_distance_cm = (uint16_t)(distance * 100.0);

            {
                char dbg[48];
                dbg[0] = 'D'; dbg[1] = '=';
                dbg[2] = "0123456789ABCDEF"[(tof_distance_cm >> 12) & 0xF];
                dbg[3] = "0123456789ABCDEF"[(tof_distance_cm >> 8) & 0xF];
                dbg[4] = "0123456789ABCDEF"[(tof_distance_cm >> 4) & 0xF];
                dbg[5] = "0123456789ABCDEF"[tof_distance_cm & 0xF];
                dbg[6] = ' '; dbg[7] = 'c'; dbg[8] = 'm';
                dbg[9] = '\0';
                test_run_info((unsigned char *)dbg);
            }
        }

        /* === Phase 5: TX TOF Final === */
        dwt_forcetrxoff();
        port_rf_switch_tx();

        tx_final[2] = frame_seq;
        tx_final[7] = (TAG_ID >> 0) & 0xFF;
        tx_final[8] = (TAG_ID >> 8) & 0xFF;
        tx_final[10] = (tof_distance_cm >> 0) & 0xFF;
        tx_final[11] = (tof_distance_cm >> 8) & 0xFF;

        dwt_writetxdata(FINAL_LEN, tx_final, 0);
        dwt_writetxfctrl(FINAL_LEN, 0, 0);
        if (dwt_starttx(DWT_START_TX_IMMEDIATE) == DWT_SUCCESS)
        {
            while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) { }
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
        }

        /* Cycle complete — wait before next (target ≥20Hz → 50ms max cycle) */
        test_run_info((unsigned char *)"CYCLE DONE");
        frame_seq++;
        port_rf_switch_rx();
        Sleep(20);
    }
}
#endif
