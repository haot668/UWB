/*! ----------------------------------------------------------------------------
 *  @file    tdoa_anchor.c
 *  @brief   TDoA Anchor — listens for blink frames and records precise
 *           RX timestamps for TDoA hyperbolic positioning.
 *
 *           In a full TDoA system, multiple anchors (≥3) at known positions
 *           each receive the same blink from a tag and record the arrival
 *           timestamp. The timestamps are collected by a location engine
 *           (e.g. PC) which computes time differences of arrival to
 *           determine the tag's position. This example implements the
 *           anchor-side timestamp capture only.
 *
 * @attention
 *
 * Copyright 2015-2020 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */
#include <deca_device_api.h>
#include <deca_regs.h>
#include <deca_spi.h>
#include <port.h>
#include <shared_defines.h>
#include <shared_functions.h>
#include <example_selection.h>

#if defined(TEST_TDOA_ANCHOR)

extern void test_run_info(unsigned char *data);

#define APP_NAME "TDOA ANCHOR v1.0"

/* Default communication configuration — must match the tag. */
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

/* Expected blink frame header (first 10 bytes, seq byte zeroed for comparison).
 * Matches the tag's blink format. */
static uint8_t rx_blink_check[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0x01, 0x00, 0xE2};
#define BLINK_COMMON_LEN 10
#define BLINK_SN_IDX      2

/* Antenna delay values for 64 MHz PRF. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

/* Buffer to store received blink frames. */
#define RX_BUF_LEN 24
static uint8_t rx_buffer[RX_BUF_LEN];

/* Hold copy of status register for debug. */
static uint32_t status_reg = 0;

/* RX timeout for blink detection, in UWB microseconds. */
#define RX_TIMEOUT_UUS 0  /* 0 = no timeout, listen forever */

/* Anchor ID — set a unique value per anchor in a multi-anchor deployment. */
#define ANCHOR_ID 1

/* Values for the PG_DELAY and TX_POWER registers. */
extern dwt_txconfig_t txconfig_options;

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn tdoa_anchor()
 */
int tdoa_anchor(void)
{
    test_run_info((unsigned char *)APP_NAME);

    port_set_dw_ic_spi_fastrate();

    reset_DWIC();
    Sleep(10);

    while (!dwt_checkidlerc())
    { };
    test_run_info((unsigned char *)"IDLE OK");

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
    {
        test_run_info((unsigned char *)"INIT FAILED");
        while (1) { };
    }
    test_run_info((unsigned char *)"INIT OK");

    if (dwt_configure(&config))
    {
        test_run_info((unsigned char *)"CONFIG FAILED");
        while (1) { };
    }
    test_run_info((unsigned char *)"CONFIG OK");

    dwt_configuretxrf(&txconfig_options);

    dwt_setrxantennadelay(RX_ANT_DLY);

    dwt_setfinegraintxseq(0);
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    test_run_info((unsigned char *)"ANCHOR READY");

    /* Init RF switch (anchor only receives). */
    port_rf_switch_init();

    /* Main anchor loop — continuously listen for blinks. */
    while (1)
    {
        uint32_t frame_len;
        uint32_t rx_timestamp;

        /* Activate reception immediately. */
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        /* Wait for a frame or RX error/timeout. */
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID))
               & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR)))
        { };

        if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
        {
            /* Clear good RX frame event. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

            /* Read frame length from RX_FINFO register. */
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;

            if (frame_len <= sizeof(rx_buffer))
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);

                /* Retrieve the precise RX timestamp (lower 32 bits). */
                rx_timestamp = dwt_readrxtimestamplo32();

                /* Save seq, then verify this is a TDoA blink (check with seq zeroed). */
                uint8_t seq = rx_buffer[BLINK_SN_IDX];
                rx_buffer[BLINK_SN_IDX] = 0;
                if (memcmp(rx_buffer, rx_blink_check, BLINK_COMMON_LEN) == 0)
                {

                    /* Read clock offset for anchor-to-anchor time sync.
                     * CO = (clockOffset / 2^26) = fractional clock offset ratio. */
                    uint16_t clockOffset = dwt_readclockoffset();

                    /* Output timestamp record in machine-parseable CSV format:
                     * A,<ANCHOR>,<SEQ>,<RX_TS_HEX>,<CLOCK_OFFSET> */
                    {
                        char out[64];
                        int i;
                        out[0] = 'A'; out[1] = ',';
                        out[2] = '0' + ANCHOR_ID;
                        out[3] = ','; out[4] = 'S'; out[5] = '=';
                        out[6] = "0123456789ABCDEF"[(seq >> 4) & 0xF];
                        out[7] = "0123456789ABCDEF"[seq & 0xF];
                        out[8] = ','; out[9] = 'R'; out[10] = '=';
                        for (i = 0; i < 8; i++)
                            out[11+i] = "0123456789ABCDEF"[(rx_timestamp >> (28 - i*4)) & 0xF];
                        out[19] = ','; out[20] = 'C'; out[21] = '=';
                        for (i = 0; i < 4; i++)
                            out[22+i] = "0123456789ABCDEF"[(clockOffset >> (12 - i*4)) & 0xF];
                        out[26] = '\0';
                        test_run_info((unsigned char *)out);
                    }
                }
            }
        }
        else
        {
            /* Clear RX error events. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        }
    }
}
#endif
