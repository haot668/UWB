/*! ----------------------------------------------------------------------------
 *  @file    tdoa_tag.c
 *  @brief   TDoA Tag — periodic blink transmitter
 *
 *           This tag periodically sends a "blink" frame without waiting for
 *           any response. Multiple TDoA anchors receive the same blink and
 *           record precise RX timestamps. A location engine uses the time
 *           differences of arrival to compute the tag's position via
 *           hyperbolic positioning (requires ≥3 anchors).
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

#if defined(TEST_TDOA_TAG)

extern void test_run_info(unsigned char *data);

#define APP_NAME "TDOA TAG v1.0"

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

/* Blink frame: minimal unicast/broadcast frame for TDoA.
 * Bytes 0-1: Frame Control (0x8841 = 16-bit addressing, data frame)
 * Byte  2:   Sequence number (incremented each blink)
 * Bytes 3-4: PAN ID (0xDECA)
 * Bytes 5-6: Destination address (0xFFFF = broadcast)
 * Bytes 7-8: Source address (tag ID, 0x0001)
 * Byte  9:   Function code (0xE2 = TDoA blink)
 * Bytes 10-11: FCS (auto-appended by DW3000) */
static uint8_t tx_blink_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0x01, 0x00, 0xE2, 0, 0};
#define BLINK_FRAME_LEN  (sizeof(tx_blink_msg))
#define BLINK_SN_IDX      2

/* Interval between blinks, in milliseconds */
#define BLINK_INTERVAL_MS 200

/* Values for the PG_DELAY and TX_POWER registers. */
extern dwt_txconfig_t txconfig_options;

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn tdoa_tag()
 */
int tdoa_tag(void)
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

    dwt_setfinegraintxseq(0);
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    test_run_info((unsigned char *)"INIT DONE");

    port_rf_switch_init();

    while (1)
    {
        /* Write blink frame to DW IC TX buffer. */
        dwt_writetxdata(BLINK_FRAME_LEN, tx_blink_msg, 0);
        dwt_writetxfctrl(BLINK_FRAME_LEN, 0, 0);

        /* Switch RF to TX and start transmission immediately. */
        port_rf_switch_tx();
        dwt_starttx(DWT_START_TX_IMMEDIATE);

        /* Wait for TX completion. */
        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK))
        { };

        /* Clear TX event. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

        /* Increment sequence number (modulo 256). */
        tx_blink_msg[BLINK_SN_IDX]++;

        test_run_info((unsigned char *)"BLINK");

        /* Switch back to RX and wait for next interval. */
        port_rf_switch_rx();
        Sleep(BLINK_INTERVAL_MS);
    }
}
#endif
