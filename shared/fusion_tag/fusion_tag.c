/*! ----------------------------------------------------------------------------
 *  @file    fusion_tag.c
 *  @brief   Downlink TDOA Tag — receives anchor beacons + blinks, solves position
 *
 *           Each cycle:
 *             1. RX Downlink Beacon (0xF0) → anchor position table + timing ref
 *             2. RX Downlink Blinks  (0xF1) → TDOA timestamps (one per slave)
 *             3. tdoa_solve() → Gauss-Newton multilateration (local FPU)
 *             4. UART output: P,SEQ,X=<cm>,Y=<cm>,Z=<cm>,Q=<rms>
 *
 *           Requires: FUSION_DOWNLINK_MODE defined in example_selection.h
 *                     tdoa_solver.c compiled into project
 *                     ≥3 anchors with DL_A<n>_X/Y/Z_CM positions configured
 *
 * @attention
 * Copyright 2015-2020 (c) Decawave Ltd, Dublin, Ireland.
 * All rights reserved.
 */
#include <deca_device_api.h>
#include <deca_regs.h>
#include <deca_spi.h>
#include <port.h>
#include <string.h>
#include <shared_defines.h>
#include <shared_functions.h>
#include <example_selection.h>
#include "downlink_protocol.h"

#if defined(TEST_FUSION_TAG)

extern void test_run_info(unsigned char *data);

#define APP_NAME "DOWNLINK TAG v3.0"

/* Default communication configuration — Channel 5, 6.8 Mbps, NO STS */
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
    DWT_STS_MODE_OFF,
    DWT_STS_LEN_64,
    DWT_PDOA_M0
};

#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

/* Increase RX buffer — beacon can be up to DL_BEACON_LEN(16)=126 bytes */
#define RX_BUF_LEN  (DL_BEACON_LEN(DL_MAX_ANCHORS) + 4)
static uint8_t rx_buffer[RX_BUF_LEN];

/* Downlink anchor table + TOA buffer */
static dl_anchor_entry_t anchor_table[DL_MAX_ANCHORS];
static uint64_t toa_dtu[DL_MAX_ANCHORS];
static uint8_t  num_anchors = 0;
static uint8_t  frame_seq   = 0;

/* Header match patterns (SN, DST, SRC wildcarded) */
#define HDR_LEN 10u
#define SN_IDX   2u
static uint8_t match_beacon[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0, 0, 0xF0};
static uint8_t match_blink[]  = {0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0, 0, 0xF1};

extern dwt_txconfig_t txconfig_options;

static const char hex[] = "0123456789ABCDEF";

/*============================================================================
 * match_header — compare rx buffer against pattern, masking SN/DST/SRC
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
 * wait_rx — Sleep(1)-based RX polling with timeout (ms)
 *============================================================================*/
static int wait_rx(uint32_t *status, uint32_t *flen, uint32_t timeout_ms)
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

/*============================================================================
 * uart_print_f32 — format a float to N fractional digits into out[].
 *                  Returns number of characters written.
 *============================================================================*/
static int uart_print_f32(char *out, float val, int frac_digits)
{
    int pos = 0;
    int int_part;
    float frac;

    if (val < 0) { out[pos++] = '-'; val = -val; }
    int_part = (int)val;
    frac = val - (float)int_part;

    /* Integer part (simple itoa) */
    if (int_part == 0) { out[pos++] = '0'; }
    else
    {
        char tmp[12]; int tp = 0;
        while (int_part > 0) { tmp[tp++] = '0' + (int_part % 10); int_part /= 10; }
        while (tp > 0) out[pos++] = tmp[--tp];
    }
    out[pos++] = '.';

    /* Fractional digits */
    {
        int d;
        for (d = 0; d < frac_digits; d++)
        {
            frac *= 10.0f;
            int digit = (int)frac;
            out[pos++] = '0' + digit;
            frac -= (float)digit;
        }
    }
    return pos;
}

/*============================================================================
 * fusion_tag — downlink TDOA receiver + solver
 *============================================================================*/
int fusion_tag(void)
{
    int timeout;

    test_run_info((unsigned char *)APP_NAME);

    /*--- DW3000 init (same as fusion tag v1.2) ---*/
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
    port_rf_switch_rx();

    /*=== Main downlink receiver loop ===*/
    while (1)
    {
        uint32_t status_reg, frame_len;
        uint8_t  cycle_seq = 0;
        int      num_blinks = 0;
        int      i;

        /* Phase 1: RX Downlink Beacon (0xF0, 200ms timeout).
         * Master broadcasts every ~40-500ms depending on tag presence. */
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        if (!wait_rx(&status_reg, &frame_len, 200))
        {
            test_run_info((unsigned char *)"BCN TIMEOUT");
            continue;
        }
        if (frame_len > sizeof(rx_buffer))
        {
            test_run_info((unsigned char *)"BCN OVER");
            continue;
        }
        dwt_readrxdata(rx_buffer, frame_len, 0);

        if (!match_header(rx_buffer, match_beacon, HDR_LEN, 1, 0, 1))
        {
            /* Not a downlink beacon — might be stray frame, ignore */
            test_run_info((unsigned char *)"NO BCN");
            continue;
        }

        /* Extract anchor position table from beacon */
        {
            uint8_t n = rx_buffer[DL_BCN_NUM_IDX];
            if (n < 3)
            {
                test_run_info((unsigned char *)"ERR ANCHORS");
                continue;
            }
            if (n > DL_MAX_ANCHORS) n = DL_MAX_ANCHORS;
            num_anchors = n;

            /* Check for duplicate cycle */
            cycle_seq = rx_buffer[DL_BCN_CYCLE_IDX];
            if (cycle_seq == frame_seq && frame_seq != 0)
            {
                /* Duplicate beacon cycle — skip */
                continue;
            }
            frame_seq = cycle_seq;

            for (i = 0; i < (int)n; i++)
            {
                unpack_anchor_entry(
                    &rx_buffer[DL_BCN_TABLE_IDX + i * DL_ANCHOR_ENTRY_SIZE],
                    &anchor_table[i]);
            }

            /* Beacon timestamp = reference TOA (anchor 0) */
            toa_dtu[0] = get_rx_timestamp_u64();
        }

        /* Phase 2: RX Downlink Blinks (0xF1) from slave anchors.
         * Each slave transmits in its TDMA slot (rank * 1000us after beacon).
         * We listen with generous per-slot timeout to catch all slots. */
        {
            int expected = (int)num_anchors - 1;
            for (i = 0; i < expected; i++)
            {
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
                if (!wait_rx(&status_reg, &frame_len, 30))
                    continue;  /* this slot missed, try next */
                if (frame_len > sizeof(rx_buffer))
                    continue;
                dwt_readrxdata(rx_buffer, frame_len, 0);

                if (!match_header(rx_buffer, match_blink, HDR_LEN, 1, 0, 1))
                    continue;

                /* Rank is encoded in source address field */
                {
                    uint8_t rank = rx_buffer[7];
                    if (rank > 0 && rank < DL_MAX_ANCHORS)
                    {
                        toa_dtu[rank] = get_rx_timestamp_u64();
                        num_blinks++;
                    }
                }
            }
        }

        /* Phase 3: TDOA multilateration */
        {
            float x_cm, y_cm, z_cm, rms;
            int ret;

            if ((num_blinks + 1) < 3)
            {
                char dbg[32];
                dbg[0] = 'P'; dbg[1] = ','; dbg[2] = '0';
                dbg[3] = ','; dbg[4] = 'E'; dbg[5] = 'R'; dbg[6] = 'R';
                dbg[7] = '='; dbg[8] = 'A'; dbg[9] = 'N'; dbg[10] = 'C';
                dbg[11] = ' '; dbg[12] = hex[num_blinks & 0xF];
                dbg[13] = '\0';
                test_run_info((unsigned char *)dbg);
                continue;
            }

            ret = tdoa_solve(anchor_table, toa_dtu, num_anchors,
                             &x_cm, &y_cm, &z_cm, &rms);

            /* Phase 4: UART output */
            if (ret == 0)
            {
                /* P,<seq>,X=<cm>,Y=<cm>,Z=<cm>,Q=<rms> */
                char out[80];
                int pos = 0;
                out[pos++] = 'P'; out[pos++] = ',';

                /* seq — 2 hex digits */
                out[pos++] = hex[(frame_seq >> 4) & 0xF];
                out[pos++] = hex[ frame_seq       & 0xF];

                out[pos++] = ','; out[pos++] = 'X'; out[pos++] = '=';
                pos += uart_print_f32(&out[pos], x_cm, 1);

                out[pos++] = ','; out[pos++] = 'Y'; out[pos++] = '=';
                pos += uart_print_f32(&out[pos], y_cm, 1);

                out[pos++] = ','; out[pos++] = 'Z'; out[pos++] = '=';
                pos += uart_print_f32(&out[pos], z_cm, 1);

                out[pos++] = ','; out[pos++] = 'Q'; out[pos++] = '=';
                pos += uart_print_f32(&out[pos], rms, 1);

                out[pos] = '\0';
                test_run_info((unsigned char *)out);
            }
            else if (ret == -2)
            {
                test_run_info((unsigned char *)"ERR ANCHORS");
            }
            else
            {
                test_run_info((unsigned char *)"ERR DIVERGE");
            }
        }
    }
}
#endif /* TEST_FUSION_TAG */
