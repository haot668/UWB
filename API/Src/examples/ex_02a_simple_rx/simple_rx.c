/*! ----------------------------------------------------------------------------
 *  @file    simple_rx.c
 *  @brief   Simple RX — Channel 9, 850Kbps, PLEN_1024
 */

#include <deca_device_api.h>
#include <deca_regs.h>
#include <deca_vals.h>
#include <deca_spi.h>
#include <port.h>
#include <example_selection.h>
#include <shared_defines.h>


#if defined(TEST_SIMPLE_RX)

extern void test_run_info(unsigned char *data);

#define APP_NAME "SIMPLE RX v9.0"

static dwt_config_t config = {
    9,               /* Channel 9 = 7987.2 MHz */
    DWT_PLEN_1024,   /* Preamble length — max for noise immunity. */
    DWT_PAC8,        /* Preamble acquisition chunk size. */
    9,               /* TX preamble code. */
    9,               /* RX preamble code. */
    1,               /* 1 = non-standard 8 symbol SFD */
    DWT_BR_850K,     /* 850 Kbps — lower rate for better range. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    DWT_PHRRATE_STD, /* PHY header rate. */
    (1024 + 1 + 8 - 8), /* SFD timeout */
    DWT_STS_MODE_OFF,
    DWT_STS_LEN_64,
    DWT_PDOA_M0
};

static uint8_t rx_buffer[FRAME_LEN_MAX];

static char hex_nib(uint8_t n)
{
    return (n < 10) ? ('0' + n) : ('A' + n - 10);
}

int simple_rx(void)
{
    uint32_t status_reg;
    uint16_t frame_len;
    uint32_t noise_count = 0;

    test_run_info((unsigned char *)APP_NAME);

    port_set_dw_ic_spi_slowrate();
    reset_DWIC();
    Sleep(50);

    {
        uint32_t t = 0;
        while (!dwt_checkidlerc()) {
            if (++t > 1000000) {
                test_run_info((unsigned char *)"IDLE T/O");
                while (1) { };
            }
        }
    }
    test_run_info((unsigned char *)"IDLE OK");

    if (dwt_check_dev_id() != DWT_SUCCESS) {
        test_run_info((unsigned char *)"DEV ID ERR");
        while (1) { };
    }
    test_run_info((unsigned char *)"DEV ID OK");

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
        test_run_info((unsigned char *)"INIT ERR");
        while (1) { };
    }
    test_run_info((unsigned char *)"INIT OK");

    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

    if (dwt_configure(&config)) {
        test_run_info((unsigned char *)"CONFIG ERR");
        while (1) { };
    }

    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
    port_rf_switch_init();
    dwt_write32bitoffsetreg(RF_SWITCH_CTRL_ID, 0, TXRXSWITCH_AUTO);

    /*
     * Write known pattern to TX buffer, then read back via FACRW
     * to verify that buffer file addressing works.
     * TX_BUFFER_ID = 0x100000 → reg_file=0x10, reg_offset=0 → FACRW
     * Expected: AB CD 12 34 → uint32 LE = 0x3412CDAB
     */
    {
        uint8_t  pat[] = {0xAB, 0xCD, 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC};
        uint32_t r0, r4;
        char info[40];
        dwt_writetxdata(8, pat, 0);
        r0 = dwt_read32bitoffsetreg(TX_BUFFER_ID, 0);
        r4 = dwt_read32bitoffsetreg(TX_BUFFER_ID, 4);
        info[0] = 'T'; info[1] = '0'; info[2] = '=';
        info[3] = hex_nib((r0 >> 28) & 0xF);
        info[4] = hex_nib((r0 >> 24) & 0xF);
        info[5] = hex_nib((r0 >> 20) & 0xF);
        info[6] = hex_nib((r0 >> 16) & 0xF);
        info[7] = hex_nib((r0 >> 12) & 0xF);
        info[8] = hex_nib((r0 >> 8) & 0xF);
        info[9] = hex_nib((r0 >> 4) & 0xF);
        info[10] = hex_nib(r0 & 0xF);
        info[11] = ' '; info[12] = 'T'; info[13] = '4'; info[14] = '=';
        info[15] = hex_nib((r4 >> 28) & 0xF);
        info[16] = hex_nib((r4 >> 24) & 0xF);
        info[17] = hex_nib((r4 >> 20) & 0xF);
        info[18] = hex_nib((r4 >> 16) & 0xF);
        info[19] = hex_nib((r4 >> 12) & 0xF);
        info[20] = hex_nib((r4 >> 8) & 0xF);
        info[21] = hex_nib((r4 >> 4) & 0xF);
        info[22] = hex_nib(r4 & 0xF);
        info[23] = 0;
        test_run_info((unsigned char *)info);
    }

    test_run_info((unsigned char *)"INIT DONE");

    while (1)
    {
        uint16_t mi;
        for (mi = 0; mi < sizeof(rx_buffer); mi++)
            rx_buffer[mi] = 0;

        dwt_write32bitreg(SYS_STATUS_ID, 0xFFFFFFFF);

        port_rf_switch_rx();
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        {
            uint32_t t = 0;
            while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID))
                   & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR)))
            {
                if (++t > 1000000) break;
            }
        }

        if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
        {
            /* FCS OK — print frame */
            uint32_t rx_finfo;
            uint16_t rd_len;
            char info[64];
            uint8_t idx, i, nprint;

            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

            rx_finfo = dwt_read32bitreg(RX_FINFO_ID);
            frame_len = rx_finfo & RX_FINFO_RXFLEN_BIT_MASK;

            rd_len = (frame_len > FRAME_LEN_MAX) ? FRAME_LEN_MAX : frame_len;
            if (rd_len >= FCS_LEN)
                dwt_readrxdata(rx_buffer, rd_len - FCS_LEN, 0);

            idx = 0;
            info[idx++] = 'G';
            info[idx++] = hex_nib((frame_len >> 4) & 0xF);
            info[idx++] = hex_nib(frame_len & 0xF);
            info[idx++] = '|';
            nprint = (uint8_t)(rd_len - FCS_LEN);
            if (nprint > 20) nprint = 20;
            for (i = 0; i < nprint; i++) {
                info[idx++] = hex_nib((rx_buffer[i] >> 4) & 0xF);
                info[idx++] = hex_nib(rx_buffer[i] & 0xF);
            }
            info[idx++] = ' ';
            info[idx++] = 'K';
            info[idx] = 0;
            test_run_info((unsigned char *)info);

            noise_count = 0;
        }
        else if (status_reg & SYS_STATUS_ALL_RX_ERR)
        {
            /* Noise frame — silently discard */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
            noise_count++;
            if ((noise_count % 100) == 0)
                test_run_info((unsigned char *)".");
        }
        else
        {
            /* Timeout — retry */
            dwt_write32bitreg(SYS_STATUS_ID, 0xFFFFFFFF);
        }
    }
}

#endif
