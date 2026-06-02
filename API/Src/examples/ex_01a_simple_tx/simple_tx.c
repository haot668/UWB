/*! ----------------------------------------------------------------------------
 *  @file    simple_tx.c
 *  @brief   Simple TX — Channel 9, 850Kbps, PLEN_1024
 */

#include <deca_device_api.h>
#include <deca_regs.h>
#include <deca_vals.h>
#include <deca_spi.h>
#include <port.h>
#include <shared_defines.h>
#include <example_selection.h>


#if defined(TEST_SIMPLE_TX)

extern void test_run_info(unsigned char *data);

#define APP_NAME "SIMPLE TX v4.0"

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

static uint8_t tx_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
#define BLINK_FRAME_SN_IDX 1
#define FRAME_LENGTH    (sizeof(tx_msg) + FCS_LEN)
#define TX_DELAY_MS 200

static dwt_txconfig_t max_power = { 0x34, 0xfefefefe, 0x0 };

int simple_tx(void)
{
    uint32_t t;

    test_run_info((unsigned char *)APP_NAME);

    port_set_dw_ic_spi_slowrate();
    test_run_info((unsigned char *)"SPI OK");

    reset_DWIC();
    Sleep(50);

    t = 0;
    while (!dwt_checkidlerc())
    {
        if (++t > 1000000) {
            test_run_info((unsigned char *)"IDLE T/O");
            while (1) { };
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

    dwt_configuretxrf(&max_power);
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
    port_rf_switch_init();

    dwt_write32bitoffsetreg(RF_SWITCH_CTRL_ID, 0, TXRXSWITCH_AUTO);

    test_run_info((unsigned char *)"INIT DONE");

    while (1)
    {
        t = 0;
        while (!dwt_checkidlerc())
        {
            if (++t > 1000000) {
                test_run_info((unsigned char *)"!IDLE");
                t = 0;
                Sleep(10);
            }
        }

        port_rf_switch_tx();

        dwt_writetxdata(FRAME_LENGTH - FCS_LEN, tx_msg, 0);
        dwt_writetxfctrl(FRAME_LENGTH, 0, 0);

        if (dwt_starttx(DWT_START_TX_IMMEDIATE) == DWT_ERROR) {
            test_run_info((unsigned char *)"START ERR");
            continue;
        }

        t = 0;
        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK))
        {
            if (++t > 1000000) {
                test_run_info((unsigned char *)"TXFRS T/O");
                break;
            }
        };

        if (t <= 1000000) {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
            test_run_info((unsigned char *)"TX OK");
        }

        Sleep(TX_DELAY_MS);
        tx_msg[BLINK_FRAME_SN_IDX]++;
    }
}

#endif
