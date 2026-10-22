/*! ----------------------------------------------------------------------------
 * @file    test_warm_reset.c
 * @brief   Minimal DW3000 warm-reset test — isolated from all protocol logic.
 *
 *          Test sequence:
 *            1. Cold-boot init (anchor_init style)
 *            2. Enter RX mode for 2 seconds (simulates slave heartbeat RX)
 *            3. Warm reset: forcetrxoff + reset_DWIC + wait IDLE
 *            4. Report PASS or FAIL
 *            5. Repeat without forcetrxoff (test alternate reset path)
 *
 *          Run this on BOTH boards to compare behaviour.
 *============================================================================*/

#include <deca_device_api.h>
#include <deca_regs.h>
#include <deca_spi.h>
#include <port.h>
#include <shared_defines.h>
#include <shared_functions.h>
#include <example_selection.h>

extern void test_run_info(unsigned char *data);

/* Same PHY config as fusion anchor */
static dwt_config_t config = {
    5,               /* Channel number. */
    DWT_PLEN_128,
    DWT_PAC8,
    9,               /* TX preamble code. */
    9,               /* RX preamble code. */
    1,               /* Non-standard 8 symbol SFD */
    DWT_BR_6M8,
    DWT_PHRMODE_STD,
    DWT_PHRRATE_STD,
    (129 + 8 - 8),   /* SFD timeout */
    DWT_STS_MODE_OFF,
    DWT_STS_LEN_64,
    DWT_PDOA_M0
};

#define RX_ANT_DLY 16385
#define TX_ANT_DLY 16385

/*============================================================================
 * warm_reset_once — try one warm-reset cycle, return 0 on success
 *============================================================================*/
static int warm_reset_once(int use_forcetrxoff)
{
    int timeout;

    if (use_forcetrxoff)
    {
        test_run_info((unsigned char *)"  forcetrxoff...");
        dwt_forcetrxoff();
    }
    else
    {
        test_run_info((unsigned char *)"  skip forcetrxoff...");
    }

    reset_DWIC();
    Sleep(100);

    timeout = 1000;
    while (!dwt_checkidlerc() && --timeout > 0)
    {
        Sleep(1);
    }

    if (timeout > 0)
    {
        test_run_info((unsigned char *)"  -> PASS");
        return 0;
    }
    else
    {
        test_run_info((unsigned char *)"  -> FAIL (IDLE TIMEOUT)");
        return -1;
    }
}

/*============================================================================
 * test_warm_reset — standalone entry point
 *============================================================================*/
int test_warm_reset(void)
{
    test_run_info((unsigned char *)"===== WARM RESET TEST =====");

    /*--- 1. Cold-boot init ---*/
    port_set_dw_ic_spi_fastrate();
    reset_DWIC();
    Sleep(100);

    if (!dwt_checkidlerc())
    {
        test_run_info((unsigned char *)"COLD IDLE FAIL — abort");
        while (1) { }
    }
    test_run_info((unsigned char *)"COLD IDLE OK");

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
    {
        test_run_info((unsigned char *)"INIT FAIL — abort");
        while (1) { }
    }
    test_run_info((unsigned char *)"INIT OK");

    if (dwt_configure(&config))
    {
        test_run_info((unsigned char *)"CONFIG FAIL — abort");
        while (1) { }
    }
    test_run_info((unsigned char *)"CONFIG OK");

    dwt_configuretxrf(&txconfig_options);
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
    dwt_setfinegraintxseq(0);

    /*--- 2. Enter RX (simulate slave heartbeat RX state) ---*/
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    test_run_info((unsigned char *)"RX ON — wait 3s...");
    Sleep(3000);

    /*--- 3. Test A: warm reset WITH forcetrxoff ---*/
    test_run_info((unsigned char *)"TEST A: with forcetrxoff");
    warm_reset_once(1);

    /* Re-init after reset */
    {
        int timeout;
        port_set_dw_ic_spi_fastrate();
        reset_DWIC();
        Sleep(100);
        timeout = 1000;
        while (!dwt_checkidlerc() && --timeout > 0) { Sleep(1); }
        if (timeout <= 0)
        {
            test_run_info((unsigned char *)"RE-INIT FAIL — stuck");
            while (1) { }
        }
        dwt_initialise(DWT_DW_INIT);
        dwt_configure(&config);
        dwt_configuretxrf(&txconfig_options);
        dwt_setrxantennadelay(RX_ANT_DLY);
        dwt_settxantennadelay(TX_ANT_DLY);
        dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
        dwt_setfinegraintxseq(0);
    }

    /*--- 4. Enter RX again ---*/
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    test_run_info((unsigned char *)"RX ON — wait 3s...");
    Sleep(3000);

    /*--- 5. Test B: warm reset WITHOUT forcetrxoff ---*/
    test_run_info((unsigned char *)"TEST B: without forcetrxoff");
    warm_reset_once(0);

    /*--- 6. Done ---*/
    test_run_info((unsigned char *)"===== TEST DONE =====");
    test_run_info((unsigned char *)"Check PASS/FAIL above.");
    while (1) { }
    return 0;
}
