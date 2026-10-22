/*! ----------------------------------------------------------------------------
 *  @file    ss_twr_initiator.c
 *  @brief   Single-sided two-way ranging (SS TWR) initiator example code
 *
 *           This is a simple code example which acts as the initiator in a SS TWR distance measurement exchange. This application sends a "poll"
 *           frame (recording the TX time-stamp of the poll), after which it waits for a "response" message from the "DS TWR responder" example
 *           code (companion to this application) to complete the exchange. The response message contains the remote responder's time-stamps of poll
 *           RX, and response TX. With this data and the local time-stamps, (of poll TX and response RX), this example application works out a value
 *           for the time-of-flight over-the-air and, thus, the estimated distance between the two devices, which it writes to the LCD.
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
#include <config_options.h>

#if defined(TEST_SS_TWR_INITIATOR)

extern void test_run_info(unsigned char *data);

/* Example application name */
#define APP_NAME "SS TWR INIT v1.1"

/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
        9,               /* Channel number. */
        DWT_PLEN_128,    /* Preamble length. Used in TX only. */
        DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
        9,               /* TX preamble code. Used in TX only. */
        9,               /* RX preamble code. Used in RX only. */
        1,               /* 0 = standard 8 symbol SFD, 1 = non-standard 8 symbol */
        DWT_BR_6M8,      /* Data rate. */
        DWT_PHRMODE_STD, /* PHY header mode. */
        DWT_PHRRATE_STD, /* PHY header rate. */
        (129 + 8 - 8),   /* SFD timeout (preamble length + 1 + SFD length - PAC size). */
        DWT_STS_MODE_OFF, /* STS disabled */
        DWT_STS_LEN_64,  /* STS length */
        DWT_PDOA_M0      /* PDOA mode off */
};

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 1000

/* Default antenna delay values for 64 MHz PRF. See NOTE 2 below. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

/* Frames used in the ranging process. See NOTE 3 below. */
static uint8_t tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
static uint8_t rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
/* Length of the common part of the message (up to and including the function code, see NOTE 3 below). */
#define ALL_MSG_COMMON_LEN 10
/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4
/* Frame sequence number, incremented after each transmission. */
static uint8_t frame_seq_nb = 0;

/* Buffer to store received response message. */
#define RX_BUF_LEN 20
static uint8_t rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32_t status_reg = 0;

/* Delay between frames, in UWB microseconds. See NOTE 1 below. */
#ifdef STM32L412xx
#define POLL_TX_TO_RESP_RX_DLY_UUS 700
#endif
/* Receive response timeout. See NOTE 5 below. */
#ifdef STM32L412xx
#define RESP_RX_TIMEOUT_UUS 2600
#endif

/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current
 * temperature. These values can be calibrated prior to taking reference measurements. See NOTE 2 below. */
extern dwt_txconfig_t txconfig_options;

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn main()
 *
 * @brief Application entry point.
 *
 * @param  none
 *
 * @return none
 */
int ss_twr_initiator(void)
{
    /* Display application name on LCD. */
    test_run_info((unsigned char *)APP_NAME);

    /* Configure SPI rate, DW3000 supports up to 38 MHz */
    port_set_dw_ic_spi_fastrate();

    /* Reset and initialize DW chip. */
    reset_DWIC(); /* Target specific drive of RSTn line into DW3000 low for a period. */

    Sleep(10); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC)

    while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before proceeding */
    { };
    test_run_info((unsigned char *)"IDLE OK");

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
    {
        test_run_info((unsigned char *)"INIT FAILED");
        while (1)
        { };
    }
    test_run_info((unsigned char *)"INIT OK");

    /* Configure DW IC. See NOTE 13 below. */
    if(dwt_configure(&config)) /* if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device */
    {
        test_run_info((unsigned char *)"CONFIG FAILED");
        while (1)
        { };
    }
    test_run_info((unsigned char *)"CONFIG OK");

    /* Configure the TX spectrum parameters (power, PG delay and PG count) */
    dwt_configuretxrf(&txconfig_options);

    /* Apply default antenna delay value. See NOTE 2 below. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    /* Next can enable TX/RX states output on GPIOs 5 and 6 to help debug, and also TX/RX LEDs
     * Note, in real low power applications the LEDs should not be used.
     * Note, before enabling PA functionality, fine grain TX sequencing must be deactivated. */
    dwt_setfinegraintxseq(0);
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    test_run_info((unsigned char *)"INIT DONE");

    /* Init RF switch (defaults to RX path) */
    port_rf_switch_init();

    /* Loop forever initiating ranging exchanges. */
    while (1)
    {
        uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
        int32_t rtd_init, rtd_resp;
        float clockOffsetRatio;
        double tof;
        double distance;

        /* Write frame data to DW IC and prepare transmission. See NOTE 7 below. */
        tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
        dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
        dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1); /* Zero offset, ranging. */

        /* Switch RF to TX and start transmission immediately.
         * NOTE: we use manual RF switching (not DWT_RESPONSE_EXPECTED) because
         * this hardware has an external RF switch (SW8112QU4) controlled by MCU
         * GPIOs that must be toggled between TX and RX paths. */
        port_rf_switch_tx();
        dwt_starttx(DWT_START_TX_IMMEDIATE);

        /* Wait for TX completion */
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & SYS_STATUS_TXFRS_BIT_MASK))
        { };

        /* Read poll TX timestamp. See NOTE 9 below. */
        poll_tx_ts = dwt_readtxtimestamplo32();

        /* Increment frame sequence number after transmission (modulo 256). */
        frame_seq_nb++;

        /* Clear status, force to IDLE, switch RF to RX, enable reception. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_MASK);
        dwt_forcetrxoff();
        port_rf_switch_rx();
        dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        test_run_info((unsigned char *)"POLL");

        /* Poll for reception of a frame or error/timeout. See NOTE 8 below. */
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID))
               & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
        { };

        if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
        {
            uint32_t frame_len;

            /* Clear good RX frame event in the DW IC status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

            /* A frame has been received, read it into the local buffer. */
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;

            if (frame_len <= sizeof(rx_buffer))
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);

                /* Debug: print frame_len AFTER read (to avoid UART delay corrupting buffer) */
                {
                    char fl[16];
                    fl[0] = 'L'; fl[1] = '=';
                    fl[2] = "0123456789ABCDEF"[(frame_len >> 4) & 0xF];
                    fl[3] = "0123456789ABCDEF"[frame_len & 0xF];
                    fl[4] = 0;
                    test_run_info((unsigned char *)fl);
                }

                /* Dump raw RX buffer — header + timestamp payload */
                {
                    char hx[48];
                    int i;
                    /* Bytes 0-9: header */
                    hx[0] = 'B'; hx[1] = ':';
                    for (i = 0; i < 10; i++) {
                        uint8_t b = rx_buffer[i];
                        hx[2+i*2] = "0123456789ABCDEF"[b >> 4];
                        hx[3+i*2] = "0123456789ABCDEF"[b & 0xF];
                    }
                    hx[22] = '\0';
                    test_run_info((unsigned char *)hx);
                    /* Bytes 10-17: timestamp payload */
                    hx[0] = 'T'; hx[1] = ':';
                    for (i = 0; i < 8; i++) {
                        uint8_t b = rx_buffer[10+i];
                        hx[2+i*2] = "0123456789ABCDEF"[b >> 4];
                        hx[3+i*2] = "0123456789ABCDEF"[b & 0xF];
                    }
                    hx[18] = '\0';
                    test_run_info((unsigned char *)hx);

                    /* Bytes 16-19: tail + FCS */
                    hx[0] = 'F'; hx[1] = ':';
                    for (i = 0; i < 4; i++) {
                        uint8_t b = rx_buffer[16+i];
                        hx[2+i*2] = "0123456789ABCDEF"[b >> 4];
                        hx[3+i*2] = "0123456789ABCDEF"[b & 0xF];
                    }
                    hx[10] = '\0';
                    test_run_info((unsigned char *)hx);
                }

                /* Check that the frame is the expected response from the companion "SS TWR responder" example.
                 * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation. */
                rx_buffer[ALL_MSG_SN_IDX] = 0;
                if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
                {
                    /* Retrieve response reception timestamp. */
                    resp_rx_ts = dwt_readrxtimestamplo32();

                    /* Read carrier integrator value and calculate clock offset ratio. See NOTE 11 below. */
                    clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1<<26);

                    /* Get timestamps embedded in response message. */
                    resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
                    resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

                    /* Compute time of flight and distance, using clock offset ratio
                     * to correct for differing local and remote clock rates */
                    rtd_init = resp_rx_ts - poll_tx_ts;
                    rtd_resp = resp_tx_ts - poll_rx_ts;

                    /* Diagnostics: dump all raw timestamps */
                    {
                        char ts[48];
                        int i;
                        ts[0] = 'P'; ts[1] = 'T'; ts[2] = '='; /* poll_tx_ts */
                        for (i = 0; i < 8; i++)
                            ts[3+i] = "0123456789ABCDEF"[(poll_tx_ts >> (28 - i*4)) & 0xF];
                        ts[11] = ' '; ts[12] = 'R'; ts[13] = 'R'; ts[14] = '='; /* resp_rx_ts */
                        for (i = 0; i < 8; i++)
                            ts[15+i] = "0123456789ABCDEF"[(resp_rx_ts >> (28 - i*4)) & 0xF];
                        ts[23] = '\0';
                        test_run_info((unsigned char *)ts);
                        ts[0] = 'p'; ts[1] = 'r'; ts[2] = '='; /* poll_rx_ts from responder */
                        for (i = 0; i < 8; i++)
                            ts[3+i] = "0123456789ABCDEF"[(poll_rx_ts >> (28 - i*4)) & 0xF];
                        ts[11] = ' '; ts[12] = 's'; ts[13] = 't'; ts[14] = '='; /* resp_tx_ts from responder */
                        for (i = 0; i < 8; i++)
                            ts[15+i] = "0123456789ABCDEF"[(resp_tx_ts >> (28 - i*4)) & 0xF];
                        ts[23] = '\0';
                        test_run_info((unsigned char *)ts);
                    }

                    tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
                    distance = tof * SPEED_OF_LIGHT;

                    /* Display computed distance on LCD. */
                    snprintf(dist_str, sizeof(dist_str), "DIST: %3.2f m", distance);
                    test_run_info((unsigned char *)dist_str);
                }
                else
                {
                    test_run_info((unsigned char *)"BAD FRAME");
                }
            }
        }
        else
        {
            /* Show error status for diagnostics */
            {
                char info[20];
                int i;
                info[0] = 'E'; info[1] = ':';
                for (i = 0; i < 8; i++) {
                    uint8_t nib = (status_reg >> (28 - i*4)) & 0xF;
                    info[2+i] = (nib < 10) ? ('0' + nib) : ('A' + nib - 10);
                }
                info[10] = '\0';
                test_run_info((unsigned char *)info);
            }

            /* Clear RX error/timeout events in the DW IC status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        }

        /* Execute a delay between ranging exchanges. */
        Sleep(RNG_DELAY_MS);
    }
}
#endif
/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. The single-sided two-way ranging scheme implemented here has to be considered carefully as the accuracy of the distance measured is highly
 *    sensitive to the clock offset error between the devices and the length of the response delay between frames. To achieve the best possible
 *    accuracy, this response delay must be kept as low as possible. In order to do so, 6.8 Mbps data rate is used in this example and the response
 *    delay between frames is defined as low as possible. The user is referred to User Manual for more details about the single-sided two-way ranging
 *    process.  NB:SEE ALSO NOTE 11.
 *
 *    Initiator: |Poll TX| ..... |Resp RX|
 *    Responder: |Poll RX| ..... |Resp TX|
 *                   ^|P RMARKER|                    - time of Poll TX/RX
 *                                   ^|R RMARKER|    - time of Resp TX/RX
 *
 *                       <--TDLY->                   - POLL_TX_TO_RESP_RX_DLY_UUS (RDLY-RLEN)
 *                               <-RLEN->            - RESP_RX_TIMEOUT_UUS   (length of response frame)
 *                    <----RDLY------>               - POLL_RX_TO_RESP_TX_DLY_UUS (depends on how quickly responder can turn around and reply)
 *
 *
 * 2. The sum of the values is the TX to RX antenna delay, this should be experimentally determined by a calibration process. Here we use a hard coded
 *    value (expected to be a little low so a positive error will be seen on the resultant distance estimate). For a real production application, each
 *    device should have its own antenna delay properly calibrated to get good precision when performing range measurements.
 * 3. The frames used here are Decawave specific ranging frames, complying with the IEEE 802.15.4 standard data frame encoding. The frames are the
 *    following:
 *     - a poll message sent by the initiator to trigger the ranging exchange.
 *     - a response message sent by the responder to complete the exchange and provide all information needed by the initiator to compute the
 *       time-of-flight (distance) estimate.
 *    The first 10 bytes of those frame are common and are composed of the following fields:
 *     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: PAN ID (0xDECA).
 *     - byte 5/6: destination address, see NOTE 4 below.
 *     - byte 7/8: source address, see NOTE 4 below.
 *     - byte 9: function code (specific values to indicate which message it is in the ranging process).
 *    The remaining bytes are specific to each message as follows:
 *    Poll message:
 *     - no more data
 *    Response message:
 *     - byte 10 -> 13: poll message reception timestamp.
 *     - byte 14 -> 17: response message transmission timestamp.
 *    All messages end with a 2-byte checksum automatically set by DW IC.
 * 4. Source and destination addresses are hard coded constants in this example to keep it simple but for a real product every device should have a
 *    unique ID. Here, 16-bit addressing is used to keep the messages as short as possible but, in an actual application, this should be done only
 *    after an exchange of specific messages used to define those short addresses for each device participating to the ranging exchange.
 * 5. This timeout is for complete reception of a frame, i.e. timeout duration must take into account the length of the expected frame. Here the value
 *    is arbitrary but chosen large enough to make sure that there is enough time to receive the complete response frame sent by the responder at the
 *    6.8M data rate used (around 200 us).
 * 6. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW IC OTP memory.
 * 7. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
 *    automatically appended by the DW IC. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
 *    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
 * 8. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
 *    refer to DW IC User Manual for more details on "interrupts". It is also to be noted that STATUS register is 5 bytes long but, as the event we
 *    use are all in the first bytes of the register, we can use the simple dwt_read32bitreg() API call to access it instead of reading the whole 5
 *    bytes.
 * 9. The high order byte of each 40-bit time-stamps is discarded here. This is acceptable as, on each device, those time-stamps are not separated by
 *    more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays can be handled by a 32-bit
 *    subtraction.
 * 10. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
 *     DW IC API Guide for more details on the DW IC driver functions.
 * 11. The use of the clock offset value to correct the TOF calculation, significantly improves the result of the SS-TWR where the remote
 *     responder unit's clock is a number of PPM offset from the local initiator unit's clock.
 *     As stated in NOTE 2 a fixed offset in range will be seen unless the antenna delay is calibrated and set correctly.
 * 12. In this example, the DW IC is put into IDLE state after calling dwt_initialise(). This means that a fast SPI rate of up to 20 MHz can be used
 *     thereafter.
 * 13. Desired configuration by user may be different to the current programmed configuration. dwt_configure is called to set desired
 *     configuration.
 ****************************************************************************************************************************************************/
