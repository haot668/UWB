/*! ----------------------------------------------------------------------------
 * @file    downlink_tag.c
 * @brief   DOWNlink TDOA Tag — listens to anchor transmissions, computes position
 *
 *          Protocol (per 100ms cycle):
 *            1. RX Master Beacon (0xF0) — extract anchor positions + record TOA
 *            2. RX N-1 Slave Blinks (0xF1) — record TOA for each
 *            3. TDOA solve → position (x,y,z) in cm
 *            4. UART output to robot
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
#include "downlink_protocol.h"

#if defined(TEST_DOWNLINK_TAG)

extern void test_run_info(unsigned char *data);

#define APP_NAME "DOWNLINK TAG v1.0"

/*============================================================================
 * PHY config — Channel 5, 6.8 Mbps (same as fusion anchors for compatibility)
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

/*============================================================================
 * RX match patterns (10-byte IEEE header: FC[2] SEQ PAN[2] DST[2] SRC[2] FUNC)
 *============================================================================*/
static uint8_t match_beacon[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0, 0, 0xF0};
static uint8_t match_blink[]  = {0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0, 0, 0xF1};
#define HDR_LEN 10u
#define SN_IDX   2u

/*============================================================================
 * RX buffer
 *============================================================================*/
#define RX_BUF_LEN 128u
static uint8_t rx_buffer[RX_BUF_LEN];

/*============================================================================
 * Anchor position table (populated from beacon)
 *============================================================================*/
static dl_anchor_entry_t anchor_table[DL_MAX_ANCHORS];
static uint8_t  num_anchors = 0;
static uint8_t  cycle_seq = 0;

/*============================================================================
 * TOA timestamps (in DWT time units = dtu)
 *============================================================================*/
static uint64_t toa_dtu[DL_MAX_ANCHORS];

/*============================================================================
 * UART helpers
 *============================================================================*/
static const char hex_chars[] = "0123456789ABCDEF";

static void uart_tag_line(uint8_t seq, float x, float y, float z, uint8_t q, float rms)
{
    char out[64];
    int i, pos = 0;
    int ix, iy, iz, irms;

    ix = (int)(x + (x >= 0 ? 0.5f : -0.5f));
    iy = (int)(y + (y >= 0 ? 0.5f : -0.5f));
    iz = (int)(z + (z >= 0 ? 0.5f : -0.5f));
    irms = (int)(rms + 0.5f);

    out[pos++] = 'P'; out[pos++] = ',';
    out[pos++] = hex_chars[(seq >> 4) & 0xF];
    out[pos++] = hex_chars[seq & 0xF];
    out[pos++] = ',';

    /* X */
    out[pos++] = 'X'; out[pos++] = '=';
    if (ix < 0) { out[pos++] = '-'; ix = -ix; }
    else        { out[pos++] = '+'; }
    for (i = 3; i >= 0; i--) out[pos++] = hex_chars[((ix >> (i*4)) & 0xF)];
    out[pos++] = ',';

    /* Y */
    out[pos++] = 'Y'; out[pos++] = '=';
    if (iy < 0) { out[pos++] = '-'; iy = -iy; }
    else        { out[pos++] = '+'; }
    for (i = 3; i >= 0; i--) out[pos++] = hex_chars[((iy >> (i*4)) & 0xF)];
    out[pos++] = ',';

    /* Z */
    out[pos++] = 'Z'; out[pos++] = '=';
    if (iz < 0) { out[pos++] = '-'; iz = -iz; }
    else        { out[pos++] = '+'; }
    for (i = 3; i >= 0; i--) out[pos++] = hex_chars[((iz >> (i*4)) & 0xF)];
    out[pos++] = ',';

    /* Quality */
    out[pos++] = 'Q'; out[pos++] = '=';
    out[pos++] = hex_chars[q & 0xF];
    out[pos++] = ',';

    /* RMS error */
    out[pos++] = 'D'; out[pos++] = '=';
    for (i = 2; i >= 0; i--) out[pos++] = hex_chars[((irms >> (i*4)) & 0xF)];

    out[pos] = '\0';
    test_run_info((unsigned char *)out);
}

static void uart_tag_err(uint8_t seq, const char *err, uint8_t avail)
{
    char out[48];
    int pos = 0;
    out[pos++] = 'P'; out[pos++] = ',';
    out[pos++] = hex_chars[(seq >> 4) & 0xF];
    out[pos++] = hex_chars[seq & 0xF];
    out[pos++] = ',';

    while (*err) out[pos++] = *err++;

    if (avail > 0)
    {
        out[pos++] = ','; out[pos++] = 'A'; out[pos++] = 'V'; out[pos++] = 'A';
        out[pos++] = 'I'; out[pos++] = 'L'; out[pos++] = '=';
        out[pos++] = hex_chars[avail & 0xF];
    }

    out[pos] = '\0';
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
 * Helper: wait for RX with timeout (uses Sleep-based polling)
 *============================================================================*/
static int wait_rx(uint32_t *status, uint32_t *flen, uint32_t timeout_ms)
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
 * tag_init — DW3000 init sequence
 *============================================================================*/
static void tag_init(void)
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

    port_rf_switch_init();
    port_rf_switch_rx();

    test_run_info((unsigned char *)"INIT DONE");
}

/*============================================================================
 * downlink_tag — main entry
 *============================================================================*/
int downlink_tag(void)
{
    uint8_t seq = 0;

    tag_init();

    while (1)
    {
        int got_beacon = 0;
        int num_blinks = 0;
        uint32_t status_reg, frame_len;
        int i;

        /*=== Phase 1: RX Master Beacon (200ms timeout) ===*/
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        if (!wait_rx(&status_reg, &frame_len, 200))
        {
            test_run_info((unsigned char *)"BCN TIMEOUT");
            continue;
        }

        if (frame_len > sizeof(rx_buffer))
        {
            test_run_info((unsigned char *)"BCN OVERSIZE");
            continue;
        }

        dwt_readrxdata(rx_buffer, frame_len, 0);

        if (match_header(rx_buffer, match_beacon, HDR_LEN, 1, 0, 1)
            && frame_len >= DL_BCN_TABLE_IDX)
        {
            num_anchors = rx_buffer[DL_BCN_NUM_IDX];
            if (num_anchors > DL_MAX_ANCHORS)
                num_anchors = DL_MAX_ANCHORS;
            cycle_seq = rx_buffer[DL_BCN_CYCLE_IDX];

            /* Skip duplicate cycle */
            if (cycle_seq == seq && seq != 0)
            {
                continue;
            }
            seq = cycle_seq;

            /* Parse anchor table */
            for (i = 0; i < (int)num_anchors; i++)
            {
                unpack_anchor_entry(
                    &rx_buffer[DL_BCN_TABLE_IDX + i * DL_ANCHOR_ENTRY_SIZE],
                    &anchor_table[i]);
            }

            /* Record beacon TOA (anchor 0) */
            toa_dtu[0] = get_rx_timestamp_u64();
            got_beacon = 1;
        }

        if (!got_beacon)
        {
            test_run_info((unsigned char *)"BCN MISMATCH");
            continue;
        }

        /*=== Phase 2: RX Slave Blinks (N-1 frames, each with 2000us timeout) ===*/
        /* Use SRC field (= anchor rank) to map TOA → correct anchor index,
         * so missed blinks don't corrupt the TDOA assignment. */
        for (i = 1; i < (int)num_anchors; i++)
        {
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            if (!wait_rx(&status_reg, &frame_len, 2))  /* 2ms timeout per slot */
            {
                continue;
            }

            if (frame_len > sizeof(rx_buffer))
                continue;

            dwt_readrxdata(rx_buffer, frame_len, 0);

            if (match_header(rx_buffer, match_blink, HDR_LEN, 1, 0, 1))
            {
                uint8_t rank = rx_buffer[7];
                if (rank > 0 && rank < num_anchors)
                {
                    toa_dtu[rank] = get_rx_timestamp_u64();
                    num_blinks++;
                }
            }
        }

        /*=== Phase 3: Solve position ===*/
        if ((num_blinks + 1) < 3)  /* need at least 3 total anchors for 2D fix */
        {
            uart_tag_err(seq, "ERR=ANCHORS", (uint8_t)(num_blinks + 1));
            continue;
        }

        {
            float x_cm, y_cm, z_cm, rms;
            int ret;

            ret = tdoa_solve(anchor_table, toa_dtu, num_anchors,
                             &x_cm, &y_cm, &z_cm, &rms);

            if (ret == 0)
            {
                uart_tag_line(seq, x_cm, y_cm, z_cm, num_anchors, rms);
            }
            else if (ret == -2)
            {
                uart_tag_err(seq, "ERR=ANCHORS", num_anchors);
            }
            else
            {
                uart_tag_err(seq, "ERR=DIVERGE", 0);
            }
        }
    }

    return 0;
}

#endif /* TEST_DOWNLINK_TAG */
