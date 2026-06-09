/*! ----------------------------------------------------------------------------
 *  @file    fusion_tag.c
 *  @brief   Downlink TDOA Tag — receives anchor beacons + blinks, solves position
 *
 *           Each cycle:
 *             1. RX Downlink Beacon (0xF0) → anchor position table + timing ref
 *             2. RX Downlink Blinks  (0xF1) → TDOA timestamps (one per slave)
 *             3. tdoa_solve() → Gauss-Newton multilateration (local FPU)
 *             4. UART output: P,SEQ,X=<cm>,Y=<cm>,Z=<cm>,Q=<rms>
 *             5. TX Position Report (0xF2) → gateway for +ONLINE/+POS
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
#include <math.h>
#include <deca_regs.h>
#include <deca_spi.h>
#include <port.h>
#include <string.h>
#include <shared_defines.h>
#include <shared_functions.h>
#include <example_selection.h>
#include "downlink_protocol.h"
#include "firmware_version.h"

#if defined(TEST_FUSION_TAG)

extern void test_run_info(unsigned char *data);

#define APP_NAME "DOWNLINK TAG v3.0"

/* Default communication configuration — Channel 5, 6.8 Mbps, NO STS */
static dwt_config_t config = {
    UWB_CHANNEL,     /* Channel number (5 or 9). */
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
#define DL_TAG_ID  0x0001u

/* Increase RX buffer — beacon can be up to DL_BEACON_LEN(16)=126 bytes */
#define RX_BUF_LEN  (DL_BEACON_LEN(DL_MAX_ANCHORS) + 4)

/* DW3210 DLY_RS calibration — AUTO-CALIBRATING per-slave.
 * Each DW3210 chip has slightly different hardware TX delay (±3m variation)
 * and the delay drifts with temperature / power-cycle.
 *
 * Auto-calibration: accumulates raw slot-compensated bias (T, value) over
 * CALIB_SAMPLES cycles at startup, then freezes the per-slave DLY_RS.
 * This removes power-on variation without absorbing the true TDOA signal.
 *
 * Defaults are used during the initial calibration window (~10s at 10 Hz). */
#define DLY_RS_DEFAULT_K1  15870   /* ~74.5m — initial guess for slave 1 */
#define DLY_RS_DEFAULT_K2  15655   /* ~73.5m — initial guess for slave 2 */
#define CALIB_SAMPLES      10      /* cycles to average for calibration   */
#define CALIB_OUTLIER_M    20      /* reject samples > ±20m from running avg */
#define CALIB_MAX_CYCLES   50      /* give up calibration after N total cycles */

/* Runtime DLY_RS state (frozen after calibration) */
static int64_t dly_rs_k1 = DLY_RS_DEFAULT_K1;
static int64_t dly_rs_k2 = DLY_RS_DEFAULT_K2;
static int64_t calib_sum_k1 = 0;   /* accumulated raw_bias for slave 1 */
static int64_t calib_sum_k2 = 0;   /* accumulated raw_bias for slave 2 */
static int32_t calib_cnt_k1 = 0;   /* accepted (non-outlier) samples, slave 1 */
static int32_t calib_cnt_k2 = 0;   /* accepted (non-outlier) samples, slave 2 */
static int32_t calib_total  = 0;   /* total cycles attempted (for timeout)  */
static int     calib_done   = 0;   /* 1 = calibration complete, frozen      */

static uint8_t rx_buffer[RX_BUF_LEN];

/* Downlink anchor table + TOA buffer */
static dl_anchor_entry_t anchor_table[DL_MAX_ANCHORS];
static uint64_t toa_dtu[DL_MAX_ANCHORS];
static int64_t  baseline_dtu[DL_MAX_ANCHORS];  /* master-to-anchor[k] distance in DTU */
static uint8_t  num_anchors = 0;
static uint8_t  frame_seq   = 0;

/* Header match patterns (SN, DST, SRC wildcarded) */
#define HDR_LEN 10u
#define SN_IDX   2u
static uint8_t match_beacon[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0, 0, 0xF0};
static uint8_t match_blink[]  = {0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0, 0, 0xF1};

extern dwt_txconfig_t txconfig_options;
extern dwt_txconfig_t txconfig_options_ch9;
#if UWB_CHANNEL == 9
#define TXCONFIG (&txconfig_options_ch9)
#else
#define TXCONFIG (&txconfig_options)
#endif

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

    /* Enable CIA diagnostics — improves RX reliability */
    dwt_configciadiag(DW_CIA_DIAG_LOG_ALL);

    /* Enable DW3000 GPIO clocks */
    dwt_enablegpioclocks();

    dwt_configuretxrf(TXCONFIG);
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_setfinegraintxseq(0);
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    test_run_info((unsigned char *)"INIT DONE");
    port_rf_switch_init();
    port_rf_switch_rx();

    /*=== Main downlink TDOA loop — fixed-window listen, then solve ===*/
    while (1)
    {
        uint32_t status_reg, frame_len;
        uint8_t  cycle_seq = 0;
        int      num_rx = 0;        /* total anchors collected this cycle */
        int      i, p;
        uint64_t tx_ts_dtu[DL_MAX_ANCHORS];  /* embedded TX timestamps */
        int16_t  cfo_raw[DL_MAX_ANCHORS];    /* CFO clock offset (raw, /16 = ppm) */

        for (i = 0; i < DL_MAX_ANCHORS; i++) { toa_dtu[i] = 0; tx_ts_dtu[i] = 0; cfo_raw[i] = 0; }
        num_anchors = 0;

        /*=== Fixed 500ms listening window — collect whatever arrives ===*/
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        for (p = 0; p < 250; p++)  /* 250×2ms=500ms */
        {
            Sleep(2);
            status_reg = dwt_read32bitreg(SYS_STATUS_ID);
            if (status_reg & SYS_STATUS_ALL_RX_ERR)
            { dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR); dwt_rxenable(DWT_START_RX_IMMEDIATE); continue; }
            if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK)) continue;
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
            if (frame_len > sizeof(rx_buffer)) { dwt_rxenable(DWT_START_RX_IMMEDIATE); continue; }
            dwt_readrxdata(rx_buffer, frame_len, 0);

            /* Beacon (0xF0): first one only */
            if (num_rx == 0 && match_header(rx_buffer, match_beacon, HDR_LEN, 1, 0, 1))
            {
                /* num_anchors from frame_len: DL_BEACON_LEN(N) = 19+N*4 → N=(len-19)/4 */
                uint8_t n = (uint8_t)(((int)frame_len - 19) / 4);
                if (n < 2 || n > DL_MAX_ANCHORS) n = rx_buffer[DL_BCN_NUM_IDX];
                if (n >= 2 && n <= DL_MAX_ANCHORS)
                {
                    num_rx    = 1;  /* master */
                    num_anchors = n;
                    cycle_seq = rx_buffer[DL_BCN_CYCLE_IDX];
                    /* Print: B,<n>,<seq> */
                    { char _b[16]; _b[0]='B'; _b[1]=','; _b[2]=hex[n&0xF]; _b[3]=',';
                      _b[4]=hex[(cycle_seq>>4)&0xF]; _b[5]=hex[cycle_seq&0xF];
                      _b[6]='\r'; _b[7]='\n'; _b[8]='\0';
                      test_run_info((unsigned char *)_b); }
                    if (cycle_seq != frame_seq || frame_seq == 0)
                    {
                        frame_seq = cycle_seq;
                        for (i = 0; i < (int)n; i++)
                            unpack_anchor_entry(
                                &rx_buffer[DL_BCN_TABLE_IDX + i * DL_ANCHOR_ENTRY_SIZE],
                                &anchor_table[i]);
                        /* Pre-compute baseline DTU: master→anchor[k] distance */
                        baseline_dtu[0] = 0;
                        for (i = 1; i < (int)n; i++)
                        {
                            float _bx = (float)(anchor_table[i].x_cm - anchor_table[0].x_cm);
                            float _by = (float)(anchor_table[i].y_cm - anchor_table[0].y_cm);
                            float _bz = (float)(anchor_table[i].z_cm - anchor_table[0].z_cm);
                            float _bcm = sqrtf(_bx*_bx + _by*_by + _bz*_bz);
                            baseline_dtu[i] = (int64_t)(_bcm / ((float)SPEED_OF_LIGHT * 100.0f / (499.2e6f * 128.0f)));
                        }
                    }
                    toa_dtu[0]  = get_rx_timestamp_u64();
                    tx_ts_dtu[0] = unpack_40bit_ts(&rx_buffer[DL_BCN_TX_TS_IDX]);
                    cfo_raw[0]   = dwt_readclockoffset();  /* CFO: f_tag/f_master */
                }
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
                continue;
            }

            /* Blink (0xF1): only after beacon, one per rank */
            if (num_rx >= 1
                && match_header(rx_buffer, match_blink, HDR_LEN, 1, 0, 1)
                && frame_len >= DL_BLINK_LEN)
            {
                /* Rank from SRC (bytes 7-8), TX ts from bytes 10-14 */
                uint16_t rank16 = (uint16_t)(rx_buffer[7])
                                | ((uint16_t)(rx_buffer[8]) << 8);
                if (rank16 > 0 && rank16 < DL_MAX_ANCHORS && toa_dtu[rank16] == 0)
                {
                    toa_dtu[rank16]  = get_rx_timestamp_u64();
                    tx_ts_dtu[rank16] = unpack_40bit_ts(&rx_buffer[DL_BLINK_TX_TS_IDX]);
                    cfo_raw[rank16]   = dwt_readclockoffset();  /* CFO: f_tag/f_slave_k */
                    num_rx++;
                    /* Print: K,<rank>,CFO=<raw> */
                    { char _k[18]; int _kp = 0;
                      _k[_kp++]='K'; _k[_kp++]=',';
                      _k[_kp++]=hex[rank16&0xF]; _k[_kp++]=',';
                      { int16_t _cv = cfo_raw[rank16]; uint16_t _uv = (uint16_t)_cv;
                        _k[_kp++]=hex[(_uv>>12)&0xF]; _k[_kp++]=hex[(_uv>>8)&0xF];
                        _k[_kp++]=hex[(_uv>>4)&0xF]; _k[_kp++]=hex[_uv&0xF];
                      }
                      _k[_kp++]='\r'; _k[_kp++]='\n'; _k[_kp]='\0';
                      test_run_info((unsigned char *)_k); }
                }
            }
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }

        /*=== Output result ===*/
        if (num_rx < 2)
            continue;

        /* Debug: slot-compensated TDOA (before per-slave DLY_RS — per-rank bias) */
        { char _d[80]; int _p = 0;
          _d[_p++]='T'; _d[_p++]=',';
          for(i=1;i<(int)num_anchors&&i<4;i++){
            if(toa_dtu[i]!=0){
              int64_t _diff = (int64_t)(toa_dtu[i]-toa_dtu[0]);
              int64_t _slot = (int64_t)i * (int64_t)DL_SLOT_DURATION_UUS * (int64_t)UUS_TO_DWT_TIME;
              int64_t _comp = _diff - _slot - baseline_dtu[i];
              int32_t _m = (int32_t)(_comp / 213);  /* DTU→m (÷213) */
              if(_m<0){_d[_p++]='-';_m=-_m;} else _d[_p++]='+';
              _d[_p++]=hex[(_m>>12)&0xF];_d[_p++]=hex[(_m>>8)&0xF];
              _d[_p++]=hex[(_m>>4)&0xF];_d[_p++]=hex[_m&0xF];
            } else { _d[_p++]='?';_d[_p++]='?';_d[_p++]='?';_d[_p++]='?';_d[_p++]='?'; }
            _d[_p++]=',';
          }
          /* anchor coords */
          for(i=0;i<(int)num_anchors&&i<3;i++){
            _d[_p++]=hex[i&0xF];_d[_p++]=':';
            { int16_t _ax=anchor_table[i].x_cm; if(_ax<0){_d[_p++]='-';_ax=-_ax;} else _d[_p++]='+';
              _d[_p++]=hex[(_ax>>8)&0xF];_d[_p++]=hex[(_ax>>4)&0xF];_d[_p++]=hex[_ax&0xF];}
            _d[_p++]=',';
            { int16_t _ay=anchor_table[i].y_cm; if(_ay<0){_d[_p++]='-';_ay=-_ay;} else _d[_p++]='+';
              _d[_p++]=hex[(_ay>>8)&0xF];_d[_p++]=hex[(_ay>>4)&0xF];_d[_p++]=hex[_ay&0xF];}
            _d[_p++]=' ';
          }
          _d[_p++]='\r';_d[_p++]='\n';_d[_p]='\0';
          test_run_info((unsigned char *)_d); }

        /*=== Compensate TOAs: slot + baseline + auto-calibrating per-slave DLY_RS.
         *   Master uses DWT_START_TX_IMMEDIATE → no DLY_RS.
         *   Slave  uses DWT_START_TX_DLY_RS     → +DLY_RS delay (~250ns).
         *
         *   Auto-calibration: accumulates raw slot-compensated bias over
         *   CALIB_SAMPLES cycles at power-on, then freezes.  The raw bias
         *   (= T, value in DTU) equals DLY_RS[k] + true_TDOA[k].
         *   Averaging over many cycles suppresses measurement noise while
         *   the true_TDOA term (bounded by anchor spacing ±1.5m) is small
         *   compared to DLY_RS (~75m).  Freezing prevents continuous
         *   adaptation from absorbing the true TDOA signal (= losing accuracy).  ===*/
        {
            uint64_t ref = toa_dtu[0];
            for (i = 1; i < (int)num_anchors && i < DL_MAX_ANCHORS; i++)
            {
                if (toa_dtu[i] != 0)
                {
                    int64_t toa_diff  = (int64_t)(toa_dtu[i] - toa_dtu[0]);
                    int64_t slot_dtu  = (int64_t)i * (int64_t)DL_SLOT_DURATION_UUS * (int64_t)UUS_TO_DWT_TIME;
                    int64_t raw_bias  = toa_diff - slot_dtu - baseline_dtu[i];  /* = DLY_RS + tdoa */

                    /* Accumulate for calibration (before DLY_RS subtraction),
                     * with outlier rejection: skip samples > ±CALIB_OUTLIER_M
                     * from the per-anchor running average to prevent a single
                     * spike from corrupting the calibration result. */
                    if (!calib_done)
                    {
                        int64_t *sum_ptr = (i == 1) ? &calib_sum_k1 : &calib_sum_k2;
                        int32_t *cnt_ptr = (i == 1) ? &calib_cnt_k1 : &calib_cnt_k2;
                        int64_t outlier_thresh_dtu = (int64_t)CALIB_OUTLIER_M * 213;
                        int accept = 1;  /* default: accept this sample */

                        if (*cnt_ptr > 0)
                        {
                            /* Compare against per-anchor running average */
                            int64_t running_avg = *sum_ptr / (*cnt_ptr);
                            int64_t diff = raw_bias - running_avg;
                            if (diff < 0) diff = -diff;
                            if (diff > outlier_thresh_dtu)
                                accept = 0;  /* outlier → reject */
                        }
                        /* else: first valid sample for this anchor, always accept */

                        if (accept)
                        {
                            *sum_ptr += raw_bias;
                            (*cnt_ptr)++;
                        }
                    }

                    /* Use calibrated or default DLY_RS */
                    int64_t dly_rs    = (i == 1) ? dly_rs_k1 : dly_rs_k2;
                    int64_t tdoa_comp = raw_bias - dly_rs;
                    toa_dtu[i] = ref + (uint64_t)tdoa_comp;
                }
            }

            /* Update calibration state (once per cycle, after all slaves).
             * Completes when BOTH anchors have enough accepted samples,
             * or times out after CALIB_MAX_CYCLES total cycles. */
            if (!calib_done && toa_dtu[1] != 0 && toa_dtu[2] != 0)
            {
                calib_total++;
                if (calib_cnt_k1 >= CALIB_SAMPLES && calib_cnt_k2 >= CALIB_SAMPLES)
                {
                    /* Enough good samples — freeze calibration */
                    dly_rs_k1 = calib_sum_k1 / calib_cnt_k1;
                    dly_rs_k2 = calib_sum_k2 / calib_cnt_k2;
                    calib_done = 1;
                    /* Print calibration result */
                    { char _c[80]; int _cp = 0;
                      _c[_cp++]='C'; _c[_cp++]=',';
                      { int32_t _m1=(int32_t)(dly_rs_k1/213); if(_m1<0){_c[_cp++]='-';_m1=-_m1;} else _c[_cp++]='+';
                        _c[_cp++]=hex[(_m1>>12)&0xF];_c[_cp++]=hex[(_m1>>8)&0xF];
                        _c[_cp++]=hex[(_m1>>4)&0xF];_c[_cp++]=hex[_m1&0xF]; }
                      _c[_cp++]=',';
                      { int32_t _m2=(int32_t)(dly_rs_k2/213); if(_m2<0){_c[_cp++]='-';_m2=-_m2;} else _c[_cp++]='+';
                        _c[_cp++]=hex[(_m2>>12)&0xF];_c[_cp++]=hex[(_m2>>8)&0xF];
                        _c[_cp++]=hex[(_m2>>4)&0xF];_c[_cp++]=hex[_m2&0xF]; }
                      /* Also print sample counts for diagnostics */
                      _c[_cp++]=','; _c[_cp++]=hex[calib_cnt_k1&0xF]; _c[_cp++]=hex[calib_cnt_k2&0xF];
                      _c[_cp++]='\r';_c[_cp++]='\n';_c[_cp]='\0';
                      test_run_info((unsigned char *)_c); }
                }
                else if (calib_total >= CALIB_MAX_CYCLES)
                {
                    /* Timeout — too many outliers, keep defaults */
                    calib_done = 1;
                    { char _c[30]; int _cp = 0;
                      _c[_cp++]='C'; _c[_cp++]=','; _c[_cp++]='T'; _c[_cp++]='O';
                      _c[_cp++]=','; _c[_cp++]=hex[calib_cnt_k1&0xF];
                      _c[_cp++]=hex[calib_cnt_k2&0xF];
                      _c[_cp++]='\r';_c[_cp++]='\n';_c[_cp]='\0';
                      test_run_info((unsigned char *)_c); }
                }
            }
        }

        /*=== Solve and output ===*/
        {
            float x_cm = 0.0f, y_cm = 0.0f, z_cm = 0.0f, rms = 0.0f;
            int ret = 0;
            int16_t  tx_xi = 0, tx_yi = 0, tx_zi = 0;
            uint16_t tx_qi = 0xFFFFu;

            /*=== Simple 2D TDOA inline solver (with fallback init) ===*/
            {
                float tdoa_cm[4];
                int _si, _k, _attempt;
                static float last_x = 0, last_y = 0;
                static int   has_last = 0;

                /* Extract TDOA measurements (in cm) */
                for (_si = 1; _si < (int)num_anchors && _si < 4; _si++)
                {
                    int64_t diff = (int64_t)(toa_dtu[_si] - toa_dtu[0]);
                    tdoa_cm[_si] = (float)diff * 0.4691f;
                }

                /* Try up to 2 initial guesses: (1) last good pos, (2) centroid */
                for (_attempt = 0; _attempt < 2; _attempt++)
                {
                    if (_attempt == 0 && has_last)
                    {
                        x_cm = last_x;
                        y_cm = last_y;
                    }
                    else
                    {
                        float _sx = 0, _sy = 0;
                        for(_k=0;_k<num_anchors;_k++){
                            _sx += (float)anchor_table[_k].x_cm;
                            _sy += (float)anchor_table[_k].y_cm;
                        }
                        x_cm = _sx / (float)num_anchors;
                        y_cm = _sy / (float)num_anchors;
                    }
                    rms = 0.0f;
                    ret = 0;

                    for (_si = 0; _si < 8; _si++)
                    {
                        float r[4];
                        float H00=0, H01=0, H11=0, g0=0, g1=0;
                        float _rms = 0;
                        float prev_rms = rms;

                        for (_k = 0; _k < (int)num_anchors; _k++)
                        {
                            float dx = x_cm - (float)anchor_table[_k].x_cm;
                            float dy = y_cm - (float)anchor_table[_k].y_cm;
                            r[_k] = sqrtf(dx*dx + dy*dy);
                            if (r[_k] < 1.0f) r[_k] = 1.0f;
                        }

                        for (_k = 1; _k < (int)num_anchors; _k++)
                        {
                            float pred = r[_k] - r[0];
                            float res  = pred - tdoa_cm[_k];
                            float dx0 = (x_cm - (float)anchor_table[0].x_cm) / r[0];
                            float dy0 = (y_cm - (float)anchor_table[0].y_cm) / r[0];
                            float dxk = (x_cm - (float)anchor_table[_k].x_cm) / r[_k];
                            float dyk = (y_cm - (float)anchor_table[_k].y_cm) / r[_k];
                            float J0 = dxk - dx0;
                            float J1 = dyk - dy0;
                            H00 += J0 * J0;
                            H01 += J0 * J1;
                            H11 += J1 * J1;
                            g0  -= J0 * res;
                            g1  -= J1 * res;
                            _rms += res * res;
                        }
                        _rms = sqrtf(_rms / (float)((int)num_anchors - 1));
                        rms = _rms;

                        { float det = H00 * H11 - H01 * H01;
                          float dx, dy;
                          if (fabsf(det) < 1e-6f) { ret = -1; break; }
                          dx = (g0 * H11 - H01 * g1) / det;
                          dy = (H00 * g1 - g0 * H01) / det;
                          /* Damping: halve step if residual increased */
                          if (_si > 0 && _rms > prev_rms) { dx *= 0.5f; dy *= 0.5f; }
                          x_cm += dx; y_cm += dy;
                          if (sqrtf(dx*dx+dy*dy) < 0.05f) break;
                        }
                    }
                    if (ret == 0) { last_x = x_cm; last_y = y_cm; has_last = 1; break; }
                }
                z_cm = 0;
            }

        /* Debug: print compensated TOA diffs + ret code */
        { char _dd[80]; int _dp = 0;
          _dd[_dp++]='D'; _dd[_dp++]=',';
          for(i=1;i<(int)num_anchors&&i<4;i++){
            int64_t _d = (int64_t)(toa_dtu[i]-toa_dtu[0]);
            int32_t _c = (int32_t)(_d / 213);
            if(_c<0){_dd[_dp++]='-';_c=-_c;} else _dd[_dp++]='+';
            _dd[_dp++]=hex[(_c>>12)&0xF];_dd[_dp++]=hex[(_c>>8)&0xF];
            _dd[_dp++]=hex[(_c>>4)&0xF];_dd[_dp++]=hex[_c&0xF];_dd[_dp++]=',';
          }
          _dd[_dp++]='R';_dd[_dp++]='=';
          _dd[_dp++]=hex[ret & 0xF];_dd[_dp++]=',';
          { union { float f; uint32_t u; } _fu; _fu.f = x_cm;
            int _j; for(_j=28;_j>=0;_j-=4) _dd[_dp++]=hex[(_fu.u>>_j)&0xF]; }
          _dd[_dp++]='\r';_dd[_dp++]='\n';_dd[_dp]='\0';
          test_run_info((unsigned char *)_dd); }

        if (ret == 0)
            {
                char out[80]; int pos = 0;
                out[pos++] = 'P'; out[pos++] = ',';
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
                tx_xi = (int16_t)x_cm; tx_yi = (int16_t)y_cm; tx_zi = (int16_t)z_cm;
                tx_qi = (uint16_t)(rms * 10.0f);
            }
            else if (ret == -2)
            {
                test_run_info((unsigned char *)"ERR ANCHORS");
            }
            else
            {
                test_run_info((unsigned char *)"ERR DIVERGE");
            }

            /* Phase 5: TX report to gateway (always — ensures tag shows online) */
            {
                uint8_t pos_report[DL_POS_REPORT_LEN];
                pos_report[0] = 0x41; pos_report[1] = 0x88;
                pos_report[2] = frame_seq;
                pos_report[3] = 0xCA; pos_report[4] = 0xDE;
                pos_report[5] = 0xFF; pos_report[6] = 0xFF;  /* broadcast */
                pos_report[7] = (DL_TAG_ID >> 0) & 0xFF;
                pos_report[8] = (DL_TAG_ID >> 8) & 0xFF;
                pos_report[9] = FUNC_DL_POS_REPORT;
                pos_report[10] = (uint8_t)(tx_xi & 0xFF);
                pos_report[11] = (uint8_t)((tx_xi >> 8) & 0xFF);
                pos_report[12] = (uint8_t)(tx_yi & 0xFF);
                pos_report[13] = (uint8_t)((tx_yi >> 8) & 0xFF);
                pos_report[14] = (uint8_t)(tx_zi & 0xFF);
                pos_report[15] = (uint8_t)((tx_zi >> 8) & 0xFF);
                pos_report[16] = (uint8_t)(tx_qi & 0xFF);
                pos_report[17] = (uint8_t)((tx_qi >> 8) & 0xFF);
                pos_report[18] = (uint8_t)(FW_VERSION_U16 & 0xFF);
                pos_report[19] = (uint8_t)((FW_VERSION_U16 >> 8) & 0xFF);
                dwt_forcetrxoff();
                port_rf_switch_tx();
                dwt_writetxdata(sizeof(pos_report), pos_report, 0);
                dwt_writetxfctrl(sizeof(pos_report), 0, 0);
                if (dwt_starttx(DWT_START_TX_IMMEDIATE) == DWT_SUCCESS)
                {
                    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) { }
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
                }
                port_rf_switch_rx();
                dwt_rxenable(DWT_START_RX_IMMEDIATE);  /* re-arm RX immediately after TX */
            }
        }
    }
}
#endif /* TEST_FUSION_TAG */
