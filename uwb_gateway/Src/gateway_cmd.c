/*! ----------------------------------------------------------------------------
 * @file    gateway_cmd.c
 * @brief   Gateway AT command handler — UART RX ring buffer + command dispatch
 *
 *          Handles AT commands received from the PC over UART:
 *            AT+PING              → +PONG
 *            AT+LIST              → +ONLINE,UID,TYPE,RANK per device
 *            AT+FIND=<UID>,<0/1>  → forward FIND_REQ via UWB
 *            AT+COORD=<UID>,<X>,<Y>,<Z> → forward COORD_WRITE via UWB
 *            AT+COORD?=<UID>      → read coordinates
 *            AT+RESET=<UID>       → remote reboot
 *            AT+OTA_START=<UID>,<SIZE>
 *            AT+OTA_DATA=<UID>,<OFFSET>,<HEX>
 *            AT+OTA_END=<UID>
 * ---------------------------------------------------------------------------*/
#include <string.h>

#include <main.h>
#include <stm32l4xx_hal.h>
#include <deca_device_api.h>
#include <deca_regs.h>
#include <port.h>
#include <shared_defines.h>

#include "gateway_cmd.h"
#include <gateway_protocol.h>
#include <downlink_protocol.h>
#include <firmware_version.h>

#ifdef FUSION_DOWNLINK_MODE
extern void dl_set_beacon_interval(uint16_t ms);
extern void dl_set_anchor_coord(uint8_t rank, int16_t x_cm, int16_t y_cm,
                                int16_t z_cm);
#endif

extern void test_run_info(unsigned char *data);

/*============================================================================
 * String formatting helpers (avoid printf dependency)
 *============================================================================*/
static const char gw_hex[] = "0123456789ABCDEF";

/* Write 4-digit uppercase hex to buf, return bytes written */
static int fmt_hex4(char *buf, uint16_t val)
{
    buf[0] = gw_hex[(val >> 12) & 0xF];
    buf[1] = gw_hex[(val >>  8) & 0xF];
    buf[2] = gw_hex[(val >>  4) & 0xF];
    buf[3] = gw_hex[ val        & 0xF];
    return 4;
}

/* Write signed decimal to buf, return bytes written (no terminator) */
static int fmt_dec(char *buf, int val)
{
    int pos = 0;
    unsigned u;
    char tmp[12];
    int i = 0;

    if (val < 0) { buf[pos++] = '-'; u = (unsigned)(-val); }
    else         { u = (unsigned)val; }

    if (u == 0) { buf[pos++] = '0'; return pos; }

    while (u > 0) { tmp[i++] = '0' + (u % 10); u /= 10; }
    while (i > 0) { buf[pos++] = tmp[--i]; }
    return pos;
}

/* Copy string, return bytes written (no terminator) */
static int fmt_str(char *buf, const char *s)
{
    int i = 0;
    while (*s) buf[i++] = *s++;
    return i;
}

/*============================================================================
 * UART RX ring buffer
 *============================================================================*/
static volatile uint8_t rx_ring[GW_RX_BUF_SIZE];
static volatile int rx_head = 0;
static volatile int rx_tail = 0;

void gateway_uart_rx_byte(uint8_t byte)
{
    int next = (rx_head + 1) % GW_RX_BUF_SIZE;
    if (next != rx_tail)  /* not full */
    {
        rx_ring[rx_head] = (uint8_t)byte;
        rx_head = next;
    }
    /* else: overflow — silently drop */
}

int uart_rx_line_available(char *buf, int maxlen)
{
    int i = 0;
    while (rx_tail != rx_head && i < maxlen - 1)
    {
        char c = (char)rx_ring[rx_tail];
        rx_tail = (rx_tail + 1) % GW_RX_BUF_SIZE;
        if (c == '\n')
        {
            /* strip trailing \r */
            if (i > 0 && buf[i - 1] == '\r') i--;
            buf[i] = '\0';
            return 1;
        }
        buf[i++] = c;
    }
    /* line too long — discard remainder */
    if (i >= maxlen - 1)
    {
        while (rx_tail != rx_head)
        {
            if (rx_ring[rx_tail] == '\n') { rx_tail = (rx_tail + 1) % GW_RX_BUF_SIZE; break; }
            rx_tail = (rx_tail + 1) % GW_RX_BUF_SIZE;
        }
        buf[0] = '\0';
        return 0;
    }
    return 0;
}

/*============================================================================
 * OTA firmware upgrade — receive from PC, forward via UWB to target device
 *============================================================================*/
static uint8_t  ota_active = 0;
static uint16_t ota_dst_uid;
static uint32_t ota_total_size;
static uint32_t ota_offset;
static uint8_t  ota_seq;

static uint16_t crc16_ccitt(const uint8_t *data, int len)
{
    uint16_t crc = 0xFFFF;
    int i, j;
    for (i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (j = 0; j < 8; j++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

/* Parse 2 hex chars → byte. Returns 0 on success, -1 on bad char. */
static int hex_byte(char hi, char lo, uint8_t *out)
{
    uint8_t val = 0;
    if      (hi >= '0' && hi <= '9') val = (uint8_t)((hi - '0') << 4);
    else if (hi >= 'a' && hi <= 'f') val = (uint8_t)((hi - 'a' + 10) << 4);
    else if (hi >= 'A' && hi <= 'F') val = (uint8_t)((hi - 'A' + 10) << 4);
    else return -1;

    if      (lo >= '0' && lo <= '9') val |= (uint8_t)(lo - '0');
    else if (lo >= 'a' && lo <= 'f') val |= (uint8_t)(lo - 'a' + 10);
    else if (lo >= 'A' && lo <= 'F') val |= (uint8_t)(lo - 'A' + 10);
    else return -1;

    *out = val;
    return 0;
}

/*============================================================================
 * Device tracking — maintained from slave reports and blink sequences
 *============================================================================*/
#define GW_MAX_DEVICES        16
#define GW_TIMEOUT_MS          6000  /* 6 s ≈ 12 missed cycles → offline */
#define GW_HEARTBEAT_MS       5000


typedef struct
{
    char     uid[8];
    char     type;    /* 'A'=Anchor, 'T'=Tag, 'G'=Gateway */
    uint8_t  rank;
    uint8_t  online;
    uint16_t fw_ver;  /* Packed firmware version (0x0001=V0.0.1) */
    uint32_t last_seen_ms;
    uint32_t last_heartbeat_ms;
} gw_dev_t;

static gw_dev_t gw_devs[GW_MAX_DEVICES];
static int      gw_dev_count = 0;
static char     master_uid[9] = "0001";

void gateway_set_master_uid(const char *uid)
{
    int i;
    for (i = 0; i < 8 && uid[i]; i++) master_uid[i] = uid[i];
    master_uid[i] = '\0';
}

void gateway_device_seen(const char *uid, const char *type, uint8_t rank)
{
    uint32_t now = HAL_GetTick();
    int i;
    for (i = 0; i < gw_dev_count; i++)
    {
        if (strcmp(gw_devs[i].uid, uid) == 0)
        {
            int was_offline = !gw_devs[i].online;
            gw_devs[i].online = 1;
            gw_devs[i].last_seen_ms = now;
            /* Only send +ONLINE if was offline or heartbeat due (every 5s),
             * not on every frame (avoids host UI flicker). */
            if (!was_offline && (now - gw_devs[i].last_heartbeat_ms) < GW_HEARTBEAT_MS)
                return;
            gw_devs[i].last_heartbeat_ms = now;
            goto send_online;
        }
    }
    if (gw_dev_count < GW_MAX_DEVICES)
    {
        gw_dev_t *d = &gw_devs[gw_dev_count++];
        strncpy(d->uid, uid, sizeof(d->uid) - 1);
        d->uid[sizeof(d->uid) - 1] = '\0';
        d->type      = (type && type[0] == 'T') ? 'T' : (type && type[0] == 'G') ? 'G' : 'A';
        d->rank      = rank;
        d->online    = 1;
        d->last_seen_ms = now;
        d->last_heartbeat_ms = now;
    }

send_online:
    {
        char out[48];
        int pos;
        /* Use STORED type for existing devices, not the passed-in parameter */
        char dev_type = 'A';
        int  dev_rank = rank;
        if (i < gw_dev_count) {  /* existing device */
            dev_type = gw_devs[i].type;
            dev_rank = gw_devs[i].rank;
        }
        out[0] = '+'; out[1] = 'O'; out[2] = 'N'; out[3] = 'L'; out[4] = 'I'; out[5] = 'N'; out[6] = 'E'; out[7] = ',';
        pos = 8;
        pos += fmt_str(&out[pos], uid);
        out[pos++] = ',';
        if (dev_type == 'T')
        { out[pos++] = 'T'; out[pos++] = 'A'; out[pos++] = 'G'; }
        else if (dev_type == 'G')
        { out[pos++] = 'G'; out[pos++] = 'A'; out[pos++] = 'T'; out[pos++] = 'E'; out[pos++] = 'W'; out[pos++] = 'A'; out[pos++] = 'Y'; }
        else
        { out[pos++] = 'A'; out[pos++] = 'N'; out[pos++] = 'C'; out[pos++] = 'H'; out[pos++] = 'O'; out[pos++] = 'R'; }
        out[pos++] = ',';
        if (dev_rank < 10) { out[pos++] = '0' + dev_rank; }
        else { pos += fmt_dec(&out[pos], dev_rank); }
        out[pos] = '\0';
        test_run_info((unsigned char *)out);
    }
}

void gateway_device_version(const char *uid, uint16_t fw_ver)
{
    int i;
    char ver[9];
    for (i = 0; i < gw_dev_count; i++)
    {
        if (strcmp(gw_devs[i].uid, uid) == 0)
        {
            gw_devs[i].fw_ver = fw_ver;
            /* Notify PC: +FWV,<type>,<uid>,<version> */
            {
                char out[48];
                int pos;
                fw_version_str(ver, fw_ver);
                out[0] = '+'; out[1] = 'F'; out[2] = 'W'; out[3] = 'V'; out[4] = ',';
                out[5] = gw_devs[i].type; out[6] = ',';
                pos = 7;
                pos += fmt_str(&out[pos], uid);
                out[pos++] = ',';
                pos += fmt_str(&out[pos], ver);
                out[pos] = '\0';
                test_run_info((unsigned char *)out);
            }
            return;
        }
    }
    /* Unknown device — still report version */
    fw_version_str(ver, fw_ver);
    {
        char out[48];
        int pos;
        out[0] = '+'; out[1] = 'F'; out[2] = 'W'; out[3] = 'V'; out[4] = ',';
        out[5] = '?'; out[6] = ',';
        pos = 7;
        pos += fmt_str(&out[pos], uid);
        out[pos++] = ',';
        pos += fmt_str(&out[pos], ver);
        out[pos] = '\0';
        test_run_info((unsigned char *)out);
    }
}

static void gateway_check_timeout(void)
{
    uint32_t now = HAL_GetTick();
    int i;
    for (i = 0; i < gw_dev_count; i++)
    {
        uint32_t age_seen, age_hb;
        if (!gw_devs[i].online) continue;

        age_seen = (uint32_t)(now - gw_devs[i].last_seen_ms);
        age_hb   = (uint32_t)(now - gw_devs[i].last_heartbeat_ms);

        if (age_seen > GW_TIMEOUT_MS)
        {
            gw_devs[i].online = 0;
            {
                char out[32];
                int pos;
                out[0] = '+'; out[1] = 'O'; out[2] = 'F'; out[3] = 'F'; out[4] = 'L';
                out[5] = 'I'; out[6] = 'N'; out[7] = 'E'; out[8] = ',';
                pos = 9;
                pos += fmt_str(&out[pos], gw_devs[i].uid);
                out[pos] = '\0';
                test_run_info((unsigned char *)out);
            }
        }
        else if (age_hb > GW_HEARTBEAT_MS)
        {
            gw_devs[i].last_heartbeat_ms = now;
            {
                char out[48];
                int pos;
                out[0] = '+'; out[1] = 'O'; out[2] = 'N'; out[3] = 'L';
                out[4] = 'I'; out[5] = 'N'; out[6] = 'E'; out[7] = ',';
                pos = 8;
                pos += fmt_str(&out[pos], gw_devs[i].uid);
                out[pos++] = ',';
                if (gw_devs[i].type == 'T')
                { out[pos++] = 'T'; out[pos++] = 'A'; out[pos++] = 'G'; }
                else if (gw_devs[i].type == 'G')
                { out[pos++] = 'G'; out[pos++] = 'A'; out[pos++] = 'T'; out[pos++] = 'E'; out[pos++] = 'W'; out[pos++] = 'A'; out[pos++] = 'Y'; }
                else
                { out[pos++] = 'A'; out[pos++] = 'N'; out[pos++] = 'C'; out[pos++] = 'H'; out[pos++] = 'O'; out[pos++] = 'R'; }
                out[pos++] = ',';
                if (gw_devs[i].rank < 10) { out[pos++] = '0' + gw_devs[i].rank; }
                else { pos += fmt_dec(&out[pos], gw_devs[i].rank); }
                out[pos] = '\0';
                test_run_info((unsigned char *)out);
            }
        }
    }
}

/*============================================================================
 * UWB frame TX helpers
 *============================================================================*/

/* Build and send a UWB data frame immediately.
 * Assumes radio is idle (caller should be in Phase 7 or after forcetrxoff).
 * src_addr: IEEE 802.15.4 short source address of sender.
 *           Master uses GW_MASTER_ADDR (0x0001).
 *           Slave uses its own rank. */
void gateway_send_uwb_ack(uint16_t dst_addr, uint8_t func,
                          uint8_t *payload, uint8_t payload_len)
{
    uint16_t src_addr = GW_MASTER_ADDR;
    gateway_send_uwb_ack_ex(dst_addr, src_addr, func, payload, payload_len);
}

void gateway_send_uwb_ack_ex(uint16_t dst_addr, uint16_t src_addr, uint8_t func,
                              uint8_t *payload, uint8_t payload_len)
{
    uint8_t frame[127];
    int i;

    if (payload_len > 127 - GW_HDR_LEN - 2) return;  /* too long */

    /* IEEE 802.15.4 MAC header */
    frame[0] = 0x41;  /* FC[0]: frame type=data, src=short, dst=short */
    frame[1] = 0x88;  /* FC[1]: no security, no pending, no ACK req, PAN compression */
    frame[2] = 0;     /* SEQ */
    frame[3] = 0xCA;  /* PAN[0] */
    frame[4] = 0xDE;  /* PAN[1] */
    frame[5] = dst_addr & 0xFF;        /* DST[0] */
    frame[6] = (dst_addr >> 8) & 0xFF; /* DST[1] */
    frame[7] = src_addr & 0xFF;         /* SRC[0] */
    frame[8] = (src_addr >> 8) & 0xFF;  /* SRC[1] */
    frame[9] = func;

    for (i = 0; i < payload_len; i++)
        frame[GW_HDR_LEN + i] = payload[i];

    {
        uint16_t total = GW_HDR_LEN + payload_len + 2;  /* +2 for FCS */

        dwt_forcetrxoff();
        port_rf_switch_tx();
        dwt_writetxdata(total, frame, 0);
        dwt_writetxfctrl(total, 0, 0);
        if (dwt_starttx(DWT_START_TX_IMMEDIATE) == DWT_SUCCESS)
        {
            while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) { }
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
        }
        port_rf_switch_rx();
    }
}

/*============================================================================
 * Gateway UWB response handler — called from Phase 1 RX
 *============================================================================*/
void gateway_handle_uwb_rx(uint8_t *frame, uint16_t len)
{
    uint8_t func = frame[9];
    uint16_t src_addr = (uint16_t)(frame[7] | (frame[8] << 8));
    char out[48];
    int pos;
    (void)len;  /* unused — frame length checked by caller */

    if (func == FUNC_FIND_ACK)
    {
        uint8_t state = frame[GW_HDR_LEN];
        /* +FIND,<UID_HEX4>,<STATE> */
        out[0] = '+'; out[1] = 'F'; out[2] = 'I'; out[3] = 'N'; out[4] = 'D'; out[5] = ',';
        pos = 6;
        pos += fmt_hex4(&out[pos], src_addr);
        out[pos++] = ',';
        out[pos++] = state ? '1' : '0';
        out[pos] = '\0';
        test_run_info((unsigned char *)out);
    }
    else if (func == FUNC_COORD_ACK)
    {
        int16_t x = (int16_t)(frame[GW_HDR_LEN]     | (frame[GW_HDR_LEN+1] << 8));
        int16_t y = (int16_t)(frame[GW_HDR_LEN+2]   | (frame[GW_HDR_LEN+3] << 8));
        int16_t z = (int16_t)(frame[GW_HDR_LEN+4]   | (frame[GW_HDR_LEN+5] << 8));
        /* +COORD,<UID_HEX4>,<X>,<Y>,<Z> */
        out[0] = '+'; out[1] = 'C'; out[2] = 'O'; out[3] = 'O'; out[4] = 'R'; out[5] = 'D'; out[6] = ',';
        pos = 7;
        pos += fmt_hex4(&out[pos], src_addr);
        out[pos++] = ',';
        pos += fmt_dec(&out[pos], x);
        out[pos++] = ',';
        pos += fmt_dec(&out[pos], y);
        out[pos++] = ',';
        pos += fmt_dec(&out[pos], z);
        out[pos] = '\0';
        test_run_info((unsigned char *)out);
    }
    else if (func == FUNC_DL_POS_REPORT)
    {
        int16_t x = (int16_t)(frame[GW_HDR_LEN]     | (frame[GW_HDR_LEN+1] << 8));
        int16_t y = (int16_t)(frame[GW_HDR_LEN+2]   | (frame[GW_HDR_LEN+3] << 8));
        int16_t z = (int16_t)(frame[GW_HDR_LEN+4]   | (frame[GW_HDR_LEN+5] << 8));
        uint16_t q = (uint16_t)(frame[GW_HDR_LEN+6] | (frame[GW_HDR_LEN+7] << 8));
        /* Mark tag online */
        {
            char uid[8];
            uid[0] = gw_hex[(src_addr >> 12) & 0xF]; uid[1] = gw_hex[(src_addr >>  8) & 0xF];
            uid[2] = gw_hex[(src_addr >>  4) & 0xF]; uid[3] = gw_hex[ src_addr        & 0xF];
            uid[4] = '\0';
            gateway_device_seen(uid, "TAG", 0);
        }
        /* +POS,<UID>,<X>,<Y>,<Z>,<Q> */
        out[0] = '+'; out[1] = 'P'; out[2] = 'O'; out[3] = 'S'; out[4] = ',';
        pos = 5;
        pos += fmt_hex4(&out[pos], src_addr);
        out[pos++] = ',';
        pos += fmt_dec(&out[pos], x);
        out[pos++] = ',';
        pos += fmt_dec(&out[pos], y);
        out[pos++] = ',';
        pos += fmt_dec(&out[pos], z);
        out[pos++] = ',';
        pos += fmt_dec(&out[pos], (int)q);
        out[pos] = '\0';
        test_run_info((unsigned char *)out);
    }
}

/*============================================================================
 * AT command handlers
 *============================================================================*/

static uint16_t parse_uid(const char *s)
{
    uint16_t val = 0;
    while (*s)
    {
        char c = *s++;
        val <<= 4;
        if      (c >= '0' && c <= '9') val |= (uint16_t)(c - '0');
        else if (c >= 'a' && c <= 'f') val |= (uint16_t)(c - 'a' + 10);
        else if (c >= 'A' && c <= 'F') val |= (uint16_t)(c - 'A' + 10);
    }
    return val;
}

static int parse_int(const char *s)
{
    int val = 0;
    int neg = 0;
    if (*s == '-') { neg = 1; s++; }
    while (*s >= '0' && *s <= '9')
    {
        val = val * 10 + (*s - '0');
        s++;
    }
    return neg ? -val : val;
}

/* Build "+FIND,<UID4>,<0/1>" and send on UART */
static void send_find_ack(uint16_t uid, int on_off)
{
    char ack[20];
    int pos;
    ack[0] = '+'; ack[1] = 'F'; ack[2] = 'I'; ack[3] = 'N'; ack[4] = 'D'; ack[5] = ',';
    pos = 6;
    pos += fmt_hex4(&ack[pos], uid);
    ack[pos++] = ',';
    ack[pos++] = on_off ? '1' : '0';
    ack[pos] = '\0';
    test_run_info((unsigned char *)ack);
}

/* AT+FIND=<UID>,<0/1> */
static void gateway_find(const char *cmd)
{
    const char *p = cmd;
    char uid_str[8];
    int on_off, i;

    if (*p != '=') return;
    p++;

    i = 0;
    while (*p && *p != ',' && i < (int)sizeof(uid_str) - 1)
        uid_str[i++] = *p++;
    uid_str[i] = '\0';

    if (*p != ',') return;
    p++;
    on_off = (*p == '1') ? 1 : 0;

    {
        uint16_t dst = parse_uid(uid_str);

        if (dst == 0)
        {
            /* Master anchor — local LED toggle */
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin,
                              on_off ? GPIO_PIN_SET : GPIO_PIN_RESET);
            send_find_ack(dst, on_off);
        }
        else
        {
            /* Forward to remote anchor via UWB */
            uint8_t payload = (uint8_t)on_off;
            gateway_send_uwb_ack(dst, FUNC_FIND_REQ, &payload, 1);
            send_find_ack(dst, on_off);
        }
    }
}

/* Build "+COORD,<UID4>,<X>,<Y>,<Z>" and send on UART */
static void send_coord_ack(uint16_t uid, int x, int y, int z)
{
    char ack[40];
    int pos;
    ack[0] = '+'; ack[1] = 'C'; ack[2] = 'O'; ack[3] = 'O'; ack[4] = 'R'; ack[5] = 'D'; ack[6] = ',';
    pos = 7;
    pos += fmt_hex4(&ack[pos], uid);
    ack[pos++] = ',';
    pos += fmt_dec(&ack[pos], x);
    ack[pos++] = ',';
    pos += fmt_dec(&ack[pos], y);
    ack[pos++] = ',';
    pos += fmt_dec(&ack[pos], z);
    ack[pos] = '\0';
    test_run_info((unsigned char *)ack);
}

/* AT+COORD=<UID>,<X>,<Y>,<Z> — retry up to 3 times, wait for anchor ACK */
static void gateway_coord_write(const char *cmd)
{
    const char *p = cmd;
    char uid_str[8];
    int x, y, z, i;

    if (*p != '=') return;
    p++;

    i = 0;
    while (*p && *p != ',' && i < (int)sizeof(uid_str) - 1)
        uid_str[i++] = *p++;
    uid_str[i] = '\0';

    if (*p != ',') return; p++;
    x = parse_int(p);
    while (*p && *p != ',') p++;
    if (*p != ',') return; p++;
    y = parse_int(p);
    while (*p && *p != ',') p++;
    if (*p != ',') return; p++;
    z = parse_int(p);

    {
        uint16_t dst = parse_uid(uid_str);
        uint8_t payload[6];
        int retry, success = 0;

        /* Direct update of beacon table (gateway on same device as Master) */
#ifdef FUSION_DOWNLINK_MODE
        dl_set_anchor_coord((uint8_t)dst, (int16_t)x, (int16_t)y, (int16_t)z);
#endif
        payload[0] = x & 0xFF;  payload[1] = (x >> 8) & 0xFF;
        payload[2] = y & 0xFF;  payload[3] = (y >> 8) & 0xFF;
        payload[4] = z & 0xFF;  payload[5] = (z >> 8) & 0xFF;

        for (retry = 0; retry < 3; retry++)
        {
            gateway_send_uwb_ack(dst, FUNC_COORD_WRITE, payload, 6);

            /* Wait up to 100ms for COORD_ACK from target anchor */
            {
                uint32_t start = HAL_GetTick();
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
                while (HAL_GetTick() - start < 100)
                {
                    Sleep(1);
                    {
                        uint32_t st = dwt_read32bitreg(SYS_STATUS_ID);
                        if (st & SYS_STATUS_ALL_RX_ERR)
                        {
                            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                            dwt_rxenable(DWT_START_RX_IMMEDIATE);
                            continue;
                        }
                        if (st & SYS_STATUS_RXFCG_BIT_MASK)
                        {
                            uint32_t flen = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
                            if (flen >= GW_HDR_LEN + 6 && flen <= 32)
                            {
                                uint8_t rx_buf[32];
                                dwt_readrxdata(rx_buf, flen, 0);
                                if (rx_buf[9] == FUNC_COORD_ACK)
                                {
                                    uint16_t src = (uint16_t)(rx_buf[7] | (rx_buf[8] << 8));
                                    if (src == dst)
                                    {
                                        success = 1;
                                        break;
                                    }
                                }
                            }
                            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
                            dwt_rxenable(DWT_START_RX_IMMEDIATE);
                        }
                    }
                }
            }
            if (success) break;
        }

        if (success)
        {
            send_coord_ack(dst, x, y, z);
        }
        else
        {
            char err[24];
            int pos = 0;
            err[pos++] = '+'; err[pos++] = 'C'; err[pos++] = 'O'; err[pos++] = 'O'; err[pos++] = 'R'; err[pos++] = 'D';
            err[pos++] = '_'; err[pos++] = 'E'; err[pos++] = 'R'; err[pos++] = 'R'; err[pos++] = ',';
            pos += fmt_hex4(&err[pos], dst);
            err[pos++] = '\r'; err[pos++] = '\n';
            err[pos] = '\0';
            test_run_info((unsigned char *)err);
        }
    }
}

/* AT+COORD?=<UID> — read coordinates (stub, returns zeros until Step 5) */
static void gateway_coord_read(const char *cmd)
{
    const char *p = cmd;
    char uid_str[8];
    int i;

    if (*p != '=') return;
    p++;
    i = 0;
    while (*p && *p != ',' && i < (int)sizeof(uid_str) - 1)
        uid_str[i++] = *p++;
    uid_str[i] = '\0';

    /* Read coordinates from flash */
    {
        char ack[40];
        int16_t fx, fy, fz;
        int pos;

        if (gateway_flash_load_coord(&fx, &fy, &fz) == 0)
        {
            ack[0] = '+'; ack[1] = 'C'; ack[2] = 'O'; ack[3] = 'O'; ack[4] = 'R'; ack[5] = 'D'; ack[6] = ',';
            pos = 7;
            pos += fmt_str(&ack[pos], uid_str);
            ack[pos++] = ',';
            pos += fmt_dec(&ack[pos], fx);
            ack[pos++] = ',';
            pos += fmt_dec(&ack[pos], fy);
            ack[pos++] = ',';
            pos += fmt_dec(&ack[pos], fz);
            ack[pos] = '\0';
        }
        else
        {
            /* Flash not written yet — return zeros */
            ack[0] = '+'; ack[1] = 'C'; ack[2] = 'O'; ack[3] = 'O'; ack[4] = 'R'; ack[5] = 'D'; ack[6] = ',';
            pos = 7;
            pos += fmt_str(&ack[pos], uid_str);
            ack[pos++] = ','; ack[pos++] = '0';
            ack[pos++] = ','; ack[pos++] = '0';
            ack[pos++] = ','; ack[pos++] = '0';
            ack[pos] = '\0';
        }
        test_run_info((unsigned char *)ack);
    }
}

/* AT+LIST — output +ONLINE for each known device */
static void gateway_send_list(void)
{
    char out[48];
    int pos, i;

    /* All known devices (not just online — so reconnecting host sees them) */
    for (i = 0; i < gw_dev_count; i++)
    {
        out[0] = '+'; out[1] = 'O'; out[2] = 'N'; out[3] = 'L'; out[4] = 'I'; out[5] = 'N'; out[6] = 'E'; out[7] = ',';
        pos = 8;
        pos += fmt_str(&out[pos], gw_devs[i].uid);
        out[pos++] = ',';
        if (gw_devs[i].type == 'T')
        { out[pos++] = 'T'; out[pos++] = 'A'; out[pos++] = 'G'; }
        else if (gw_devs[i].type == 'G')
        { out[pos++] = 'G'; out[pos++] = 'A'; out[pos++] = 'T'; out[pos++] = 'E'; out[pos++] = 'W'; out[pos++] = 'A'; out[pos++] = 'Y'; }
        else
        { out[pos++] = 'A'; out[pos++] = 'N'; out[pos++] = 'C'; out[pos++] = 'H'; out[pos++] = 'O'; out[pos++] = 'R'; }
        out[pos++] = ',';
        out[pos++] = '0' + (gw_devs[i].rank % 10);
        out[pos] = '\0';
        test_run_info((unsigned char *)out);
    }
}

/*============================================================================
 * Main poll entry point — called from Phase 7
 *============================================================================*/
void gateway_poll_uart(void)
{
    char cmd[GW_CMD_MAX_LEN];

    gateway_check_timeout();

    gateway_device_seen("0000", "GATEWAY", 0);  /* self keepalive */
    if (!uart_rx_line_available(cmd, sizeof(cmd)))
        return;

    if (strncmp(cmd, "AT+PING", 7) == 0)
    {
        test_run_info((unsigned char *)"+PONG");
    }
    else if (strncmp(cmd, "AT+LIST", 7) == 0)
    {
        gateway_send_list();
    }
    else if (strncmp(cmd, "AT+FIND", 7) == 0)
    {
        gateway_find(cmd + 7);
    }
    else if (strncmp(cmd, "AT+COORD?=", 10) == 0)
    {
        gateway_coord_read(cmd + 9);
    }
    else if (strncmp(cmd, "AT+COORD=", 9) == 0)
    {
        gateway_coord_write(cmd + 8);
    }
    else if (strncmp(cmd, "AT+RATE=", 8) == 0)
    {
        int ms = parse_int(cmd + 8);
        if (ms >= 20 && ms <= 5000)
        {
            dl_set_beacon_interval((uint16_t)ms);
            {
                char out[24];
                int pos = 0;
                out[pos++] = '+'; out[pos++] = 'R'; out[pos++] = 'A'; out[pos++] = 'T'; out[pos++] = 'E';
                out[pos++] = '_'; out[pos++] = 'O'; out[pos++] = 'K'; out[pos++] = ',';
                pos += fmt_dec(&out[pos], ms);
                out[pos++] = '\r'; out[pos++] = '\n';
                out[pos] = '\0';
                test_run_info((unsigned char *)out);
            }
        }
        else
        {
            test_run_info((unsigned char *)"+RATE_ERR");
        }
    }
    else if (strncmp(cmd, "AT+RESET=", 9) == 0)
    {
        /* +RESET,<UID>,OK — reset acknowledged (actual UWB forward in future) */
        char ack[32];
        int pos;
        ack[0] = '+'; ack[1] = 'R'; ack[2] = 'E'; ack[3] = 'S'; ack[4] = 'E'; ack[5] = 'T'; ack[6] = ',';
        pos = 7;
        pos += fmt_str(&ack[pos], cmd + 9);
        ack[pos++] = ','; ack[pos++] = 'O'; ack[pos++] = 'K';
        ack[pos] = '\0';
        test_run_info((unsigned char *)ack);
    }
    else if (strncmp(cmd, "AT+OTA_START=", 13) == 0)
    {
        /* AT+OTA_START=<UID>,<SIZE> → +OTA_READY,<UID> */
        const char *p = cmd + 13;
        char uid_str[8];
        int i;
        i = 0;
        while (*p && *p != ',' && i < (int)sizeof(uid_str) - 1)
            uid_str[i++] = *p++;
        uid_str[i] = '\0';

        if (*p == ',') p++;
        ota_total_size = 0;
        while (*p >= '0' && *p <= '9')
            ota_total_size = ota_total_size * 10 + (uint32_t)(*p++ - '0');

        ota_dst_uid = parse_uid(uid_str);
        ota_offset  = 0;
        ota_seq     = 0;
        ota_active  = 1;

        /* +OTA_READY,<UID> */
        {
            char out[32];
            int pos;
            out[0] = '+'; out[1] = 'O'; out[2] = 'T'; out[3] = 'A'; out[4] = '_'; out[5] = 'R';
            out[6] = 'E'; out[7] = 'A'; out[8] = 'D'; out[9] = 'Y'; out[10] = ',';
            pos = 11;
            pos += fmt_str(&out[pos], uid_str);
            out[pos] = '\0';
            test_run_info((unsigned char *)out);
        }
    }
    else if (strncmp(cmd, "AT+OTA_DATA=", 12) == 0)
    {
        /* AT+OTA_DATA=<UID>,<OFFSET>,<HEX_DATA> */
        if (!ota_active)
        {
            test_run_info((unsigned char *)"+OTA_ERROR,OTA,Not started");
            return;
        }
        const char *p = cmd + 12;
        char uid_str[8];
        int i;
        i = 0;
        while (*p && *p != ',' && i < (int)sizeof(uid_str) - 1)
            uid_str[i++] = *p++;
        uid_str[i] = '\0';

        if (*p == ',') p++;
        uint32_t offset = 0;
        while (*p >= '0' && *p <= '9')
            offset = offset * 10 + (uint32_t)(*p++ - '0');

        if (*p == ',') p++;
        /* Remaining chars are hex data — 2 chars per byte */
        {
            uint8_t ota_buf[251];  /* seq(1) + data(248) + crc(2) = 251 */
            int hex_len = 0;
            const char *hp = p;
            while (*hp && *hp != '\r' && *hp != '\n') { hex_len++; hp++; }
            int data_len = hex_len / 2;
            if (data_len > 248) data_len = 248;

            ota_buf[0] = ota_seq;
            for (i = 0; i < data_len; i++)
            {
                uint8_t b;
                if (hex_byte(p[i*2], p[i*2+1], &b) != 0) break;
                ota_buf[1 + i] = b;
            }
            {
                uint16_t crc = crc16_ccitt(&ota_buf[1], data_len);
                ota_buf[1 + data_len]     = crc & 0xFF;
                ota_buf[1 + data_len + 1] = (crc >> 8) & 0xFF;
            }
            gateway_send_uwb_ack(ota_dst_uid, FUNC_OTA_DATA, ota_buf, (uint8_t)(1 + data_len + 2));

            ota_offset = offset + (uint32_t)data_len;
            ota_seq++;

            /* +OTA_PROGRESS,<UID>,<OFFSET> */
            {
                char out[48];
                int pos;
                out[0] = '+'; out[1] = 'O'; out[2] = 'T'; out[3] = 'A'; out[4] = '_'; out[5] = 'P';
                out[6] = 'R'; out[7] = 'O'; out[8] = 'G'; out[9] = 'R'; out[10] = 'E';
                out[11] = 'S'; out[12] = 'S'; out[13] = ',';
                pos = 14;
                pos += fmt_str(&out[pos], uid_str);
                out[pos++] = ',';
                pos += fmt_dec(&out[pos], (int)ota_offset);
                out[pos] = '\0';
                test_run_info((unsigned char *)out);
            }
        }
    }
    else if (strncmp(cmd, "AT+OTA_END=", 11) == 0)
    {
        /* AT+OTA_END=<UID> → +OTA_DONE,<UID> */
        ota_active = 0;
        {
            char out[32];
            int pos;
            out[0] = '+'; out[1] = 'O'; out[2] = 'T'; out[3] = 'A'; out[4] = '_'; out[5] = 'D';
            out[6] = 'O'; out[7] = 'N'; out[8] = 'E'; out[9] = ',';
            pos = 10;
            pos += fmt_str(&out[pos], cmd + 11);
            out[pos] = '\0';
            test_run_info((unsigned char *)out);
        }
    }
    else if (strncmp(cmd, "AT+FWV", 6) == 0)
    {
        /* AT+FWV → +FWV,<type>,<uid>,<version> for all known devices */
        char out[48], ver[9];
        int pos;
        /* Gateway itself */
        fw_version_str(ver, FW_VERSION_U16);
        out[0] = '+'; out[1] = 'F'; out[2] = 'W'; out[3] = 'V'; out[4] = ',';
        out[5] = 'G'; out[6] = 'W'; out[7] = ',';
        out[8] = '0'; out[9] = '0'; out[10] = '0'; out[11] = '0'; out[12] = ',';
        pos = 13;
        pos += fmt_str(&out[pos], ver);
        out[pos] = '\0';
        test_run_info((unsigned char *)out);
        /* (Device versions are auto-detected via beacon/slave reports) */
    }
    /* Unknown command — silently ignore */
}

/*============================================================================
 * Flash config storage — STM32L412 last page (page 63 @ 0x0801F800, 2 KB)
 *============================================================================*/
#define FLASH_CONFIG_ADDR  0x0801F800UL
#define FLASH_CONFIG_MAGIC 0x55AA55AAUL

int gateway_flash_save_coord(int16_t x, int16_t y, int16_t z)
{
    /* TODO: Enable HAL_FLASH_MODULE_ENABLED when code size budget allows */
    (void)x; (void)y; (void)z;
    return 0;
}

int gateway_flash_load_coord(int16_t *x, int16_t *y, int16_t *z)
{
    /* TODO: Enable HAL_FLASH_MODULE_ENABLED when code size budget allows */
    (void)x; (void)y; (void)z;
    return -1;  /* never written */
}
