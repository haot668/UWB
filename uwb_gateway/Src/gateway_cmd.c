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
#include <example_selection.h>
#include <firmware_version.h>

/* Stub implementations live near fusion_gateway() below */

extern void test_run_info(unsigned char *data);

/* Gateway mode: 0=config(IDLE/TX), 1=TDOA(RX sniff). Set by START/STOP ACK. */
int gw_tdoa_active = 0;

/*============================================================================
 * String formatting helpers (avoid printf dependency)
 *============================================================================*/
static const char gw_hex[] = "0123456789ABCDEF";

/* Forward declaration — called from command handlers defined before it */
static void gateway_rx_flush(void);

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

int gateway_uart_rx_pending(void)
{
    if (rx_head >= rx_tail) return rx_head - rx_tail;
    else return GW_RX_BUF_SIZE - rx_tail + rx_head;
}

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
    int tail = rx_tail;  /* snapshot — don't consume until \n found */
    while (tail != rx_head && i < maxlen - 1)
    {
        char c = (char)rx_ring[tail];
        if (c == '\n')
        {
            /* strip trailing \r */
            if (i > 0 && buf[i - 1] == '\r') i--;
            buf[i] = '\0';
            rx_tail = (tail + 1) % GW_RX_BUF_SIZE;  /* commit: consume up to \n */
            return 1;
        }
        buf[i++] = c;
        tail = (tail + 1) % GW_RX_BUF_SIZE;
    }
    /* line too long — discard by consuming up to next \n */
    if (i >= maxlen - 1)
    {
        tail = rx_tail;
        while (tail != rx_head)
        {
            if (rx_ring[tail] == '\n') {
                rx_tail = (tail + 1) % GW_RX_BUF_SIZE;
                buf[0] = '\0';
                return 0;
            }
            tail = (tail + 1) % GW_RX_BUF_SIZE;
        }
    }
    /* No complete line yet — don't consume anything, return 0 */
    return 0;
}

/*============================================================================
 * OTA firmware upgrade — SW-ARQ (Stop-and-Wait Automatic Repeat reQuest)
 *
 *   Flow:  Host → AT+OTA_DATA → Gateway → UWB OTA_DATA → Device
 *          Device → UWB OTA_ACK → Gateway → +OTA_PROGRESS → Host → next chunk
 *
 *   Gateway sends one chunk, waits ≤100ms for ACK, retries up to 3×.
 *   CRC failures on device are SILENT → Gateway timeout triggers retry.
 *============================================================================*/
#define OTA_ARQ_TIMEOUT_MS  50u    /* Flash write 1.2ms + TDMA 2-10ms + ACK TX 0.1ms ≈ 12ms max */
#define OTA_ARQ_TIMEOUT_BLK0_MS 1000u  /* Block-0: staging erase ~660ms + margin */
#define OTA_ARQ_MAX_RETRIES 20
#define OTA_ARQ_BACKOFF_BASE_MS 10u  /* exponential backoff: 10,20,40,80...ms */

typedef enum {
    OTA_ARQ_IDLE = 0,
    OTA_ARQ_WAIT_ACK
} ota_arq_state_t;

static ota_arq_state_t ota_arq_state = OTA_ARQ_IDLE;
static uint16_t ota_dst_uid;
static uint32_t ota_total_size;
static uint32_t ota_offset;        /* cumulative offset (for +OTA_PROGRESS) */
static uint16_t ota_arq_seq;       /* current block sequence number (16-bit: max 65536 chunks) */
static uint8_t  ota_arq_buf[253];     /* pending chunk: seq(2)+data+crc(2)+pad(2) */
static uint8_t  ota_arq_buf_len;     /* total bytes in buf                           */
static uint8_t  ota_arq_data_len;    /* actual firmware data bytes (excl. overhead)  */
static uint32_t ota_arq_deadline;    /* HAL_GetTick() deadline for ACK                 */
static int      ota_arq_retries;
static uint8_t  ota_active;          /* OTA session active (for AT+OTA_START/END)      */

/*=== Broadcast OTA types and state ===*/
#define BCAST_OTA_MAX_DEVICES  4u
#define BCAST_OTA_ACK_WINDOW_MS  300u   /* normal chunk: 300ms ACK window (tag blinks may fill RX buffer) */
#define BCAST_OTA_ACK_BLK0_MS   3000u   /* block 0: 3.0s (device erases 660ms flash + safety margin) */
#define BCAST_OTA_MAX_RETRIES     5u    /* keep total poll time <15s (host timeout) */

typedef enum {
    BCAST_OTA_IDLE = 0,
    BCAST_OTA_PREP,
    BCAST_OTA_ACTIVE,
    BCAST_OTA_DONE
} bcast_ota_state_t;

typedef struct {
    uint16_t uid;
    uint8_t  present;
    uint8_t  done;
} bcast_ota_dev_t;

static bcast_ota_state_t bcast_state = BCAST_OTA_IDLE;
static bcast_ota_dev_t   bcast_devs[BCAST_OTA_MAX_DEVICES];
static uint8_t           bcast_num_devs;
static uint16_t          bcast_arq_seq;
static uint8_t           bcast_arq_buf[253];
static uint8_t           bcast_arq_buf_len;
static uint8_t           bcast_arq_data_len;
static uint32_t          bcast_arq_deadline;
static int               bcast_arq_retries;
static uint32_t          bcast_total_size;

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
#define GW_MAX_DEVICES        48
#define GW_TIMEOUT_MS         15000  /* 15 s → offline */
#define GW_HEARTBEAT_MS        8000  /*  8 s heartbeat */


typedef struct
{
    char     uid[8];
    char     type;    /* 'A'=Anchor, 'T'=Tag, 'G'=Gateway */
    uint16_t rank;    /* 16-bit anchor rank (short address) — may exceed 255 */
    uint8_t  online;
    uint8_t  seen_count; /* debounce: ≥2 sightings before reporting online */
    uint16_t fw_ver;  /* Packed firmware version (0x0001=V0.0.1) */
    uint32_t last_seen_ms;
    uint32_t last_heartbeat_ms;
    /* P1-4: UID fingerprint of the device.  The rank is a CRC16 of the
     * 96-bit UID, so two different anchors can share a rank — when the
     * fingerprint changes for the same rank we report +RANKCOLL. */
    uint64_t uid_fp;
    uint8_t  uid_fp_valid;
    uint8_t  net_state;   /* GW_NETSTATE_* — derived from the anchor's beacon */
} gw_dev_t;

static gw_dev_t gw_devs[GW_MAX_DEVICES];
static int      gw_dev_count = 0;
static char     master_uid[9] = "0001";

/* Network formation roster (P1-1).
 *
 * The gateway sends the participant list inside NETWORK_START, so every
 * anchor knows exactly how many members the network must have.  The Root
 * publishes a presence bitmap over the SAME ordered list in its TimeSync
 * beacon; the gateway keeps a copy of the list here so it can tell the host
 * which anchors never joined the network (a Root that only learns its
 * neighbours from what it happens to hear can never detect a one-way link).
 */
static uint16_t gw_net_roster[NET_MAX_PARTICIPANTS];
static int      gw_net_roster_n = 0;
static uint16_t gw_net_root = 0;
static uint16_t gw_net_present = 0;
static int      gw_net_present_valid = 0;

/* Find a device by its uid string.  Returns NULL if it is not known. */
static gw_dev_t *gw_find_dev(const char *uid)
{
    int i;
    for (i = 0; i < gw_dev_count; i++) {
        if (strcmp(gw_devs[i].uid, uid) == 0) return &gw_devs[i];
    }
    return NULL;
}

void gateway_set_master_uid(const char *uid)
{
    int i;
    for (i = 0; i < 8 && uid[i]; i++) master_uid[i] = uid[i];
    master_uid[i] = '\0';
}

void gateway_device_seen(const char *uid, const char *type, uint16_t rank)
{
    uint32_t now = HAL_GetTick();
    int i;
    for (i = 0; i < gw_dev_count; i++)
    {
        if (strcmp(gw_devs[i].uid, uid) == 0)
        {
            int was_offline = !gw_devs[i].online;
            /* Always refresh rank — it may have been truncated/stale on a
             * previous sighting (e.g. device re-registered with a new rank). */
            gw_devs[i].rank = rank;
            gw_devs[i].last_seen_ms = now;
            /* Increment debounce counter (saturating at 255) */
            if (gw_devs[i].seen_count < 255)
                gw_devs[i].seen_count++;
            /* Require ≥2 sightings before marking online (phantom filter) */
            if (gw_devs[i].seen_count < 2)
                return;
            gw_devs[i].online = 1;
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
        d->fw_ver    = 0;
        d->online    = 0;       /* stay offline until debounce satisfied */
        d->seen_count = 1;      /* first sighting */
        d->last_seen_ms = now;
        d->last_heartbeat_ms = now;
        d->uid_fp = 0;
        d->uid_fp_valid = 0;
        d->net_state = GW_NETSTATE_IDLE;
        /* Do NOT send +ONLINE yet — wait for 2nd sighting */
        return;
    }

    /* Table full and this is an unknown device → evict the LRU entry.
     * Prefer the oldest OFFLINE device (a genuinely departed one); if every
     * slot is still online, evict the oldest overall so a fresh device can
     * still register (the evicted one will re-register on its next sighting). */
    {
        int victim = -1;
        uint32_t oldest = 0xFFFFFFFFu;
        for (i = 0; i < gw_dev_count; i++) {
            if (!gw_devs[i].online && gw_devs[i].last_seen_ms < oldest) {
                oldest = gw_devs[i].last_seen_ms; victim = i;
            }
        }
        if (victim < 0) {
            oldest = 0xFFFFFFFFu;
            for (i = 0; i < gw_dev_count; i++) {
                if (gw_devs[i].last_seen_ms < oldest) {
                    oldest = gw_devs[i].last_seen_ms; victim = i;
                }
            }
        }
        if (victim < 0) return;   /* table empty — impossible */

        {
            gw_dev_t *d = &gw_devs[victim];
            strncpy(d->uid, uid, sizeof(d->uid) - 1);
            d->uid[sizeof(d->uid) - 1] = '\0';
            d->type       = (type && type[0] == 'T') ? 'T' : (type && type[0] == 'G') ? 'G' : 'A';
            d->rank       = rank;
            d->fw_ver     = 0;
            d->online     = 0;      /* stay offline until debounce satisfied */
            d->seen_count = 1;      /* first sighting */
            d->last_seen_ms = now;
            d->last_heartbeat_ms = now;
            d->uid_fp = 0;
            d->uid_fp_valid = 0;
            d->net_state = GW_NETSTATE_IDLE;
        }
        /* Like a fresh registration: no +ONLINE until the 2nd sighting. */
        return;
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

/* P1-4: rank collision detection.
 * The anchor rank is a CRC16 over the 96-bit STM32 UID, so two different
 * anchors can end up with the SAME 16-bit short address.  Both would then
 * be one and the same device in the host list (and one of them could steal
 * the other's TDMA slot).  The discovery frame carries a 64-bit fingerprint
 * of the UID; when it changes for a known rank we report the collision. */
void gateway_device_uid_fp(const char *uid, uint64_t fp)
{
    int i;
    for (i = 0; i < gw_dev_count; i++)
    {
        if (strcmp(gw_devs[i].uid, uid) != 0) continue;
        if (!gw_devs[i].uid_fp_valid) {
            gw_devs[i].uid_fp = fp;
            gw_devs[i].uid_fp_valid = 1;
            return;
        }
        if (gw_devs[i].uid_fp == fp) return;

        /* Different anchor behind the same rank — report once per pair. */
        {
            char out[80]; int pos = 0;
            out[pos++] = '+'; out[pos++] = 'R'; out[pos++] = 'A';
            out[pos++] = 'N'; out[pos++] = 'K'; out[pos++] = 'C';
            out[pos++] = 'O'; out[pos++] = 'L'; out[pos++] = 'L';
            out[pos++] = ',';
            pos += fmt_str(&out[pos], uid);
            out[pos++] = ',';
            pos += fmt_dec(&out[pos], (int)gw_devs[i].rank);
            out[pos++] = ',';
            out[pos++] = '1'; out[pos++] = '6';   /* 16-bit rank */
            out[pos] = '\0';
            test_run_info((unsigned char *)out);
        }
        /* Keep the newest fingerprint so the warning is not repeated. */
        gw_devs[i].uid_fp = fp;
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
            /* Only report when version changes (not on every blink) */
            if (gw_devs[i].fw_ver != fw_ver)
            {
                gw_devs[i].fw_ver = fw_ver;
                fw_version_str(ver, fw_ver);
                char out[48]; int pos;
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
    /* Unknown device — report once */
    fw_version_str(ver, fw_ver);
    {
        char out[48]; int pos;
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

void gateway_check_timeout(void)
{
    uint32_t now = HAL_GetTick();
    int i;

    /* P1-5: gateway self-heartbeat.  The gateway used to report its own
     * presence only inside AT+LIST, so the host had no way to tell "gateway
     * is alive but nothing else is talking" from "gateway is dead".  Emit
     * +ONLINE,GW01,GATEWAY,0 every 5 s from the main loop. */
    {
        static uint32_t last_gw_hb = 0;
        if ((uint32_t)(now - last_gw_hb) >= 5000u) {
            last_gw_hb = now;
            test_run_info((unsigned char *)"+ONLINE,GW01,GATEWAY,0");
        }
    }

    for (i = 0; i < gw_dev_count; i++)
    {
        uint32_t age_seen, age_hb;
        if (!gw_devs[i].online) continue;

        age_seen = (uint32_t)(now - gw_devs[i].last_seen_ms);
        age_hb   = (uint32_t)(now - gw_devs[i].last_heartbeat_ms);

        if (age_seen > GW_TIMEOUT_MS)
        {
            gw_devs[i].online = 0;
            gw_devs[i].last_heartbeat_ms = now;  /* prevent immediate re-online */
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
        else if (!gw_devs[i].online)
        {
            ; /* was marked offline above — skip heartbeat this round */
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

/* Send a UWB command and wait for ACK with a specific func code.
 * Returns 1 if ACK received, 0 if all retries exhausted.
 * delay_ms: per-attempt RX poll window
 * retries: max TX attempts */
static int gateway_send_cmd_wait_ack(uint16_t dst, uint8_t cmd_func,
                                      uint8_t *payload, uint8_t payload_len,
                                      uint8_t ack_func, int delay_ms, int retries)
{
    int _r, ack_ok = 0;
    for (_r = 0; _r < retries && !ack_ok; _r++) {
        gateway_send_uwb_ack(dst, cmd_func, payload, payload_len);
        {
            uint32_t dl = HAL_GetTick() + (uint32_t)delay_ms;
            /* Disable DW3000 EXTI IRQ during polling — the IRQ handler's
             * dwt_forcetrxoff() issues SPI commands that collide with our
             * own SPI reads (SYS_STATUS, RX_FINFO, rxdata), corrupting
             * frame data.  The polling loop handles all RX itself. */
            port_DisableEXT_IRQ();
            while (HAL_GetTick() < dl) {
                uint32_t st = dwt_read32bitreg(SYS_STATUS_ID);

                /* Check for received frame FIRST. */
                if (st & SYS_STATUS_RXFCG_BIT_MASK) {
                    uint32_t fl = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
                    if (fl >= GW_HDR_LEN && fl <= 32) {
                        uint8_t rb[32]; dwt_readrxdata(rb, fl, 0);
                        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
                        /* Diagnostic: print func code of every RX frame */
                        { char _rx[20]; int _rxp=0;
                          _rx[_rxp++]='R'; _rx[_rxp++]='X'; _rx[_rxp++]=':';
                          _rx[_rxp++]=gw_hex[(rb[9]>>4)&0xF]; _rx[_rxp++]=gw_hex[rb[9]&0xF];
                          _rx[_rxp++]=','; _rx[_rxp++]='L'; _rx[_rxp++]='=';
                          _rx[_rxp++]=gw_hex[(fl>>4)&0xF]; _rx[_rxp++]=gw_hex[fl&0xF];
                          _rx[_rxp++]='\r'; _rx[_rxp++]='\n'; _rx[_rxp]='\0';
                          test_run_info((unsigned char *)_rx); }
                        if (rb[9] == ack_func) { ack_ok = 1; break; }
                    } else {
                        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
                    }
                    /* MUST re-enable RX after consuming a frame: the EXTI
                     * IRQ handler calls dwt_forcetrxoff() which forces the
                     * DW3000 to IDLE — RXAUTR can't keep RX running. */
                    dwt_rxenable(DWT_START_RX_IMMEDIATE);
                }

                /* Clean RX errors AFTER handling any good frame. */
                if (st & SYS_STATUS_ALL_RX_ERR) {
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                    dwt_rxenable(DWT_START_RX_IMMEDIATE);
                }

                /* No unconditional dwt_rxenable() — prevents aborting
                 * incoming frames mid-preamble.  RX is maintained by the
                 * explicit dwt_rxenable() calls above (after frame/error). */
                Sleep(1);
            }
            /* Re-enable DW3000 EXTI IRQ after polling — the sniffer loop
             * (or next TX) needs IRQ for normal operation.  Clear stale
             * pending bit first so a frame received during the final ms
             * of polling doesn't trigger a spurious IRQ. */
            EXTI->PR1 = (1u << 12);
            port_EnableEXT_IRQ();
        }
    }
    return ack_ok;
}

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
        uint16_t total = GW_HDR_LEN + payload_len;  /* DW3000 auto-appends FCS */
        int tx_ok = 0;

        /* Disable DW3000 IRQ during TX to prevent EXTI handler's
         * dwt_forcetrxoff() from aborting our transmission. */
        port_DisableEXT_IRQ();

        dwt_forcetrxoff();

        port_rf_switch_tx();  /* PA7=TX_EN=1, PA6=RX_EN=0 */

        dwt_writetxdata(total, frame, 0);
        dwt_writetxfctrl(total, 0, 0);
        if (dwt_starttx(DWT_START_TX_IMMEDIATE) == DWT_SUCCESS) {
            { uint32_t tx_dl = HAL_GetTick() + 10u;
              while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {
                if (HAL_GetTick() > tx_dl) break;
              }
            }
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
            tx_ok = 1;
        }

        port_rf_switch_rx();  /* PA7=TX_EN=0, PA6=RX_EN=1 */
        dwt_forcetrxoff();

        /*=== Print TX confirm BEFORE re-enabling EXTI ===
         * UART TX takes ~1ms — if EXTI were enabled during this time,
         * a received frame would trigger IRQ → dwt_forcetrxoff() → DW3000=IDLE,
         * breaking any inline polling loop that follows.
         * Printing NOW (EXTI still disabled) eliminates this gap entirely. */
        if (func == FUNC_PREP_CMD    || func == FUNC_COORD_WRITE ||
            func == FUNC_OTA_PREPARE || func == FUNC_STOP_TDOA  ||
            func == FUNC_START_TDOA  || func == FUNC_OTA_END    ||
            func == FUNC_OTA_ABORT   || func == FUNC_OTA_DATA) {
            char _tx[24]; int _txp = 0;
            _tx[_txp++]='T'; _tx[_txp++]='X'; _tx[_txp++]=',';
            _tx[_txp++]='F'; _tx[_txp++]='=';
            _tx[_txp++]=gw_hex[(func>>4)&0xF]; _tx[_txp++]=gw_hex[func&0xF];
            _tx[_txp++]=',';
            if (tx_ok) {
                _tx[_txp++]='O';
            } else {
                _tx[_txp++]='E'; _tx[_txp++]='R'; _tx[_txp++]='R';
            }
            _tx[_txp++]='\r'; _tx[_txp++]='\n'; _tx[_txp]='\0';
            test_run_info((unsigned char *)_tx);
        }

        /* Clear any EXTI pending that may have been set during TX,
         * THEN re-enable IRQ — gap from here to caller's disable is
         * just a few instructions (function epilogue). */
        EXTI->PR1 = (1u << 12);
        port_EnableEXT_IRQ();

        dwt_rxenable(DWT_START_RX_IMMEDIATE);
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
        /* Register the device so it appears in the host device list. */
        {
            char uid[8];
            uid[0] = gw_hex[(src_addr >> 12) & 0xF]; uid[1] = gw_hex[(src_addr >>  8) & 0xF];
            uid[2] = gw_hex[(src_addr >>  4) & 0xF]; uid[3] = gw_hex[ src_addr        & 0xF];
            uid[4] = '\0';
            gateway_device_seen(uid, "ANCHOR", src_addr);
        }
        /* +FIND,<UID_HEX4>,<STATE> */
        out[0] = '+'; out[1] = 'F'; out[2] = 'I'; out[3] = 'N'; out[4] = 'D'; out[5] = ',';
        pos = 6;
        pos += fmt_hex4(&out[pos], src_addr);
        out[pos++] = ',';
        out[pos++] = state ? '1' : '0';
        out[pos] = '\0';
        test_run_info((unsigned char *)out);
    }
    else if (func == FUNC_ANCHOR_DISCOVERY)
    {
        /* Anchor discovery beacon (0xEC) — an anchor that is powered on but
         * not yet in a network is announcing itself.  Register it so the
         * operator can pick it (or another anchor) as Root.
         * src_addr = anchor rank (16-bit short address). */
        char uid[8];
        uid[0] = gw_hex[(src_addr >> 12) & 0xF]; uid[1] = gw_hex[(src_addr >>  8) & 0xF];
        uid[2] = gw_hex[(src_addr >>  4) & 0xF]; uid[3] = gw_hex[ src_addr        & 0xF];
        uid[4] = '\0';
        gateway_device_seen(uid, "ANCHOR", src_addr);
        /* Firmware version carried in the discovery payload [12..13] */
        if (len >= (uint16_t)(GW_HDR_LEN + 4)) {
            uint16_t fw = (uint16_t)(frame[GW_HDR_LEN+2] | (frame[GW_HDR_LEN+3] << 8));
            gateway_device_version(uid, fw);
        }
        /* P1-4: UID fingerprint — detect two different anchors that ended up
         * behind the same 16-bit rank (they would collapse into one device
         * in the host list and fight over one TDMA slot). */
        if (len >= (uint16_t)(DL_TS_DISC_UID_IDX + 8)) {
            uint64_t fp = 0;
            int k;
            for (k = 0; k < 8; k++)
                fp |= ((uint64_t)frame[DL_TS_DISC_UID_IDX + k]) << (k * 8);
            gateway_device_uid_fp(uid, fp);
        }
        /* P1-3: report the anchor's network state (0=idle, 1=starting,
         * 2=wait_confirm, 3=running) so the host can verify — 2..3 s after
         * NETSTART — that every roster member actually reached RUNNING.
         * The state byte lives at payload offset 4 (frame index 14). */
        if (len >= (uint16_t)(GW_HDR_LEN + 5) && frame[GW_HDR_LEN + 4] <= 3) {
            char sout[24];
            int sp = 0;
            sout[sp++]='+'; sout[sp++]='N'; sout[sp++]='E'; sout[sp++]='T';
            sout[sp++]='S'; sout[sp++]='T'; sout[sp++]='A'; sout[sp++]='T';
            sout[sp++]='E'; sout[sp++]=',';
            sp += fmt_hex4(&sout[sp], src_addr);
            sout[sp++]=',';
            sout[sp++] = (char)('0' + frame[GW_HDR_LEN + 4]);
            sout[sp]='\0';
            test_run_info((unsigned char *)sout);
        }
    }
    else if (func == FUNC_DL_POS_REPORT)
    {
        int16_t x = (int16_t)(frame[GW_HDR_LEN]     | (frame[GW_HDR_LEN+1] << 8));
        int16_t y = (int16_t)(frame[GW_HDR_LEN+2]   | (frame[GW_HDR_LEN+3] << 8));
        int16_t z = (int16_t)(frame[GW_HDR_LEN+4]   | (frame[GW_HDR_LEN+5] << 8));
        uint16_t q = (uint16_t)(frame[GW_HDR_LEN+6] | (frame[GW_HDR_LEN+7] << 8));
        /* Mark tag online + extract firmware version */
        {
            char uid[8];
            uid[0] = gw_hex[(src_addr >> 12) & 0xF]; uid[1] = gw_hex[(src_addr >>  8) & 0xF];
            uid[2] = gw_hex[(src_addr >>  4) & 0xF]; uid[3] = gw_hex[ src_addr        & 0xF];
            uid[4] = '\0';
            /* The tag's source address IS its id (CRC16 over its UID), so
             * the host can tell two tags apart.  Rank 0 stays reserved for
             * the gateway. */
            gateway_device_seen(uid, "TAG", src_addr);
            /* Extract firmware version from position report (bytes 18-19) */
            if (len >= GW_HDR_LEN + 10) {
                uint16_t fw = (uint16_t)(frame[GW_HDR_LEN+8] | (frame[GW_HDR_LEN+9] << 8));
                gateway_device_version(uid, fw);
            }
        }
        /* +POS,<UID>,<X.XX>,<Y.YY>,<Z.ZZ>,<Q> — coords in meters */
        out[0] = '+'; out[1] = 'P'; out[2] = 'O'; out[3] = 'S'; out[4] = ',';
        pos = 5;
        pos += fmt_hex4(&out[pos], src_addr);
        out[pos++] = ',';
        pos += fmt_coord(&out[pos], (int)x);
        out[pos++] = ',';
        pos += fmt_coord(&out[pos], (int)y);
        out[pos++] = ',';
        pos += fmt_coord(&out[pos], (int)z);
        out[pos++] = ',';
        pos += fmt_dec(&out[pos], (int)q);
        out[pos] = '\0';
        test_run_info((unsigned char *)out);
    }
    else if (func == FUNC_OTA_ACK)
    {
        /* +OTA_ACK,<UID_HEX4>,<SEQ>,<STATUS>
         * seq    = frame[10..11] 16-bit echo of chunk sequence number
         * status = frame[12]     0=OK, 1=CRC error, 2=flash error */
        uint16_t seq    = (uint16_t)(frame[GW_HDR_LEN]) | ((uint16_t)(frame[GW_HDR_LEN + 1]) << 8);
        uint8_t  status = frame[GW_HDR_LEN + 2];

        /* Broadcast OTA: track multi-device ACKs.  If consumed by
         * broadcast handler, skip unicast ARQ processing below. */
        if (bcast_state == BCAST_OTA_ACTIVE) {
            gateway_bcast_ota_on_ack(seq, status, src_addr);
            /* Don't print per-device ACK in broadcast mode — host
             * gets summary via +OTA_PROGRESS. */
            return;
        }

        /* Feed into SW-ARQ state machine first (may print +OTA_PROGRESS).
         * Only print +OTA_ACK if this was the FIRST copy — suppress duplicates
         * from the device's 2× burst to avoid flooding the host UART parser. */
        if (gateway_ota_arq_on_ack(seq, status)) {
            out[0] = '+'; out[1] = 'O'; out[2] = 'T'; out[3] = 'A';
            out[4] = '_'; out[5] = 'A'; out[6] = 'C'; out[7] = 'K';
            out[8] = ','; pos = 9;
            pos += fmt_hex4(&out[pos], src_addr);
            out[pos++] = ',';
            out[pos++] = gw_hex[(seq >> 12) & 0xF];
            out[pos++] = gw_hex[(seq >> 8) & 0xF];
            out[pos++] = gw_hex[(seq >> 4) & 0xF];
            out[pos++] = gw_hex[seq & 0xF];
            out[pos++] = ',';
            out[pos++] = status == 0 ? '0'
                       : status == 1 ? '1'
                       : '2';
            out[pos] = '\0';
            test_run_info((unsigned char *)out);
        }
    }
    else if (func == FUNC_DOWNLINK_BEACON)
    {
        // Anchor TimeSync beacon (0xEE) — mark anchor online
        // src_addr = anchor's rank (short address from frame[7..8])
        char uid[8];
        uid[0] = gw_hex[(src_addr >> 12) & 0xF];
        uid[1] = gw_hex[(src_addr >>  8) & 0xF];
        uid[2] = gw_hex[(src_addr >>  4) & 0xF];
        uid[3] = gw_hex[ src_addr        & 0xF];
        uid[4] = '\0';
        gateway_device_seen(uid, "ANCHOR", src_addr);

        /* P1-5: derive the anchor's network state from the beacon it just
         * sent, and report it to the host.
         *
         * The gateway HEARS every beacon of the grid, so it knows which
         * anchors are really on it — no extra uplink is needed:
         *   level 0                → the Root, running
         *   level 1, slot != 255   → admitted to the grid, running
         *   level 1, slot == 255   → still waiting for the Root to confirm
         *
         * Nothing ever emitted +NETSTATE before, so the host's post-
         * formation check reported EVERY member as "未上报" and the map
         * could not tell who was in the network.  The line is sent only
         * when the state CHANGES, so the UART stays quiet.
         */
        if (len >= (uint16_t)(DL_TS_LEVEL_IDX + 1)) {
            uint8_t st = GW_NETSTATE_IDLE;
            uint8_t level = frame[DL_TS_LEVEL_IDX];
            uint8_t slot  = frame[DL_TS_SLOT_IDX];
            gw_dev_t *d;

            if (level == 0)        st = GW_NETSTATE_RUNNING;
            else if (slot != 255u) st = GW_NETSTATE_RUNNING;
            else                   st = GW_NETSTATE_WAIT;

            d = gw_find_dev(uid);
            if (d && d->net_state != st) {
                char sout[32];
                int sp = 0;
                d->net_state = st;
                sout[sp++]='+'; sout[sp++]='N'; sout[sp++]='E'; sout[sp++]='T';
                sout[sp++]='S'; sout[sp++]='T'; sout[sp++]='A'; sout[sp++]='T';
                sout[sp++]='E'; sout[sp++]=',';
                sp += fmt_str(&sout[sp], uid);
                sout[sp++]=',';
                sout[sp++] = (char)('0' + st);
                sout[sp]='\0';
                test_run_info((unsigned char *)sout);
            }
        }

        /* P1-1: the Root publishes a presence bitmap over the roster it was
         * given in NETWORK_START.  Compare it with our copy of the roster and
         * report the members that are still missing — that is the only way to
         * detect an anchor that never joined (one-way link, out of range). */
        if (gw_net_roster_n > 0 && src_addr == gw_net_root &&
            len >= (uint16_t)(DL_TS_PRESENT_IDX + 2))
        {
            uint16_t mask = (uint16_t)(frame[DL_TS_PRESENT_IDX] |
                        ((uint16_t)frame[DL_TS_PRESENT_IDX + 1] << 8));
            if (!gw_net_present_valid || mask != gw_net_present)
            {
                uint16_t missing[NET_MAX_PARTICIPANTS];
                int miss_n = 0, i2;
                gw_net_present = mask;
                gw_net_present_valid = 1;
                for (i2 = 0; i2 < gw_net_roster_n; i2++) {
                    if ((mask & (uint16_t)(1u << i2)) == 0)
                        missing[miss_n++] = gw_net_roster[i2];
                }
                {
                    char out[128];
                    int pos = 0;
                    out[pos++]='+'; out[pos++]='N'; out[pos++]='E'; out[pos++]='T';
                    out[pos++]='M'; out[pos++]='I'; out[pos++]='S'; out[pos++]='S';
                    out[pos++]=',';
                    pos += fmt_dec(&out[pos], (int)gw_net_root);
                    out[pos++]=',';
                    pos += fmt_dec(&out[pos], miss_n);
                    for (i2 = 0; i2 < miss_n; i2++) {
                        out[pos++]=',';
                        pos += fmt_dec(&out[pos], (int)missing[i2]);
                    }
                    out[pos]='\0';
                    test_run_info((unsigned char *)out);
                }
            }
        }
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
    int val = 0, neg = 0;
    if (*s == '-') { neg = 1; s++; }
    while (*s >= '0' && *s <= '9') { val = val * 10 + (*s - '0'); s++; }
    return neg ? -val : val;
}

/* Parse coordinate — returns cm units.
 * Accepts optional decimal point with up to 2 fractional digits.
 *    "2.50" → 250    "1.0" → 100    "-3.5" → -350    "0" → 0   */
static int parse_coord(const char *s)
{
    int val = 0, neg = 0, frac = 0, frac_digits = 0;
    if (*s == '-') { neg = 1; s++; }
    while (*s >= '0' && *s <= '9') { val = val * 10 + (*s - '0'); s++; }
    if (*s == '.') {
        s++;
        while (*s >= '0' && *s <= '9' && frac_digits < 2) {
            frac = frac * 10 + (*s - '0');
            frac_digits++;
            s++;
        }
        /* skip any remaining fractional digits beyond 2 */
        while (*s >= '0' && *s <= '9') s++;
    }
    /* Normalize: if only 1 fractional digit, scale to 2 digits */
    if (frac_digits == 1) frac *= 10;
    val = val * 100 + frac;
    return neg ? -val : val;
}

/* Format coordinate (cm) as meters string with 2 fractional digits.
 *   250 → "2.50"    100 → "1.00"    -350 → "-3.50"    0 → "0.00"   */
static int fmt_coord(char *buf, int val_cm)
{
    int pos = 0;
    unsigned u, int_part, frac_part;
    char tmp[12];
    int i;

    if (val_cm < 0) { buf[pos++] = '-'; u = (unsigned)(-val_cm); }
    else             { u = (unsigned)val_cm; }

    int_part = u / 100u;
    frac_part = u % 100u;

    /* integer part */
    if (int_part == 0) { buf[pos++] = '0'; }
    else {
        i = 0;
        while (int_part > 0) { tmp[i++] = '0' + (int_part % 10); int_part /= 10; }
        while (i > 0) { buf[pos++] = tmp[--i]; }
    }

    /* decimal point + 2 fractional digits */
    buf[pos++] = '.';
    buf[pos++] = '0' + (char)(frac_part / 10u);
    buf[pos++] = '0' + (char)(frac_part % 10u);

    return pos;
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

extern uint32_t g_find_blink_until;  /* fusion_anchor.c */

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

        /* Forward FIND to anchor via UWB, wait for FIND_ACK (0xF3).
         * FIND_REQ is only 11B — DW3210 corrupts last 2B of short frames.
         * Pad 2 extra bytes (13B total) to move func+payload out of danger zone. */
        {
            int _r, ack_ok = 0;
            uint8_t payload[3];
            payload[0] = (uint8_t)on_off;
            payload[1] = 0xCC;  /* padding — avoids DW3210 last-2B RX bug */
            payload[2] = 0xDD;  /* padding */
            for (_r = 0; _r < 3 && !ack_ok; _r++) {
                gateway_send_uwb_ack(dst, FUNC_FIND_REQ, payload, 3);
                { uint32_t dl = HAL_GetTick() + (_r < 2 ? 30u : 100u);
                  port_DisableEXT_IRQ();
                  while (HAL_GetTick() < dl) {
                    uint32_t st = dwt_read32bitreg(SYS_STATUS_ID);
                    if (st & SYS_STATUS_ALL_RX_ERR) {
                        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                        dwt_rxenable(DWT_START_RX_IMMEDIATE);
                        Sleep(1); continue; }
                    if (st & SYS_STATUS_RXFCG_BIT_MASK) {
                        uint32_t fl = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
                        if (fl >= GW_HDR_LEN && fl <= 32) {
                            uint8_t rb[32]; dwt_readrxdata(rb, fl, 0);
                            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
                            if (rb[9] == FUNC_FIND_ACK) { ack_ok = 1; break; }
                        } else { dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK); }
                        /* IRQ handler calls dwt_forcetrxoff() → must re-enter RX */
                        dwt_rxenable(DWT_START_RX_IMMEDIATE);
                    }
                    Sleep(1);
                  }
                  EXTI->PR1 = (1u << 12);
                  port_EnableEXT_IRQ();
                }
            }
            if (ack_ok) send_find_ack(dst, on_off);
            else { char _e[24]; int _p=0; _e[_p++]='+'; _e[_p++]='F'; _e[_p++]='I'; _e[_p++]='N'; _e[_p++]='D';
                   _e[_p++]='_'; _e[_p++]='E'; _e[_p++]='R'; _e[_p++]='R'; _e[_p++]=',';
                   _p += fmt_hex4(&_e[_p], dst); _e[_p++]='\r'; _e[_p++]='\n'; _e[_p]='\0';
                   test_run_info((unsigned char *)_e); }
            gateway_rx_flush();  /* clean RX state after TX+ACK polling */
        }
    }
}

/* Build "+COORDAck,<UID4>,<X.XX>,<Y.YY>,<Z.ZZ>" and send on UART.
 * x/y/z are in cm. */
static void send_coord_ack(uint16_t uid, int x, int y, int z)
{
    char ack[48];
    int pos;
    ack[0] = '+'; ack[1] = 'C'; ack[2] = 'O'; ack[3] = 'O'; ack[4] = 'R'; ack[5] = 'D'; ack[6] = 'A';
    ack[7] = 'c'; ack[8] = 'k'; ack[9] = ',';
    pos = 10;
    pos += fmt_hex4(&ack[pos], uid);
    ack[pos++] = ',';
    pos += fmt_coord(&ack[pos], x);
    ack[pos++] = ',';
    pos += fmt_coord(&ack[pos], y);
    ack[pos++] = ',';
    pos += fmt_coord(&ack[pos], z);
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
    x = parse_coord(p);
    while (*p && *p != ',') p++;
    if (*p != ',') return; p++;
    y = parse_coord(p);
    while (*p && *p != ',') p++;
    if (*p != ',') return; p++;
    z = parse_coord(p);

    {
        uint16_t dst = parse_uid(uid_str);
        uint8_t payload[8];
        int ack_ok = 0;
#ifdef FUSION_DOWNLINK_MODE
        dl_set_anchor_coord((uint8_t)dst, (int16_t)x, (int16_t)y, (int16_t)z);
#endif
        payload[0] = x & 0xFF;  payload[1] = (x >> 8) & 0xFF;
        payload[2] = y & 0xFF;  payload[3] = (y >> 8) & 0xFF;
        payload[4] = z & 0xFF;  payload[5] = (z >> 8) & 0xFF;
        payload[6] = 0xCC;       /* pad — avoid DW3210 last-2B RX corruption */
        payload[7] = 0xDD;       /* pad */

        /* Write coordinate directly — do NOT send PREP_CMD first.
         * PREP_CMD put the Master into config mode (pausing TDMA) and was
         * never resumed, so after a coordinate write during TDOA every
         * slave dropped off the air.  The Master's RX phase (~470ms of each
         * 500ms cycle) hears COORD_WRITE reliably with retries, and the
         * anchor-side handler re-syncs DLY_TS after the ACK TX. */
        ack_ok = gateway_send_cmd_wait_ack(dst, FUNC_COORD_WRITE,
                                           payload, 8,
                                           FUNC_COORD_ACK, 200, 5);

        if (ack_ok) send_coord_ack(dst, x, y, z);
        else { char err[24]; int pos = 0;
            err[pos++]='+'; err[pos++]='C'; err[pos++]='O'; err[pos++]='O'; err[pos++]='R'; err[pos++]='D';
            err[pos++]='_'; err[pos++]='E'; err[pos++]='R'; err[pos++]='R'; err[pos++]=',';
            pos += fmt_hex4(&err[pos], dst);
            err[pos++]='\r'; err[pos++]='\n'; err[pos]='\0';
            test_run_info((unsigned char *)err); }
        /* Diagnostic */
        {
            char _dx[80]; int _dxp = 0;
            _dxp += fmt_str(&_dx[_dxp], "TX,C=");
            _dxp += fmt_hex4(&_dx[_dxp], dst);
            _dxp += fmt_str(&_dx[_dxp], ",(");
            _dxp += fmt_coord(&_dx[_dxp], x);
            _dxp += fmt_str(&_dx[_dxp], ",");
            _dxp += fmt_coord(&_dx[_dxp], y);
            _dxp += fmt_str(&_dx[_dxp], ",");
            _dxp += fmt_coord(&_dx[_dxp], z);
            _dxp += fmt_str(&_dx[_dxp], "),");
            _dx[_dxp++] = ack_ok ? 'O' : 'F';
            _dx[_dxp++] = '\r'; _dx[_dxp++] = '\n'; _dx[_dxp] = '\0';
            test_run_info((unsigned char *)_dx);
        }
        gateway_rx_flush();
    }
}

/* AT+COORD?=<UID> — read the coordinate back FROM THE ANCHOR (P1-2).
 *
 * The coordinate is the one thing the TDOA solve depends on, and it lives
 * on the anchor.  The old implementation read the GATEWAY's flash — which
 * is never written (the stub always fails) — so every query answered
 * 0,0,0 and the host could not tell a real (0,0,0) from a coordinate the
 * anchor had lost.  Now the gateway asks the anchor over UWB and reports
 * what the anchor says; when the anchor does not answer it says so with
 * "+COORDERR,<uid>" instead of pretending the coordinate is zero. */
static void gateway_coord_read(const char *cmd)
{
    const char *p = cmd;
    char uid_str[8];
    int i;
    uint16_t dst;
    uint8_t payload[4];
    uint8_t resp[8];
    int ok = 0;
    int r;

    if (*p != '=') return;
    p++;
    i = 0;
    while (*p && *p != ',' && i < (int)sizeof(uid_str) - 1)
        uid_str[i++] = *p++;
    uid_str[i] = '\0';
    dst = parse_uid(uid_str);

    payload[0] = 0; payload[1] = 0; payload[2] = 0; payload[3] = 0;

    /* Ask the anchor and wait for its answer — the same shape as the write
     * path (a few tries, a short window each). */
    for (r = 0; r < 3 && !ok; r++) {
        gateway_send_uwb_ack(dst, FUNC_COORD_READ, payload, 4);
        {
            uint32_t dl = HAL_GetTick() + 200u;
            port_DisableEXT_IRQ();
            while (HAL_GetTick() < dl) {
                uint32_t st = dwt_read32bitreg(SYS_STATUS_ID);
                if (st & SYS_STATUS_ALL_RX_ERR) {
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                    dwt_rxenable(DWT_START_RX_IMMEDIATE);
                    Sleep(1); continue;
                }
                if (st & SYS_STATUS_RXFCG_BIT_MASK) {
                    uint32_t fl = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
                    if (fl >= GW_HDR_LEN && fl <= 32) {
                        uint8_t rb[32];
                        dwt_readrxdata(rb, fl, 0);
                        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
                        if (rb[9] == FUNC_COORD_REPORT) {
                            uint16_t src_rank = (uint16_t)(rb[7] | ((uint16_t)rb[8] << 8));
                            if (src_rank == dst) {
                                resp[0] = rb[10]; resp[1] = rb[11];
                                resp[2] = rb[12]; resp[3] = rb[13];
                                resp[4] = rb[14]; resp[5] = rb[15];
                                ok = 1; break;
                            }
                        }
                    } else {
                        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
                    }
                    dwt_rxenable(DWT_START_RX_IMMEDIATE);
                }
                Sleep(1);
            }
            EXTI->PR1 = (1u << 12);
            port_EnableEXT_IRQ();
        }
    }
    gateway_rx_flush();

    if (ok) {
        int16_t fx = (int16_t)((uint16_t)resp[0] | ((uint16_t)resp[1] << 8));
        int16_t fy = (int16_t)((uint16_t)resp[2] | ((uint16_t)resp[3] << 8));
        int16_t fz = (int16_t)((uint16_t)resp[4] | ((uint16_t)resp[5] << 8));
        char ack[48];
        int pos = 0;
        ack[pos++] = '+'; ack[pos++] = 'C'; ack[pos++] = 'O'; ack[pos++] = 'O';
        ack[pos++] = 'R'; ack[pos++] = 'D'; ack[pos++] = ',';
        pos += fmt_str(&ack[pos], uid_str);
        ack[pos++] = ',';
        pos += fmt_coord(&ack[pos], fx);
        ack[pos++] = ',';
        pos += fmt_coord(&ack[pos], fy);
        ack[pos++] = ',';
        pos += fmt_coord(&ack[pos], fz);
        ack[pos] = '\0';
        test_run_info((unsigned char *)ack);
    } else {
        char err[32];
        int pos = 0;
        err[pos++] = '+'; err[pos++] = 'C'; err[pos++] = 'O'; err[pos++] = 'O';
        err[pos++] = 'R'; err[pos++] = 'D'; err[pos++] = 'E'; err[pos++] = 'R';
        err[pos++] = 'R'; err[pos++] = ',';
        pos += fmt_str(&err[pos], uid_str);
        err[pos] = '\0';
        test_run_info((unsigned char *)err);
    }
}

/* Emit one device line: "<head>,<uid>,<TYPE>,<rank>"
 * head is "+ONLINE" (confirmed device) or "+SEEN" (sighted once — the
 * debounce in gateway_device_seen() has not been satisfied yet). */
static void gateway_emit_dev_line(const char *head, gw_dev_t *d)
{
    char out[48];
    int pos = 0, i;

    for (i = 0; head[i]; i++) out[pos++] = head[i];
    out[pos++] = ',';
    pos += fmt_str(&out[pos], d->uid);
    out[pos++] = ',';
    if (d->type == 'T')
    { out[pos++] = 'T'; out[pos++] = 'A'; out[pos++] = 'G'; }
    else if (d->type == 'G')
    { out[pos++] = 'G'; out[pos++] = 'A'; out[pos++] = 'T'; out[pos++] = 'E'; out[pos++] = 'W'; out[pos++] = 'A'; out[pos++] = 'Y'; }
    else
    { out[pos++] = 'A'; out[pos++] = 'N'; out[pos++] = 'C'; out[pos++] = 'H'; out[pos++] = 'O'; out[pos++] = 'R'; }
    out[pos++] = ',';
    /* Full 16-bit rank in DECIMAL — the host sends this value back in
     * AT+NETSTART, so truncating it here breaks Root selection. */
    pos += fmt_dec(&out[pos], (int)d->rank);
    out[pos] = '\0';
    test_run_info((unsigned char *)out);
}

/* AT+LIST — output +ONLINE for each known device.
 * Gateway sniffs continuously, so device list is always current.
 * Always include Gateway itself first — the host needs this to show it online
 * on reconnect (the startup +ONLINE was already consumed).
 *
 * P2-3: devices that have been sighted ONCE but have not yet satisfied the
 * 2-sighting debounce are reported as "+SEEN".  Without this line a freshly
 * powered anchor stays invisible to the host until the second sighting
 * arrives, which is exactly the "上位机搜不到基站" symptom: the host polls
 * AT+LIST faster than the debounce can complete. */
static void gateway_send_list(void)
{
    int i;

    /* Gateway self-registration — must appear in every AT+LIST response */
    test_run_info((unsigned char *)"+ONLINE,GW01,GATEWAY,0");

    for (i = 0; i < gw_dev_count; i++)
    {
        if (gw_devs[i].online)
            gateway_emit_dev_line("+ONLINE", &gw_devs[i]);
        else if (gw_devs[i].seen_count >= 1)
            gateway_emit_dev_line("+SEEN", &gw_devs[i]);
    }
}

/*============================================================================
 * RX flush — force-clean DW3000 RX after TX+ACK sequence.
 * Prevents "stuck RX" state that causes the sniffer to miss frames.
 * Must be called after any handler that does TX + inline ACK polling
 * (COORD, FIND, START, STOP, NETWORK) before returning to sniffer loop.
 *============================================================================*/
static void gateway_rx_flush(void)
{
    int buf_idx;

    /* 1. Force DW3000 out of whatever state it's in → IDLE */
    dwt_forcetrxoff();

    /* 2. Drain both RX buffers (DW3000 has double-buffered RX).
     *    During ACK polling, non-ACK frames may have been received
     *    and left unread, corrupting the buffer state.
     *
     *    P2-5: frames that arrived WHILE the command was running are real
     *    traffic (anchor discovery beacons, TimeSync beacons) — dropping
     *    them blind meant an anchor that announced itself exactly during the
     *    command was lost for good ("上位机搜不到基站").  Parse them into the
     *    device table first, then flush. */
    port_DisableEXT_IRQ();
    for (buf_idx = 0; buf_idx < 2; buf_idx++)
    {
        uint32_t st = dwt_read32bitreg(SYS_STATUS_ID);
        if (st & SYS_STATUS_RXFCG_BIT_MASK)
        {
            uint32_t fl = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
            if (fl > 0 && fl <= 127)
            {
                uint8_t dummy[128];
                dwt_readrxdata(dummy, fl, 0);
                if (fl >= GW_HDR_LEN && fl <= 127)
                    gateway_handle_uwb_rx(dummy, (uint16_t)fl);
            }
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
        }
    }
    EXTI->PR1 = (1u << 12);
    port_EnableEXT_IRQ();

    /* 3. Clear all RX error / overflow flags accumulated during polling */
    dwt_write32bitreg(SYS_STATUS_ID,
        SYS_STATUS_ALL_RX_ERR | SYS_STATUS_RXFCG_BIT_MASK
        | SYS_STATUS_RXOVRR_BIT_MASK);

    /* 4. Clean IDLE→RX transition with settling time.
     *    PLL re-lock after TX takes ~150 µs; 2 ms is generous. */
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    Sleep(2);
}

/*============================================================================
 * OTA SW-ARQ — called from sniffer loop and UWB RX handler
 *============================================================================*/

/* Called every 1ms from the Gateway sniffer loop.
 * Checks if we're waiting for an ACK and whether the deadline has passed. */
void gateway_ota_arq_poll(void)
{
    if (ota_arq_state != OTA_ARQ_WAIT_ACK) return;

    if (HAL_GetTick() < ota_arq_deadline) return;  /* still waiting */

    /* Timeout — retry or abort */
    if (ota_arq_retries < OTA_ARQ_MAX_RETRIES) {
        ota_arq_retries++;
        gateway_send_uwb_ack(ota_dst_uid, FUNC_OTA_DATA,
                             ota_arq_buf, ota_arq_buf_len);
        ota_arq_deadline = HAL_GetTick()
            + (ota_arq_seq == 0 ? OTA_ARQ_TIMEOUT_BLK0_MS : OTA_ARQ_TIMEOUT_MS);
    } else {
        /* All retries exhausted for this chunk — skip it and advance.
         * The device writes at seq-based offset, so a gap in the middle
         * is harmless.  At OTA_END, gw_ota_finish() detects missing
         * chunks (OTA GAP) and the host can decide to retry or abort.
         * This is strictly better than aborting the entire session:
         * a single failed chunk doesn't kill the whole upgrade. */
        {
            char _skip[48]; int _skp = 0;
            _skp += fmt_str(&_skip[_skp], "+OTA_SKIP,");
            _skp += fmt_hex4(&_skip[_skp], ota_dst_uid);
            _skip[_skp++] = ','; _skp += fmt_hex4(&_skip[_skp], ota_arq_seq);
            _skip[_skp++] = '\r'; _skip[_skp++] = '\n'; _skip[_skp] = '\0';
            test_run_info((unsigned char *)_skip);
        }
        /* Advance offset and seq as-if-ACKed, so host continues */
        ota_offset += (uint32_t)ota_arq_data_len;
        ota_arq_seq++;
        ota_arq_state = OTA_ARQ_IDLE;
        /* Print +OTA_PROGRESS so host sends next chunk */
        {
            char out[48]; int pos;
            out[0]='+'; out[1]='O'; out[2]='T'; out[3]='A'; out[4]='_';
            out[5]='P'; out[6]='R'; out[7]='O'; out[8]='G'; out[9]='R';
            out[10]='E'; out[11]='S'; out[12]='S'; out[13]=',';
            pos = 14;
            { uint16_t u = ota_dst_uid;
              out[pos++]=gw_hex[(u>>12)&0xF]; out[pos++]=gw_hex[(u>>8)&0xF];
              out[pos++]=gw_hex[(u>>4)&0xF]; out[pos++]=gw_hex[u&0xF]; }
            out[pos++]=',';
            pos += fmt_dec(&out[pos], (int)ota_offset);
            out[pos++]='\r'; out[pos++]='\n'; out[pos]='\0';
            test_run_info((unsigned char *)out);
        }
    }
}

/* Called from gateway_handle_uwb_rx when an OTA_ACK frame is received.
 * Returns 1 if this ACK was processed by the ARQ state machine (first copy),
 * 0 if it was a duplicate/stale ACK that should not trigger +OTA_ACK print. */
int gateway_ota_arq_on_ack(uint16_t seq, uint8_t status)
{
    if (ota_arq_state != OTA_ARQ_WAIT_ACK) return 0;
    if (seq != ota_arq_seq) return 0;  /* stale ACK for a different chunk */

    if (status == 0) {
        /* ACK OK — chunk successfully stored by device */
        ota_arq_state = OTA_ARQ_IDLE;

        /* Advance offset and print progress → host sends next chunk */
        ota_offset += (uint32_t)ota_arq_data_len;
        ota_arq_seq++;

        {
            char out[48]; int pos;
            out[0]='+'; out[1]='O'; out[2]='T'; out[3]='A'; out[4]='_';
            out[5]='P'; out[6]='R'; out[7]='O'; out[8]='G'; out[9]='R';
            out[10]='E'; out[11]='S'; out[12]='S'; out[13]=',';
            pos = 14;
            { uint16_t u = ota_dst_uid;
              out[pos++]=gw_hex[(u>>12)&0xF]; out[pos++]=gw_hex[(u>>8)&0xF];
              out[pos++]=gw_hex[(u>>4)&0xF]; out[pos++]=gw_hex[u&0xF]; }
            out[pos++]=',';
            pos += fmt_dec(&out[pos], (int)ota_offset);
            out[pos++]='\r'; out[pos++]='\n'; out[pos]='\0';
            test_run_info((unsigned char *)out);
        }
    } else {
        /* NACK (flash error) — retry immediately */
        if (ota_arq_retries < OTA_ARQ_MAX_RETRIES) {
            ota_arq_retries++;
            gateway_send_uwb_ack(ota_dst_uid, FUNC_OTA_DATA,
                                 ota_arq_buf, ota_arq_buf_len);
            ota_arq_deadline = HAL_GetTick()
                + (ota_arq_seq == 0 ? OTA_ARQ_TIMEOUT_BLK0_MS : OTA_ARQ_TIMEOUT_MS);
        } else {
            /* All NACK retries exhausted — skip chunk and advance.
             * Device keeps reporting flash error → gap is better than abort. */
            {
                char _nsk[48]; int _nkp = 0;
                _nkp += fmt_str(&_nsk[_nkp], "+OTA_SKIP_NACK,");
                _nkp += fmt_hex4(&_nsk[_nkp], ota_dst_uid);
                _nsk[_nkp++] = ','; _nkp += fmt_hex4(&_nsk[_nkp], ota_arq_seq);
                _nsk[_nkp++] = '\r'; _nsk[_nkp++] = '\n'; _nsk[_nkp] = '\0';
                test_run_info((unsigned char *)_nsk);
            }
            ota_offset += (uint32_t)ota_arq_data_len;
            ota_arq_seq++;
            ota_arq_state = OTA_ARQ_IDLE;
            {
                char out[48]; int pos;
                out[0]='+'; out[1]='O'; out[2]='T'; out[3]='A'; out[4]='_';
                out[5]='P'; out[6]='R'; out[7]='O'; out[8]='G'; out[9]='R';
                out[10]='E'; out[11]='S'; out[12]='S'; out[13]=',';
                pos = 14;
                { uint16_t u = ota_dst_uid;
                  out[pos++]=gw_hex[(u>>12)&0xF]; out[pos++]=gw_hex[(u>>8)&0xF];
                  out[pos++]=gw_hex[(u>>4)&0xF]; out[pos++]=gw_hex[u&0xF]; }
                out[pos++]=',';
                pos += fmt_dec(&out[pos], (int)ota_offset);
                out[pos++]='\r'; out[pos++]='\n'; out[pos]='\0';
                test_run_info((unsigned char *)out);
            }
        }
    }
    return 1;  /* ACK was processed by ARQ state machine */
}

/*============================================================================
 * Broadcast OTA — parallel upgrade of all anchors
 *
 * The Gateway sends every chunk once with DST=0xFFFF.  Each anchor ACKs
 * in a rank-based TDMA slot (rank × 5ms delay).  The Gateway collects
 * ACKs from all devices in a 50ms window before sending the next chunk.
 *============================================================================*/

/* Register a device for broadcast OTA (by UID).  Returns device index or -1. */
static int bcast_ota_register(uint16_t uid)
{
    uint8_t i;
    for (i = 0; i < bcast_num_devs; i++) {
        if (bcast_devs[i].uid == uid) return (int)i;
    }
    if (bcast_num_devs >= BCAST_OTA_MAX_DEVICES) return -1;
    bcast_devs[bcast_num_devs].uid     = uid;
    bcast_devs[bcast_num_devs].present = 0;
    bcast_devs[bcast_num_devs].done    = 0;
    bcast_num_devs++;
    return (int)(bcast_num_devs - 1);
}

/* Mark a device as having ACKed the current chunk. */
static void bcast_ota_mark_acked(uint16_t uid)
{
    uint8_t i;
    for (i = 0; i < bcast_num_devs; i++) {
        if (bcast_devs[i].uid == uid) {
            bcast_devs[i].present = 1;
            return;
        }
    }
}

/* Check if all registered devices have ACKed the current chunk. */
static int bcast_ota_all_acked(void)
{
    uint8_t i;
    for (i = 0; i < bcast_num_devs; i++) {
        if (!bcast_devs[i].present && !bcast_devs[i].done) return 0;
    }
    return 1;
}

/* Reset per-chunk ACK flags for next chunk. */
static void bcast_ota_reset_acks(void)
{
    uint8_t i;
    for (i = 0; i < bcast_num_devs; i++) {
        bcast_devs[i].present = 0;
    }
}

/* Handle FUNC_OTA_ACK in broadcast mode — check seq and mark device.
 * Returns 1 if this ACK was processed (seq match), 0 if stale. */
int gateway_bcast_ota_on_ack(uint16_t seq, uint8_t status, uint16_t src_addr)
{
    if (bcast_state != BCAST_OTA_ACTIVE) return 0;
    if (seq != bcast_arq_seq) return 0;  /* wrong chunk */
    if (status != 0) return 0;           /* NACK — will be handled by retry */
    bcast_ota_mark_acked(src_addr);
    return 1;
}

/* Poll broadcast OTA ARQ — called every 1ms from sniffer loop. */
void gateway_bcast_ota_poll(void)
{
    uint8_t i;

    if (bcast_state != BCAST_OTA_ACTIVE) return;
    if (HAL_GetTick() < bcast_arq_deadline) return;  /* still collecting */

    /* Timeout — check who didn't ACK */
    if (bcast_ota_all_acked()) {
        /* All ACKed — report progress and advance seq for next chunk */
        {
            char out[32]; int pos;
            out[0]='+'; out[1]='O'; out[2]='T'; out[3]='A'; out[4]='_';
            out[5]='P'; out[6]='R'; out[7]='O'; out[8]='G'; out[9]='R';
            out[10]='E'; out[11]='S'; out[12]='S'; out[13]=',';
            pos=14;
            pos += fmt_dec(&out[pos], (int)(bcast_arq_seq * 96u + (uint32_t)bcast_arq_data_len));
            out[pos++] = ','; out[pos++] = 'O'; out[pos++] = 'K';
            out[pos] = '\0';
            test_run_info((unsigned char *)out);
        }
        bcast_arq_seq++;
        bcast_state = BCAST_OTA_PREP;  /* ready for next AT+OTA_BCAST_DATA */
        return;
    }

    /* Some devices missed — retry or abort */
    if (bcast_arq_retries < BCAST_OTA_MAX_RETRIES) {
        bcast_arq_retries++;
        /* Diagnostic: print which device(s) missed the ACK */
        {
            char _d[48]; int _dp = 0;
            _d[_dp++]='R'; _d[_dp++]='E'; _d[_dp++]='T'; _d[_dp++]='R'; _d[_dp++]='Y';
            _d[_dp++]=' '; _d[_dp++]=gw_hex[(bcast_arq_seq>>4)&0xF]; _d[_dp++]=gw_hex[bcast_arq_seq&0xF];
            _d[_dp++]='/'; _d[_dp++]=bcast_arq_retries + '0';
            _d[_dp++]=' '; _d[_dp++]='M'; _d[_dp++]='I'; _d[_dp++]='S'; _d[_dp++]='S'; _d[_dp++]=':';
            for (i = 0; i < bcast_num_devs; i++) {
                if (!bcast_devs[i].present && !bcast_devs[i].done) {
                    _d[_dp++]=' ';
                    _d[_dp++]=gw_hex[(bcast_devs[i].uid>>12)&0xF];
                    _d[_dp++]=gw_hex[(bcast_devs[i].uid>>8)&0xF];
                    _d[_dp++]=gw_hex[(bcast_devs[i].uid>>4)&0xF];
                    _d[_dp++]=gw_hex[bcast_devs[i].uid&0xF];
                }
            }
            _d[_dp++]='\r'; _d[_dp++]='\n'; _d[_dp]='\0';
            test_run_info((unsigned char *)_d);
        }
        /* Retransmit to broadcast — devices that already got it will
         * re-ACK (duplicate), devices that missed it will process.
         *
         * Exponential backoff: each retry gets a longer ACK window
         * to reduce air collisions and give devices more time.
         *   Retry 1:  300ms    Retry 5: 1500ms
         *   Retry 2:  600ms    Retry 6: 2000ms
         *   Retry 3:  900ms    Retry 7: 2000ms
         *   Retry 4: 1200ms    Retry 8: 2000ms (cap) */
        bcast_ota_reset_acks();
        gateway_send_uwb_ack(0xFFFFu, FUNC_OTA_DATA,
                             bcast_arq_buf, bcast_arq_buf_len);
        {
            uint32_t backoff_ms;
            if (bcast_arq_seq == 0) {
                backoff_ms = BCAST_OTA_ACK_BLK0_MS;  /* block 0: fixed long window */
            } else if (bcast_arq_retries <= 1) {
                backoff_ms = 300u;
            } else {
                backoff_ms = 300u * (uint32_t)bcast_arq_retries;
                if (backoff_ms > 2000u) backoff_ms = 2000u;
            }
            bcast_arq_deadline = HAL_GetTick() + backoff_ms;
        }
    } else {
        /* Max retries exhausted — abort */
        uint8_t pad[3] = { 0xCC, 0xDD, 0xEE };
        for (i = 0; i < 3; i++) {
            gateway_send_uwb_ack(0xFFFFu, FUNC_OTA_ABORT, pad, 3);
            Sleep(20);
        }
        bcast_state = BCAST_OTA_IDLE;
        bcast_num_devs = 0;
        test_run_info((unsigned char *)"+OTA_BCAST_ERROR,Timeout\r\n");
    }
}

/*============================================================================
 * Main poll entry point — called from Phase 7
 *============================================================================*/
void gateway_poll_uart(void)
{
    char cmd[GW_CMD_MAX_LEN];

    gateway_check_timeout();

    if (!uart_rx_line_available(cmd, sizeof(cmd)))
        return;

    /* DEBUG: echo received command to diagnose COORD issue */
    {
        char _dbg[48]; int _d = 0, _i;
        _dbg[_d++]='C'; _dbg[_d++]='M'; _dbg[_d++]='D'; _dbg[_d++]=':';
        for (_i = 0; _i < 25 && cmd[_i]; _i++)
            _dbg[_d++] = cmd[_i];
        if (cmd[25]) { _dbg[_d++]='.'; _dbg[_d++]='.'; }
        _dbg[_d++]='\r'; _dbg[_d++]='\n'; _dbg[_d]='\0';
        test_run_info((unsigned char *)_dbg);
    }

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
#if 0 /* === Broadcast OTA disabled — use sequential unicast instead === */
    else if (strncmp(cmd, "AT+OTA_BCAST=", 13) == 0)
    {
        /* AT+OTA_BCAST=<SIZE>
         * Broadcast OTA: send PREP_CMD to 0xFFFF, collect PREP_ACKs
         * from all anchors (TDMA: rank*5ms delay).  Returns device count. */
        const char *p = cmd + 13;
        uint32_t size = 0;
        while (*p >= '0' && *p <= '9')
            size = size * 10 + (uint32_t)(*p++ - '0');

        bcast_num_devs = 0;
        bcast_total_size = size;
        bcast_arq_seq = 0;
        bcast_state = BCAST_OTA_PREP;

        /* 30-burst broadcast PREP_CMD */
        {
            uint8_t prep_pay[3] = { 'F', 0xCC, 0xDD };
            int burst;
            for (burst = 0; burst < 30; burst++) {
                gateway_send_uwb_ack(0xFFFFu, FUNC_PREP_CMD, prep_pay, 3);
                {
                    uint32_t dl = HAL_GetTick() + 10u;
                    port_DisableEXT_IRQ();
                    while (HAL_GetTick() < dl) {
                        uint32_t st = dwt_read32bitreg(SYS_STATUS_ID);
                        if (st & SYS_STATUS_RXFCG_BIT_MASK) {
                            uint32_t fl = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
                            if (fl >= GW_HDR_LEN && fl <= 32) {
                                uint8_t rb[32]; dwt_readrxdata(rb, fl, 0);
                                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
                                if (rb[9] == FUNC_PREP_ACK) {
                                    uint16_t dev_uid = (uint16_t)(rb[7] | (rb[8] << 8));
                                    bcast_ota_register(dev_uid);
                                }
                            } else {
                                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
                            }
                            dwt_rxenable(DWT_START_RX_IMMEDIATE);
                        }
                        if (st & SYS_STATUS_ALL_RX_ERR) {
                            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                            dwt_rxenable(DWT_START_RX_IMMEDIATE);
                        }
                        Sleep(1);
                    }
                    EXTI->PR1 = (1u << 12);
                    port_EnableEXT_IRQ();
                }
            }
        }

        if (bcast_num_devs > 0) {
            char out[32]; int pos;
            out[0]='+'; out[1]='O'; out[2]='T'; out[3]='A'; out[4]='_'; out[5]='B';
            out[6]='C'; out[7]='A'; out[8]='S'; out[9]='T'; out[10]='_'; out[11]='R';
            out[12]='E'; out[13]='A'; out[14]='D'; out[15]='Y'; out[16]=',';
            pos=17; pos+=fmt_dec(&out[pos],(int)bcast_num_devs); out[pos]='\0';
            test_run_info((unsigned char *)out);
        } else {
            bcast_state = BCAST_OTA_IDLE;
            test_run_info((unsigned char *)"+OTA_BCAST_ERROR,No devices\r\n");
        }
        gateway_rx_flush();
    }
    else if (strncmp(cmd, "AT+OTA_BCAST_DATA=", 18) == 0)
    {
        /* AT+OTA_BCAST_DATA=<HEX_DATA> — send one chunk to all anchors.
         * Chunk format: hex-encoded firmware data (96 bytes = 192 hex chars). */
        if (bcast_state != BCAST_OTA_PREP) {
            /* Only accept data chunks when PREP is set (after AT+OTA_BCAST).
             * Reject if IDLE (session ended/aborted) or ACTIVE (shouldn't happen). */
            return;
        }
        {
            const char *p = cmd + 18;
            int hex_len = 0;
            while (*p && *p != '\r' && *p != '\n') { hex_len++; p++; }
            int data_len = hex_len / 2;
            if (data_len > 248) data_len = 248;
            p = cmd + 18;

            /* Build chunk buffer: seq(2B) + data(960B) + crc(2B) + pad(2B).
             * All chunks identical — total_size was already sent in PREP_CMD. */
            bcast_arq_buf[0] = bcast_arq_seq & 0xFF;
            bcast_arq_buf[1] = (bcast_arq_seq >> 8) & 0xFF;
            {
                int i;
                for (i = 0; i < data_len && i < 248; i++) {
                    uint8_t b;
                    if (hex_byte(p[i*2], p[i*2+1], &b) != 0) break;
                    bcast_arq_buf[2 + i] = b;
                }
                uint16_t crc = crc16_ccitt(&bcast_arq_buf[2], data_len);
                bcast_arq_buf[2 + data_len]     = crc & 0xFF;
                bcast_arq_buf[2 + data_len + 1] = (crc >> 8) & 0xFF;
                bcast_arq_buf[2 + data_len + 2] = 0xCC;
                bcast_arq_buf[2 + data_len + 3] = 0xDD;
            }
            bcast_arq_buf_len  = (uint8_t)(2 + data_len + 4);
            bcast_arq_data_len = (uint8_t)data_len;

            /*=== Inline polling: send + collect ACKs from all registered devices.
             * Replaces old fire-and-forget approach that relied on sniffer loop
             * timing — sniffer RX windows missed ACKs during flash erase gap.
             * Disables EXTI and polls DW3000 directly, same pattern as
             * gateway_send_cmd_wait_ack().  ACKs arrive after ~670ms (block 0)
             * or ~10ms (normal chunks). ===*/
            {
                int retry;
                int all_acked = 0;

                /* Initial send */
                bcast_ota_reset_acks();
                gateway_send_uwb_ack(0xFFFFu, FUNC_OTA_DATA,
                                     bcast_arq_buf, bcast_arq_buf_len);

                for (retry = 0; retry < (int)BCAST_OTA_MAX_RETRIES && !all_acked; retry++) {
                    uint32_t ack_window_ms;

                    if (retry > 0) {
                        /* Retry: print diagnostic, brief settle, re-send */
                        {
                            char _d[48]; int _dp = 0; uint8_t _j;
                            _d[_dp++]='R'; _d[_dp++]='E'; _d[_dp++]='T'; _d[_dp++]='R'; _d[_dp++]='Y';
                            _d[_dp++]=' '; _d[_dp++]=gw_hex[(bcast_arq_seq>>4)&0xF]; _d[_dp++]=gw_hex[bcast_arq_seq&0xF];
                            _d[_dp++]='/'; _d[_dp++]=retry + '0';
                            _d[_dp++]=' '; _d[_dp++]='M'; _d[_dp++]='I'; _d[_dp++]='S'; _d[_dp++]='S'; _d[_dp++]=':';
                            for (_j = 0; _j < bcast_num_devs; _j++) {
                                if (!bcast_devs[_j].present && !bcast_devs[_j].done) {
                                    _d[_dp++]=' ';
                                    _d[_dp++]=gw_hex[(bcast_devs[_j].uid>>12)&0xF];
                                    _d[_dp++]=gw_hex[(bcast_devs[_j].uid>>8)&0xF];
                                    _d[_dp++]=gw_hex[(bcast_devs[_j].uid>>4)&0xF];
                                    _d[_dp++]=gw_hex[bcast_devs[_j].uid&0xF];
                                }
                            }
                            _d[_dp++]='\r'; _d[_dp++]='\n'; _d[_dp]='\0';
                            test_run_info((unsigned char *)_d);
                        }
                        bcast_ota_reset_acks();
                        Sleep(50);
                        gateway_send_uwb_ack(0xFFFFu, FUNC_OTA_DATA,
                                             bcast_arq_buf, bcast_arq_buf_len);
                    }

                    /* Exponential backoff for ACK window.
                     * Block 0: 3000ms first try (flash erase ~660ms), 2000ms retries.
                     * Normal:  300ms first try, 300×retry cap 2000ms. */
                    if (bcast_arq_seq == 0 && retry == 0)
                        ack_window_ms = BCAST_OTA_ACK_BLK0_MS;
                    else if (bcast_arq_seq == 0)
                        ack_window_ms = 2000u;
                    else if (retry <= 1)
                        ack_window_ms = 300u;
                    else {
                        ack_window_ms = 300u * (uint32_t)retry;
                        if (ack_window_ms > 2000u) ack_window_ms = 2000u;
                    }

                    {
                        uint32_t deadline = HAL_GetTick() + ack_window_ms;

                        port_DisableEXT_IRQ();
                        while (HAL_GetTick() < deadline && !all_acked) {
                            uint32_t st = dwt_read32bitreg(SYS_STATUS_ID);

                            if (st & SYS_STATUS_RXFCG_BIT_MASK) {
                                uint32_t fl = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
                                if (fl >= GW_HDR_LEN && fl <= 32) {
                                    uint8_t rb[32]; dwt_readrxdata(rb, fl, 0);
                                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
                                    if (rb[9] == FUNC_OTA_ACK) {
                                        uint16_t aseq  = (uint16_t)(rb[GW_HDR_LEN]) | ((uint16_t)(rb[GW_HDR_LEN+1]) << 8);
                                        uint8_t  ast   = rb[GW_HDR_LEN + 2];
                                        uint16_t auid  = (uint16_t)(rb[7] | (rb[8] << 8));
                                        if (aseq == bcast_arq_seq && ast == 0) {
                                            bcast_ota_mark_acked(auid);
                                            if (bcast_ota_all_acked()) {
                                                all_acked = 1;
                                            }
                                        }
                                    }
                                } else {
                                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
                                }
                                dwt_rxenable(DWT_START_RX_IMMEDIATE);
                            }

                            if (st & SYS_STATUS_ALL_RX_ERR) {
                                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                                dwt_rxenable(DWT_START_RX_IMMEDIATE);
                            }

                            Sleep(1);
                        }
                        EXTI->PR1 = (1u << 12);
                        port_EnableEXT_IRQ();
                    }
                }

                if (all_acked) {
                    /* All devices ACKed — report progress, advance seq */
                    {
                        char out[32]; int pos;
                        out[0]='+'; out[1]='O'; out[2]='T'; out[3]='A'; out[4]='_';
                        out[5]='P'; out[6]='R'; out[7]='O'; out[8]='G'; out[9]='R';
                        out[10]='E'; out[11]='S'; out[12]='S'; out[13]=',';
                        pos=14;
                        pos += fmt_dec(&out[pos], (int)(bcast_arq_seq * 96u + (uint32_t)bcast_arq_data_len));
                        out[pos++] = ','; out[pos++] = 'O'; out[pos++] = 'K';
                        out[pos] = '\0';
                        test_run_info((unsigned char *)out);
                    }
                    bcast_arq_seq++;
                    bcast_state = BCAST_OTA_PREP;  /* ready for next chunk */
                } else {
                    /* All retries exhausted — abort broadcast OTA */
                    {
                        uint8_t _pad[3] = { 0xCC, 0xDD, 0xEE };
                        int _z;
                        for (_z = 0; _z < 3; _z++) {
                            gateway_send_uwb_ack(0xFFFFu, FUNC_OTA_ABORT, _pad, 3);
                            Sleep(20);
                        }
                    }
                    bcast_num_devs = 0;
                    bcast_state = BCAST_OTA_IDLE;
                    test_run_info((unsigned char *)"+OTA_BCAST_ERROR,Timeout\r\n");
                }
                gateway_rx_flush();
            }
        }
    }
    else if (strncmp(cmd, "AT+OTA_BCAST_END", 16) == 0)
    {
        /* AT+OTA_BCAST_END — finish broadcast OTA.
         * Send OTA_END 5× to 0xFFFF, then resume Master TDMA. */
        uint8_t pad[3] = { 0xCC, 0xDD, 0xEE };
        int i;
        for (i = 0; i < 5; i++) {
            gateway_send_uwb_ack(0xFFFFu, FUNC_OTA_END, pad, 3);
            Sleep(100);
        }
        gateway_send_uwb_ack(0x0000u, FUNC_START_TDOA, pad, 3);
        bcast_state = BCAST_OTA_IDLE;
        bcast_num_devs = 0;
        test_run_info((unsigned char *)"+OTA_BCAST_DONE\r\n");
    }
#endif /* 0 — broadcast OTA disabled */
    else if (strncmp(cmd, "AT+OTA_START=", 13) == 0)
    {
        /* AT+OTA_START=<UID>,<SIZE>
         * Send OTA_PREPARE on Ch9 → device ACKs + stops ranging
         * OTA data/ACK on same Ch9 (no channel switch needed) */
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

        ota_dst_uid    = parse_uid(uid_str);
        ota_offset     = 0;
        ota_arq_seq    = 0;
        ota_active     = 1;
        ota_arq_state  = OTA_ARQ_IDLE;

        /* Send unified prepare command (0xF0) via 30-burst fire-and-poll.
         * Master responds with 0xF1 ACK.  30 rapid sends at 10ms intervals
         * (300ms total) guarantees at least one lands in an RX window.
         * Payload: total_size(4B LE) + pad(2B) = 6B.
         * Pad absorbs DW3210 last-2B RX corruption — without it the
         * upper 2 bytes of total_size get clobbered. */
        {
            int ack_ok = 0, burst;
            uint8_t prep_pay[6];
            prep_pay[0] = (uint8_t)(ota_total_size & 0xFF);
            prep_pay[1] = (uint8_t)((ota_total_size >> 8) & 0xFF);
            prep_pay[2] = (uint8_t)((ota_total_size >> 16) & 0xFF);
            prep_pay[3] = (uint8_t)((ota_total_size >> 24) & 0xFF);
            prep_pay[4] = 0xCC;  /* pad — absorbed by DW3210 RX corruption */
            prep_pay[5] = 0xDD;  /* pad — absorbed by DW3210 RX corruption */

            /* Diagnostic: print ota_total_size and prep_pay content */
            { char _d[32]; const char _h[] = "0123456789ABCDEF"; int _di = 0;
              _d[_di++]='P'; _d[_di++]='S'; _d[_di++]='=';
              _d[_di++]=_h[(ota_total_size>>28)&0xF]; _d[_di++]=_h[(ota_total_size>>24)&0xF];
              _d[_di++]=_h[(ota_total_size>>20)&0xF]; _d[_di++]=_h[(ota_total_size>>16)&0xF];
              _d[_di++]=_h[(ota_total_size>>12)&0xF]; _d[_di++]=_h[(ota_total_size>>8)&0xF];
              _d[_di++]=_h[(ota_total_size>>4)&0xF]; _d[_di++]=_h[ota_total_size&0xF];
              _d[_di++]=' '; _d[_di++]='P'; _d[_di++]='P'; _d[_di++]='=';
              _d[_di++]=_h[(prep_pay[3]>>4)&0xF]; _d[_di++]=_h[prep_pay[3]&0xF];
              _d[_di++]=_h[(prep_pay[2]>>4)&0xF]; _d[_di++]=_h[prep_pay[2]&0xF];
              _d[_di++]=_h[(prep_pay[1]>>4)&0xF]; _d[_di++]=_h[prep_pay[1]&0xF];
              _d[_di++]=_h[(prep_pay[0]>>4)&0xF]; _d[_di++]=_h[prep_pay[0]&0xF];
              _d[_di++]='\r'; _d[_di++]='\n'; _d[_di]='\0';
              test_run_info((unsigned char *)_d); }

            /* 30-burst PREP_CMD at 10ms intervals — 300ms firestorm */
            for (burst = 0; burst < 30 && !ack_ok; burst++) {
                gateway_send_uwb_ack(ota_dst_uid, FUNC_PREP_CMD, prep_pay, 6);
                {
                    uint32_t dl = HAL_GetTick() + 10u;
                    port_DisableEXT_IRQ();
                    while (HAL_GetTick() < dl) {
                        uint32_t st = dwt_read32bitreg(SYS_STATUS_ID);
                        if (st & SYS_STATUS_RXFCG_BIT_MASK) {
                            uint32_t fl = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
                            if (fl >= GW_HDR_LEN && fl <= 32) {
                                uint8_t rb[32]; dwt_readrxdata(rb, fl, 0);
                                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
                                /* Diagnostic: print func of every RX frame */
                                { char _rx[20]; int _rxp=0;
                                  _rx[_rxp++]='R'; _rx[_rxp++]='X'; _rx[_rxp++]=':';
                                  _rx[_rxp++]=gw_hex[(rb[9]>>4)&0xF]; _rx[_rxp++]=gw_hex[rb[9]&0xF];
                                  _rx[_rxp++]=','; _rx[_rxp++]='L'; _rx[_rxp++]='=';
                                  _rx[_rxp++]=gw_hex[(fl>>4)&0xF]; _rx[_rxp++]=gw_hex[fl&0xF];
                                  _rx[_rxp++]='\r'; _rx[_rxp++]='\n'; _rx[_rxp]='\0';
                                  test_run_info((unsigned char *)_rx); }
                                if (rb[9] == FUNC_PREP_ACK) { ack_ok = 1; break; }
                            } else {
                                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
                            }
                            /* IRQ handler calls dwt_forcetrxoff() — re-enter RX */
                            dwt_rxenable(DWT_START_RX_IMMEDIATE);
                        }
                        if (st & SYS_STATUS_ALL_RX_ERR) {
                            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                            dwt_rxenable(DWT_START_RX_IMMEDIATE);
                        }
                        Sleep(1);
                    }
                    EXTI->PR1 = (1u << 12);
                    port_EnableEXT_IRQ();
                }
            }

            if (ack_ok) {
                /* +OTA_READY,<UID> */
                { char out[32]; int pos;
                  out[0]='+'; out[1]='O'; out[2]='T'; out[3]='A'; out[4]='_'; out[5]='R';
                  out[6]='E'; out[7]='A'; out[8]='D'; out[9]='Y'; out[10]=',';
                  pos=11; pos+=fmt_str(&out[pos],uid_str); out[pos]='\0';
                  test_run_info((unsigned char *)out); }
            } else {
                ota_active = 0;
                test_run_info((unsigned char *)"+OTA_ERROR,PREPARE,No ACK\r\n");
            }
            /* Diagnostic */
            {
                char _dx[16]; int _dxp = 0;
                _dxp += fmt_str(&_dx[_dxp], "TX,P=");
                _dxp += fmt_hex4(&_dx[_dxp], ota_dst_uid);
                _dx[_dxp++] = ','; _dx[_dxp++] = ack_ok ? 'O' : 'F';
                _dx[_dxp++] = '\r'; _dx[_dxp++] = '\n'; _dx[_dxp] = '\0';
                test_run_info((unsigned char *)_dx);
            }
            gateway_rx_flush();
        }
    }
    else if (strncmp(cmd, "AT+OTA_DATA=", 12) == 0)
    {
        /* P0-3: Inline-polling unicast OTA_DATA handler.
         * Send chunk → poll DW3000 directly for ACK (EXTI disabled).
         * No dependency on sniffer loop timing.  Retries up to 15×.
         * Block 0: 3000ms window (device erases flash ~660ms).
         * Normal:  300ms window per attempt. */
        const char *p;
        char uid_str[8];
        int i, hex_len, data_len;

        if (!ota_active) {
            test_run_info((unsigned char *)"+OTA_ERROR,OTA,Not started");
            return;
        }

        /* Parse: AT+OTA_DATA=<UID>,<OFFSET>,<HEX> */
        p = cmd + 12;
        i = 0;
        while (*p && *p != ',' && i < (int)sizeof(uid_str) - 1) uid_str[i++] = *p++;
        uid_str[i] = '\0';
        if (*p == ',') p++;
        while (*p >= '0' && *p <= '9') p++;  /* skip offset */
        if (*p == ',') p++;
        hex_len = 0; { const char *hp = p; while (*hp && *hp != '\r' && *hp != '\n') { hex_len++; hp++; } }
        data_len = hex_len / 2;
        if (data_len > 248) data_len = 248;

        /* Build OTA_DATA payload — seq(2B) + data(960B) + crc(2B) + pad(2B).
         * total_size was already sent in PREP_CMD (0xF0); no per-chunk header needed. */
        ota_arq_buf[0] = ota_arq_seq & 0xFF;
        ota_arq_buf[1] = (ota_arq_seq >> 8) & 0xFF;
        for (i = 0; i < data_len; i++) { uint8_t b; if (hex_byte(p[i*2], p[i*2+1], &b) != 0) break; ota_arq_buf[2 + i] = b; }
        { uint16_t crc = crc16_ccitt(&ota_arq_buf[2], data_len);
          ota_arq_buf[2 + data_len] = crc & 0xFF; ota_arq_buf[2 + data_len + 1] = (crc >> 8) & 0xFF;
          ota_arq_buf[2 + data_len + 2] = 0xCC; ota_arq_buf[2 + data_len + 3] = 0xDD; }
        ota_arq_buf_len  = (uint8_t)(2 + data_len + 4);
        ota_arq_data_len = (uint8_t)data_len;

        /*=== Inline polling: send + wait for ACK ===*/
        {
            int _retry, _acked = 0;

            /* Initial send */
            gateway_send_uwb_ack(ota_dst_uid, FUNC_OTA_DATA, ota_arq_buf, ota_arq_buf_len);

            for (_retry = 0; _retry < (int)OTA_ARQ_MAX_RETRIES && !_acked; _retry++) {
                uint32_t _timeout;

                if (_retry > 0) {
                    /* Exponential backoff: 10,20,40,80...ms up to 2s.
                     * Gives UWB channel time to recover from deep fades.
                     * 20 retries × ~100ms avg ≈ 2s worst case. */
                    {
                        uint32_t _bo = OTA_ARQ_BACKOFF_BASE_MS << (uint32_t)(_retry - 1);
                        if (_bo > 2000u) _bo = 2000u;  /* cap at 2s */
                        Sleep(_bo);
                    }
                    gateway_send_uwb_ack(ota_dst_uid, FUNC_OTA_DATA, ota_arq_buf, ota_arq_buf_len);
                }

                /* ACK window: block 0 = 3000ms first try, 1000ms retries */
                if (ota_arq_seq == 0 && _retry == 0)
                    _timeout = OTA_ARQ_TIMEOUT_BLK0_MS;   /* 1000ms: staging erase ~660ms */
                else if (ota_arq_seq == 0)
                    _timeout = 500u;                       /* blk0 retry: 500ms */
                else
                    _timeout = OTA_ARQ_TIMEOUT_MS;         /* 50ms: flash write + ACK */

                {
                    uint32_t _dl = HAL_GetTick() + _timeout;
                    port_DisableEXT_IRQ();
                    dwt_rxenable(DWT_START_RX_IMMEDIATE);  /* MUST re-enter RX — EXTI may have killed it */
                    while (HAL_GetTick() < _dl && !_acked) {
                        uint32_t _st = dwt_read32bitreg(SYS_STATUS_ID);

                        if (_st & SYS_STATUS_RXFCG_BIT_MASK) {
                            uint32_t _fl = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
                            if (_fl >= GW_HDR_LEN && _fl <= 32) {
                                uint8_t _rb[32]; dwt_readrxdata(_rb, _fl, 0);
                                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
                                /* Diagnostic disabled for OTA speed — was printing <XX> per frame */
                                if (_rb[9] == FUNC_OTA_ACK) {
                                    uint16_t _aseq = (uint16_t)(_rb[GW_HDR_LEN]) | ((uint16_t)(_rb[GW_HDR_LEN+1]) << 8);
                                    uint8_t  _ast  = _rb[GW_HDR_LEN + 2];
                                    if (_aseq == ota_arq_seq && _ast == 0) { _acked = 1; }
                                    /* ACK rejected (wrong seq/status) — silent, ARQ will retry */
                                }
                            } else {
                                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
                            }
                            dwt_rxenable(DWT_START_RX_IMMEDIATE);
                        }

                        if (_st & SYS_STATUS_ALL_RX_ERR) {
                            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                            dwt_rxenable(DWT_START_RX_IMMEDIATE);
                        }

                        Sleep(1);
                    }
                    EXTI->PR1 = (1u << 12);
                    port_EnableEXT_IRQ();
                }
            }

            if (_acked) {
                ota_offset += (uint32_t)ota_arq_data_len;
                ota_arq_seq++;
                { char _out[48]; int _pos;
                  _out[0]='+'; _out[1]='O'; _out[2]='T'; _out[3]='A'; _out[4]='_'; _out[5]='P';
                  _out[6]='R'; _out[7]='O'; _out[8]='G'; _out[9]='R'; _out[10]='E'; _out[11]='S'; _out[12]='S'; _out[13]=',';
                  _pos=14; { uint16_t _u=ota_dst_uid; _out[_pos++]=gw_hex[(_u>>12)&0xF]; _out[_pos++]=gw_hex[(_u>>8)&0xF];
                  _out[_pos++]=gw_hex[(_u>>4)&0xF]; _out[_pos++]=gw_hex[_u&0xF]; }
                  _out[_pos++]=','; _pos+=fmt_dec(&_out[_pos],(int)ota_offset); _out[_pos++]='\r'; _out[_pos++]='\n'; _out[_pos]='\0';
                  test_run_info((unsigned char *)_out); }
            } else {
                /* All retries exhausted — ABORT, don't fake success.
                 * Skipping created staging gaps → gw_ota_finish GAP abort → "fake success". */
                ota_active = 0;
                test_run_info((unsigned char *)"+OTA_ERROR,ARQ,All 15 retries exhausted\r\n");
            }
            gateway_rx_flush();
        }
    }
    else if (strncmp(cmd, "AT+OTA_END=", 11) == 0)
    {
        /* AT+OTA_END=<UID> → send OTA_END via UWB 3×, then +OTA_DONE */
        uint16_t uid = parse_uid(cmd + 11);
        ota_active    = 0;
        ota_arq_state = OTA_ARQ_IDLE;

        /* Pad to 13B to avoid DW3210 last-2B RX corruption (func=byte9).
         * Send 3× with 100ms spacing — OTA_END has no ACK, must not be lost. */
        { int _r;
          for (_r = 0; _r < 3; _r++) {
            uint8_t pad[3] = {0xCC, 0xDD, 0xEE};
            gateway_send_uwb_ack(uid, FUNC_OTA_END, pad, 3);
            gateway_rx_flush();
            if (_r < 2) Sleep(100);
          }
        }

        /* Resume Master TDMA if it was paused before OTA */
        if (uid == 0x0000) {
            uint8_t start_pad[3] = {0xCC, 0xDD, 0xEE};
            gateway_send_cmd_wait_ack(uid, FUNC_START_TDOA,
                                      start_pad, 3, FUNC_START_ACK, 150, 5);
        }

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
        /* AT+FWV → +FWV,<type>,<uid>,<version> for gateway + all known devices */
        char out[48], ver[9];
        int pos, i;

        /* Gateway itself */
        fw_version_str(ver, FW_VERSION_U16);
        out[0] = '+'; out[1] = 'F'; out[2] = 'W'; out[3] = 'V'; out[4] = ',';
        out[5] = 'G'; out[6] = 'W'; out[7] = ',';
        out[8] = 'G'; out[9] = 'W'; out[10] = '0'; out[11] = '1'; out[12] = ',';
        pos = 13;
        pos += fmt_str(&out[pos], ver);
        out[pos] = '\0';
        test_run_info((unsigned char *)out);

        /* All known devices with version info */
        for (i = 0; i < gw_dev_count; i++)
        {
            if (gw_devs[i].fw_ver == 0) continue;  /* version not yet known */
            fw_version_str(ver, gw_devs[i].fw_ver);
            out[0] = '+'; out[1] = 'F'; out[2] = 'W'; out[3] = 'V'; out[4] = ',';
            out[5] = gw_devs[i].type; out[6] = ',';
            pos = 7;
            pos += fmt_str(&out[pos], gw_devs[i].uid);
            out[pos++] = ',';
            pos += fmt_str(&out[pos], ver);
            out[pos] = '\0';
            test_run_info((unsigned char *)out);
        }
    }
    else if (strncmp(cmd, "AT+NETWORK=", 11) == 0)
    {
        /* AT+NETWORK=<hex_mask> — set which anchors participate */
        unsigned int mask = 0;
        const char *hp = cmd + 11;
        while (*hp) {
            char c = *hp++;
            mask <<= 4;
            if (c >= '0' && c <= '9') mask |= (c - '0');
            else if (c >= 'A' && c <= 'F') mask |= (c - 'A' + 10);
            else if (c >= 'a' && c <= 'f') mask |= (c - 'a' + 10);
            else break;
        }
        {
            uint8_t payload[2]; int _r;
            payload[0] = mask & 0xFF;
            payload[1] = (mask >> 8) & 0xFF;
            /* NETWORK_CFG — send and wait for ACK */
            {
                int _r, ack_ok = 0;
                for (_r = 0; _r < 3 && !ack_ok; _r++) {
                    gateway_send_uwb_ack(0x0000, FUNC_NETWORK_CFG, payload, 2);
                    { uint32_t dl = HAL_GetTick() + (_r < 2 ? 30u : 100u);
                      port_DisableEXT_IRQ();
                      while (HAL_GetTick() < dl) {
                        uint32_t st = dwt_read32bitreg(SYS_STATUS_ID);
                        if (st & SYS_STATUS_ALL_RX_ERR) {
                            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                            dwt_rxenable(DWT_START_RX_IMMEDIATE);
                            Sleep(1); continue; }
                        if (st & SYS_STATUS_RXFCG_BIT_MASK) {
                            uint32_t fl = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
                            if (fl >= GW_HDR_LEN && fl <= 32) {
                                uint8_t rb[32]; dwt_readrxdata(rb, fl, 0);
                                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
                                if (rb[9] == FUNC_NETWORK_CFG_ACK) { ack_ok = 1; break; }
                            } else { dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK); }
                            dwt_rxenable(DWT_START_RX_IMMEDIATE);
                        }
                        Sleep(1);
                      }
                      EXTI->PR1 = (1u << 12);
                      port_EnableEXT_IRQ();
                    }
                }
                if (ack_ok) {
                    char out[24]; int pos=0;
                    out[pos++]='+'; out[pos++]='N'; out[pos++]='E'; out[pos++]='T';
                    out[pos++]='_'; out[pos++]='O'; out[pos++]='K'; out[pos++]=',';
                    pos += fmt_hex4(&out[pos], (uint16_t)mask);
                    out[pos++]='\r'; out[pos++]='\n'; out[pos]='\0';
                    test_run_info((unsigned char *)out);
                } else {
                    test_run_info((unsigned char *)"+NET_ERR\r\n");
                }
                gateway_rx_flush();  /* clean RX state after TX+ACK polling */
            }
        }
    }
    else if (strncmp(cmd, "AT+NETSTART=", 12) == 0)
    {
        /* AT+NETSTART=<root_rank>[,<r1>,<r2>,...]
         *
         * Broadcasts FUNC_NETWORK_START to all anchors.  The root rank is a
         * 16-bit value identifying the designated root anchor; anchors reply
         * with FUNC_NETWORK_ACK (0xE9) unicast to the gateway.
         *
         * The optional comma-separated list is the PARTICIPANT ROSTER (P1-1).
         * It is sent to every anchor inside the same frame, so all nodes
         * agree on the network membership, and bit N of the Root's presence
         * bitmap always refers to the same participant.  Without a roster a
         * Root can only learn its neighbours from the frames it happens to
         * hear, so a single one-way link makes network formation impossible
         * and undetectable.  When the host sends no roster the gateway falls
         * back to "every known anchor", which keeps the old command working.
         */
        const char *p = cmd + 12;
        unsigned int root_rank = 0;
        uint16_t roster[NET_MAX_PARTICIPANTS];
        int roster_n = 0;

        while (*p >= '0' && *p <= '9')
            root_rank = root_rank * 10 + (unsigned int)(*p++ - '0');
        if (root_rank > 0xFFFF) root_rank = 0xFFFF;

        /* --- optional explicit roster --- */
        if (*p == ',') {
            p++;
            while (*p && roster_n < (int)NET_MAX_PARTICIPANTS) {
                unsigned int r = 0;
                int got = 0;
                while (*p >= '0' && *p <= '9') { r = r * 10 + (unsigned int)(*p++ - '0'); got = 1; }
                if (got && r > 0 && r <= 0xFFFF) roster[roster_n++] = (uint16_t)r;
                if (*p == ',') p++;
                else break;
            }
        }

        {
            int i, j, dup;
            /* --- fallback roster: root + every known anchor --- */
            if (roster_n == 0) {
                roster[roster_n++] = (uint16_t)root_rank;
                for (i = 0; i < gw_dev_count && roster_n < (int)NET_MAX_PARTICIPANTS; i++) {
                    uint16_t r;
                    if (gw_devs[i].type != 'A') continue;
                    if (!gw_devs[i].online) continue;
                    r = gw_devs[i].rank;
                    if (r == 0 || r == (uint16_t)root_rank) continue;
                    dup = 0;
                    for (j = 0; j < roster_n; j++) if (roster[j] == r) { dup = 1; break; }
                    if (!dup) roster[roster_n++] = r;
                }
            }
            /* Keep our own copy so the presence mask can be decoded later */
            /* The Root is always a member of the network (it owns slot 0), so
             * make sure it is part of the roster even when the host listed
             * only the other participants.  The order must stay identical to
             * the order inside the frame — bit N of the presence mask refers
             * to roster[N] on every node. */
            {
                int has_root = 0;
                for (i = 0; i < roster_n; i++)
                    if (roster[i] == (uint16_t)root_rank) { has_root = 1; break; }
                if (!has_root && roster_n < (int)NET_MAX_PARTICIPANTS) {
                    for (i = roster_n; i > 0; i--) roster[i] = roster[i - 1];
                    roster[0] = (uint16_t)root_rank;
                    roster_n++;
                }
            }
            for (i = 0; i < roster_n; i++) gw_net_roster[i] = roster[i];
            gw_net_roster_n = roster_n;
            gw_net_root = (uint16_t)root_rank;
            gw_net_present_valid = 0;

            /* Build NETWORK_START payload:
             * [0-1] root_rank (uint16 LE)
             * [2]   flags (bit[0]=1 force reset)
             * [3]   n_participants
             * [4-5] reserved
             * [6..] participant ranks (n × uint16 LE) */
            uint8_t payload[FUNC_NETWORK_START_MAX];
            int plen = 0;
            payload[plen++] = (uint8_t)(root_rank & 0xFF);
            payload[plen++] = (uint8_t)((root_rank >> 8) & 0xFF);
            payload[plen++] = NET_FLAG_FORCE_RESET;  /* force reset on anchors */
            payload[plen++] = (uint8_t)roster_n;
            payload[plen++] = 0; payload[plen++] = 0;   /* reserved */
            for (i = 0; i < roster_n; i++) {
                payload[plen++] = (uint8_t)(roster[i] & 0xFF);
                payload[plen++] = (uint8_t)((roster[i] >> 8) & 0xFF);
            }

            /* Broadcast to all anchors (0xFFFF) via UWB.
             * P1-3: the burst must outlast the anchor-side timing — the
             * anchor de-bounces repeated NETWORK_START for 800 ms, answers
             * with an ALOHA beacon every 500 ms and needs up to 3 s to
             * confirm its slot.  A 300 ms burst therefore missed most
             * anchors; 100 × 20 ms = 2 s covers all of it. */
            int ack_count = 0;
            uint16_t ack_ranks[GW_MAX_DEVICES];
            int burst;
            for (burst = 0; burst < 100; burst++) {
                gateway_send_uwb_ack(0xFFFFu, FUNC_NETWORK_START, payload, (uint8_t)plen);
                {
                    uint32_t dl = HAL_GetTick() + 20u;
                    port_DisableEXT_IRQ();
                    while (HAL_GetTick() < dl) {
                        uint32_t st = dwt_read32bitreg(SYS_STATUS_ID);
                        if (st & SYS_STATUS_ALL_RX_ERR) {
                            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                            dwt_rxenable(DWT_START_RX_IMMEDIATE);
                            Sleep(1); continue;
                        }
                        if (st & SYS_STATUS_RXFCG_BIT_MASK) {
                            uint32_t fl = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
                            if (fl >= GW_HDR_LEN && fl <= 32) {
                                uint8_t rb[32]; dwt_readrxdata(rb, fl, 0);
                                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
                                if (rb[9] == FUNC_NETWORK_ACK) {
                                    /* Got NETWORK_ACK from an anchor.
                                     * Parse: [10-11]=root_rank, [12]=cs_level,
                                     *         [13]=slot_idx, [14]=n_neighbors */
                                    uint16_t ack_root = (uint16_t)(rb[DL_TS_NETACK_ROOT_IDX] |
                                                    ((uint16_t)rb[DL_TS_NETACK_ROOT_IDX+1] << 8));
                                    uint8_t cs_level  = rb[DL_TS_NETACK_LEVEL_IDX];
                                    uint8_t slot_idx  = rb[DL_TS_NETACK_SLOT_IDX];
                                    uint8_t n_neigh   = rb[DL_TS_NETACK_NNEIGH_IDX];
                                    uint16_t src_rank = (uint16_t)(rb[7] | ((uint16_t)rb[8] << 8));
                                    (void)ack_root; (void)cs_level; (void)slot_idx; (void)n_neigh;
                                    /* Register the anchor as online (P1-4) and count
                                     * each anchor only once, even though it ACKs on
                                     * every broadcast burst. */
                                    {
                                        char auid[8];
                                        int dup = 0, k;
                                        for (k = 0; k < ack_count; k++) {
                                            if (ack_ranks[k] == src_rank) { dup = 1; break; }
                                        }
                                        if (!dup && ack_count < GW_MAX_DEVICES) {
                                            ack_ranks[ack_count++] = src_rank;
                                            auid[0] = gw_hex[(src_rank >> 12) & 0xF];
                                            auid[1] = gw_hex[(src_rank >>  8) & 0xF];
                                            auid[2] = gw_hex[(src_rank >>  4) & 0xF];
                                            auid[3] = gw_hex[ src_rank        & 0xF];
                                            auid[4] = '\0';
                                            gateway_device_seen(auid, "ANCHOR", src_rank);
                                            /* P1-1: report how many anchors this one
                                             * hears.  The Root's neighbour count is
                                             * its coverage — a Root that hears
                                             * almost nobody cannot complete the
                                             * network, so the host can suggest a
                                             * better Root. */
                                            {
                                                char nout[32]; int np = 0;
                                                nout[np++]='+'; nout[np++]='N'; nout[np++]='E';
                                                nout[np++]='T'; nout[np++]='N'; nout[np++]='B';
                                                nout[np++]='R'; nout[np++]=',';
                                                np += fmt_dec(&nout[np], (int)src_rank);
                                                nout[np++]=',';
                                                np += fmt_dec(&nout[np], (int)n_neigh);
                                                nout[np]='\0';
                                                test_run_info((unsigned char *)nout);
                                            }
                                        }
                                    }
                                    /* Don't break — keep collecting ACKs from other anchors */
                                }
                            } else {
                                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
                            }
                            dwt_rxenable(DWT_START_RX_IMMEDIATE);
                        }
                        Sleep(1);
                    }
                    EXTI->PR1 = (1u << 12);
                    port_EnableEXT_IRQ();
                }
            }

            /* Report result: +NETSTART_OK,<ack_count>,<root_rank>,<miss_cnt>,<r1>,...
             * The missing list holds the roster members that never answered
             * the NETWORK_START — the host must warn about them and keep them
             * out of the TDOA solve set.  If nobody answered at all, emit
             * +NETSTART_ERR so the host can tell "command sent but no anchor
             * heard it" from "success". */
            {
                char out[128]; int pos = 0;
                if (ack_count == 0) {
                    out[pos++]='+'; out[pos++]='N'; out[pos++]='E'; out[pos++]='T';
                    out[pos++]='S'; out[pos++]='T'; out[pos++]='A'; out[pos++]='R';
                    out[pos++]='T'; out[pos++]='_'; out[pos++]='E'; out[pos++]='R';
                    out[pos++]='R';
                    out[pos++]='\r'; out[pos++]='\n'; out[pos]='\0';
                } else {
                    uint16_t missing[NET_MAX_PARTICIPANTS];
                    int miss_n = 0, i2, j2;
                    for (i2 = 0; i2 < roster_n; i2++) {
                        int acked = 0;
                        for (j2 = 0; j2 < ack_count; j2++)
                            if (ack_ranks[j2] == roster[i2]) { acked = 1; break; }
                        if (!acked) missing[miss_n++] = roster[i2];
                    }
                    out[pos++]='+'; out[pos++]='N'; out[pos++]='E'; out[pos++]='T';
                    out[pos++]='S'; out[pos++]='T'; out[pos++]='A'; out[pos++]='R';
                    out[pos++]='T'; out[pos++]='_'; out[pos++]='O'; out[pos++]='K';
                    out[pos++]=',';
                    pos += fmt_dec(&out[pos], ack_count);
                    out[pos++]=',';
                    pos += fmt_dec(&out[pos], (int)root_rank);
                    out[pos++]=',';
                    pos += fmt_dec(&out[pos], miss_n);
                    for (i2 = 0; i2 < miss_n; i2++) {
                        out[pos++]=',';
                        pos += fmt_dec(&out[pos], (int)missing[i2]);
                    }
                    out[pos++]='\r'; out[pos++]='\n'; out[pos]='\0';
                }
                test_run_info((unsigned char *)out);
            }
            gateway_rx_flush();
        }
    }
    else if (strncmp(cmd, "AT+STOP", 7) == 0)
    {
        /* Send STOP_TDOA to Master via UWB, wait for ACK */
        {
            uint8_t pad[3] = {0xCC, 0xDD, 0xEE}; int _r, ack_ok = 0;
            for (_r = 0; _r < 3 && !ack_ok; _r++) {
                gateway_send_uwb_ack(0x0000, FUNC_STOP_TDOA, pad, 3);
                { uint32_t dl = HAL_GetTick() + (_r < 2 ? 30u : 100u);
                  port_DisableEXT_IRQ();
                  while (HAL_GetTick() < dl) {
                    uint32_t st = dwt_read32bitreg(SYS_STATUS_ID);
                    if (st & SYS_STATUS_ALL_RX_ERR) {
                        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                        dwt_rxenable(DWT_START_RX_IMMEDIATE);
                        Sleep(1); continue; }
                    if (st & SYS_STATUS_RXFCG_BIT_MASK) {
                        uint32_t fl = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
                        if (fl >= GW_HDR_LEN && fl <= 32) {
                            uint8_t rb[32]; dwt_readrxdata(rb, fl, 0);
                            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
                            if (rb[9] == FUNC_STOP_ACK) { ack_ok = 1; break; }
                        } else { dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK); }
                        dwt_rxenable(DWT_START_RX_IMMEDIATE);
                    }
                    Sleep(1);
                  }
                  EXTI->PR1 = (1u << 12);
                  port_EnableEXT_IRQ();
                }
            }
            if (ack_ok) {
                gw_tdoa_active = 0;
                test_run_info((unsigned char *)"+STOP_OK\r\n");
            } else {
                test_run_info((unsigned char *)"+STOP_ERR\r\n");
            }
            gateway_rx_flush();  /* clean RX state after TX+ACK polling */
        }
    }
    else if (strncmp(cmd, "AT+START", 8) == 0)
    {
        /* Send START_TDOA to Master via UWB, wait for ACK */
        {
            uint8_t pad[3] = {0xCC, 0xDD, 0xEE}; int _r, ack_ok = 0;
            for (_r = 0; _r < 3 && !ack_ok; _r++) {
                gateway_send_uwb_ack(0x0000, FUNC_START_TDOA, pad, 3);
                { uint32_t dl = HAL_GetTick() + (_r < 2 ? 30u : 100u);
                  port_DisableEXT_IRQ();
                  while (HAL_GetTick() < dl) {
                    uint32_t st = dwt_read32bitreg(SYS_STATUS_ID);
                    if (st & SYS_STATUS_ALL_RX_ERR) {
                        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                        dwt_rxenable(DWT_START_RX_IMMEDIATE);
                        Sleep(1); continue; }
                    if (st & SYS_STATUS_RXFCG_BIT_MASK) {
                        uint32_t fl = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
                        if (fl >= GW_HDR_LEN && fl <= 32) {
                            uint8_t rb[32]; dwt_readrxdata(rb, fl, 0);
                            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
                            if (rb[9] == FUNC_START_ACK) { ack_ok = 1; break; }
                        } else { dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK); }
                        dwt_rxenable(DWT_START_RX_IMMEDIATE);
                    }
                    Sleep(1);
                  }
                  EXTI->PR1 = (1u << 12);
                  port_EnableEXT_IRQ();
                }
            }
            if (ack_ok) {
                gw_tdoa_active = 1;
                test_run_info((unsigned char *)"+START_OK\r\n");
            } else {
                test_run_info((unsigned char *)"+START_ERR\r\n");
            }
            gateway_rx_flush();  /* clean RX state after TX+ACK polling */
        }
    }
    /* Unknown command — silently ignore */
}

/*============================================================================
 * Gateway main entry — pure sniffer + AT command bridge
 *============================================================================*/
extern void reset_DWIC(void);
extern void Sleep(uint32_t ms);
extern void port_DisableEXT_IRQ(void);
extern void port_EnableEXT_IRQ(void);
extern dwt_txconfig_t txconfig_options;
#define RX_ANT_DLY 16385
#define TX_ANT_DLY 16385

/* DW3000 channel config — channel 9, 128 preamble, 6.8Mbps (matches Anchor/Tag) */
static dwt_config_t gw_config = {
    UWB_CHANNEL, DWT_PLEN_128, DWT_PAC8,
    9, 9, 1, DWT_BR_6M8, DWT_PHRMODE_STD, DWT_PHRRATE_STD,
    (129 + 8 - 8), DWT_STS_MODE_OFF, DWT_STS_LEN_64, DWT_PDOA_M0
};

/* Stub implementations for linkage — Gateway receives coords via AT commands */
void dl_set_anchor_coord(uint16_t rank, int16_t x_cm, int16_t y_cm, int16_t z_cm)
{
    (void)rank; (void)x_cm; (void)y_cm; (void)z_cm;
}
void dl_set_beacon_interval(uint16_t ms)
{
    (void)ms;
}

int fusion_gateway(void)
{
    test_run_info((unsigned char *)"GATEWAY START\r\n");

    reset_DWIC();
    Sleep(50);

    while (!dwt_checkidlerc()) { }
    if (dwt_check_dev_id() != DWT_SUCCESS) {
        test_run_info((unsigned char *)"DEV ERR\r\n");
        return -1;
    }
    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
        test_run_info((unsigned char *)"INIT ERR\r\n");
        return -1;
    }

    dwt_configure(&gw_config);
    dwt_configuretxrf(&txconfig_options);
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_setleds(DWT_LEDS_ENABLE);
    port_rf_switch_init();  /* Set RF switch to RX mode: PA6=RX_EN, PA7=TX_EN */

    test_run_info((unsigned char *)"CONFIG OK\r\n");

    /* Heartbeat: toggle LED every second to confirm main loop is running */
    int led_state = 0;
    uint32_t loop_ctr = 0;

    /* Disable EXTI around RX start */
    port_DisableEXT_IRQ();
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    EXTI->PR1 = (1u << 12);
    port_EnableEXT_IRQ();

    while (1) {
        uint32_t status;

        port_DisableEXT_IRQ();
        status = dwt_read32bitreg(SYS_STATUS_ID);
        if (status & SYS_STATUS_ALL_RX_ERR) {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }
        if (status & SYS_STATUS_RXFCG_BIT_MASK) {
            uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
            if (frame_len >= GW_HDR_LEN && frame_len <= 127) {
                uint8_t rx_buf[127];
                dwt_readrxdata(rx_buf, frame_len, 0);
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
                port_EnableEXT_IRQ();
                gateway_handle_uwb_rx(rx_buf, frame_len);
                port_DisableEXT_IRQ();
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
                port_EnableEXT_IRQ();
                continue;
            } else {
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
            }
        }
        port_EnableEXT_IRQ();

        gateway_poll_uart();
        gateway_ota_arq_poll();
        gateway_bcast_ota_poll();
        gateway_check_timeout();

        /* Heartbeat: LED toggle every ~5s */
        if (++loop_ctr >= 5000) {
            loop_ctr = 0;
            led_state = !led_state;
            dwt_setleds(led_state ? 0x3 : 0x0);
        }

        Sleep(1);
    }
    return 0;
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

