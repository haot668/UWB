/*! ----------------------------------------------------------------------------
 * @file    downlink_protocol.h
 * @brief   BD8NCF-style mesh sync protocol for UWB TDOA positioning.
 *
 *          Architecture: every anchor is a peer.  Each anchor broadcasts a
 *          compact TimeSync frame on its own cadence (~100 ms).  Slaves sync
 *          to other anchors via embedded TX timestamp + per-anchor Kalman
 *          clock filter (clock_sync.h).
 *
 *          Frame types:
 *            0xEE  TIMESYNC          Anchor → broadcast   periodic sync
 *            0xED  CLOCK_FEEDBACK    Observer → unicast    sync error report
 *            0xE3  POS_REPORT        Tag → broadcast       solved position
 *
 *          No TDMA.  No master/slave.  Cascading sync via cs_level field.
 *
 * @attention
 * Copyright 2026 (c) UWB project authors.
 * --------------------------------------------------------------------------- */
#ifndef DOWNLINK_PROTOCOL_H_
#define DOWNLINK_PROTOCOL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*============================================================================
 * Function codes
 *============================================================================*/
#define FUNC_DOWNLINK_BEACON   0xEEu   /* Anchor → All: TimeSync frame (BD8NCF) */
#define FUNC_CLOCK_FEEDBACK    0xEDu   /* Observer → Unicast: sync error       */
#define FUNC_DL_POS_REPORT     0xE3u   /* Tag → GW: solved position             */

/*============================================================================
 * Timing constants — per-anchor self-broadcast (BD8NCF style)
 *============================================================================*/
#define DL_TIMESYNC_PERIOD_MS      100u   /* Each anchor broadcasts every 100 ms  */
#define DL_TIMESYNC_PERIOD_UUS     100000u
#define DL_RX_WINDOW_MS            95u    /* Tag listens for 95 ms within 100 ms slot */
#define DL_MAX_TRACKED_ANCHORS      8u     /* Tag tracks up to 8 anchors          */

/* Anchor election (used during Phase 1.2 cascading sync) ----------------- */
#define DL_CS_LEVEL_UNINIT         0xFFu  /* Anchor not yet assigned a level     */
#define DL_CS_LEVEL_ROOT            0u     /* Level 0 = root clock source         */
#define DL_PARENT_LOST_TIMEOUT_MS   30000u /* If parent silent 30 s, re-elect   */

/*============================================================================
 * TimeSync frame layout (26 bytes payload)
 *   [0-1]   frame_ctrl       (0x41, 0x88)
 *   [2]     seq8
 *   [3-4]   pan_id           (0xCA, 0xDE)
 *   [5-6]   dest_addr        (0xFFFF broadcast)
 *   [7-8]   source_addr      (anchor rank, LSB MSB)
 *   [9]     func             (0xEE)
 *   [10-14] tx_timestamp40   (40-bit, predicted TX time)
 *   [15]    cs_level         (0xFF uninit, 0=root, 1+=child)
 *   [16-23] parent_eui64     (anchor's parent EUI; 0 if cs_level=0)
 *   [24-25] anchor_count + flags byte + reserved
 * DW3000 appends 2-byte CRC automatically; we still pad 2 bytes
 * before CRC zone to absorb DW3210 RX last-2-byte corruption.
 *============================================================================*/
#define DL_TIMESYNC_LEN            26u
#define DL_TS_TX_TS_IDX            10u    /* 5-byte TX timestamp               */
#define DL_TS_CS_LEVEL_IDX         15u
#define DL_TS_PARENT_EUI_IDX       16u    /* 8-byte EUI64                       */
#define DL_TS_RESERVED_IDX         24u    /* anchor_count + flags              */

/*============================================================================
 * Clock Feedback frame layout (Observer → unicast, 27 bytes payload)
 *   [0-9]   header (10 B, dst = target anchor EUI64)
 *   [10-17] dst_eui64 (8 B, mirror of header dst for clarity)
 *   [18]    func = 0xED
 *   [19-22] error_ticks     (int32 LE, signed, DW3000 ticks)
 *   [24-25] confidence      (uint16 LE, 0..65535)
 *============================================================================*/
#define DL_FEEDBACK_LEN            27u
#define DL_FB_ERROR_TICKS_IDX      19u
#define DL_FB_CONFIDENCE_IDX       24u

/*============================================================================
 * Tag Position Report (22 bytes — unchanged for host compatibility)
 *   [0-9]   header
 *   [10-11] int16 x_cm (Kalman-filtered)
 *   [12-13] int16 y_cm
 *   [14-15] int16 z_cm
 *   [16-17] uint16 quality_score (0..1000 = 0.0..1.0)
 *   [18-19] uint16 fw_version
 *   [20-21] pad (DW3210 RX corruption)
 *============================================================================*/
#define DL_POS_REPORT_LEN          22u
#define DL_MAX_ANCHORS             16u    /* Backward-compat for tdoa_solver.c */

/*============================================================================
 * Per-anchor position entry (7 bytes, int16 cm — full field range)
 *   [0]     anchor_id  (uint8)
 *   [1-2]   x_cm       (int16 LE, cm)
 *   [3-4]   y_cm       (int16 LE, cm)
 *   [5-6]   z_cm       (int16 LE, cm)
 *============================================================================*/
#define DL_ANCHOR_ENTRY_SIZE  7u

/* TDOA conversion: 1 DW3000 device time unit ≈ 15.65 ps ≈ 0.4691 cm */
#define TDOA_CM_PER_DTU         0.4691f

typedef struct {
    uint8_t  id;
    int16_t  x_cm;
    int16_t  y_cm;
    int16_t  z_cm;
} dl_anchor_entry_t;

/*============================================================================
 * Inline pack/unpack helpers
 *============================================================================*/
static inline void pack_anchor_entry(uint8_t *buf, const dl_anchor_entry_t *a)
{
    buf[0] = a->id;
    buf[1] = (uint8_t)( a->x_cm        & 0xFF);
    buf[2] = (uint8_t)((a->x_cm >> 8)  & 0xFF);
    buf[3] = (uint8_t)( a->y_cm        & 0xFF);
    buf[4] = (uint8_t)((a->y_cm >> 8)  & 0xFF);
    buf[5] = (uint8_t)( a->z_cm        & 0xFF);
    buf[6] = (uint8_t)((a->z_cm >> 8)  & 0xFF);
}

static inline void unpack_anchor_entry(const uint8_t *buf, dl_anchor_entry_t *a)
{
    a->id   = buf[0];
    a->x_cm = (int16_t)((uint16_t)buf[1] | ((uint16_t)buf[2] << 8));
    a->y_cm = (int16_t)((uint16_t)buf[3] | ((uint16_t)buf[4] << 8));
    a->z_cm = (int16_t)((uint16_t)buf[5] | ((uint16_t)buf[6] << 8));
}

/*============================================================================
 * Anchor runtime state (set via AT+COORD from host, broadcast in TimeSync)
 *============================================================================*/
void dl_set_anchor_coord(uint8_t rank, int16_t x_cm, int16_t y_cm, int16_t z_cm);
void dl_get_anchor_coord(uint8_t rank, dl_anchor_entry_t *out);

#ifdef __cplusplus
}
#endif

#endif /* DOWNLINK_PROTOCOL_H_ */