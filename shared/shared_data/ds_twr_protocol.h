/*! ----------------------------------------------------------------------------
 * @file    ds_twr_protocol.h
 * @brief   DS-TWR (Double-Sided Two-Way Ranging) protocol — 4-anchor equal topology
 *
 *          Frame types:
 *            0xD0  DS_TWR_POLL    Tag     → All  anchors (broadcast)
 *            0xD1  DS_TWR_RESP    Anchor  → Tag  in slot (rank-based TDMA)
 *            0xD2  DS_TWR_FINAL   Tag     → All  anchors (broadcast, w/ RX ts)
 *            0xD3  DS_TWR_REPORT  Anchor  → Tag  in slot (computed distance)
 *
 *          Cycle: 50ms (20 Hz).
 *          Slot duration: 3ms (matches existing DL_SLOT_DURATION_UUS).
 *
 *          All 4 anchors are equal (no master). Tag initiates ranging.
 *          Anchor rank assigned dynamically on power-up by listening + UID
 *          sort — see auto_anchor.h.  Anchor slot index = my_rank (1..4).
 *
 *          Timing (relative to cycle start at tag TX POLL):
 *            t=0.0ms      tag TX POLL
 *            t=R*3ms      anchor rank R TX RESP    (R=1..4)
 *            t=15.0ms     tag TX FINAL (broadcast with 4 RX timestamps)
 *            t=15+R*3ms   anchor rank R TX REPORT  (R=1..4, distance computed)
 *            t=29ms       tag has all 4 distances
 *            t=29-35ms    tag: multilateration → Kalman → POS_REPORT
 *
 *          Total airtime: ~10 messages × ~150 µs = 1.5 ms.
 *          Plenty of margin in 50ms cycle (97% idle).
 *
 * @attention
 * Copyright 2026 (c) MiniMax.
 * Replaces TDOA scheme for ±10 cm precision under temperature drift and
 * multipath.
 */
#ifndef DS_TWR_PROTOCOL_H_
#define DS_TWR_PROTOCOL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*============================================================================
 * Function codes (avoid collision with existing 0xE0-0xEF TDOA / 0xF0-0xFE
 * gateway / 0xE4-0xE6 OTA ranges)
 *============================================================================*/
#define FUNC_DS_TWR_POLL    0xD0u   /* Tag → All anchors (broadcast)         */
#define FUNC_DS_TWR_RESP    0xD1u   /* Anchor → Tag (in rank slot)           */
#define FUNC_DS_TWR_FINAL   0xD2u   /* Tag → All anchors (broadcast)         */
#define FUNC_DS_TWR_REPORT  0xD3u   /* Anchor → Tag (in rank slot)           */

/*============================================================================
 * Timing constants
 *============================================================================*/
#define DSTWR_CYCLE_MS        50u   /* Full cycle period (ms) — 20 Hz         */
#define DSTWR_SLOT_US         3000u /* Per-anchor slot duration (us)          */
#define DSTWR_RESP_BASE_MS    0u    /* RESP slot 0 starts at POLL TX          */
                                    /* slot R fires at R * DSTWR_SLOT_US      */
#define DSTWR_FINAL_OFFSET_MS 15u   /* FINAL TX at this offset from POLL      */
#define DSTWR_REPORT_BASE_MS  15u   /* REPORT slot 0 starts at FINAL TX       */

/*============================================================================
 * Limits
 *============================================================================*/
#define DSTWR_MAX_ANCHORS     4u    /* Hard-coded to 4 anchors per tag       */

/*============================================================================
 * Frame sizes (DW3000 auto-appends 2-byte FCS; +2B pad for DW3210 RX bug)
 *
 * POLL  (tag → all): 18 bytes payload
 *   [10] session_id (1B)
 *   [11-15] T_poll_tx tag-clock (40-bit, 5B)
 *   [16-17] reserved pad
 *
 * RESP  (anchor → tag): 25 bytes payload
 *   [10]   session_id (1B)
 *   [11]   anchor_rank (1B, 1..4)
 *   [12]   rssi_dbm (1B, signed, offset+128)
 *   [13-17] T_poll_rx anchor-clock (40-bit, 5B)
 *   [18-22] T_resp_tx anchor-clock (40-bit, 5B)
 *   [23-24] reserved pad
 *
 * FINAL (tag → all): 38 bytes payload
 *   [10]   session_id (1B)
 *   [11]   num_anchors (1B, 1..4)
 *   [12-16] T_poll_tx tag-clock (40-bit, redundant w/ POLL)
 *   [17-21] T_resp_rx_1 (40-bit)
 *   [22-26] T_resp_rx_2 (40-bit)
 *   [27-31] T_resp_rx_3 (40-bit)
 *   [32-36] T_resp_rx_4 (40-bit)
 *   [37]    reserved pad
 *
 * REPORT (anchor → tag): 22 bytes payload
 *   [10]   session_id (1B)
 *   [11]   anchor_rank (1B)
 *   [12]   rssi_dbm (1B)
 *   [13-14] reserved (2B, future use: AOA / PDoA phase)
 *   [15-18] distance_mm (int32 LE, signed mm — covers ±2147 m)
 *   [19-21] reserved pad (3B to align)
 *============================================================================*/
#define DSTWR_POLL_LEN     18u
#define DSTWR_RESP_LEN     25u
#define DSTWR_FINAL_LEN    38u
#define DSTWR_REPORT_LEN   22u

/*============================================================================
 * Payload field indices (within RX buffer, after 10-byte IEEE header)
 *============================================================================*/
#define DSTWR_SESSION_IDX     10u   /* All frames: session_id                */

#define DSTWR_POLL_TX_IDX     11u   /* POLL: T_poll_tx (40-bit)              */

#define DSTWR_RESP_RANK_IDX   11u   /* RESP: anchor_rank                     */
#define DSTWR_RESP_RSSI_IDX   12u   /* RESP: rssi (1B)                      */
#define DSTWR_RESP_RX_IDX     13u   /* RESP: T_poll_rx (40-bit)             */
#define DSTWR_RESP_TX_IDX     18u   /* RESP: T_resp_tx (40-bit)             */

#define DSTWR_FINAL_NUM_IDX   11u   /* FINAL: num_anchors                   */
#define DSTWR_FINAL_TX_IDX    12u   /* FINAL: T_poll_tx (40-bit)            */
#define DSTWR_FINAL_RX1_IDX   17u   /* FINAL: T_resp_rx_1 (40-bit)          */
#define DSTWR_FINAL_RX2_IDX   22u   /* FINAL: T_resp_rx_2                   */
#define DSTWR_FINAL_RX3_IDX   27u   /* FINAL: T_resp_rx_3                   */
#define DSTWR_FINAL_RX4_IDX   32u   /* FINAL: T_resp_rx_4                   */

#define DSTWR_REPORT_RANK_IDX 11u   /* REPORT: anchor_rank                  */
#define DSTWR_REPORT_RSSI_IDX 12u   /* REPORT: rssi                         */
#define DSTWR_REPORT_DIST_IDX 15u   /* REPORT: distance_mm (int32 LE)       */

/*============================================================================
 * Session id: 8-bit counter (cycles 0..255, wraps)
 *   Used to disambiguate late-arriving frames from previous cycles when
 *   tag is unsynced.  Anchor also echoes session_id so tag can match.
 *============================================================================*/

/*============================================================================
 * Per-anchor ranging result (filled by anchor after computing DS-TWR TOF,
 * echoed by tag after RX for downstream processing / logging).
 *============================================================================*/
typedef struct {
    uint8_t  rank;          /* 1..4 (anchor rank in election)         */
    uint8_t  rssi_dbm;      /* Average RSSI of POLL+FINAL             */
    int32_t  distance_mm;   /* Computed one-way distance (signed mm)  */
    uint32_t tof_dtu;       /* Time-of-flight in DW3210 time units    */
                             /*   (~15.65 ps each; 213 DTU = ~1 m)     */
} dst_twr_result_t;

/*============================================================================
 * DS-TWR compute (anchor-side)
 *   Inputs: 4 timestamps from this ranging (2 anchor-side, 2 tag-side).
 *   Output: distance_mm, tof_dtu, rssi_dbm.
 *
 *   Decawave formula cancels both clock offset AND first-order clock
 *   rate error:
 *
 *     Tr1 = T_resp_rx_tag    - T_poll_tx_tag    (tag-clock)
 *     Tre = T_resp_tx_anchor - T_poll_rx_anchor  (anchor-clock)
 *     Tr2 = T_final_rx_anchor - T_resp_tx_anchor (anchor-clock)
 *     Tfe = T_final_tx_tag   - T_resp_rx_tag    (tag-clock)
 *
 *     TOF_dtu = (Tr1*Tr2 - Tre*Tfe) / (Tr1 + Tr2 + Tre + Tfe)
 *============================================================================*/
typedef struct {
    uint64_t T_poll_rx;     /* anchor clock, 40-bit padded to uint64  */
    uint64_t T_resp_tx;     /* anchor clock                          */
    uint64_t T_resp_rx;     /* tag clock, from FINAL                 */
    uint64_t T_poll_tx;     /* tag clock, from POLL                  */
    uint64_t T_final_rx;    /* anchor clock                          */
    uint64_t T_final_tx;    /* tag clock, from FINAL                 */
} dst_twr_timestamps_t;

int dstwr_compute(const dst_twr_timestamps_t *ts,
                  int32_t *distance_mm, uint32_t *tof_dtu);

#ifdef __cplusplus
}
#endif

#endif /* DS_TWR_PROTOCOL_H_ */