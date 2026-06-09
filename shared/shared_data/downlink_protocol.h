/*! ----------------------------------------------------------------------------
 * @file    downlink_protocol.h
 * @brief   DOWNlink TDOA protocol definitions — anchors TX, tag RX + solves
 *
 *          Frame types:
 *            0xF0  DOWNLINK_BEACON  Master → All   anchor positions table
 *            0xF1  DOWNLINK_BLINK   Slave  → All   per-anchor TDOA reference
 *
 *          TDMA: Master Beacon at slot 0, each slave at slot = rank.
 *          Cycle period = 100ms, slot duration = 1000us.
 *
 * @attention
 */

#ifndef DOWNLINK_PROTOCOL_H_
#define DOWNLINK_PROTOCOL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*============================================================================
 * Function codes
 *============================================================================*/
#define FUNC_DOWNLINK_BEACON    0xF0u   /* Master → All: anchor positions table */
#define FUNC_DOWNLINK_BLINK     0xF1u   /* Slave → All: blink code (all slaves) */
#define FUNC_DL_POS_REPORT      0xF2u   /* Tag   → GW: solved position            */

/*============================================================================
 * Timing constants
 *============================================================================*/
#define DL_SLOT_DURATION_UUS     5000u   /* TDMA slot duration (microseconds)   */
#define DL_CYCLE_PERIOD_MS       100u    /* Full cycle period (milliseconds)     */
#define DL_CYCLE_PERIOD_UUS      100000u /* Full cycle in uus                    */
#define DL_MASTER_LOST_THRESHOLD 6u      /* Consecutive missed beacons → re-disc */
/*============================================================================
 * Frame lengths (payload only — FCS auto-appended by DW3000)
 *============================================================================*/
#define DL_BEACON_LEN(N)    (19u + (N) * 4u)  /* 10B hdr+2B meta+5B ts+N×4B+2B pad (DW3210 bug) */
#define DL_BLINK_LEN         19u               /* 10B hdr+5B TX ts+2B rank+2B pad (DW3210 bug) */
#define DL_POS_REPORT_LEN    22u               /* 10B header+4×int16+2B ver+2B pad (DW3210 bug) */
#define DL_FRAME_PAD         2u                /* Padding to absorb DW3210 RX last-2-byte corruption */
#define DL_MAX_ANCHORS       16u               /* Max anchors per group       */

/*============================================================================
 * Beacon field indices (REVISED for DW3210: bytes 15-16 are RX-corrupted)
 *   [0-9]   IEEE 802.15.4 header + function code (10 bytes)
 *   [10-14] Master TX timestamp (40-bit, 5 bytes) — safe zone
 *   [15]    Number of anchors (uint8) — recoverable from frame_len
 *   [16]    Cycle sequence number (uint8) — optional
 *   [17..]  Anchor entries table (4 bytes each) — safe zone
 *============================================================================*/
#define DL_BCN_TX_TS_IDX     10u   /* Master TX timestamp (40-bit, 5 bytes)  */
#define DL_BCN_NUM_IDX       15u   /* Number of anchors (uint8)              */
#define DL_BCN_CYCLE_IDX     16u   /* Cycle sequence number (uint8)          */
#define DL_BCN_TABLE_IDX     17u   /* Start of anchor entries table          */

/*============================================================================
 * Blink field indices (slave TDOA reference)
 *   [0-9]   IEEE 802.15.4 header + function code 0xF1 (10 bytes)
 *   [10-14] Slave TX timestamp (40-bit, embedded for tag TDOA)
 *   [15-16] Slave rank (uint16, for identification)
 *============================================================================*/
#define DL_BLINK_TX_TS_IDX   10u   /* Slave TX timestamp (40-bit, 5 bytes)  */
#define DL_BLINK_RANK_IDX    14u   /* Slave rank (1 byte at position 14)     */
                                    /* Bytes 15-16: unused (DW3210 RX bug)   */

/*============================================================================
 * Per-anchor entry: 4 bytes (compressed for short-frame compatibility)
 *   [0]     anchor_id  (uint8,  0=master, 1..N-1=slaves)
 *   [1]     x_5cm      (int8,   sign-extend ×5 = cm)
 *   [2]     y_5cm      (int8,   sign-extend ×5 = cm)
 *   [3]     z_5cm      (int8,   sign-extend ×5 = cm)
 * Range: ±635 cm per axis.  DL_BEACON_LEN(3)=26 bytes.
 *============================================================================*/
#define DL_ANCHOR_ENTRY_SIZE  4u

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
    int8_t sx = (int8_t)(a->x_cm / 5);
    int8_t sy = (int8_t)(a->y_cm / 5);
    int8_t sz = (int8_t)(a->z_cm / 5);
    buf[0] = a->id;
    buf[1] = (uint8_t)sx;
    buf[2] = (uint8_t)sy;
    buf[3] = (uint8_t)sz;
}

static inline void unpack_anchor_entry(const uint8_t *buf, dl_anchor_entry_t *a)
{
    a->id   = buf[0];
    a->x_cm = (int16_t)((int8_t)buf[1] * 5);
    a->y_cm = (int16_t)((int8_t)buf[2] * 5);
    a->z_cm = (int16_t)((int8_t)buf[3] * 5);
}

/*============================================================================
 * Downlink tag solver interface (implemented in tdoa_solver.c)
 *============================================================================*/
int tdoa_solve(const dl_anchor_entry_t *anchors,
               const uint64_t *toa_dtu,
               uint8_t num_anchors,
               float *x_cm_out, float *y_cm_out, float *z_cm_out,
               float *rms_out);

void     dl_set_beacon_interval(uint16_t ms);
uint16_t dl_get_beacon_interval(void);
void     dl_set_anchor_coord(uint8_t rank, int16_t x_cm, int16_t y_cm,
                             int16_t z_cm);

#ifdef __cplusplus
}
#endif

#endif /* DOWNLINK_PROTOCOL_H_ */
