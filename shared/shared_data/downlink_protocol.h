/*! ----------------------------------------------------------------------------
 * @file    downlink_protocol.h
 * @brief   DOWNlink TDOA protocol definitions — anchors TX, tag RX + solves
 *
 *          Frame types:
 *            0xEE  DOWNLINK_BEACON  Master → All   anchor positions table
 *            0xEF  DOWNLINK_BLINK   Slave  → All   per-anchor TDOA reference
 *
 *          TDMA: Master Beacon at slot 0, each slave at slot = rank.
 *          Cycle period = 500ms, slot duration = 3000us.
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
#define FUNC_DOWNLINK_BEACON    0xEEu   /* Master → All: anchor positions table */
#define FUNC_DOWNLINK_BLINK     0xEFu   /* Slave → All: blink code (all slaves) */
#define FUNC_DL_POS_REPORT      0xE3u   /* Tag → GW: solved position (E3=E2+1,  */
                                        /* avoids collision with FUNC_FIND_REQ=0xF2) */

/*============================================================================
 * Timing constants
 *============================================================================*/
#define DL_SLOT_DURATION_UUS     3000u   /* TDMA slot duration (microseconds)   */
#define DL_CYCLE_PERIOD_MS       50u     /* Full cycle period (ms) — 20 Hz       */
#define DL_CYCLE_PERIOD_UUS      50000u  /* Full cycle in uus (beacon + 4 blinks) */
#define DL_MASTER_LOST_THRESHOLD 10u     /* Consecutive missed beacons → re-disc */
/*============================================================================
 * Frame lengths (payload only — FCS auto-appended by DW3000)
 *============================================================================*/
#define DL_BEACON_LEN(N)    (19u + (N) * 7u)  /* 10B hdr+5B ts+2B meta+N×7B+2B pad (DW3210 bug) */
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
#define DL_BCN_NUM_IDX       15u   /* [7]=TDOA_ACTIVE, [6:0]=num anchors     */
#define DL_BCN_CYCLE_IDX     16u   /* Cycle sequence number (uint8)          */
#define DL_BCN_TABLE_IDX     17u   /* Start of anchor entries table          */

/* DL_BCN_NUM_IDX bit layout:
 *   bit 7:   DL_BCN_TDOA_ACTIVE — 0=config/stopped, 1=TDOA running
 *   bit 6-0: number of anchors (0-15)
 * Slave checks this bit in the Master beacon before sending TDMA blinks. */
#define DL_BCN_TDOA_ACTIVE    0x80u

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
 * Per-anchor entry: 7 bytes (int16 cm — full field range)
 *   [0]     anchor_id  (uint8,  0=master, 1..N-1=slaves)
 *   [1-2]   x_cm       (int16,  little-endian, cm)
 *   [3-4]   y_cm       (int16,  little-endian, cm)
 *   [5-6]   z_cm       (int16,  little-endian, cm)
 * Range: ±327.67 m per axis.  DL_BEACON_LEN(4)=47 bytes.
 *============================================================================*/
#define DL_ANCHOR_ENTRY_SIZE  7u

typedef struct {
    uint8_t  id;
    int16_t  x_cm;     /* cm, e.g. 250 = 2.50 m */
    int16_t  y_cm;     /* cm */
    int16_t  z_cm;     /* cm */
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
