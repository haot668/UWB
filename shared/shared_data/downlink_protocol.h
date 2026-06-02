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
#define FUNC_DOWNLINK_BLINK     0xF1u   /* Slave  → All: per-anchor reference  */

/*============================================================================
 * Timing constants
 *============================================================================*/
#define DL_SLOT_DURATION_UUS     1000u   /* TDMA slot duration (microseconds)   */
#define DL_CYCLE_PERIOD_MS       100u    /* Full cycle period (milliseconds)     */
#define DL_CYCLE_PERIOD_UUS      100000u /* Full cycle in uus                    */
#define DL_MASTER_LOST_THRESHOLD 6u      /* Consecutive missed beacons → re-disc */

/*============================================================================
 * Frame lengths (payload only — FCS auto-appended by DW3000)
 *============================================================================*/
#define DL_BEACON_LEN(N)    (14u + (N) * 7u)  /* N = number of anchors       */
#define DL_BLINK_LEN         12u               /* 10B header + 2B FCS         */
#define DL_MAX_ANCHORS       16u               /* Max anchors per group       */

/*============================================================================
 * Beacon field indices
 *============================================================================*/
#define DL_BCN_NUM_IDX       10u   /* Number of anchors (uint8)              */
#define DL_BCN_CYCLE_IDX     11u   /* Cycle sequence number (uint8)          */
#define DL_BCN_TABLE_IDX     12u   /* Start of anchor entries table          */

/*============================================================================
 * Per-anchor entry: 7 bytes
 *   [0]     anchor_id  (uint8,  0=master, 1..N-1=slaves)
 *   [1-2]   x_cm       (int16,  little-endian)
 *   [3-4]   y_cm       (int16,  little-endian)
 *   [5-6]   z_cm       (int16,  little-endian)
 *============================================================================*/
#define DL_ANCHOR_ENTRY_SIZE  7u

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
    buf[1] = (uint8_t)(a->x_cm & 0xFF);
    buf[2] = (uint8_t)((a->x_cm >> 8) & 0xFF);
    buf[3] = (uint8_t)(a->y_cm & 0xFF);
    buf[4] = (uint8_t)((a->y_cm >> 8) & 0xFF);
    buf[5] = (uint8_t)(a->z_cm & 0xFF);
    buf[6] = (uint8_t)((a->z_cm >> 8) & 0xFF);
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

#ifdef __cplusplus
}
#endif

#endif /* DOWNLINK_PROTOCOL_H_ */
