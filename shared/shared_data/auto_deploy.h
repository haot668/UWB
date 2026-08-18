/*! ----------------------------------------------------------------------------
 * @file    auto_deploy.h
 * @brief   Anchor auto-deploy (anchor-to-anchor SS-TWR self-ranging) protocol.
 *
 *          The host clicks "自动部署": gateway → master (FUNC_AUTODEPLOY), the
 *          master runs a TWR session to measure all pairwise distances needed
 *          for 2D trilateration, computes the anchor coordinates, and reports
 *          them back (FUNC_AUTODEPLOY_RES) so the host can refresh the map.
 *
 *          Function codes are chosen to NOT collide with the existing sets:
 *            0xE2..0xEF  downlink TDOA (beacon/blink/pos-report)
 *            0xE4..0xE6  OTA (END/PREPARE/ABORT)
 *            0xE7..0xEA  auto-discovery (beacon/claim/join/ack)
 *            0xF0..0xFE  gateway control (0xFD = LIST_REQ, 0xFE = STOP_ACK)
 *          So we use 0xE0/0xE1 (TWR poll/resp) and 0xDC/0xDD/0xDE/0xEC.
 * ---------------------------------------------------------------------------*/
#ifndef AUTO_DEPLOY_H
#define AUTO_DEPLOY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*============================================================================
 * Function codes
 *============================================================================*/
#define FUNC_RANGE_POLL      0xE0u  /* initiator→responder: begin SS-TWR        */
#define FUNC_RANGE_RESP      0xE1u  /* responder→initiator: poll_rx_ts + resp_tx_ts */
#define FUNC_RANGE_CMD       0xDCu  /* master→slave: "range target T, report back" */
#define FUNC_RANGE_RPT       0xDDu  /* slave-initiator→master: target + distance  */
#define FUNC_AUTODEPLOY_RES  0xDEu  /* master→GW(bcast): rank + x,y,z result      */
#define FUNC_AUTODEPLOY      0xECu  /* GW→master: run auto-deploy, 2B anchor mask */

/*============================================================================
 * Frame payload indices (payload starts at byte 10 = GW_HDR_LEN)
 *   Frame header [0-9]: FC[2] SEQ[1] PAN[2] DST[2] SRC[2] FUNC[1]
 *   DST = [5-6], SRC = [7-8], FUNC = [9]
 *============================================================================*/

/* FUNC_RANGE_POLL (initiator→responder): DST=target rank, SRC=initiator rank.
 * No payload beyond the header — just 2B pad to absorb DW3210 RX tail bug. */
#define RANGE_POLL_LEN       12u    /* 10B header + 2B pad                        */

/* FUNC_RANGE_RESP (responder→initiator):
 *   [10-13] poll_rx_ts  (32-bit, little-endian) — low 32 bits of adjusted RX ts
 *   [14-17] resp_tx_ts  (32-bit, little-endian) — low 32 bits of predicted TX ts
 *   [18-19] pad
 * 32-bit timestamps are sufficient for SS-TWR (round-trip << 67ms wrap) and
 * match the proven Decawave ex_06a/06b reference exactly. */
#define RANGE_RESP_POLL_IDX  10u
#define RANGE_RESP_TX_IDX    14u
#define RANGE_RESP_LEN       20u    /* 10B header + 4B + 4B + 2B pad              */

/* FUNC_RANGE_CMD (master→slave initiator):
 *   [10-11] target rank (uint16 LE)
 *   [12-13] pad */
#define RANGE_CMD_TARGET_IDX 10u
#define RANGE_CMD_LEN        14u

/* FUNC_RANGE_RPT (slave-initiator→master):
 *   [10-11] target rank (uint16 LE)
 *   [12-15] distance_cm (int32 LE, can be negative on error)
 *   [16-17] pad */
#define RANGE_RPT_TARGET_IDX 10u
#define RANGE_RPT_DIST_IDX   12u
#define RANGE_RPT_LEN        18u

/* FUNC_AUTODEPLOY (GW→master):
 *   [10-11] anchor mask (uint16 LE) — bit i = rank i participates
 *   [12-13] pad */
#define AUTODEPLOY_MASK_IDX  10u
#define AUTODEPLOY_LEN       14u

/* FUNC_AUTODEPLOY_RES (master→GW bcast):
 *   [10]    rank
 *   [11-12] x_cm (int16 LE)
 *   [13-14] y_cm (int16 LE)
 *   [15-16] z_cm (int16 LE)
 *   [17-18] pad
 * rank = 0xFF marks the END of the result stream (master sent all anchors). */
#define AUTODEPLOY_RES_RANK_IDX 10u
#define AUTODEPLOY_RES_X_IDX    11u
#define AUTODEPLOY_RES_Y_IDX    13u
#define AUTODEPLOY_RES_Z_IDX    15u
#define AUTODEPLOY_RES_LEN      19u
#define AUTODEPLOY_RES_END      0xFFu

/* Max anchors handled by one auto-deploy session (matches DL_MAX_ANCHORS). */
#define AUTODEPLOY_MAX_ANCHORS  16u

#ifdef __cplusplus
}
#endif

#endif /* AUTO_DEPLOY_H */
