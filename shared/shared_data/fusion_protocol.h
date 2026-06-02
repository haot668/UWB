/*! ----------------------------------------------------------------------------
 *  @file    fusion_protocol.h
 *  @brief   TDOA+TOF Fusion Protocol — shared frame formats, timing, and roles
 *
 *           Supports multi-anchor (up to 16) and multi-tag (up to 16) via TDMA.
 *
 * @attention
 * Copyright 2015-2020 (c) Decawave Ltd, Dublin, Ireland.
 * All rights reserved.
 */
#ifndef FUSION_PROTOCOL_H_
#define FUSION_PROTOCOL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*============================================================================
 * Frame Function Codes
 *============================================================================*/
#define FUNC_TDOA_BLINK     0xE2u  /* Tag → All anchors (TDOA timestamp)       */
#define FUNC_CLOCK_SYNC     0xAAu  /* Main Anchor → Tag + Slaves (clock sync)  */
#define FUNC_TOF_POLL       0xE3u  /* Tag → Main Anchor (SS-TWR poll)          */
#define FUNC_TOF_RESP       0xE4u  /* Main Anchor → Tag (SS-TWR response)      */
#define FUNC_TOF_FINAL      0xE5u  /* Tag → All anchors (TOF distance result)  */
#define FUNC_SLAVE_REPORT   0xE6u  /* Slave Anchor → Main Anchor (timestamp rpt)*/

/*============================================================================
 * Anchor Roles
 *============================================================================*/
#define ANCHOR_ROLE_MAIN    0u
#define ANCHOR_ROLE_SLAVE   1u

#define ANCHOR_MAX          16u
#define TAG_MAX             16u

/*============================================================================
 * TDMA Timing (UWB microseconds)
 *============================================================================*/
/* Delay from Final RX to first slave report slot */
#define TDMA_BASE_DELAY_UUS      2000u

/* Duration of each slave report slot */
#define TDMA_SLOT_DURATION_UUS   5000u

/* Delay from blink RX to clock sync TX */
#define BLINK_RX_TO_SYNC_TX_DLY_UUS  1500u

/* Delay from poll RX to response TX */
#define POLL_RX_TO_RESP_TX_DLY_UUS   1500u

/*============================================================================
 * Frame Length Constants
 *============================================================================*/
#define BLINK_LEN       12u   /* TDOA Blink frame length  */
#define SYNC_LEN        9u    /* Clock Sync frame length   */
#define POLL_LEN        14u   /* TOF Poll frame length     */
#define RESP_LEN        20u   /* TOF Response frame length */
#define FINAL_LEN       16u   /* TOF Final frame length    */
#define REPORT_LEN      24u   /* Slave Report frame length */

#define HDR_COMMON_LEN  10u   /* Common header: FC[2],SEQ,PAN[2],ADDR[4],FUNC */

/*============================================================================
 * Frame Field Indices (within TX/RX buffer)
 *============================================================================*/
#define IDX_FC0          0u
#define IDX_FC1          1u
#define IDX_SEQ          2u
#define IDX_PAN0         3u
#define IDX_PAN1         4u
#define IDX_DEST0        5u
#define IDX_DEST1        6u
#define IDX_SRC0         7u
#define IDX_SRC1         8u
#define IDX_FUNC         9u

/* TOF Response payload indices */
#define RESP_POLL_RX_TS_IDX   10u
#define RESP_RESP_TX_TS_IDX   14u

/* TOF Final payload: TOF distance at bytes 10-11 */
#define FINAL_TOF_DIST_IDX    10u

/* Slave Report payload indices */
#define REPORT_BLINK_RX_TS_IDX   10u
#define REPORT_SYNC_RX_TS_IDX    14u
#define REPORT_CLK_OFF_IDX       18u
#define REPORT_TOF_DIST_IDX      20u
#define REPORT_FCS_IDX           22u

/*============================================================================
 * Default PAN ID and addresses
 *============================================================================*/
#define PAN_ID          0xDECAu
#define BCAST_ADDR      0xFFFFu

/*============================================================================
 * Build helper — pack 16-bit value into 2 bytes (little-endian)
 *============================================================================*/
static inline void pack_u16(uint8_t *buf, uint16_t val)
{
    buf[0] = (uint8_t)(val & 0xFFu);
    buf[1] = (uint8_t)((val >> 8) & 0xFFu);
}

/*============================================================================
 * UART Output Format Strings
 *============================================================================*/
/* Main Anchor: "F,SEQ,R=BLINK_TS,D=DIST,C=CLK_OFF[,S=N:R=TS:C=CO]" */
/* Slave Anchor: "S,RANK,BLINK_TS,SYNC_TS,CLK_OFF,TOF_DIST"           */

#ifdef __cplusplus
}
#endif

#endif /* FUSION_PROTOCOL_H_ */
