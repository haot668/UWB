/*! ----------------------------------------------------------------------------
 *  @file    auto_anchor.h
 *  @brief   Auto-discovery protocol for multi-anchor networks
 *
 *           Phases:
 *             LISTEN (2s) → hear Beacon → JOIN
 *             LISTEN timeout  → ELECT → claim + collect → determine master
 *             JOIN            → request rank from master
 *             RUN             → enter normal fusion protocol loop
 *
 *           Election: lowest STM32 UID[31:0] wins (deterministic, no collision)
 *
 * @attention
 * Copyright 2015-2020 (c) Decawave Ltd, Dublin, Ireland.
 * All rights reserved.
 */
#ifndef AUTO_ANCHOR_H_
#define AUTO_ANCHOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*============================================================================
 * Auto-discovery function codes
 *============================================================================*/
#define FUNC_ANCHOR_BEACON   0xE7u  /* Master → All (periodic heartbeat)     */
#define FUNC_ELECTION_CLAIM  0xE8u  /* Candidate → All (UID claim)          */
#define FUNC_JOIN_REQUEST    0xE9u  /* Slave → Master (join request)        */
#define FUNC_JOIN_ACK        0xEAu  /* Master → Slave (assign rank)         */

/*============================================================================
 * Timing constants (milliseconds)
 *============================================================================*/
#define AUTO_LISTEN_TIMEOUT_MS     2000u   /* How long to listen for beacon  */
#define AUTO_ELECT_WINDOW_MS       1500u   /* How long to collect claims     */
#define AUTO_SLOT_DURATION_MS      100u    /* Per-candidate election slot    */
#define AUTO_JOIN_TIMEOUT_MS       2000u   /* Join request / ack timeout     */
#define AUTO_BEACON_INTERVAL_S     10u     /* Master beacon interval (sec)   */

/*============================================================================
 * Max anchors (rank 0 = master, 1..7 = slaves)
 *============================================================================*/
#define AUTO_ANCHOR_MAX            8u

/*============================================================================
 * Frame lengths
 *============================================================================*/
#define CLAIM_LEN   14u   /* FC[2],SEQ,PAN[2],DEST[2],SRC[2],FUNC, UID[4],  FCS[2] */
#define JOIN_REQ_LEN 14u  /* FC[2],SEQ,PAN[2],DEST[2],SRC[2],FUNC, UID[4],  FCS[2] */
#define JOIN_ACK_LEN 13u  /* FC[2],SEQ,PAN[2],DEST[2],SRC[2],FUNC, RANK,NODE,FCS[2] */
#define BEACON_LEN   20u  /* FC[2],SEQ,PAN[2],DEST[2],SRC[2],FUNC, GROUP,COUNT, UID[4], VER[2], FCS[2] */

/*============================================================================
 * Payload field indices (within TX/RX buffer, after 10-byte IEEE header)
 *============================================================================*/
#define AUTO_PAYLOAD_IDX    10u

/* UID is at bytes 10-13 in CLAIM / JOIN_REQ / JOIN_ACK / BEACON */
#define AUTO_UID_IDX        10u
/* ACK rank at byte 14 */
#define AUTO_ACK_RANK_IDX   14u
#define AUTO_ACK_NODE_IDX   15u
/* Beacon group/count at bytes 14-15 */
#define AUTO_BCN_GROUP_IDX  14u
#define AUTO_BCN_COUNT_IDX  15u
/* Beacon firmware version at bytes 16-17 */
#define AUTO_BCN_FWVER_IDX  16u

/*============================================================================
 * Auto-discovery state machine
 *============================================================================*/
typedef enum {
    AUTO_STATE_LISTEN = 0,   /* Listening for existing master beacon     */
    AUTO_STATE_ELECT,        /* Sending claim, collecting candidates     */
    AUTO_STATE_JOIN,         /* Requesting rank from master              */
    AUTO_STATE_RUN,          /* Discovery complete, entering main loop   */
    AUTO_STATE_SLAVE_READY,  /* Rank assigned, waiting for tag frames    */
    AUTO_STATE_ERROR         /* Discovery failed (timeout / no master)   */
} auto_state_t;

/*============================================================================
 * Per-anchor discovery runtime data
 *============================================================================*/
typedef struct {
    auto_state_t state;
    uint8_t      my_rank;           /* 0 = master, 1..7 = slave         */
    uint8_t      group_id;          /* Anchor group (from master beacon) */
    uint8_t      node_count;        /* Total anchors in group            */
    uint32_t     my_uid32;          /* STM32 UID[31:0] for election      */
    uint8_t      master_addr[2];    /* Short address of master (or self) */
    uint32_t     best_uid;          /* Lowest UID seen during election   */
    uint8_t      candidates_seen;   /* Count of claims received          */
} auto_disc_t;

/*============================================================================
 * Helpers — pack/unpack UID
 *============================================================================*/
static inline void pack_uid(uint8_t *buf, uint32_t uid)
{
    buf[0] = (uint8_t)(uid & 0xFFu);
    buf[1] = (uint8_t)((uid >> 8) & 0xFFu);
    buf[2] = (uint8_t)((uid >> 16) & 0xFFu);
    buf[3] = (uint8_t)((uid >> 24) & 0xFFu);
}

static inline uint32_t unpack_uid(const uint8_t *buf)
{
    return (uint32_t)buf[0]
         | ((uint32_t)buf[1] << 8)
         | ((uint32_t)buf[2] << 16)
         | ((uint32_t)buf[3] << 24);
}

#ifdef __cplusplus
}
#endif

#endif /* AUTO_ANCHOR_H_ */
