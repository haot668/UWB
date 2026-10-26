/*! ----------------------------------------------------------------------------
 * @file    downlink_protocol.h
 * @brief   BD8NCF-style mesh sync protocol for UWB TDOA positioning.
 *
 *          Architecture: ONE TDMA grid shared by the whole network.  The
 *          Root (cs_level = 0) owns slot 0 and its beacon defines the
 *          origin of every round.  Every anchor broadcasts one compact
 *          TimeSync frame per round (50 ms → 20 frames/s) at
 *              round_origin + slot × 6.25 ms
 *          where round_origin is taken from the DW3000 SYS_TIME axis, so
 *          all anchors share ONE physical time grid.  A round carries the
 *          embedded TX timestamp, so the Tag derives the anchor clock
 *          model from it (clock_sync.h) and the slot compensation cancels
 *          the grid offsets from the TDOA.
 *
 *          Frame types:
 *            0xEE  TIMESYNC          Anchor → broadcast   periodic sync
 *            0xED  CLOCK_FEEDBACK    Observer → unicast    sync error report
 *            0xE3  POS_REPORT        Tag → broadcast       solved position
 *
 *          One "cycle" (超周期) = DL_ROUNDS_PER_CYCLE rounds = 100 ms.
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

/* Manual network startup — dedicated codes, MUST NOT collide with
 * gateway_protocol.h (FUNC_PREP_CMD 0xF0 / FUNC_PREP_ACK 0xF1). */
#define FUNC_NETWORK_START     0xE8u   /* GW → Anchor broadcast: start network  */
#define FUNC_NETWORK_ACK      0xE9u   /* Anchor → GW unicast: network ack      */

/* Anchor discovery — anchor announces itself while not yet networked */
#define FUNC_ANCHOR_DISCOVERY  0xECu   /* Anchor → GW broadcast: I exist       */

/*============================================================================
 * Manual network configuration
 *============================================================================*/
#define NET_CHILD_TX_DELAY_MS   200u   /* Child waits this long after NET_START before TX */
#define NET_MAX_SLOTS          8u      /* Max TDMA slots (same as ANCHOR_MAX_SLOTS) */

/* Anchor network state — reported by the gateway to the host as
 * "+NETSTATE,<uid>,<state>".  The gateway derives it from the beacons it
 * hears on the grid, so no extra uplink is needed:
 *   IDLE      — the anchor is not on the grid (or was never heard)
 *   STARTING  — the anchor announces itself but has no slot yet
 *   WAIT      — the anchor transmits but the Root has not confirmed its slot
 *   RUNNING   — the anchor is on the grid and its beacon is on the slot
 */
#define GW_NETSTATE_IDLE       0u
#define GW_NETSTATE_STARTING   1u
#define GW_NETSTATE_WAIT       2u
#define GW_NETSTATE_RUNNING    3u
#define NET_CONFIRM_TIMEOUT_MS  3000u  /* Child waits up to 3s for Root slot confirmation */
#define NET_DISCOVERY_PERIOD_MS 300u   /* Anchor discovery beacon period while NET_IDLE */
#define NET_RUNNING_DISCOVERY_MS 2000u /* Low-rate discovery heartbeat once networked (P2-4) */
#define NET_START_DEDUP_MS      800u   /* Ignore repeated NETWORK_START of same root */
#define NET_ALOHA_RETRY_MS      500u   /* Retry interval for the Child's ALOHA beacon */

/* Roster of participating anchors carried by NETWORK_START (P1-1).
 * Every anchor receives the SAME ordered list, so bit N of the Root's
 * presence mask always refers to the same participant on every node. */
#define NET_MAX_PARTICIPANTS   8u
#define NET_MIN_PARTICIPANTS   3u     /* fewer anchors -> TDOA is degenerate */

/*============================================================================
 * ANCHOR_DISCOVERY frame (Anchor → GW broadcast, 24 bytes)
 *   Sent periodically by every anchor while it is NOT yet in a network
 *   (NET_IDLE), so the gateway / host can list candidate anchors and let
 *   the operator pick a Root.
 *   [0-1]   frame_ctrl       (0x41, 0x88)
 *   [2]     seq8
 *   [3-4]   pan_id           (0xCA, 0xDE)
 *   [5-6]   dest_addr        (0xFFFF broadcast)
 *   [7-8]   source_addr      (Anchor rank)
 *   [9]     func             (0xEC)
 *   [10-11] anchor_rank      (uint16 LE, redundant w/ source_addr)
 *   [12-13] fw_version       (uint16 LE)
 *   [14]    net_state        (0=idle, 1=starting, 2=wait_confirm, 3=running)
 *   [15]    reserved
 *   [16-23] uid_fp           (8 bytes, low 64 bit of the 96-bit STM32 UID)
 *           The rank is a CRC16 of the full UID, so two anchors can end up
 *           with the SAME short address.  The gateway compares the
 *           fingerprint and reports +RANKCOLL when it changes (P1-4).
 *============================================================================*/
#define FUNC_ANCHOR_DISCOVERY_LEN  24u
#define DL_TS_DISC_RANK_IDX       10u
#define DL_TS_DISC_FW_IDX         12u
#define DL_TS_DISC_STATE_IDX      14u
#define DL_TS_DISC_UID_IDX        16u   /* 8 bytes */

/*============================================================================
 * NETWORK_START frame (Gateway → All anchors, broadcast)
 *   [0-1]   frame_ctrl       (0x41, 0x88)
 *   [2]     seq8
 *   [3-4]   pan_id           (0xCA, 0xDE)
 *   [5-6]   dest_addr        (0xFFFF broadcast)
 *   [7-8]   source_addr      (Gateway rank=0)
 *   [9]     func             (0xE8)
 *   [10-11] root_rank        (uint16 LE, the designated root)
 *   [12]    flags           (bit[0]=1 force reset)
 *   [13]    n_participants   (N, number of anchors taking part in the network)
 *   [14-15] reserved
 *   [16..]  participant ranks (N × uint16 LE, same order on every anchor)
 *
 * The roster solves P1-1: a Root that only learns its neighbours from the
 * frames it happens to hear can never complete the network when one link is
 * one-way.  With the roster, every anchor knows how many members the network
 * must have and the Root can report which ones it is still missing.
 *============================================================================*/
#define FUNC_NETWORK_START_LEN    16u     /* header + fixed fields            */
#define FUNC_NETWORK_START_MAX    (FUNC_NETWORK_START_LEN + NET_MAX_PARTICIPANTS * 2u)
#define NET_FLAG_FORCE_RESET     0x01u
#define DL_TS_NET_ROOT_IDX      10u
#define DL_TS_NET_FLAGS_IDX     12u
#define DL_TS_NET_NPART_IDX     13u
#define DL_TS_NET_ROSTER_IDX    16u

/*============================================================================
 * NETWORK_ACK frame (Anchor → Gateway, unicast to rank 0)
 *   [0-1]   frame_ctrl       (0x41, 0x88)
 *   [2]     seq8
 *   [3-4]   pan_id           (0xCA, 0xDE)
 *   [5-6]   dest_addr        (GW_MASTER_ADDR = 0x8001)
 *   [7-8]   source_addr      (Anchor rank)
 *   [9]     func             (0xE9)
 *   [10-11] root_rank        (confirmed root rank, uint16 LE)
 *   [12]    cs_level         (0=root, 1=child)
 *   [13]    slot_idx        (TDMA slot, 255=unassigned)
 *   [14]    n_neighbors      (discovered neighbor count)
 *============================================================================*/
#define FUNC_NETWORK_ACK_LEN      15u
#define DL_TS_NETACK_ROOT_IDX    10u
#define DL_TS_NETACK_LEVEL_IDX   12u
#define DL_TS_NETACK_SLOT_IDX    13u
#define DL_TS_NETACK_NNEIGH_IDX  14u

/*============================================================================
 * Timing constants — per-anchor self-broadcast (BD8NCF style)
 *
 * TDMA grid (P0-2/P0-3): the network runs in ROUNDS of
 *   NET_MAX_SLOTS × DL_SLOT_DTU_US = 8 × 6.25 ms = 50 ms.
 * Slot 0 belongs to the Root; the Root's beacon defines the round origin.
 * Every anchor transmits exactly once per round, at
 *   round_origin + slot × 6.25 ms
 * where round_origin is derived from the DW3000 SYS_TIME axis (never from
 * HAL_GetTick()), so all anchors share ONE physical time grid.
 *
 * 一周期（超周期）= DL_ROUNDS_PER_CYCLE 轮 = 100 ms：
 *   • 锚站每轮广播一帧 → 20 帧/s；
 *   • 标签每轮解算一次 → 20 Hz 输出。
 *
 * 槽宽由 10 ms 收到 6.25 ms 的两个好处：
 *   • 频率：轮长 80 ms → 50 ms，帧率 12.5 → 20 Hz；
 *   • 精度：槽补偿后的残余误差项 (slot_k − slot_j) × S × δf 正比于轮内
 *     跨度 S，跨度 70 ms → 43.75 ms，主导误差项降到原来的 5/8。
 *
 * 网络规模上限 8 个锚站（1 Root + 7 子站），槽表同步减半，per-anchor 状态
 * 数组（ANCHOR_MAX_NEIGHBORS / DL_MAX_TRACKED_ANCHORS）随之减半 —— 这才是
 * 64 KB STM32L412 上真正的收益点。
 *============================================================================*/
#define DL_TIMESYNC_PERIOD_MS      50u     /* 一轮 = 8 槽 × 6.25 ms = 50 ms      */
#define DL_TIMESYNC_PERIOD_UUS     50000u
#define DL_SLOT_DTU_US             6250u   /* 一个 TDMA 槽 = 6.25 ms             */
#define DL_ROUNDS_PER_CYCLE        2u      /* 一个超周期 = 2 轮                   */
#define DL_SUPERCYCLE_MS           100u    /* 超周期（一周期）= 100 ms            */
#define DL_SOLVE_INTERVAL_MS       50u     /* 标签解算节拍 = 一轮 = 20 Hz         */
#define DL_RX_WINDOW_MS            100u    /* 标签 RX 窗口 = 一个超周期           */
#define DL_MAX_TRACKED_ANCHORS     8u      /* Tag tracks up to 8 anchors          */

/* Anchor election (used during Phase 1.2 cascading sync) ----------------- */
#define DL_CS_LEVEL_UNINIT         0xFFu  /* Anchor not yet assigned a level     */
#define DL_CS_LEVEL_ROOT            0u     /* Level 0 = root clock source         */
#define DL_PARENT_LOST_TIMEOUT_MS   30000u /* If parent silent 30 s, re-elect   */
#define DL_SCAN_TIMEOUT_MS          5000u  /* 5-second scan period on boot          */

/*============================================================================
 * TimeSync frame layout (63 bytes total, including the 10-byte header)
 *   [0-1]   frame_ctrl       (0x41, 0x88)
 *   [2]     seq8
 *   [3-4]   pan_id           (0xCA, 0xDE)
 *   [5-6]   dest_addr        (0xFFFF broadcast)
 *   [7-8]   source_addr      (anchor rank, LSB MSB)
 *   [9]     func             (0xEE)
 *   [10-14] tx_timestamp40   (40-bit, THIS frame's TX instant)
 *   [15]    flags             (bit0 = TX_TS_PRECISE)              <-- P0-2
 *   [16-17] reserved
 *   [18]    slot             (own TDMA slot, 255 = unassigned/ALOHA)
 *   [19-26] parent_eui64     (8 bytes, 0 if cs_level=0)
 *   [27]    cs_level         (0xFF uninit, 0=root, 1+=child)   <-- P0-1
 *   [28-43] slot_table       (8 × uint16 rank, 0 = empty)      <-- P0-2
 *   [44-51] x/y/z coords (int16 each) + 2 pad bytes
 *   [52-53] cycle_id         (uint16, TDMA cycle counter)      <-- P0-4
 *   [54-55] present_mask     (uint16, roster presence bitmap)   <-- P1-1
 *   [56-60] rx_ref_ts40      (40-bit, RX instant of the frame this
 *                             beacon used as its clock reference) <-- DS-TWR
 *   [61-62] rx_ref_cycle     (uint16, cycle id of that frame)     <-- DS-TWR
 * DW3000 appends 2-byte CRC automatically; we still pad 2 bytes
 * before CRC zone to absorb DW3210 RX last-2-byte corruption.
 *
 * P0-1: cs_level and slot used to share byte 18 — writing the slot
 *       overwrote the level, so every frame looked like level == slot.
 *       They are now separate bytes with separate indices.
 *
 * P0-4: cycle_id tells the Tag which TDMA round a beacon belongs to.  The
 *       slot-compensated TDOA formula
 *           toa = rx_local - slot × 6.25 ms
 *       only cancels the round origin when all beacons come from the SAME
 *       round; one round of skew is 50 ms = 3.19 M DTU, far beyond the
 *       ±1.06 M DTU acceptance window.  The Tag now only pairs beacons
 *       that carry the same cycle_id.
 *
 * P0-2 (clock domain): tx_timestamp40 used to carry the PREVIOUS frame's
 *       TX instant while the receiver paired it with THIS frame's RX
 *       instant — a whole TDMA period (and the propagation delay) leaked
 *       into the clock offset estimate.  It now carries THIS frame's TX
 *       instant.  For a delayed (on-grid) frame that instant is the
 *       programmed target of dwt_setdelayedtrxtime() and is known exactly
 *       before the frame is built, so flags bit0 (TX_TS_PRECISE) is set.
 *       For an immediate frame (ALOHA / bootstrap) the real TX instant is
 *       only readable after the transmission, so the field holds an
 *       estimate (SPI write + starttx ≈ 30 µs) and the PRECISE bit is
 *       CLEARED — receivers must ignore imprecise timestamps for the
 *       clock model and for the grid origin.
 *
 * DS-TWR: rx_ref_ts40 / rx_ref_cycle let the Root close the round trip
 *       with the sender:
 *           (r2 - T1) = prop_R->X + f * (T2 - r1) + prop_X->R
 *       with r1 = the sender's RX instant of the Root frame (rx_ref_ts40),
 *       T1 = that frame's TX instant (known from rx_ref_cycle), T2 = the
 *       sender's TX instant (tx_timestamp40) and r2 = the Root's RX
 *       instant.  The Root solves the propagation delay and unicasts the
 *       correction back (FUNC_TWR_CORR), so the child can place its own
 *       beacon exactly on the Root's grid.
 *============================================================================*/
#define DL_TIMESYNC_LEN            63u
#define DL_TS_TX_TS_IDX           10u    /* 5-byte TX timestamp (lower 40 bits)    */
#define DL_TS_FLAGS_IDX           15u    /* bit0 = TX_TS_PRECISE                    */
#define DL_TS_SLOT_IDX            18u    /* own TDMA slot (255 = unassigned)        */
#define DL_TS_PARENT_EUI_IDX      19u    /* 8-byte EUI64                             */
#define DL_TS_LEVEL_IDX           27u    /* cs_level (P0-1: was DL_TS_CS_LEVEL_IDX) */
#define DL_TS_SLOT_TABLE_IDX      28u    /* 8 × uint16_t rank, 0 = empty            */
#define DL_TS_COORD_IDX           44u    /* x/y/z coords appended after slot table  */
#define DL_TS_CYCLE_IDX           52u    /* uint16 TDMA cycle counter                */
#define DL_TS_PRESENT_IDX         54u    /* uint16 roster presence bitmap            */
#define DL_TS_RX_REF_TS_IDX       56u    /* 5-byte RX instant of the reference frame */
#define DL_TS_RX_REF_CYCLE_IDX    61u    /* uint16 cycle id of that reference frame  */

/* TimeSync frame flags */
#define DL_TS_FLAG_TX_TS_PRECISE  0x01u  /* tx_timestamp40 is exact (delayed TX)    */

/*============================================================================
 * TWR correction frame (Root → broadcast, variable length)
 *   The Root solved the Root↔Child propagation delay from the round trip
 *   and tells the child how far its own beacon was off the grid.  The child
 *   shifts its next slot instant accordingly, which removes the systematic
 *   (root->child propagation delay) bias from the Tag's slot-compensated
 *   TDOA.
 *
 *   The delay is carried in DEVICE TIME UNITS, not centimetres: the DS-TWR
 *   solver works entirely in DTU and a cm round-trip would throw away the
 *   sub-cm resolution (1 cm = 2.132 DTU) — and it keeps the clock-domain
 *   question out of the wire format.  The child's clock differs from the
 *   Root's by f_root, so the value sent is the delay expressed in the
 *   CHILD's clock (p_root / f_root), which is the unit the child needs to
 *   shift its own grid origin.
 *
 *   P1-3: the corrections of ALL children travel in ONE frame, one batch
 *   per TDMA cycle.  The previous scheme (one unicast per cycle, round
 *   robin over the children) delayed a child's correction by up to N
 *   cycles — 0.64 s for 8 anchors — which made a "2 cycle" freshness
 *   limit impossible and let the path delay go badly stale.  With the
 *   batch every child learns its own delay every cycle.
 *
 *   [0-9]   header (10 B, dst = 0xFFFF broadcast)
 *   [10]    entry count N (1..DL_TWR_CORR_MAX_ENTRIES)
 *   [11]    quality: age of the OLDEST entry, in TDMA cycles
 *   [12..]  N entries of DL_TWR_CORR_ENTRY_LEN bytes:
 *             target_rank  (uint16 LE, the child this correction is for)
 *             corr_dtu     (uint16 LE, Root→Child path delay in DTU)
 *
 *   corr_dtu fits in 16 bit: the plausible range is 40..22,000 DTU.
 *============================================================================*/
#define FUNC_TWR_CORR           0xE7u   /* Root → broadcast: path corrections     */
#define DL_TWR_CORR_HDR_LEN     12u    /* header + count + quality               */
#define DL_TWR_CORR_COUNT_IDX   10u
#define DL_TWR_CORR_QUAL_IDX    11u
#define DL_TWR_CORR_ENTRY_IDX   12u
#define DL_TWR_CORR_ENTRY_LEN   4u     /* rank (u16) + corr_dtu (u16)            */
#define DL_TWR_CORR_MAX_ENTRIES 8u
#define DL_TWR_CORR_MAX_LEN     (DL_TWR_CORR_HDR_LEN \
                                 + DL_TWR_CORR_ENTRY_LEN * DL_TWR_CORR_MAX_ENTRIES)

/* Plausible Root→Child path delay, in DTU (child clock):
 *   0.2 m = 42.6 DTU .. 100 m = 21,320 DTU. */
#define DL_TWR_CORR_MIN_DTU     40u
#define DL_TWR_CORR_MAX_DTU     22000u

/* Freshness limit (P1-3): if our own entry has not been refreshed for more
 * than this many TDMA cycles, the correction is stale — the path delay may
 * have changed (the anchor moved) — and the child leaves the grid. */
#define DL_TWR_CORR_MAX_AGE_CYCLES 2u

/* Backwards-compatible alias — the cs_level byte moved to 27 (P0-1). */
#define DL_TS_CS_LEVEL_IDX        DL_TS_LEVEL_IDX

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
 *   [0-1]   frame type 0x41 0x88
 *   [2]     sequence (per tag, incremented per report)
 *   [3-4]   source EUI64 tag 0xCA 0xDE
 *   [5-6]   destination (0xFFFF = broadcast)
 *   [7-8]   source id (CRC16 over the tag's 96-bit STM32 UID)
 *   [9]     FUNC_DL_POS_REPORT
 *   [10-11] int16 x_cm (solved by the tag)
 *   [12-13] int16 y_cm
 *   [14-15] int16 z_cm
 *   [16-17] uint16 quality (0..1000 = 0.0..1.0; 65535 = no solution)
 *   [18-19] uint16 fw_version
 *   [20-21] pad (DW3210 RX corruption)
 *
 * The tag is a pure receiver in the TDMA grid.  Its report goes out in the
 * GUARD TIME OF SLOT 0: slot 0 belongs to the Root alone, so the interval
 * between the end of the Root's beacon and slot 1's beacon is guaranteed to
 * be free of any anchor transmission — no matter how many anchors are in the
 * network.  The tag therefore needs no slot of its own, and the anchor slot
 * table (8 anchors / 50 ms round / 100 ms super-cycle / 20 Hz) is untouched.
 *
 * The report instant is measured from the arrival of the ROOT's beacon at the
 * tag (rx_local), i.e. from the round origin itself:
 *
 *     report = root_rx_local + TAG_REPORT_DELAY_US + ALOHA jitter
 *
 * with DELAY + JITTER < DL_SLOT_DTU_US (6.25 ms), so the report always lands
 * inside slot 0's guard and never disturbs an anchor's reception.
 *
 * The position in the report is solved BY THE TAG; the gateway only relays
 * it.  The precision of the TX instant therefore does not affect the
 * accuracy of the position at all — the delayed TX exists only to place the
 * frame inside the guard window.
 *==========================================================================*/
#define DL_POS_REPORT_LEN          22u
#define TAG_REPORT_DELAY_US       2200u  /* 2.2 ms after the Root's beacon   */
#define TAG_REPORT_JITTER_US      2000u  /* ALOHA jitter between tags        */
#define TAG_REPORT_QUALITY_NONE  65535u  /* "no solution" marker             */
#define DL_MAX_ANCHORS             8u     /* Backward-compat for tdoa_solver.c */

/*============================================================================
 * Per-anchor position entry (7 bytes, int16 cm — full field range)
 *   [0]     anchor_id  (uint8)
 *   [1-2]   x_cm       (int16 LE, cm)
 *   [3-4]   y_cm       (int16 LE, cm)
 *   [5-6]   z_cm       (int16 LE, cm)
 *============================================================================*/
#define DL_ANCHOR_ENTRY_SIZE  7u

/* TDOA conversion: 1 DW3000 device time unit ≈ 15.65 ps ≈ 0.46907 cm.
 * P2-9: the canonical value is CM_PER_DTU_F in shared_defines.h — this
 * alias must not drift from it. */
#define TDOA_CM_PER_DTU         0.46907f

typedef struct {
    uint16_t id;    /* anchor rank (16-bit to support ranks > 255) */
    int16_t  x_cm;
    int16_t  y_cm;
    int16_t  z_cm;
} dl_anchor_entry_t;

/*============================================================================
 * Inline pack/unpack helpers
 *============================================================================*/
static inline void pack_anchor_entry(uint8_t *buf, const dl_anchor_entry_t *a)
{
    buf[0] = (uint8_t)( a->id        & 0xFF);
    buf[1] = (uint8_t)((a->id >> 8)  & 0xFF);
    buf[2] = (uint8_t)( a->x_cm        & 0xFF);
    buf[3] = (uint8_t)((a->x_cm >> 8)  & 0xFF);
    buf[4] = (uint8_t)( a->y_cm        & 0xFF);
    buf[5] = (uint8_t)((a->y_cm >> 8)  & 0xFF);
    buf[6] = (uint8_t)( a->z_cm        & 0xFF);
    buf[7] = (uint8_t)((a->z_cm >> 8)  & 0xFF);
}

static inline void unpack_anchor_entry(const uint8_t *buf, dl_anchor_entry_t *a)
{
    a->id   = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    a->x_cm = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8));
    a->y_cm = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8));
    a->z_cm = (int16_t)((uint16_t)buf[6] | ((uint16_t)buf[7] << 8));
}

/*============================================================================
 * TDOA solver — improved Gauss-Newton with outlier rejection
 *============================================================================*/
int tdoa_solve(const dl_anchor_entry_t *anchors,
               const uint64_t *toa_dtu,
               uint8_t num_anchors,
               float *x_cm_out, float *y_cm_out, float *z_cm_out,
               float *rms_out);

/*============================================================================
 * Anchor runtime state (set via AT+COORD from host, broadcast in TimeSync)
 *============================================================================*/
void dl_set_anchor_coord(uint16_t rank, int16_t x_cm, int16_t y_cm, int16_t z_cm);
void dl_get_anchor_coord(uint16_t rank, dl_anchor_entry_t *out);

#ifdef __cplusplus
}
#endif

#endif /* DOWNLINK_PROTOCOL_H_ */