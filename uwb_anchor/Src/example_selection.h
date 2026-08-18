/*! ----------------------------------------------------------------------------
 * @file    example_selection.h
 * @brief   Anchor project — TEST_FUSION_ANCHOR (static rank, Channel 9)
 * ---------------------------------------------------------------------------*/
#ifndef TEST_SELECTION_
#define TEST_SELECTION_

#ifdef __cplusplus
extern "C" {
#endif

#define TEST_FUSION_ANCHOR

/* Dynamic role: auto-discovery assigns rank at runtime.
 * Master = 0, slaves = 1..N.  All slaves share the same firmware binary.
 * Define FUSION_ANCHOR_RANK=0 to force Master (only Master project). */
#define AUTO_ANCHOR_ENABLE

#ifndef FUSION_ANCHOR_RANK
#define FUSION_ANCHOR_RANK 255  /* 255 = unassigned, auto-discovery determines rank */
#endif

/*============================================================================
 * FUSION_DOWNLINK_MODE — anchors TX downlink beacon (master) / blink (slave)
 *   Comment out to use original fusion protocol only.
 *============================================================================*/
#define FUSION_DOWNLINK_MODE

/* UWB Channel selection — 5 or 9.
 * Channel 5: 6489.6 MHz, Channel 9: 7987.2 MHz
 * Both use PRF 64M, preamble code 9, 6.8 Mbps data rate. */
#define UWB_CHANNEL 9

/* Downlink TDOA: anchor default positions (cm) — cross layout */
#if defined(FUSION_DOWNLINK_MODE)
#define DL_NUM_ANCHORS   4u  /* total anchors (master + slaves) */
#define DL_A0_X_CM       0
#define DL_A0_Y_CM       0
#define DL_A0_Z_CM       0
#define DL_A1_X_CM    2741
#define DL_A1_Y_CM       0
#define DL_A1_Z_CM       0
#define DL_A2_X_CM    2747
#define DL_A2_Y_CM   -2170
#define DL_A2_Z_CM       0
#define DL_A3_X_CM      -8
#define DL_A3_Y_CM   -2139
#define DL_A3_Z_CM       0
#define DL_A4_X_CM       0
#define DL_A4_Y_CM    -500
#define DL_A4_Z_CM     150
#define DL_A5_X_CM     500
#define DL_A5_Y_CM     500
#define DL_A5_Z_CM     150
#define DL_A6_X_CM    -500
#define DL_A6_Y_CM     500
#define DL_A6_Z_CM     150
#define DL_A7_X_CM     500
#define DL_A7_Y_CM    -500
#define DL_A7_Z_CM     150
#define DL_A8_X_CM    -500
#define DL_A8_Y_CM    -500
#define DL_A8_Z_CM     150
#define DL_A9_X_CM    1000
#define DL_A9_Y_CM       0
#define DL_A9_Z_CM     150
#endif

#ifdef __cplusplus
}
#endif

#endif
