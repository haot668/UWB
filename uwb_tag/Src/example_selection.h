/*! ----------------------------------------------------------------------------
 * @file    example_selection.h
 * @brief   Tag project — always TEST_FUSION_TAG
 * ---------------------------------------------------------------------------*/
#ifndef TEST_SELECTION_
#define TEST_SELECTION_

#ifdef __cplusplus
extern "C" {
#endif

#define TEST_FUSION_TAG

/* UWB Channel selection — 5 or 9 */
#define UWB_CHANNEL 5

/*============================================================================
 * FUSION_DOWNLINK_MODE — tag receives anchor beacons + blinks, solves TDOA
 *   Comment out to revert to original fusion tag (TX Blink/Poll/Final).
 *============================================================================*/
#define FUSION_DOWNLINK_MODE

/* Downlink TDOA: anchor default positions (cm) — cross layout, 5 anchors */
#if defined(FUSION_DOWNLINK_MODE)
#define DL_NUM_ANCHORS   3u  /* 1 master + 2 slaves */
#define DL_A0_X_CM       0
#define DL_A0_Y_CM       0
#define DL_A0_Z_CM       0
#define DL_A1_X_CM     300
#define DL_A1_Y_CM       0
#define DL_A1_Z_CM       0
#define DL_A2_X_CM       0
#define DL_A2_Y_CM     300
#define DL_A2_Z_CM       0
#define DL_A3_X_CM    -500
#define DL_A3_Y_CM       0
#define DL_A3_Z_CM     150
#define DL_A4_X_CM       0
#define DL_A4_Y_CM    -500
#define DL_A4_Z_CM     150
#endif

#ifdef __cplusplus
}
#endif

#endif
