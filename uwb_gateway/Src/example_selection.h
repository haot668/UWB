/*! ----------------------------------------------------------------------------
 * @file    example_selection.h
 * @brief   Gateway project — pure sniffer, no anchor role
 * ---------------------------------------------------------------------------*/
#ifndef TEST_SELECTION_
#define TEST_SELECTION_

#ifdef __cplusplus
extern "C" {
#endif

#define TEST_FUSION_ANCHOR

/* UWB Channel selection — 5 or 9 */
#define UWB_CHANNEL 5

/*============================================================================
 * FUSION_DOWNLINK_MODE — required for decoding beacons (0xF0) and blinks (0xF1)
 *============================================================================*/
#define FUSION_DOWNLINK_MODE

/* Downlink TDOA: anchor default positions (cm) */
#if defined(FUSION_DOWNLINK_MODE)
#define DL_NUM_ANCHORS   3u
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
#define DL_A3_Z_CM       0
#define DL_A4_X_CM       0
#define DL_A4_Y_CM    -500
#define DL_A4_Z_CM       0
#endif

#ifdef __cplusplus
}
#endif

#endif
