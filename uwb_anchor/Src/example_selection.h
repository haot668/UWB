/*! ----------------------------------------------------------------------------
 * @file    example_selection.h
 * @brief   Anchor project — always TEST_FUSION_ANCHOR (slave rank)
 * ---------------------------------------------------------------------------*/
#ifndef TEST_SELECTION_
#define TEST_SELECTION_

#ifdef __cplusplus
extern "C" {
#endif

#define TEST_FUSION_ANCHOR

/* Auto-discovery: anchors negotiate master/slave roles at boot */
#define AUTO_ANCHOR_ENABLE

/* Default to slave anchor (rank >= 1) when AUTO_ANCHOR_ENABLE fails */
#define FUSION_ANCHOR_RANK 1

/*============================================================================
 * FUSION_DOWNLINK_MODE — anchors TX downlink beacon (master) / blink (slave)
 *   Comment out to use original fusion protocol only.
 *============================================================================*/
#define FUSION_DOWNLINK_MODE

/* Downlink TDOA: anchor default positions (cm) — cross layout, 5 anchors */
#if defined(FUSION_DOWNLINK_MODE)
#define DL_NUM_ANCHORS   5u
#define DL_A0_X_CM       0
#define DL_A0_Y_CM       0
#define DL_A0_Z_CM     150
#define DL_A1_X_CM     500
#define DL_A1_Y_CM       0
#define DL_A1_Z_CM     150
#define DL_A2_X_CM       0
#define DL_A2_Y_CM     500
#define DL_A2_Z_CM     150
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
