/*! ----------------------------------------------------------------------------
 * @file    example_selection.h
 * @brief   Gateway project — always TEST_FUSION_ANCHOR (master rank + UART bridge)
 * ---------------------------------------------------------------------------*/
#ifndef TEST_SELECTION_
#define TEST_SELECTION_

#ifdef __cplusplus
extern "C" {
#endif

#define TEST_FUSION_ANCHOR

/* Auto-discovery: anchors negotiate master/slave roles at boot */
#define AUTO_ANCHOR_ENABLE

/* Default to master anchor (rank 0) — gateway always acts as master */
#define FUSION_ANCHOR_RANK 0

/*============================================================================
 * FUSION_DOWNLINK_MODE — gateway TX downlink beacon with anchor positions
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
