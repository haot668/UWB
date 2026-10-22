/*! ----------------------------------------------------------------------------
 * @file    example_selection.h
 * @brief   Example selection is configured here
 *
 * @attention
 *
 * Copyright 2013 - 2020(c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */

#ifndef TEST_SELECTION_
#define TEST_SELECTION_

#ifdef __cplusplus
extern "C" {
#endif

//Enable the needed example/test. Please enable only one example/test!

//#define TEST_READING_DEV_ID
//#define TEST_SIMPLE_TX
//#define TEST_SIMPLE_TX_PDOA
//#define TEST_TX_SLEEP
//#define TEST_TX_SLEEP_IDLE_RC
//#define TEST_TX_SLEEP_AUTO
//#define TEST_TX_SLEEP_TIMED
//#define TEST_TX_WITH_CCA

//#define TEST_SIMPLE_RX
//#define TEST_RX_DIAG
//#define TEST_RX_SNIFF
//#define TEST_RX_TRIM
//#define TEST_SIMPLE_RX_PDOA

//#define TEST_SIMPLE_TX_STS_SDC
//#define TEST_SIMPLE_RX_STS_SDC

//#define TEST_SIMPLE_TX_AES
//#define TEST_SIMPLE_RX_AES

//#define TEST_TX_WAIT_RESP
//#define TEST_TX_WAIT_RESP_INT
//#define TEST_RX_SEND_RESP

//#define TEST_CONTINUOUS_WAVE
//#define TEST_CONTINUOUS_FRAME

//#define TEST_DS_TWR_INITIATOR_STS
//#define TEST_DS_TWR_RESPONDER_STS

//#define TEST_DS_TWR_INITIATOR
//#define TEST_DS_TWR_RESPONDER

//#define TEST_DS_TWR_STS_SDC_INITIATOR
//#define TEST_DS_TWR_STS_SDC_RESPONDER

//#define TEST_SS_TWR_INITIATOR
//#define TEST_SS_TWR_RESPONDER
//#define TEST_SIMPLE_TX
//#define TEST_SIMPLE_RX
//#define TEST_SS_TWR_INITIATOR

//#define TEST_SS_TWR_INITIATOR_STS
//#define TEST_SS_TWR_RESPONDER_STS

//#define TEST_SS_TWR_INITIATOR_STS_NO_DATA
//#define TEST_SS_TWR_RESPONDER_STS_NO_DATA

//#define TEST_AES_SS_TWR_INITIATOR
//#define TEST_AES_SS_TWR_RESPONDER

//#define TEST_ACK_DATA_TX
//#define TEST_ACK_DATA_RX

//#define TEST_ACK_DATA_RX_DBL_BUFF

//#define TEST_SPI_CRC
//#define TEST_GPIO

//#define TEST_OTP_WRITE

//#define TEST_LE_PEND_TX
//#define TEST_LE_PEND_RX

//#define TEST_FRAME_FILTERING_TX
//#define TEST_FRAME_FILTERING_RX

//#define TEST_TDOA_TAG
//#define TEST_TDOA_ANCHOR

//#define TEST_FUSION_TAG
//#define TEST_FUSION_ANCHOR

/* DOWNlink TDOA: anchors transmit, tag receives + solves position locally.
 * Enable ONE of the two below: */
//#define TEST_DOWNLINK_TAG
//#define TEST_DOWNLINK_ANCHOR

/* Auto-discovery: anchors negotiate master/slave roles at boot.
 *   Disable (comment out) to use static FUSION_ANCHOR_RANK below. */
#define AUTO_ANCHOR_ENABLE

/* Static anchor rank (ignored when AUTO_ANCHOR_ENABLE is active):
 *   0 = Main Anchor (full protocol + TDMA collection),
 *   1+ = Slave Anchor (RX-only + TDMA report) */
#define FUSION_ANCHOR_RANK 1

/*============================================================================
 * DOWNlink anchor position configuration (compile-time).
 * Master auto-discovers via UID; each anchor needs its own (x,y,z) in cm.
 * When TEST_DOWNLINK_ANCHOR is active, the MASTER uses ALL defines to build
 * the beacon anchor table. When TEST_DOWNLINK_TAG is active, these defines
 * are NOT needed — tag learns positions from the beacon.
 *============================================================================*/
#if defined(TEST_DOWNLINK_ANCHOR) || defined(TEST_DOWNLINK_TAG)

#define DL_NUM_ANCHORS   5u

/* Anchor 0 — Master (also serves as TDOA reference) */
#define DL_A0_X_CM       0
#define DL_A0_Y_CM       0
#define DL_A0_Z_CM       150

/* Anchor 1 — Slave */
#define DL_A1_X_CM       500
#define DL_A1_Y_CM       0
#define DL_A1_Z_CM       150

/* Anchor 2 — Slave */
#define DL_A2_X_CM       0
#define DL_A2_Y_CM       500
#define DL_A2_Z_CM       150

/* Anchor 3 — Slave */
#define DL_A3_X_CM      -500
#define DL_A3_Y_CM       0
#define DL_A3_Z_CM       150

/* Anchor 4 — Slave */
#define DL_A4_X_CM       0
#define DL_A4_Y_CM      -500
#define DL_A4_Z_CM       150

#endif /* TEST_DOWNLINK_ANCHOR || TEST_DOWNLINK_TAG */

#ifdef __cplusplus
}
#endif


#endif
