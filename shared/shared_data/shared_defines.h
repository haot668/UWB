/*! ----------------------------------------------------------------------------
 * @file    shared_defines.h
 * @brief   Shared definitions for UWB project
 *
 * @attention
 * Copyright 2013-2020 (c) Decawave Ltd, Dublin, Ireland.
 * All rights reserved.
 */

#ifndef _SHARE_DEF_
#define _SHARE_DEF_

#ifdef __cplusplus
extern "C" {
#endif

#ifdef STM32F429xx
#include <main.h>
#endif

#define SPEED_OF_LIGHT      (299702547)
#define FRAME_LEN_MAX       (127)
#define FRAME_LEN_MAX_EX    (1023)

#define RXFLEN_MASK     0x0000007FUL   /* Receive Frame Length (0 to 127) */
#define RXFL_MASK_1023  0x000003FFUL   /* Receive Frame Length Extension (0 to 1023) */

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor */
#define UUS_TO_DWT_TIME 63898

/* ── Distance / time unit conversions ──────────────────────────────────────
 * 1 DTU = 1 / (499.2 MHz x 128) = 15.649 ps.
 * Light needs 3.3365 ns to travel 1 m  ->  1 m = 213.20 DTU, 1 cm = 2.1320 DTU.
 *
 * The figure "2131 DTU per cm" that appeared in the TWR code was wrong by
 * 1000x: it confused the NANOSECOND flight time of 1 m (3.34 ns) with a
 * microsecond one.  Everything that converts between cm and DTU must use the
 * constants below — never a bare number.
 *
 *   1 m   = 213.20 DTU        5 ms slot = 319,490,000 DTU
 *   1 cm  =   2.1320 DTU       40-bit wrap = 17.21 s
 *   20 cm =  42.64 DTU         80 ms cycle = 5,111,840,000 DTU
 *---------------------------------------------------------------------------*/
#define DTU_PER_CM_F     2.1320f      /* float: DTU per centimetre          */
#define DTU_PER_M_F      213.20f      /* float: DTU per metre               */
#define CM_PER_DTU_F     0.46907f     /* float: centimetres per DTU         */

/* Integer variants (x1000), for the places that must not use floating point. */
#define DTU_PER_CM_MILLI 2132
#define CM_PER_DTU_MILLI 469

/* DTU -> cm, rounded.  dtu may be negative (differences). */
#define DTU_TO_CM(dtu)   (((int32_t)(dtu) * CM_PER_DTU_MILLI + 500) / 1000)
/* cm -> DTU, rounded. */
#define CM_TO_DTU(cm)    (((int32_t)(cm) * DTU_PER_CM_MILLI + 500) / 1000)

typedef enum
{
    AES_RES_OK = 0,
    AES_RES_ERROR_LENGTH = -1,
    AES_RES_ERROR = -2,
    AES_RES_ERROR_FRAME = -3,
    AES_RES_ERROR_IGNORE_FRAME = -4
} aes_results_e;

#ifdef __cplusplus
}
#endif

#endif
