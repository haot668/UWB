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
