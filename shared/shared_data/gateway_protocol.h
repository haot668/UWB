/*! ----------------------------------------------------------------------------
 * @file    gateway_protocol.h
 * @brief   Gateway UWB function codes and shared protocol defines
 * ---------------------------------------------------------------------------*/
#ifndef GATEWAY_PROTOCOL_H
#define GATEWAY_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Gateway UWB function codes — PC command forwarding */
#define FUNC_PREP_CMD    0xF0u  /* GW→Dev: unified prepare (enter cmd mode)    */
#define FUNC_PREP_ACK    0xF1u  /* Dev→GW: ACK to prepare, in cmd mode now     */
#define FUNC_FIND_REQ     0xF2u  /* GW→Dev: 1B on/off */
#define FUNC_FIND_ACK     0xF3u  /* Dev→GW: 1B on/off */
#define FUNC_COORD_WRITE  0xF4u  /* GW→Dev: x(2B)+y(2B)+z(2B) */
#define FUNC_COORD_ACK    0xF5u  /* Dev→GW: x(2B)+y(2B)+z(2B) */
#define FUNC_OTA_DATA     0xF6u  /* GW→Dev: seq(1B)+data[248B]+crc16(2B) */
#define FUNC_OTA_ACK      0xF7u  /* Dev→GW: seq(1B)+status(1B) */
#define FUNC_OTA_END      0xE4u  /* GW→Dev: OTA complete, reboot now       */
#define FUNC_OTA_PREPARE  0xE5u  /* GW→Dev: prepare OTA, switch to Ch5    */
#define FUNC_OTA_PREP_ACK 0xE6u  /* Dev→GW: ACK OTA prepare, switching now */
#define FUNC_OTA_ABORT    0xE6u  /* GW→Dev: abort OTA, switch back to Ch9   */
#define FUNC_START_TDOA   0xF8u  /* GW→Master: exit config mode, start TDMA */
#define FUNC_NETWORK_CFG  0xF9u  /* GW→Master: 2B bitmask of active anchors */
#define FUNC_STOP_TDOA    0xFAu  /* GW→Master: stop TDMA, re-enter config mode */
#define FUNC_START_ACK    0xFBu  /* Master→GW: START received */
#define FUNC_NETWORK_ACK  0xFCu  /* Master→GW: NETWORK received */
#define FUNC_LIST_REQ     0xFDu  /* GW→Master: request online device list */
#define FUNC_STOP_ACK     0xFEu  /* Master→GW: STOP received */

/* Gateway UWB addressing */
#define GW_MASTER_ADDR    0x8001u  /* unique addr — avoids collision with slave rank (0x0001) */
#define GW_BCAST_ADDR     0xFFFFu

/* Frame header size: FC[2]+SEQ[1]+PAN[2]+DST[2]+SRC[2]+FUNC[1] */
#define GW_HDR_LEN        10u

/* Max AT command line length — must fit AT+OTA_DATA with 248B hex payload (~526B worst) */
#define GW_CMD_MAX_LEN    600

/* RX ring buffer size — must hold at least one full max-length command line */
#define GW_RX_BUF_SIZE    1024

#ifdef __cplusplus
}
#endif

#endif /* GATEWAY_PROTOCOL_H */
