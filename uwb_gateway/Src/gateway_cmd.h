/*! ----------------------------------------------------------------------------
 * @file    gateway_cmd.h
 * @brief   Gateway AT command handler — UART RX + command dispatch
 * ---------------------------------------------------------------------------*/
#ifndef GATEWAY_CMD_H
#define GATEWAY_CMD_H

#include <stdint.h>
#include <gateway_protocol.h>

/* Feed one received byte into the RX ring buffer.
 * Called from HAL_UART_RxCpltCallback in main.c */
void gateway_uart_rx_byte(uint8_t byte);

/* Return number of pending unread bytes in ring buffer (diagnostic) */
int gateway_uart_rx_pending(void);

/* Check if a complete AT command line is available (non-blocking).
 * Returns 1 and copies the null-terminated line into buf on success. */
int uart_rx_line_available(char *buf, int maxlen);

/* Non-blocking poll: check for pending AT commands and handle them.
 * Called from main anchor loop Phase 7. */
void gateway_poll_uart(void);

/* Handle a received UWB frame addressed to the gateway.
 * Called from Phase 1 of main_anchor_loop for func codes 0xF3/0xF5/0xF7. */
void gateway_handle_uwb_rx(uint8_t *frame, uint16_t len);

/* Register a device (called when an anchor/tag is discovered) */
void gateway_device_seen(const char *uid, const char *type, uint16_t rank);

/* Set firmware version for a device */
void gateway_device_version(const char *uid, uint16_t fw_ver);

/* P1-4: UID fingerprint of a device — detects two different anchors that
 * share the same 16-bit rank (reported as +RANKCOLL). */
void gateway_device_uid_fp(const char *uid, uint64_t fp);

/* Set master UID (called from fusion_anchor after discovery) */
void gateway_set_master_uid(const char *uid);

/* Expose known node count to anchor */
void gateway_set_node_count(uint8_t count);

/* Flash config storage — anchor coordinates */
int  gateway_flash_save_coord(int16_t x, int16_t y, int16_t z);
int  gateway_flash_load_coord(int16_t *x, int16_t *y, int16_t *z);

/* Send a UWB data frame with master as SRC (0x0001).
 * Uses immediate TX; caller must handle radio state. */
void gateway_send_uwb_ack(uint16_t dst_addr, uint8_t func,
                          uint8_t *payload, uint8_t payload_len);

/* Send a UWB data frame with custom SRC address.
 * Used by slave anchors to respond to gateway commands. */
void gateway_send_uwb_ack_ex(uint16_t dst_addr, uint16_t src_addr, uint8_t func,
                              uint8_t *payload, uint8_t payload_len);

/* Gateway mode flag: 0=config(IDLE/TX), 1=TDOA(RX sniff) */
extern int gw_tdoa_active;

/* Check and expire offline devices (called from sniffer loop) */
void gateway_check_timeout(void);

/* OTA SW-ARQ: poll ACK timeout (called every 1ms from sniffer loop) */
void gateway_ota_arq_poll(void);

/* OTA SW-ARQ: handle received ACK frame (called from gateway_handle_uwb_rx).
 * Returns 1 if ACK was processed by ARQ (first copy), 0 if duplicate/stale. */
int gateway_ota_arq_on_ack(uint16_t seq, uint8_t status);

/* Broadcast OTA: poll multi-ACK collection (called every 1ms from sniffer loop). */
void gateway_bcast_ota_poll(void);

/* Format 16-bit value as 4-digit hex string (from fusion_anchor.c) */
extern char *fmt_uid4(char *buf, uint16_t val);

#endif /* GATEWAY_CMD_H */
