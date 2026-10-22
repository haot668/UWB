#ifndef _SHARE_FUNC_
#define _SHARE_FUNC_

#ifdef __cplusplus
extern "C" {
#endif

/* DW3000 40-bit timestamp read (TX / RX / SYS_TIME) */
uint64_t get_tx_timestamp_u64(void);
uint64_t get_rx_timestamp_u64(void);
uint64_t get_sys_timestamp_u64(void);

/* 40-bit timestamp pack/unpack (DW3000 timestamps are 40 bits) */
void     pack_40bit_ts(uint8_t *buf, uint64_t ts);
uint64_t unpack_40bit_ts(const uint8_t *buf);

/* 64-bit extended timestamp pack/unpack (for cross-anchor timestamp comparison) */
void     pack_64bit_ts(uint8_t *buf, uint64_t ts);
uint64_t unpack_64bit_ts(const uint8_t *buf);

#ifdef __cplusplus
}
#endif

#endif
