/*! ----------------------------------------------------------------------------
 * @file    firmware_version.h
 * @brief   Firmware version — single source of truth for all projects
 *
 *          Packed as 16-bit: [15:8]=major, [7:0]=minor
 *          V0.1  → 0x0001,  V0.2  → 0x0002
 *          V1.15 → 0x010F,  V2.0  → 0x0200
 * ---------------------------------------------------------------------------*/
#ifndef FIRMWARE_VERSION_H_
#define FIRMWARE_VERSION_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* === Edit these two macros to bump the version === */
/* NO 'u' suffix — used in stringification via VER_STR() */
#define FW_VERSION_MAJOR  2
#define FW_VERSION_MINOR  7

/* Packed version for embedding in frames (2 bytes) */
#define FW_VERSION_U16 \
    ((uint16_t)(((FW_VERSION_MAJOR) << 8) | (FW_VERSION_MINOR)))

/* Single-byte packed version for frame SEQ field (4+4 bit split).
 *   bits 7-4: major (0-15),  bits 3-0: minor (0-15)
 *   Example: V0.1→0x01, V1.2→0x12, V15.15→0xFF */
#define FW_VERSION_BYTE \
    ((uint8_t)((((FW_VERSION_MAJOR) & 0xF) << 4) | ((FW_VERSION_MINOR) & 0xF)))

/* Unpack a FW_VERSION_BYTE back to uint16_t for fw_version_str() */
static inline uint16_t fw_byte_to_u16(uint8_t b)
{
    return (uint16_t)(((b >> 4) & 0xF) << 8) | (uint16_t)(b & 0xF);
}

/* Unpack macros */
#define FW_VER_MAJOR(v)  ((uint8_t)(((v) >> 8) & 0xFF))
#define FW_VER_MINOR(v)  ((uint8_t)((v) & 0xFF))

/* Format version string into buffer (needs ≥8 chars).
 * Output examples: "V0.1", "V1.15", "V2.0" */
static inline void fw_version_str(char *buf, uint16_t ver)
{
    uint8_t maj = FW_VER_MAJOR(ver);
    uint8_t min = FW_VER_MINOR(ver);
    int pos = 0;
    buf[pos++] = 'V';
    /* major (0-255) */
    if (maj >= 100)      { buf[pos++] = '0' + (maj / 100); maj %= 100; }
    if (maj >= 10 || pos > 1) { buf[pos++] = '0' + (maj / 10); }
    buf[pos++] = '0' + (maj % 10);
    buf[pos++] = '.';
    /* minor (0-255) */
    if (min >= 100)      { buf[pos++] = '0' + (min / 100); min %= 100; }
    if (min >= 10)       { buf[pos++] = '0' + (min / 10); }
    buf[pos++] = '0' + (min % 10);
    buf[pos] = '\0';
}

#ifdef __cplusplus
}
#endif

#endif /* FIRMWARE_VERSION_H_ */
