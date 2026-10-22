/*! ----------------------------------------------------------------------------
 * @file    gw_ota.h
 * @brief   Shared OTA receiver API
 * ---------------------------------------------------------------------------*/
#ifndef GW_OTA_H
#define GW_OTA_H

#include <stdint.h>

/* Init / reset OTA state */
void gw_ota_init(void);

/* Handle OTA prepare (PREP_CMD 0xF0) — erase staging, set total_size.
 * total_size: firmware image size in bytes from PREP_CMD payload (4B LE).
 * All subsequent OTA_DATA chunks must carry exactly 104 bytes of data. */
void gw_ota_handle_prepare(uint32_t total_size);

/* Check if OTA transfer is in progress */
int  gw_ota_is_active(void);

/* Poll for OTA timeout — call from main loop every cycle.
 * Returns 1 if OTA timed out (caller should switch to Ch9).
 * Returns 0 if no timeout / OTA not active. */
int  gw_ota_poll(void);

/* Force-reset OTA state (called on OTA_ABORT frame from Gateway).
 * Caller should also switch channel to Ch9 after this. */
void gw_ota_abort(void);

/* P2: Confirm successful boot — call once after app initialisation.
 * Erases flag page + staging area.  If not called (app crashes),
 * bootloader re-copies from staging and retries boot (up to 3×). */
void gw_ota_confirm_boot(void);

/* Handle one OTA_DATA frame (0xF6).  Sends OTA_ACK (0xF7) inline with
 * a 2ms settling delay before TX (DW3000 PLL stabilisation after flash write).
 * Returns 1 if frame was handled, 0 if not an OTA frame. */
int  gw_ota_handle_data(uint8_t *frame, uint16_t len, uint16_t src_rank);

/* Called when OTA transfer is complete — write flag + reboot.
 * Bootloader handles staging→app copy with verification on next boot. */
void gw_ota_finish(uint16_t src_rank);

#endif /* GW_OTA_H */
