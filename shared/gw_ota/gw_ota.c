/*! ----------------------------------------------------------------------------
 * @file    gw_ota.c
 * @brief   Shared OTA receiver — handles OTA_DATA frames on anchor/tag devices
 *
 *          Flow:
 *            1. Gateway forwards AT+OTA_START/DATA/END via UWB (0xF6)
 *            2. Device receives OTA_DATA → validate CRC → write to flash staging
 *            3. Device sends OTA_ACK (0xF7) after each chunk
 *            4. On OTA_END → write OTA flag → reboot → bootloader copies staging→app
 *
 *          L1 rollback: 5s timeout → resume ranging, original firmware intact
 *          L3 rollback: bootloader validates staging before erasing old app
 *
 *          Flash layout (STM32L412KB, 128KB, pages 2KB×64):
 *            0x08000000  Bootloader         (pages 0-3,   8 KB)
 *            0x08002000  Application        (pages 4-31, 56 KB)
 *            0x08010000  OTA staging        (pages 32-61, 60 KB)
 *            0x0801F000  OTA flag page      (page 62,     2 KB)
 *            0x0801F800  Config             (page 63,     2 KB)
 * ---------------------------------------------------------------------------*/
#include <string.h>
#include <deca_device_api.h>
#include <deca_regs.h>
#include <port.h>
#include <main.h>
#include <stm32l4xx_hal.h>
#include <gateway_protocol.h>
#include <shared_defines.h>

/*============================================================================
 * Flash staging area
 *============================================================================*/
#define OTA_FLASH_BASE    0x08010000UL   /* Staging area (64KB offset)            */
#define OTA_FLASH_END     0x0801F000UL   /* End before flag page                  */
#define OTA_PAGE_SIZE     2048u           /* STM32L4 page size (2 KB)             */
#define OTA_MAX_SIZE      (OTA_FLASH_END - OTA_FLASH_BASE)  /* ~60KB available   */

/*============================================================================
 * OTA flag page (page 62 @ 0x0801F000, 2 KB)
 *
 *   Offset 0:  MAGIC  (4B) — 0x4F544152 = "OTAR" (OTA Ready)
 *   Offset 4:  STATE  (4B) — 0=IDLE, 1=READY, 2=COMMIT
 *   Offset 8:  SIZE   (4B) — firmware size in bytes
 *   Offset 12: RETRY  (4B) — copy retry counter (bootloader-managed)
 *   Offset 16: CRC32  (4B) — CRC-32/MPEG-2 of complete firmware image
 *
 * P0-1: gw_ota_finish() writes MAGIC+STATE+SIZE+CRC32 then reboots.
 * Bootloader verifies CRC32 before erasing old app — eliminates the
 * brick-risk window that existed when gw_ota_finish() directly erased
 * app flash from runtime code.
 *============================================================================*/
#define OTA_FLAG_ADDR       0x0801F000UL
#define OTA_FLAG_MAGIC      0x4F544152UL   /* "OTAR" */
#define OTA_STATE_IDLE      0x00000000UL  /* flag page: no OTA pending */
#define OTA_STATE_READY     0x00000001UL  /* flag page: staging has valid fw */
#define OTA_STATE_COMMIT    0x00000002UL  /* flag page: reserved */
#define OTA_STATE_BOOTING   0x00000003UL  /* P2: firmware copied, waiting for app to confirm boot */
#define OTA_FLAG_MAX_RETRIES 3u
#define OTA_FLAG_OFF_MAGIC  0u
#define OTA_FLAG_OFF_STATE  4u
#define OTA_FLAG_OFF_SIZE   8u
#define OTA_FLAG_OFF_RETRY  12u
#define OTA_FLAG_OFF_CRC32  16u
/* P1-1: Breakpoint resume — saved on timeout, checked on seq 0 */
#define OTA_FLAG_OFF_RESUME_MAGIC  20u
#define OTA_FLAG_OFF_RESUME_TOTAL  24u
#define OTA_FLAG_OFF_RESUME_BYTES  28u
#define OTA_RESUME_MAGIC  0x5253554DUL   /* "RSUM" */

/*============================================================================
 * OTA timeout — if no data for this long, abort and return to Ch9
 *============================================================================*/
#define OTA_TIMEOUT_MS    15000u  /* must exceed Gateway ARQ max (300ms×15=4500ms) */
#define OTA_FINISH_DELAY_MS 3000u /* wait for OTA_END before auto-finish (lets GW complete protocol) */

/*============================================================================
 * OTA state machine — explicit enum (P0-2)
 *
 *   IDLE       → no OTA in progress
 *   RECEIVING  → active transfer (seq 0 received, staging erased)
 *   ALL_RCVD   → all bytes received, waiting for OTA_END or auto-finish
 *
 * Replaces the old implicit ota_active + ota_all_received flags that
 * caused the auto-finish bug (ota_all_received cleared before
 * gw_ota_finish call = state inconsistency).
 *============================================================================*/
typedef enum {
    OTA_S_IDLE = 0,
    OTA_STATE_RECEIVING,
    OTA_STATE_ALL_RCVD,
    OTA_STATE_REBOOT_PENDING
} ota_state_t;

#define OTA_REBOOT_DELAY_MS  2000u  /* wait 2s after flag write before SCB_AIRCR */

static ota_state_t ota_state         = OTA_S_IDLE;
static uint32_t    ota_bytes_written  = 0;
static uint32_t    ota_total_size     = 0;  /* expected firmware size from seq 0 header */
static uint16_t    ota_last_seq      = 0xFFFF;
static uint32_t    ota_all_rcv_tick   = 0;  /* when ALL_RCVD was entered */
static uint32_t    ota_last_data_tick = 0;  /* for timeout detection */
static uint32_t    ota_resume_offset  = 0;  /* P1-1: bytes already in staging from prior session */
static uint32_t    ota_reboot_tick    = 0;  /* when REBOOT_PENDING was entered */
extern void test_run_info(unsigned char *data);

/* Debug output — set to 0 to silence OTA diagnostic prints */
#define DEBUG_ENABLE  1
#if DEBUG_ENABLE
#define DBG_PRINT(msg) test_run_info((unsigned char *)(msg))
#else
#define DBG_PRINT(msg) do {} while(0)
#endif

/*============================================================================
 * Flash helpers — use HAL flash driver
 *============================================================================*/

/* Feed the independent watchdog during long flash operations.
 * Flash page erase can take ~22ms per page; 30 pages = 660ms.
 * Use direct register access — HAL_IWDG_Refresh requires the hiwdg handle
 * which isn't available in this shared module. */
#define OTA_IWDG_FEED()  do { IWDG->KR = 0xAAAA; } while(0)

/* Erase flash pages from start_addr to end_addr (inclusive page boundaries).
 * Uses direct register access (same method as bootloader) — bypasses
 * HAL_FLASHEx_Erase which can fail due to tick/timeout interaction.
 * Caller MUST call port_DisableEXT_IRQ() before invoking. */
static int ota_flash_erase(uint32_t start_addr, uint32_t end_addr)
{
    uint32_t first_page, last_page, page;

    OTA_IWDG_FEED();  /* feed dog before long erase (up to ~660ms for 30 pages) */

    /* Unlock flash (same sequence as bootloader) */
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = 0x45670123UL;
        FLASH->KEYR = 0xCDEF89ABUL;
    }

    first_page = (start_addr - FLASH_BASE) / OTA_PAGE_SIZE;
    last_page  = (end_addr   - FLASH_BASE) / OTA_PAGE_SIZE;

    for (page = first_page; page <= last_page; page++) {
        uint32_t cr;

        /* Check for errors from previous operation */
        if (FLASH->SR & (FLASH_SR_PGSERR | FLASH_SR_SIZERR |
                         FLASH_SR_PGAERR | FLASH_SR_WRPERR)) {
            FLASH->SR = (FLASH_SR_PGSERR | FLASH_SR_SIZERR |
                         FLASH_SR_PGAERR | FLASH_SR_WRPERR);
        }

        /* Page erase */
        cr  = FLASH_CR_PER;
        cr |= (page << 3) & 0x3F8UL;   /* PNB[7:0] in CR[10:3] */
        FLASH->CR = cr;
        FLASH->CR = cr | FLASH_CR_STRT;

        /* Wait for erase to complete */
        while (FLASH->SR & FLASH_SR_BSY) {
            OTA_IWDG_FEED();  /* feed dog during long erase */
        }

        FLASH->CR &= ~FLASH_CR_PER;

        /* Check for errors after erase */
        if (FLASH->SR & (FLASH_SR_PGSERR | FLASH_SR_SIZERR |
                         FLASH_SR_PGAERR | FLASH_SR_WRPERR)) {
            FLASH->CR = FLASH_CR_LOCK;  /* lock flash before returning error */
            return -1;
        }
    }

    FLASH->CR = FLASH_CR_LOCK;  /* lock flash */
    return 0;
}

/* Erase a single flash page */
static int ota_flash_erase_page(uint32_t page_addr)
{
    return ota_flash_erase(page_addr, page_addr);
}

/* Write one 8-byte double-word to flash (STM32L4 requires 64-bit write).
 * Uses direct register access — same as bootloader.  Verified reliable.
 * Caller MUST call port_DisableEXT_IRQ() before invoking.
 * Retries up to 5 times — transient flash busy can occur under EMI. */
static int ota_flash_write_dword(uint32_t addr, uint64_t data)
{
    int retry;
    for (retry = 0; retry < 5; retry++) {
        OTA_IWDG_FEED();

        /* Unlock flash if locked */
        if (FLASH->CR & FLASH_CR_LOCK) {
            FLASH->KEYR = 0x45670123UL;
            FLASH->KEYR = 0xCDEF89ABUL;
        }

        /* Clear error flags */
        if (FLASH->SR & (FLASH_SR_PGSERR | FLASH_SR_SIZERR |
                         FLASH_SR_PGAERR | FLASH_SR_WRPERR)) {
            FLASH->SR = (FLASH_SR_PGSERR | FLASH_SR_SIZERR |
                         FLASH_SR_PGAERR | FLASH_SR_WRPERR);
        }

        /* Program double-word */
        FLASH->CR |= FLASH_CR_PG;
        *(volatile uint32_t *)(addr)      = (uint32_t)(data & 0xFFFFFFFFUL);
        *(volatile uint32_t *)(addr + 4)  = (uint32_t)(data >> 32);
        while (FLASH->SR & FLASH_SR_BSY) { }
        FLASH->CR &= ~FLASH_CR_PG;

        /* Read-back verify — simple direct read.  The ART cache is cold
         * (just invalidated by the flash write), so this read misses cache
         * and hits physical cells.  No DCEN/ICEN toggle needed here —
         * the bootloader handles full verification with cache disabled. */
        if (*(volatile uint32_t *)(addr)     == (uint32_t)(data & 0xFFFFFFFFUL) &&
            *(volatile uint32_t *)(addr + 4) == (uint32_t)(data >> 32)) {
            FLASH->CR = FLASH_CR_LOCK;
            return 0;
        }

        /* Check for errors */
        if (FLASH->SR & (FLASH_SR_PGSERR | FLASH_SR_SIZERR |
                         FLASH_SR_PGAERR | FLASH_SR_WRPERR)) {
            FLASH->SR = (FLASH_SR_PGSERR | FLASH_SR_SIZERR |
                         FLASH_SR_PGAERR | FLASH_SR_WRPERR);
        }
    }
    FLASH->CR = FLASH_CR_LOCK;  /* lock flash */
    return -1;
}

/* Write a buffer to flash (must be 8-byte aligned address and size).
 * Retries once on HAL failure. */
static int ota_flash_write(uint32_t addr, const uint8_t *data, uint32_t len)
{
    uint32_t i;
    for (i = 0; i < len; i += 8) {
        uint64_t dw = 0;
        int b;
        for (b = 0; b < 8 && (i + b) < len; b++) {
            dw |= ((uint64_t)data[i + b]) << (b * 8);
        }
        if (ota_flash_write_dword(addr + i, dw) != 0) {
            /* Retry once on HAL failure */
            if (ota_flash_write_dword(addr + i, dw) != 0) {
                return -1;
            }
        }
    }
    return 0;
}

/* Write one 64-bit value to the flag page (no locking — caller handles it).
 * Flag page is page 62 (0x0801F000). Each call writes at offset 0/8/16/24.
 * Uses direct register access for reliability. */
static int ota_flag_write_dword(uint32_t offset, uint64_t data)
{
    int retry;
    uint32_t addr = OTA_FLAG_ADDR + offset;
    for (retry = 0; retry < 3; retry++) {
        OTA_IWDG_FEED();

        if (FLASH->CR & FLASH_CR_LOCK) {
            FLASH->KEYR = 0x45670123UL;
            FLASH->KEYR = 0xCDEF89ABUL;
        }

        if (FLASH->SR & (FLASH_SR_PGSERR | FLASH_SR_SIZERR |
                         FLASH_SR_PGAERR | FLASH_SR_WRPERR)) {
            FLASH->SR = (FLASH_SR_PGSERR | FLASH_SR_SIZERR |
                         FLASH_SR_PGAERR | FLASH_SR_WRPERR);
        }

        FLASH->CR |= FLASH_CR_PG;
        *(volatile uint32_t *)(addr)      = (uint32_t)(data & 0xFFFFFFFFUL);
        *(volatile uint32_t *)(addr + 4)  = (uint32_t)(data >> 32);
        while (FLASH->SR & FLASH_SR_BSY) { }
        FLASH->CR &= ~FLASH_CR_PG;

        /* Read-back verify — simple direct read (cache cold after write) */
        if (*(volatile uint32_t *)(addr)     == (uint32_t)(data & 0xFFFFFFFFUL) &&
            *(volatile uint32_t *)(addr + 4) == (uint32_t)(data >> 32)) {
            FLASH->CR = FLASH_CR_LOCK;
            return 0;
        }

        if (FLASH->SR & (FLASH_SR_PGSERR | FLASH_SR_SIZERR |
                         FLASH_SR_PGAERR | FLASH_SR_WRPERR)) {
            FLASH->SR = (FLASH_SR_PGSERR | FLASH_SR_SIZERR |
                         FLASH_SR_PGAERR | FLASH_SR_WRPERR);
        }
    }
    FLASH->CR = FLASH_CR_LOCK;
    return -1;
}

/*============================================================================
 * RAM-resident staging→app copy — DEPRECATED by P0-1 flag-chain architecture.
 * Was called from gw_ota_finish() to erase app flash and copy staging→app
 * from runtime code (risk: power loss during erase = brick).  P0-1 replaced
 * this with gw_ota_finish() → write OTA flag → reboot → Bootloader does safe copy.
 * Kept as reference; excluded from build to save ~600 bytes flash space.
 *============================================================================*/
#if 0
#define APP_FLASH_BASE  0x08000000UL
#define APP_FLASH_SIZE  0x00010000UL   /* 64 KB app area */

__attribute__((section(".ramfunc"), long_call))
void ota_copy_staging_to_app(uint32_t copy_size)
{
    uint32_t app_end, page, num_pages, off;
    volatile uint32_t *src, *dst;
    uint32_t err_count = 0;

    __disable_irq();

    /*--- Validate staging area: check SP in SRAM, PC in flash range ---*/
    {
        uint32_t sp = *(volatile uint32_t *)OTA_FLASH_BASE;
        uint32_t pc = *(volatile uint32_t *)(OTA_FLASH_BASE + 4);
        if (sp < 0x20000000UL || sp > 0x20008000UL ||
            pc < APP_FLASH_BASE || pc > (APP_FLASH_BASE + APP_FLASH_SIZE)) {
            /* Invalid firmware in staging — abort, old app still intact */
            __enable_irq();
            return;
        }
    }

    /*--- Erase application pages ---*/
    app_end   = APP_FLASH_BASE + copy_size;
    num_pages = ((app_end - APP_FLASH_BASE) + OTA_PAGE_SIZE - 1u) / OTA_PAGE_SIZE;

    /* Unlock flash */
    while (FLASH->SR & FLASH_SR_BSY) { }
    FLASH->KEYR = 0x45670123UL;
    FLASH->KEYR = 0xCDEF89ABUL;

    for (page = 0; page < num_pages; page++) {
        FLASH->CR &= ~FLASH_CR_PG;   /* ensure PG is clear before PER */
        FLASH->CR |= FLASH_CR_PER;
        FLASH->CR = (FLASH->CR & ~(0xFFUL << 3)) | ((page & 0xFF) << 3);  /* PNB[7:0] in CR[10:3] */
        FLASH->CR |= FLASH_CR_STRT;
        while (FLASH->SR & FLASH_SR_BSY) { }
        FLASH->CR &= ~FLASH_CR_PER;
        /* Each page erase takes ~22ms — feed IWDG to prevent watchdog reset
         * during the ~660ms erase phase (28 pages × 22ms). */
        IWDG->KR = 0xAAAA;
        if (FLASH->SR & (FLASH_SR_PGSERR | FLASH_SR_SIZERR |
                          FLASH_SR_PGAERR | FLASH_SR_WRPERR)) {
            FLASH->SR = (FLASH_SR_PGSERR | FLASH_SR_SIZERR |
                          FLASH_SR_PGAERR | FLASH_SR_WRPERR);
            err_count++;
        }
    }

    /*--- Copy staging → app (double-word with read-back verify) ---*/
    src = (volatile uint32_t *)OTA_FLASH_BASE;
    dst = (volatile uint32_t *)APP_FLASH_BASE;

    for (off = 0; off < copy_size; off += 8) {
        uint32_t w0 = src[off / 4];
        uint32_t w1 = src[off / 4 + 1];

        /* Program double-word */
        FLASH->CR |= FLASH_CR_PG;
        dst[off / 4]     = w0;
        dst[off / 4 + 1] = w1;
        while (FLASH->SR & FLASH_SR_BSY) { }
        FLASH->CR &= ~FLASH_CR_PG;

        /* Verify */
        if (dst[off / 4] != w0 || dst[off / 4 + 1] != w1) {
            err_count++;
        }

        if (FLASH->SR & (FLASH_SR_PGSERR | FLASH_SR_SIZERR |
                          FLASH_SR_PGAERR | FLASH_SR_WRPERR)) {
            FLASH->SR = (FLASH_SR_PGSERR | FLASH_SR_SIZERR |
                          FLASH_SR_PGAERR | FLASH_SR_WRPERR);
            err_count++;
        }

        /* Feed IWDG every 64 double-words (~512 bytes).
         * Each dword write is ~90µs; 64 writes ≈ 5.8ms, well under
         * typical IWDG timeout.  For 48KB firmware: ~96 feed points. */
        if ((off & 0x1FFu) == 0) {
            IWDG->KR = 0xAAAA;
        }
    }

    FLASH->CR &= ~FLASH_CR_PG;
    FLASH->CR |= FLASH_CR_LOCK;

    if (err_count) {
        /* SOS blink: LED fast forever */
        __enable_irq();
        while (1) {
            HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
            { volatile uint32_t d; for (d = 0; d < 2000000; d++) { __NOP(); } }
        }
    }

    /* Success — reboot into new firmware */
    __enable_irq();  /* not strictly needed before reset, but clean */
    NVIC_SystemReset();
    while (1) { }
}
#endif /* 0 — deprecated ota_copy_staging_to_app */

/*============================================================================
 * CRC-16/CCITT (same as gateway)
 *============================================================================*/
static uint16_t ota_crc16(const uint8_t *data, int len)
{
    uint16_t crc = 0xFFFF;
    int i, j;
    for (i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (j = 0; j < 8; j++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

/*============================================================================
 * CRC-32/MPEG-2 — whole-firmware integrity verification
 * Polynomial: 0x04C11DB7, initial value: 0xFFFFFFFF
 *============================================================================*/
static uint32_t ota_crc32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFF;
    uint32_t i;
    for (i = 0; i < len; i++) {
        int j;
        crc ^= (uint32_t)data[i] << 24;
        for (j = 0; j < 8; j++) {
            if (crc & 0x80000000)
                crc = (crc << 1) ^ 0x04C11DB7;
            else
                crc = (crc << 1);
        }
    }
    return crc;
}

/*============================================================================
 * Write OTA flag page — signals bootloader that staging has valid firmware.
 * Erases flag page, then writes MAGIC+STATE+SIZE+CRC32 in 3 double-words.
 * After this, reboot — the bootloader handles staging→app copy safely. */
static int ota_flag_write(uint32_t fw_size, uint32_t fw_crc)
{
    int ret;

    ret = ota_flash_erase_page(OTA_FLAG_ADDR);
    if (ret != 0) return -1;

    /* Write MAGIC (offset 0) + STATE=READY (offset 4) as one double-word */
    {
        uint64_t dw = ((uint64_t)OTA_STATE_READY << 32) | (uint64_t)OTA_FLAG_MAGIC;
        if (ota_flag_write_dword(0, dw) != 0) return -2;
    }

    /* Write SIZE (offset 8) + RETRY=0 (offset 12) */
    {
        uint64_t dw = (uint64_t)fw_size;
        if (ota_flag_write_dword(8, dw) != 0) return -2;
    }

    /* Write CRC32 (offset 16) + padding (offset 20) */
    {
        uint64_t dw = (uint64_t)fw_crc;
        if (ota_flag_write_dword(16, dw) != 0) return -2;
    }

    return 0;
}

/*============================================================================
 * P1-1: Breakpoint resume — save/restore OTA progress in flag page
 *
 * On timeout: ota_resume_save() writes resume info to flag page
 *   (RESUME_MAGIC + total_size + bytes_written) so the next OTA
 *   attempt can continue from where it left off.
 *
 * On seq 0: ota_resume_check() reads the flag page.  If the saved
 *   total_size matches this OTA's total_size, skip staging erase
 *   and restore bytes_written.  Already-received chunks are ACKed
 *   instantly as duplicates (no re-write to flash).
 *============================================================================*/

/* Save resume point to flag page — called on OTA timeout */
static void ota_resume_save(void)
{
    /* Erase flag page first */
    if (ota_flash_erase_page(OTA_FLAG_ADDR) != 0) return;

    /* Write RESUME_MAGIC */
    {
        uint64_t dw = (uint64_t)OTA_RESUME_MAGIC;
        ota_flag_write_dword(OTA_FLAG_OFF_RESUME_MAGIC, dw);
    }
    /* Write total_size + bytes_written */
    {
        uint64_t dw = ((uint64_t)ota_bytes_written << 32) | (uint64_t)ota_total_size;
        ota_flag_write_dword(OTA_FLAG_OFF_RESUME_TOTAL, dw);
    }
}

/* Check for valid resume state.  Returns bytes_written to restore,
 * or 0 if no valid resume / firmware mismatch. */
static uint32_t ota_resume_check(uint32_t new_total_size)
{
    uint32_t magic, saved_total, saved_bytes;

    magic       = *(volatile uint32_t *)(OTA_FLAG_ADDR + OTA_FLAG_OFF_RESUME_MAGIC);
    saved_total = *(volatile uint32_t *)(OTA_FLAG_ADDR + OTA_FLAG_OFF_RESUME_TOTAL);
    saved_bytes = *(volatile uint32_t *)(OTA_FLAG_ADDR + OTA_FLAG_OFF_RESUME_BYTES);

    if (magic != OTA_RESUME_MAGIC) return 0;

    /* Firmware must be the same binary (same total_size) */
    if (saved_total != new_total_size) {
        /* Different firmware — invalidate resume, full re-download needed */
        ota_flash_erase_page(OTA_FLAG_ADDR);
        return 0;
    }

    /* Validate saved_bytes is reasonable */
    if (saved_bytes == 0 || saved_bytes > OTA_MAX_SIZE) {
        ota_flash_erase_page(OTA_FLAG_ADDR);
        return 0;
    }

    /* Invalidate resume after reading — one-shot use */
    ota_flash_erase_page(OTA_FLAG_ADDR);

    return saved_bytes;
}

/*============================================================================
 * Direct UWB TX — send OTA_ACK back to Gateway
 * Sleep(2) before TX gives DW3000 time to settle after flash write.
 *============================================================================*/
static void ota_send_ack(uint16_t dst_addr, uint16_t src_addr,
                          uint16_t seq, uint8_t status)
{
    /* 16 bytes = 10B header + 3B payload + 3B pad.
     * DW3210 corrupts last 2 bytes of SPI read; 3B pad ensures the
     * critical status byte (ack[12]) is 4 bytes from frame end.
     * Seq is now 16-bit (2B) — fixes overflow at 256 chunks. */
    uint8_t ack[16];

    /* TDMA ACK delay — 2ms minimum for DW3000 PLL settling after flash write */
    {
        uint16_t slot_ms = 2u;
        if (src_addr > 0 && src_addr < 16) {
            slot_ms = (uint16_t)(src_addr * 5u);
            if (slot_ms < 2u) slot_ms = 2u;
        }
        Sleep(slot_ms);
    }

    ack[0] = 0x41; ack[1] = 0x88;
    ack[2] = 0;                    /* SEQ */
    ack[3] = 0xCA; ack[4] = 0xDE; /* PAN */
    ack[5] = dst_addr & 0xFF;      /* DST[0] — Gateway */
    ack[6] = (dst_addr >> 8) & 0xFF;
    ack[7] = src_addr & 0xFF;      /* SRC[0] — this device */
    ack[8] = (src_addr >> 8) & 0xFF;
    ack[9] = FUNC_OTA_ACK;         /* 0xF7 */
    ack[10] = seq & 0xFF;          /* seq low */
    ack[11] = (seq >> 8) & 0xFF;   /* seq high */
    ack[12] = status;              /* 0=OK, 1=CRC error, 2=flash error */
    ack[13] = 0xCC;                /* pad */
    ack[14] = 0xDD;                /* pad — these last 2 bytes may be */
    ack[15] = 0xEE;                /*       corrupted by DW3210 SPI bug */

    /* 1× burst — sufficient for reliable ACK.
     * MUST disable EXTI IRQ during TX: a frame arriving during the
     * spin-wait would trigger EXTI15_10_IRQHandler → dwt_forcetrxoff()
     * → clear SYS_STATUS (including TXFRS) → infinite hang. */
    port_DisableEXT_IRQ();
    dwt_forcetrxoff();
    port_rf_switch_tx();
    dwt_writetxdata(sizeof(ack), ack, 0);
    dwt_writetxfctrl(sizeof(ack), 0, 0);
    if (dwt_starttx(DWT_START_TX_IMMEDIATE) == DWT_SUCCESS) {
        uint32_t tx_dl = HAL_GetTick() + 10u;  /* 10ms timeout */
        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {
            if (HAL_GetTick() > tx_dl) break;   /* TX hung — bail out */
        }
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    }
    port_rf_switch_rx();
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    EXTI->PR1 = (1u << 12);
    port_EnableEXT_IRQ();
}

/*============================================================================
 * Public API
 *============================================================================*/

/* Initialize or reset OTA state */
void gw_ota_init(void)
{
    ota_state          = OTA_S_IDLE;
    ota_bytes_written  = 0;
    ota_total_size     = 0;
    ota_last_seq       = 0xFFFF;
    ota_last_data_tick = 0;
    ota_all_rcv_tick   = 0;
    ota_resume_offset  = 0;
    ota_reboot_tick    = 0;
}

/* Check if OTA data transfer is in progress.
 * Returns true only during RECEIVING — caller skips normal
 * beacon/blink TX to keep air clear for OTA frames. */
int gw_ota_is_active(void)
{
    return (ota_state == OTA_STATE_RECEIVING);
}

/* Handle OTA prepare — called when PREP_CMD (0xF0) is received from Gateway.
 * total_size: firmware image size in bytes (4B LE from PREP_CMD payload).
 * Erases staging area and initialises OTA state machine.
 * This replaces the old seq==0 total_size header — now ALL OTA_DATA chunks
 * carry exactly 104 bytes of pure firmware data (13×8B = always aligned). */
void gw_ota_handle_prepare(uint32_t total_size)
{
    if (ota_state == OTA_STATE_REBOOT_PENDING) return;
    if (total_size == 0 || total_size > OTA_MAX_SIZE) return;

    gw_ota_init();
    ota_total_size = total_size;

    DBG_PRINT("ERS\r\n");
    if (ota_flash_erase(OTA_FLASH_BASE, OTA_FLASH_END - OTA_PAGE_SIZE) != 0) {
        ota_state = OTA_S_IDLE;
        ota_total_size = 0;
        DBG_PRINT("ERF\r\n");
        return;
    }

    ota_state = OTA_STATE_RECEIVING;
    ota_last_data_tick = HAL_GetTick();
}

/* Poll for OTA timeout — call from main loop.
 * Returns 1 if OTA timed out (caller should resume normal operation).
 * Returns 0 if no timeout. */
int gw_ota_poll(void)
{
    switch (ota_state) {
    case OTA_S_IDLE:
        return 0;

    case OTA_STATE_RECEIVING:
        if (ota_last_data_tick == 0) return 0;  /* no data yet, wait */
        if (HAL_GetTick() - ota_last_data_tick < OTA_TIMEOUT_MS) return 0;

        /* Timeout — save resume point, then abort.
         * P1-1: On next OTA attempt, ota_resume_check() will restore
         * progress so we don't re-download already-received chunks. */
        DBG_PRINT("TMO\r\n");
        if (ota_bytes_written > 0 && ota_total_size > 0) {
            ota_resume_save();
            DBG_PRINT("RS\r\n");
        }
        gw_ota_init();
        return 1;

    case OTA_STATE_ALL_RCVD:
        /* Delayed auto-finish: all bytes received, waiting for OTA_END.
         * If OTA_END arrives, gw_ota_finish() handles it immediately.
         * If it doesn't arrive within OTA_FINISH_DELAY_MS, auto-finish. */
        if (HAL_GetTick() - ota_all_rcv_tick >= OTA_FINISH_DELAY_MS) {
            DBG_PRINT("AF\r\n");
            gw_ota_finish(0);
            ota_all_rcv_tick = HAL_GetTick();
        }
        return 0;

    case OTA_STATE_REBOOT_PENDING:
        /* Delayed soft-reset: flag write is done, flash is idle.
         * Wait OTA_REBOOT_DELAY_MS to let serial output flush and flash
         * controller fully settle, then SCB_AIRCR (proven reliable when
         * no flash ops are in progress — RST test passed). */
        if (HAL_GetTick() - ota_reboot_tick >= OTA_REBOOT_DELAY_MS) {
            DBG_PRINT("RBT\r\n");
            __disable_irq();
            __DSB();
            SCB->AIRCR = (0x5FAUL << 16U) | 0x04U;
            __DSB();
            __ISB();
            while (1) { __NOP(); }
        }
        return 0;

    default:
        return 0;
    }
}

/* Force-reset OTA state (called on OTA_ABORT frame from Gateway) */
void gw_ota_abort(void)
{
    if (ota_state == OTA_S_IDLE) return;
    gw_ota_init();
    DBG_PRINT("ABT\r\n");
}

/* P2: Confirm successful boot — call once after app initialisation is complete
 * (ranging started, UART working, etc.).  Erases flag page + staging area to
 * signal the bootloader that this firmware is good.  If this is NOT called
 * (app crashes), the bootloader will re-copy from staging and retry boot.
 * After MAX_BOOT_RETRIES (3) failed boots → SOS.
 *
 * Caller must ensure flash is idle before calling.  This function disables
 * IRQs during flash operations for safety. */
void gw_ota_confirm_boot(void)
{
    uint32_t magic, state;
    int need_cleanup = 0;

    /* Check if boot confirmation is needed */
    magic = *(volatile uint32_t *)(OTA_FLAG_ADDR + OTA_FLAG_OFF_MAGIC);
    state = *(volatile uint32_t *)(OTA_FLAG_ADDR + OTA_FLAG_OFF_STATE);

    if (magic == OTA_FLAG_MAGIC && state == OTA_STATE_BOOTING) {
        need_cleanup = 1;
    }

    if (!need_cleanup) return;  /* no pending confirmation — normal boot */

    DBG_PRINT("BOOT OK\r\n");

    __disable_irq();

    /* Erase flag page — clears STATE=BOOTING → STATE=IDLE */
    if (ota_flash_erase_page(OTA_FLAG_ADDR) == 0) {
        /* Erase staging area — no longer needed */
        ota_flash_erase(OTA_FLASH_BASE, OTA_FLASH_END - OTA_PAGE_SIZE);
    }

    __enable_irq();
    DBG_PRINT("CLN\r\n");
}

/* Handle OTA_DATA frame received via UWB.
 * frame: raw received buffer
 * len:   frame length in bytes
 * src_rank: this device's rank (used as SRC in ACK)
 *
 * Uses seq-based addressing: write_addr = OTA_FLASH_BASE + seq * chunk_size.
 * This is robust against restarts and out-of-order delivery. */
#define OTA_CHUNK_SIZE  104u  /* max: 127 - 10hdr - 2seq - 4totalSz - 2crc - 2pad = 107→104 aligned */

int gw_ota_handle_data(uint8_t *frame, uint16_t len, uint16_t src_rank)
{
    uint16_t seq;
    uint16_t dst_addr;
    uint8_t *payload;
    int      data_len;
    uint16_t crc_rx, crc_calc;
    uint32_t write_addr;

    if (frame[9] != FUNC_OTA_DATA)
        return 0;

    /* Reject OTA data when reboot is pending — prevents Gateway retries
     * from resetting the REBOOT_PENDING state and re-entering OTA. */
    if (ota_state == OTA_STATE_REBOOT_PENDING)
        return 0;

    /* Check minimum length: header(10) + seq(2) + data(≥1) + crc(2) + pad(2) */
    if (len < GW_HDR_LEN + 5)
        return 0;

    dst_addr = (uint16_t)(frame[7] | (frame[8] << 8));  /* Gateway address */
    seq      = (uint16_t)(frame[GW_HDR_LEN]) | ((uint16_t)(frame[GW_HDR_LEN + 1]) << 8);
    payload  = &frame[GW_HDR_LEN + 2];
    data_len = (int)len - GW_HDR_LEN - 6;  /* minus seq(2)+crc(2)+pad(2) */

    if (data_len <= 0 || data_len > 248) return 0;

    /* Update last-data tick for timeout detection */
    ota_last_data_tick = HAL_GetTick();

    /* Validate CRC — SW-ARQ: on CRC error, be SILENT.
     * Gateway will time out and retransmit the chunk. */
    crc_rx   = (uint16_t)(payload[data_len] | (payload[data_len + 1] << 8));
    crc_calc = ota_crc16(payload, data_len);
    if (crc_rx != crc_calc) {
        DBG_PRINT("CRC\r\n");
        return 0;  /* silent — gateway ARQ timeout will retry */
    }

    /*=== Trap-window fix: disable EXTI during flash operations ===
     * dwt_rxenable() at the call site already put DW3000 in RX.
     * During ota_flash_erase/write, __enable_irq() between dwords
     * would let the EXTI handler fire → dwt_forcetrxoff() → DW3000
     * goes IDLE → next OTA chunk lost.
     *
     * port_DisableEXT_IRQ() disables at NVIC level — survives the
     * __enable_irq() calls inside flash HAL.  dwt_forcetrxoff()
     * prevents stray frames from filling the RX double-buffer while
     * IRQs are masked (stale frames would collide with the next
     * OTA_DATA after ACK). */
    port_DisableEXT_IRQ();
    dwt_forcetrxoff();

    /* Skip duplicate sequence (retransmission) — MUST come FIRST,
     * before any write logic.  Gateway ARQ retries chunks whose ACK
     * was lost; re-writing already-programmed flash cells would fail
     * (PGSERR).  Just re-ACK and move on. */
    if (seq == ota_last_seq) {
        port_EnableEXT_IRQ();
        ota_send_ack(dst_addr, src_rank, seq, 0);
        return 1;
    }

    if (ota_state != OTA_STATE_RECEIVING) {
        /* Not started — ignore (PREP_CMD was lost, wait for retransmission) */
        port_EnableEXT_IRQ();
        return 0;
    }

    /*=== Simplified write: all chunks are 104B = 13×8B aligned ==========
     * PREP_CMD (0xF0) now carries total_size → every OTA_DATA chunk is
     * pure firmware data (104 bytes).  write_addr = OTA_FLASH_BASE +
     * seq * 104 is always 8-byte aligned (104 = 13 × 8).  No more
     * alignment buffer, no ota_flash_pos, no gap. */
    write_addr = OTA_FLASH_BASE + (uint32_t)seq * OTA_CHUNK_SIZE;

    if (write_addr + (uint32_t)data_len > OTA_FLASH_END) {
        port_EnableEXT_IRQ();
        ota_send_ack(dst_addr, src_rank, seq, 2);
        DBG_PRINT("OVF\r\n");
        return 1;
    }

    /* Write all dwords — data_len must be a multiple of 8.
     * Each chunk is 960 bytes = 120 perfect dwords. */
    {
        int i;
        for (i = 0; i + 8 <= data_len; i += 8) {
            uint64_t dw = 0;
            int b;
            for (b = 0; b < 8; b++)
                dw |= ((uint64_t)payload[i + b]) << (b * 8);
            if (ota_flash_write_dword(write_addr + (uint32_t)i, dw) != 0) {
                /* Retry once on genuine flash error (ART cache false-
                 * negative was fixed in ota_flash_write_dword above). */
                if (ota_flash_write_dword(write_addr + (uint32_t)i, dw) != 0) {
                    port_EnableEXT_IRQ();
                    ota_send_ack(dst_addr, src_rank, seq, 2);
                    DBG_PRINT("WRF\r\n");
                    return 1;
                }
            }
        }
    }

    ota_bytes_written += (uint32_t)data_len;
    ota_last_seq = seq;

    /* Re-enable EXTI before sending ACK — ota_send_ack() does its own
     * disable/enable cycle around the TX burst for IRQ safety. */
    port_EnableEXT_IRQ();
    ota_send_ack(dst_addr, src_rank, seq, 0);

    /* Delayed auto-finish: when all expected bytes received, mark complete
     * but DON'T reboot yet.  Wait for OTA_END from gateway (up to 3 s).
     * If OTA_END arrives, gw_ota_finish() handles it immediately.
     * If not, gw_ota_poll() auto-finishes after OTA_FINISH_DELAY_MS.
     *
     * This gives the Gateway time to:
     *   1. Receive our ACK
     *   2. Print +OTA_PROGRESS
     *   3. Host sends AT+OTA_BCAST_END
     *   4. Gateway sends OTA_END → we receive it → clean finish.
     *
     * Without this delay, the device reboots immediately and the Gateway
     * sees a timeout on the next chunk → false failure at 99%. */
    if (ota_total_size > 0 && (ota_resume_offset + ota_bytes_written) >= ota_total_size) {
        ota_state = OTA_STATE_ALL_RCVD;
        ota_all_rcv_tick = HAL_GetTick();
        DBG_PRINT("RCV\r\n");
    }

    return 1;
}

/*============================================================================
 * gw_ota_finish — compute CRC, write flag, reboot.
 * Bootloader handles staging→app copy with full verification on next boot.
 *============================================================================*/
void gw_ota_finish(uint16_t src_rank)
{
    if (ota_state == OTA_S_IDLE) return;
    if (ota_state == OTA_STATE_REBOOT_PENDING) return;

    {
        uint32_t total_rcvd = ota_resume_offset + ota_bytes_written;
        if (total_rcvd < ota_total_size) {
            ota_state = OTA_S_IDLE;
            return;
        }
    }

    ota_state = OTA_S_IDLE;

    {
        uint32_t fw_size = ota_total_size;  /* exact firmware size — no padding */
        uint32_t fw_crc;
        /* Compute CRC with ART cache bypassed — the staging area was just
         * written via ota_flash_write_dword() which toggles DCEN per dword,
         * but this explicit bypass guarantees correctness regardless of
         * future changes to the write path. */
        {
            uint32_t _save_acr = FLASH->ACR;
            FLASH->ACR &= ~((1UL << 9) | (1UL << 10));  /* DCEN=0, ICEN=0 */
            __DSB(); __ISB();
            fw_crc = ota_crc32((const uint8_t *)OTA_FLASH_BASE, fw_size);
            FLASH->ACR = _save_acr; __DSB(); __ISB();
        }

        DBG_PRINT("CRC=");
        {   char _h[9]; const char _x[] = "0123456789ABCDEF";
            _h[0]=_x[(fw_crc>>28)&0xF]; _h[1]=_x[(fw_crc>>24)&0xF];
            _h[2]=_x[(fw_crc>>20)&0xF]; _h[3]=_x[(fw_crc>>16)&0xF];
            _h[4]=_x[(fw_crc>>12)&0xF]; _h[5]=_x[(fw_crc>>8)&0xF];
            _h[6]=_x[(fw_crc>>4)&0xF]; _h[7]=_x[fw_crc&0xF]; _h[8]='\0';
            DBG_PRINT(_h);
        }
        DBG_PRINT("\r\n");

        __disable_irq();
        if (ota_flag_write(fw_size, fw_crc) == 0) {
            /* Read-back verify BEFORE reset — confirms flash content */
            {
                uint32_t v0 = *(volatile uint32_t *)(OTA_FLAG_ADDR);
                uint32_t v1 = *(volatile uint32_t *)(OTA_FLAG_ADDR + 4);
                uint32_t v2 = *(volatile uint32_t *)(OTA_FLAG_ADDR + 8);
                uint32_t v3 = *(volatile uint32_t *)(OTA_FLAG_ADDR + 16);
                DBG_PRINT("V=");
                {   char _h[9]; const char _x[] = "0123456789ABCDEF";
                    _h[0]=_x[(v0>>28)&0xF]; _h[1]=_x[(v0>>24)&0xF]; _h[2]=_x[(v0>>20)&0xF]; _h[3]=_x[(v0>>16)&0xF];
                    _h[4]=_x[(v0>>12)&0xF]; _h[5]=_x[(v0>>8)&0xF]; _h[6]=_x[(v0>>4)&0xF]; _h[7]=_x[v0&0xF]; _h[8]='\0';
                    DBG_PRINT(_h);
                }
                DBG_PRINT(",");
                {   char _h[9]; const char _x[] = "0123456789ABCDEF";
                    _h[0]=_x[(v3>>28)&0xF]; _h[1]=_x[(v3>>24)&0xF]; _h[2]=_x[(v3>>20)&0xF]; _h[3]=_x[(v3>>16)&0xF];
                    _h[4]=_x[(v3>>12)&0xF]; _h[5]=_x[(v3>>8)&0xF]; _h[6]=_x[(v3>>4)&0xF]; _h[7]=_x[v3&0xF]; _h[8]='\0';
                    DBG_PRINT(_h);
                }
                DBG_PRINT("\r\n");
            }
            DBG_PRINT("OK\r\n");
            ota_state       = OTA_STATE_REBOOT_PENDING;
            ota_reboot_tick = HAL_GetTick();
        }
        __enable_irq();
    }
}
