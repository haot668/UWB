/*! ----------------------------------------------------------------------------
 * @file    bootloader.c
 * @brief   OTA bootloader at 0x08000000 (pages 0-3, 8 KB) — STANDALONE BINARY
 *
 *          Flow on every boot:
 *            1. Read OTA flag page (0x0801F000) for pending firmware
 *            2. If MAGIC=="OTAR" & STATE==READY:
 *               a. Validate staging vector table (SP/PC sanity check)
 *                  → Invalid: erase staging+flag, boot existing app (rollback!)
 *               b. Erase app pages (4-31)
 *               c. Copy staging→app, verify every double-word
 *               d. Verify app VTOR matches staging VTOR
 *                  → OK: erase staging+flag, boot new app
 *                  → FAIL: RETRY++; if <3 reboot, else erase bad staging+flag → rollback
 *            3. If no flag / STATE!=READY: boot app directly
 *
 *          Safety guarantees:
 *            - Staging is VALIDATED before erasing old app (never bricks old firmware)
 *            - Retry up to 3 times on copy failure (staging still has good data)
 *            - Recovery mode LED SOS if all retries exhausted and app may be corrupt
 *
 *          Flash layout (STM32L412KB, 128 KB, 2 KB pages):
 *            0x08000000  Bootloader         (pages 0-3,   8 KB)
 *            0x08002000  Application        (pages 4-31, 56 KB)
 *            0x08010000  OTA staging area   (pages 32-61, 60 KB)
 *            0x0801F000  OTA flag page      (page 62,     2 KB)
 *            0x0801F800  Config             (page 63,     2 KB)
 *
 *          Deployment:
 *            - Bootloader is flashed ONCE at 0x08000000 (JTAG/SWD)
 *            - App is flashed at 0x08002000 (or OTA-updated via staging)
 *            - App scatter file starts at 0x08002000
 * ---------------------------------------------------------------------------*/
#include <stdint.h>

/*============================================================================
 * Hardware registers (no HAL — self-contained, position-independent safe)
 *============================================================================*/

/* Flash controller */
#define FLASH_REGS_BASE  0x40022000UL
#define FLASH_ACR        (*(volatile uint32_t *)(FLASH_REGS_BASE + 0x00))
#define FLASH_KEYR       (*(volatile uint32_t *)(FLASH_REGS_BASE + 0x08))
#define FLASH_SR         (*(volatile uint32_t *)(FLASH_REGS_BASE + 0x10))
#define FLASH_CR         (*(volatile uint32_t *)(FLASH_REGS_BASE + 0x14))

#define FLASH_KEY1       0x45670123UL
#define FLASH_KEY2       0xCDEF89ABUL
#define FLASH_SR_BSY     (1UL << 16)

#define FLASH_CR_PG      (1UL << 0)
#define FLASH_CR_PER     (1UL << 1)
#define FLASH_CR_STRT    (1UL << 16)
#define FLASH_CR_LOCK    (1UL << 31)

/* System Control Block */
#define SCB_VTOR         (*(volatile uint32_t *)0xE000ED08UL)
#define SCB_AIRCR        (*(volatile uint32_t *)0xE000ED0CUL)

/* RCC — enable clocks */
#define RCC_BASE         0x40021000UL
#define RCC_CR           (*(volatile uint32_t *)(RCC_BASE + 0x00))
#define RCC_AHB2ENR      (*(volatile uint32_t *)(RCC_BASE + 0x4C))
#define RCC_APB1ENR1     (*(volatile uint32_t *)(RCC_BASE + 0x58))
#define RCC_CCIPR         (*(volatile uint32_t *)(RCC_BASE + 0x88))
#define RCC_CR_HSION      (1UL << 8)
#define RCC_CR_HSIRDY     (1UL << 10)
#define RCC_AHB2ENR_GPIOAEN  (1UL << 0)
#define RCC_APB1ENR1_USART2EN (1UL << 17)

/* GPIOA — LED on PA5, USART2 TX on PA2 */
#define GPIOA_BASE       0x48000000UL
#define GPIOA_MODER      (*(volatile uint32_t *)(GPIOA_BASE + 0x00))
#define GPIOA_OTYPER     (*(volatile uint32_t *)(GPIOA_BASE + 0x04))
#define GPIOA_OSPEEDR    (*(volatile uint32_t *)(GPIOA_BASE + 0x08))
#define GPIOA_PUPDR      (*(volatile uint32_t *)(GPIOA_BASE + 0x0C))
#define GPIOA_AFRL       (*(volatile uint32_t *)(GPIOA_BASE + 0x20))
#define GPIOA_BSRR       (*(volatile uint32_t *)(GPIOA_BASE + 0x18))

/* USART2 */
#define USART2_BASE      0x40004400UL
#define USART2_CR1       (*(volatile uint32_t *)(USART2_BASE + 0x00))
#define USART2_BRR       (*(volatile uint32_t *)(USART2_BASE + 0x0C))
#define USART2_ISR       (*(volatile uint32_t *)(USART2_BASE + 0x1C))
#define USART2_TDR       (*(volatile uint32_t *)(USART2_BASE + 0x28))

/*============================================================================
 * Constants
 *============================================================================*/
#define APP_START        0x08002000UL   /* Application entry point            */
#define STAGING_START    0x08010000UL   /* OTA staging area                   */
#define FLAG_PAGE        0x0801F000UL   /* OTA flag storage page              */
#define CONFIG_PAGE      0x0801F800UL   /* Config storage page                */
#define PAGE_SIZE        2048UL         /* STM32L4 page size                  */

/* Bootloader diagnostic log in SRAM2 (not cleared by C startup).
 * App reads this on boot and prints via its working UART. */
#define BL_LOG_ADDR      0x10001F00UL   /* top 256 bytes of SRAM2 */
#define BL_LOG_MAGIC     0x424C4F47UL   /* "BLOG" */
volatile uint32_t * const bl_log = (volatile uint32_t *)BL_LOG_ADDR;
static uint32_t bl_log_idx = 0;  /* word offset from bl_log base */

static void bl_log_init(void)
{
    bl_log[0] = BL_LOG_MAGIC;
    bl_log[1] = 0;  /* status code: 0=running, 1=success, 2=rollback */
    bl_log[2] = 0;  /* error step */
    bl_log[3] = 0;  /* detail value 1 */
    bl_log[4] = 0;  /* detail value 2 */
    bl_log_idx = 5;
}

static void bl_log_set(uint32_t status, uint32_t step, uint32_t d1, uint32_t d2)
{
    bl_log[1] = status;
    bl_log[2] = step;
    bl_log[3] = d1;
    bl_log[4] = d2;
}

static void bl_log_word(uint32_t w)
{
    if (bl_log_idx < 63) {
        bl_log[bl_log_idx++] = w;
    }
}
#define BOOT_PAGES       4U             /* Pages 0-3 reserved for bootloader   */
#define STAGING_PAGES    ((FLAG_PAGE - STAGING_START) / PAGE_SIZE)  /* pages 32-61 = 30 */
#define APP_PAGES        ((STAGING_START - APP_START) / PAGE_SIZE)  /* pages 4-31 = 28 */

/* Flag page structure offsets */
#define FLAG_OFF_MAGIC   0u
#define FLAG_OFF_STATE   4u
#define FLAG_OFF_SIZE    8u
#define FLAG_OFF_RETRY   12u
#define FLAG_OFF_CRC32   16u   /* P0-1: CRC-32 of complete firmware image */

#define MAGIC_OTA_READY  0x4F544152UL   /* "OTAR" */
#define OTA_STATE_IDLE     0x00000000UL
#define OTA_STATE_READY    0x00000001UL  /* Staging has valid firmware, needs copy */
#define OTA_STATE_BOOTING  0x00000003UL  /* Firmware copied, waiting for app to confirm boot */
#define MAX_COPY_RETRIES   3u            /* Copy retry limit */
#define MAX_BOOT_RETRIES   3u            /* Boot confirmation retry limit → SOS */

/* SRAM bounds for SP validation */
#define SRAM1_START      0x20000000UL
#define SRAM1_END        0x20008000UL   /* 32 KB SRAM1 */

/*============================================================================
 * Minimal LED helpers (no HAL)
 *============================================================================*/
static void led_init(void)
{
    RCC_AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    /* PA5 as output (MODER bits 11:10 = 01) */
    GPIOA_MODER &= ~(3UL << 10);
    GPIOA_MODER |=  (1UL << 10);
}

static void led_on(void)  { GPIOA_BSRR = (1UL << 5); }
static void led_off(void) { GPIOA_BSRR = (1UL << 21); }

static void delay_ms(volatile uint32_t ms)
{
    volatile uint32_t i;
    while (ms--) for (i = 0; i < 4000; i++) { }
}

/* LED blink codes — each step encodes its result:
 *   N slow blinks (300ms on/300ms off) = step N passed
 *   Fast blink (100ms on/100ms off) for 3s = ERROR at current step
 *   Solid 1s = SUCCESS (copy+verify complete)
 *
 *   Step 1 = Flag valid,   Step 2 = VTOR valid,   Step 3 = CRC OK,
 *   Step 4 = Erase done,   Step 5 = Copy done,     Step 6 = VTOR verify OK
 */
static void led_slow_blink(int n)
{
    int _i;
    for (_i = 0; _i < n; _i++) {
        led_on();  delay_ms(300);
        led_off(); delay_ms(300);
    }
    delay_ms(500);  /* gap between sequences */
}

static void led_error_blink(void)
{
    int _i;
    for (_i = 0; _i < 15; _i++) {  /* ~3 seconds of fast blinking */
        led_on();  delay_ms(100);
        led_off(); delay_ms(100);
    }
}

/*============================================================================
 * Minimal USART2 TX (PA2, AF7, 115200 @ MSI 4MHz) — no FIFO, polling
 *============================================================================*/
static void uart_init(void)
{
    /* STM32L412 boots on MSI @ 4MHz.  Use it directly — no HSI16 needed. */

    /* Enable GPIOA and USART2 clocks */
    RCC_AHB2ENR  |= RCC_AHB2ENR_GPIOAEN;
    RCC_APB1ENR1 |= RCC_APB1ENR1_USART2EN;

    /* PA2 → AF7 (USART2_TX), push-pull, high speed */
    GPIOA_MODER   = (GPIOA_MODER & ~(3UL << 4)) | (2UL << 4);  /* AF mode */
    GPIOA_AFRL    = (GPIOA_AFRL  & ~(0xFUL << 8)) | (7UL << 8); /* AF7 */
    GPIOA_OSPEEDR |= (3UL << 4);  /* PA2 = very high speed */

    /* USART2 clock = PCLK = MSI 4MHz (USART2SEL=00, default after reset) */

    /* Baud rate: 4,000,000 / 115,200 = 34.72
     * Mantissa=34(0x22), Fraction=12(0xC) → actual=115,108 (0.08% error) */
    USART2_BRR  = (34UL << 4) | 0x0CUL;
    USART2_CR1  = (1UL << 3);           /* TE=1 */
    USART2_CR1 |= (1UL << 0);           /* UE=1 */
    /* Wait for TX ready, then flush with a dummy byte */
    { volatile int _w = 0; while (_w < 10000) _w++; }  /* short settle delay */
    while (!(USART2_ISR & (1UL << 7))) { }   /* wait TXE */
    USART2_TDR = '\r';
    while (!(USART2_ISR & (1UL << 6))) { }   /* wait TC */
}

static void uart_putc(char c)
{
    while (!(USART2_ISR & (1UL << 7))) { }  /* wait TXE */
    USART2_TDR = (uint32_t)(uint8_t)c;
}

static void uart_puts(const char *s)
{
    while (*s) { uart_putc(*s++); }
}

static void uart_puthex8(uint8_t v)
{
    static const char hx[] = "0123456789ABCDEF";
    uart_putc(hx[v >> 4]);
    uart_putc(hx[v & 0xF]);
}

static void uart_puthex32(uint32_t v)
{
    uart_puthex8((uint8_t)(v >> 24));
    uart_puthex8((uint8_t)(v >> 16));
    uart_puthex8((uint8_t)(v >> 8));
    uart_puthex8((uint8_t)(v));
}

static void uart_putdec(uint32_t v)
{
    char buf[12]; int i = 0, j;
    if (v == 0) { uart_putc('0'); return; }
    while (v) { buf[i++] = '0' + (char)(v % 10); v /= 10; }
    for (j = i - 1; j >= 0; j--) uart_putc(buf[j]);
}

/* SOS pattern: ... --- ...  (never returns) */
static void led_sos(void)
{
    int _k;
    while (1) {
        /* S: 3 short */
        for (_k = 0; _k < 3; _k++) {
            led_on();  delay_ms(150);
            led_off(); delay_ms(150);
        }
        delay_ms(300);
        /* O: 3 long */
        for (_k = 0; _k < 3; _k++) {
            led_on();  delay_ms(500);
            led_off(); delay_ms(150);
        }
        delay_ms(300);
        /* S: 3 short */
        for (_k = 0; _k < 3; _k++) {
            led_on();  delay_ms(150);
            led_off(); delay_ms(150);
        }
        delay_ms(1000);
    }
}

/*============================================================================
 * Fault handlers — override weak defaults from startup_stm32l412xx.s.
 *
 * Each has a DISTINCT LED blink pattern to identify WHICH fault fired:
 *   HardFault:  2 fast blinks, pause, repeat  →  "..  ..  .."
 *   NMI:        3 fast blinks, pause, repeat  →  "...  ...  ..."
 *   BusFault:   4 fast blinks, pause, repeat  →  "....  ....  ...."
 *   UsageFault: 5 fast blinks, pause, repeat  →  ".....  .....  ....."
 *
 * These use RAW register access — no function calls — so they work even
 * when the stack is suspect or ART cache is in an odd state.
 *============================================================================*/

#define FAULT_BLINK_ON()   do { GPIOA_BSRR = (1UL << 5);                      } while(0)
#define FAULT_BLINK_OFF()  do { GPIOA_BSRR = (1UL << 21);                     } while(0)
#define FAULT_DELAY_SHORT() do { volatile uint32_t _fd = 0;                    \
                                 while (_fd < 400000) { _fd++; }              } while(0)
#define FAULT_DELAY_PAUSE() do { volatile uint32_t _fp = 0;                    \
                                 while (_fp < 2000000) { _fp++; }             } while(0)

static void fault_led_init(void)
{
    RCC_AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    GPIOA_MODER &= ~(3UL << 10);
    GPIOA_MODER |=  (1UL << 10);
}

/* Pattern: N fast blinks then long pause, repeating forever */
static void fault_blink_n(int n)
{
    fault_led_init();
    while (1) {
        int _i;
        for (_i = 0; _i < n; _i++) {
            FAULT_BLINK_ON();
            FAULT_DELAY_SHORT();
            FAULT_BLINK_OFF();
            FAULT_DELAY_SHORT();
        }
        FAULT_DELAY_PAUSE();
    }
}

void HardFault_Handler(void)  { fault_blink_n(2); }
void NMI_Handler(void)        { fault_blink_n(3); }
void BusFault_Handler(void)   { fault_blink_n(4); }
void UsageFault_Handler(void) { fault_blink_n(5); }

/*============================================================================
 * Flash helpers — direct register access, no HAL dependency
 *============================================================================*/
static void flash_unlock(void)
{
    if (FLASH_CR & FLASH_CR_LOCK) {
        FLASH_KEYR = FLASH_KEY1;
        FLASH_KEYR = FLASH_KEY2;
    }
}

/* Independent watchdog — feed during long flash ops */
#define IWDG_KR           (*(volatile uint32_t *)0x40003000UL)

static void flash_wait_busy(void)
{
    while (FLASH_SR & FLASH_SR_BSY) {
        IWDG_KR = 0xAAAA;  /* feed IWDG to prevent reset during long ops */
    }
}

#define FLASH_SR_EOP      (1UL << 0)
#define FLASH_SR_WRPERR   (1UL << 17)
#define FLASH_SR_PGAERR   (1UL << 18)
#define FLASH_SR_SIZERR   (1UL << 19)
#define FLASH_SR_PGSERR   (1UL << 20)
#define FLASH_SR_ALL_ERR   (FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_SIZERR | FLASH_SR_PGSERR)

/* Clear all flash status flags and check for errors.
 * Returns 0 on success, non-zero if any error flag was set. */
static int flash_check_errors(void)
{
    uint32_t sr = FLASH_SR;
    FLASH_SR = FLASH_SR_EOP | FLASH_SR_ALL_ERR;  /* w1c all flags */
    if (sr & FLASH_SR_ALL_ERR) return -1;
    return 0;
}

static void flash_erase_page(uint32_t page)
{
    uint32_t cr;
    flash_unlock();
    /* Clear all status flags before starting */
    FLASH_SR = FLASH_SR_EOP | FLASH_SR_ALL_ERR;
    cr  = FLASH_CR_PER;
    cr |= (page << 3) & 0x3F8UL;
    FLASH_CR = cr;
    FLASH_CR = cr | FLASH_CR_STRT;
    flash_wait_busy();
    FLASH_CR &= ~FLASH_CR_PER;   /* clear only PER, preserve LOCK */
    FLASH_CR  = FLASH_CR_LOCK;   /* lock flash (same as gw_ota pattern) */
}

/* Write one 8-byte double-word to flash.
 * Uses the EXACT same sequence as gw_ota's ota_flash_write_dword() —
 * proven to write 32KB+ of staging data reliably.
 *   - OR/AND-NOT FLASH_CR (don't clobber other bits)
 *   - Clear error flags before each attempt
 *   - Retry up to 5 times
 *   - Lock flash on success */
static int flash_program_dword(uint32_t addr, uint32_t w0, uint32_t w1)
{
    int retry;
    for (retry = 0; retry < 5; retry++) {
        /* Feed IWDG */
        IWDG_KR = 0xAAAA;

        /* Unlock flash if locked */
        if (FLASH_CR & FLASH_CR_LOCK) {
            FLASH_KEYR = FLASH_KEY1;
            FLASH_KEYR = FLASH_KEY2;
        }

        /* Clear error flags before programming */
        {
            uint32_t _sr = FLASH_SR;
            if (_sr & (FLASH_SR_PGSERR | FLASH_SR_SIZERR |
                       FLASH_SR_PGAERR | FLASH_SR_WRPERR)) {
                FLASH_SR = (FLASH_SR_PGSERR | FLASH_SR_SIZERR |
                            FLASH_SR_PGAERR | FLASH_SR_WRPERR);
            }
        }

        /* Program double-word */
        FLASH_CR |= FLASH_CR_PG;
        *(volatile uint32_t *)(addr)     = w0;
        *(volatile uint32_t *)(addr + 4) = w1;
        flash_wait_busy();
        FLASH_CR &= ~FLASH_CR_PG;

        /* Read-back verify with DCEN/ICEN disabled.
         * TWO-PASS: immediate + delayed (~50µs settling).
         * Marginal flash cells can pass immediate read-back but
         * fail after the programming charge dissipates.  The
         * delayed pass catches this — if it fails, we retry. */
        {
            uint32_t _save_acr = FLASH_ACR;
            FLASH_ACR &= ~((1UL << 9) | (1UL << 10));  /* DCEN=0, ICEN=0 */
            __asm { dsb } __asm { isb }

            if (*(volatile uint32_t *)(addr)     == w0 &&
                *(volatile uint32_t *)(addr + 4) == w1) {
                /* Immediate pass. Wait ~50µs for cell settling,
                 * then re-read to catch marginal programming. */
                { volatile uint32_t _sd; for (_sd = 0; _sd < 200; _sd++) { } }
                if (*(volatile uint32_t *)(addr)     == w0 &&
                    *(volatile uint32_t *)(addr + 4) == w1) {
                    FLASH_ACR = _save_acr; __asm { dsb } __asm { isb }
                    FLASH_CR = FLASH_CR_LOCK;  /* lock flash */
                    return 0;
                }
                /* Delayed read-back failed — cell marginal, retry */
            }
            FLASH_ACR = _save_acr; __asm { dsb } __asm { isb }
        }

        /* Check error flags for next retry */
        {
            uint32_t _sr2 = FLASH_SR;
            if (_sr2 & (FLASH_SR_PGSERR | FLASH_SR_SIZERR |
                        FLASH_SR_PGAERR | FLASH_SR_WRPERR)) {
                FLASH_SR = (FLASH_SR_PGSERR | FLASH_SR_SIZERR |
                            FLASH_SR_PGAERR | FLASH_SR_WRPERR);
            }
        }
    }
    FLASH_CR = FLASH_CR_LOCK;  /* lock flash */
    return -1;
}

static void flash_lock(void)
{
    FLASH_CR = FLASH_CR_LOCK;
}

/*============================================================================
 * P3: Enable WRP (Write Protection) on bootloader pages 0-3.
 *
 * WARNING: This is a ONE-TIME production step.  After WRP is enabled,
 * the bootloader pages CANNOT be erased or programmed without first
 * disabling WRP via SWD/JTAG.  This protects against accidental
 * corruption but also prevents OTA updates to the bootloader itself.
 *
 * Call ONLY during initial factory flashing or via a dedicated debug
 * command.  Do NOT call on every boot — Option Bytes have limited
 * write endurance (~1000 cycles).
 *
 * STM32L4 Option Bytes programming sequence:
 *   1. Unlock FLASH_CR
 *   2. Unlock FLASH_OPTKEYR
 *   3. Write WRP1AR (protect pages 0–3)
 *   4. Set OPTSTRT → wait BSY → lock
 *============================================================================*/
#define FLASH_OPTKEYR    (*(volatile uint32_t *)(FLASH_REGS_BASE + 0x0C))
#define FLASH_OPTR       (*(volatile uint32_t *)(FLASH_REGS_BASE + 0x20))
#define FLASH_WRP1AR     (*(volatile uint32_t *)(FLASH_REGS_BASE + 0x2C))
#define FLASH_CR_OPTSTRT (1UL << 1)
#define FLASH_OPTKEY1    0x08192A3BUL
#define FLASH_OPTKEY2    0x4C5D6E7FUL

static int bl_enable_wrp(void)
{
    uint32_t wrp_val;

    flash_unlock();  /* unlock FLASH_CR via KEYR */

    /* Unlock Option Bytes */
    FLASH_OPTKEYR = FLASH_OPTKEY1;
    FLASH_OPTKEYR = FLASH_OPTKEY2;

    if (FLASH_OPTR & 1UL) {  /* OPTLOCK still set → unlock failed */
        flash_lock();
        return -1;
    }

    /* Protect bootloader pages 0–3 (8 KB at 0x08000000).
     * WRP1AR: WRP1A_STRT[7:0] = 0, WRP1A_END[7:0] = 3 */
    wrp_val = FLASH_WRP1AR;
    wrp_val &= ~(0xFFUL | (0xFFUL << 16));  /* clear old values */
    wrp_val |= (0UL) | (3UL << 16);          /* start=0, end=3 */
    FLASH_WRP1AR = wrp_val;

    /* Start Option Byte programming (erases + programs entire option block) */
    FLASH_CR |= FLASH_CR_OPTSTRT;
    flash_wait_busy();

    /* Lock everything */
    FLASH_CR |= FLASH_CR_LOCK;

    /* Verify — read back WRP registers */
    if ((FLASH_WRP1AR & 0xFFUL) == 0 && ((FLASH_WRP1AR >> 16) & 0xFFUL) == 3) {
        return 0;  /* success */
    }
    return -2;  /* verification failed */
}

/* Write to flag page (page must be pre-erased).
 * Each call writes one 64-bit word at the given offset (0, 8, 16, 24). */
static int flag_write_dword(uint32_t offset, uint32_t w0, uint32_t w1)
{
    return flash_program_dword(FLAG_PAGE + offset, w0, w1);
}

/*============================================================================
 * CRC-32 via STM32L4 hardware peripheral (guarantees identical result to
 * gw_ota software CRC — both use CRC-32/MPEG-2: poly 0x04C11DB7, init 0xFF)
 *============================================================================*/
#define CRC_BASE         0x40023000UL
#define CRC_DR           (*(volatile uint32_t *)(CRC_BASE + 0x00))
#define CRC_IDR          (*(volatile uint32_t *)(CRC_BASE + 0x04))
#define CRC_CR           (*(volatile uint32_t *)(CRC_BASE + 0x08))
#define CRC_INIT         (*(volatile uint32_t *)(CRC_BASE + 0x10))
#define CRC_POL          (*(volatile uint32_t *)(CRC_BASE + 0x14))
#define RCC_AHB1ENR      (*(volatile uint32_t *)(0x40021000UL + 0x48))
#define RCC_AHB1ENR_CRCEN (1UL << 12)

static void hw_crc32_init(void)
{
    RCC_AHB1ENR |= RCC_AHB1ENR_CRCEN;  /* enable CRC clock */
    CRC_CR  |= (1UL << 0);              /* RESET */
    CRC_POL  = 0x04C11DB7UL;            /* CRC-32/MPEG-2 polynomial */
    CRC_INIT = 0xFFFFFFFFUL;            /* initial value */
    CRC_CR   = 0;                       /* release reset */
}

/* Byte-swap a 32-bit word: 0x12345678 → 0x78563412.
 * ARMCC optimises this to a single REV instruction. */
static uint32_t bswap32(uint32_t x)
{
    return (x >> 24) | ((x >> 8) & 0xFF00U) | ((x << 8) & 0xFF0000U) | (x << 24);
}

static uint32_t hw_crc32(const uint8_t *data, uint32_t len)
{
    uint32_t i;
    uint32_t crc;
    uint32_t aligned_len = len & ~3u;  /* round down to multiple of 4 */
    CRC_CR |= (1UL << 0);  /* reset CRC to INIT value */
    /* STM32L4 CRC peripheral processes bit 31 first. On little-endian ARM,
     * *(uint32_t*)(data+i) has data[i+3] in bit 31, so bytes are processed
     * in REVERSE order.  We byte-swap each word so data[i+0] is in bits 31-24,
     * matching the software CRC byte order. */
    for (i = 0; i < aligned_len; i += 4) {
        uint32_t w = *(volatile uint32_t *)(data + i);
        CRC_DR = bswap32(w);
    }
    /* Read hardware CRC result, then finish trailing 0-3 bytes in software.
     * Writing a byte to CRC_DR zero-extends to 32 bits, adding spurious zero
     * bits — so tail computation cannot use the hardware peripheral. */
    crc = CRC_DR;
    {
        uint32_t rem = len & 3u;
        uint32_t j;
        for (j = 0; j < rem; j++) {
            int b;
            crc ^= (uint32_t)data[i + j] << 24;
            for (b = 0; b < 8; b++) {
                if (crc & 0x80000000)
                    crc = (crc << 1) ^ 0x04C11DB7;
                else
                    crc = (crc << 1);
            }
        }
    }
    return crc;
}

/* Software CRC-32/MPEG-2 for self-test only */
static uint32_t bl_crc32(const uint8_t *data, uint32_t len)
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
 * Validation
 *============================================================================*/

/* Returns 1 if staging vector table looks valid */
static int validate_staging_vtor(void)
{
    volatile uint32_t *staging = (volatile uint32_t *)STAGING_START;
    uint32_t sp = staging[0];
    uint32_t pc = staging[1];

    /* SP must be in SRAM1 range */
    if (sp < SRAM1_START || sp > SRAM1_END)
        return 0;

    /* PC must be in app flash range */
    if (pc < APP_START || pc > STAGING_START)
        return 0;

    /* PC must have Thumb bit set */
    if ((pc & 1UL) == 0)
        return 0;

    return 1;
}

/* Scan backwards from end of staging to find firmware size.
 * Returns size rounded up to 8 bytes, or 0 if invalid. */
static uint32_t scan_firmware_size(void)
{
    uint32_t max_size = STAGING_PAGES * PAGE_SIZE;
    uint32_t fw_size;
    uint32_t *p;

    if (max_size > 0xF000) max_size = 0xF000;  /* sanity cap ~60KB */
    fw_size = max_size;
    p = (uint32_t *)(STAGING_START + fw_size);

    while (fw_size > 1024 && p[-1] == 0xFFFFFFFFUL) {
        p--;
        fw_size -= 4;
    }

    fw_size = (fw_size + 7) & ~7UL;
    if (fw_size < 1024 || fw_size > max_size)
        return 0;

    return fw_size;
}

/*============================================================================
 * Erase helpers
 *============================================================================*/

/* Erase staging pages (32-61) and flag page (62) */
static void erase_staging_and_flag(void)
{
    uint32_t first_page = STAGING_START / PAGE_SIZE;
    uint32_t last_page  = FLAG_PAGE / PAGE_SIZE;  /* page 62 */
    uint32_t page;
    for (page = first_page; page <= last_page; page++) {
        flash_erase_page(page);
    }
}

/*============================================================================
 * Forward declarations
 *============================================================================*/
void bootloader_main(void);

/* __set_MSP for ARMCC 5 — not available as a library symbol.
 * Defined here as inline asm because we don't include cmsis_armcc.h. */
__asm void __set_MSP(uint32_t sp)
{
    MSR MSP, r0
    BX lr
}

/*============================================================================
 * Minimal SystemInit — startup_stm32l412xx.s calls this before __main.
 * Bootloader runs at HSI 16 MHz default; no PLL or clock setup needed.
 *============================================================================*/
void SystemInit(void)
{
    /* Enable FPU on Cortex-M4 */
    *(volatile uint32_t *)0xE000ED88UL |= (0xFUL << 20);
}
uint32_t SystemCoreClock = 16000000;

/*============================================================================
 * main — called from __main after C runtime init
 *============================================================================*/
int main(void)
{
    bootloader_main();
    /* bootloader_main() either jumps to app or loops forever (SOS) */
    while (1) { }
}

/*============================================================================
 * Bootloader entry point
 *
 * This is a standalone binary flashed at 0x08000000.
 * Application is at 0x08002000 with its own Reset_Handler.
 *============================================================================*/
void bootloader_main(void)
{
    uint32_t magic, state, fw_size, retry_cnt;
    uint32_t offset, page;
    uint32_t staging_sp, staging_pc;

    led_init();
    led_on();  /* LED ON = bootloader running. If stays ON → hung. */
    uart_init();
    uart_puts("\r\n\r\n[BL] === BOOTLOADER @ 0x08000000 ===\r\n");

    bl_log_init();

    /* 1. Read OTA flag page (flash only — simple and reliable) */
    magic     = *(volatile uint32_t *)(FLAG_PAGE + FLAG_OFF_MAGIC);
    state     = *(volatile uint32_t *)(FLAG_PAGE + FLAG_OFF_STATE);
    fw_size   = *(volatile uint32_t *)(FLAG_PAGE + FLAG_OFF_SIZE);
    retry_cnt = *(volatile uint32_t *)(FLAG_PAGE + FLAG_OFF_RETRY);

    bl_log_word(magic); bl_log_word(state);
    bl_log_word(fw_size); bl_log_word(retry_cnt);

    /* P2: If flag page is erased (0xFFFFFFFF) or STATE=IDLE, no OTA pending — boot app.
     *     STATE_BOOTING → app crashed before confirming boot — retry copy.
     *     STATE_READY  → normal OTA — copy staging→app. */
    if (magic != MAGIC_OTA_READY || (state != OTA_STATE_READY && state != OTA_STATE_BOOTING)) {
        bl_log_set(0, 0, magic, state);
        led_off();  /* bootloader done, no OTA flag */
        goto boot_app;
    }

    /* P2: Boot counter — if STATE_BOOTING, app crashed before confirming boot.
     * Re-copy from staging (still intact) and increment retry counter.
     * After MAX_BOOT_RETRIES crashes, enter SOS (firmware is defective). */
    if (state == OTA_STATE_BOOTING) {
        if (retry_cnt >= MAX_BOOT_RETRIES) {
            bl_log_set(2, 8, retry_cnt, 0);
            uart_puts("[BL] BOOT RETRY EXHAUSTED → SOS\r\n");
            erase_staging_and_flag();
            flash_lock();
            led_sos();
        }
        uart_puts("[BL] STATE=BOOTING retry="); uart_putdec(retry_cnt);
        uart_puts(" → re-copy staging→app\r\n");
        /* Skip validation steps 1-3 (staging was already validated on first copy).
         * Jump directly to erase+copy.  But first, re-validate staging CRC
         * to guard against flash cell degradation. */
        goto bl_retry_copy;
    }

    /* Step 1: Flag valid — 1 slow blink */
    led_slow_blink(1);
    uart_puts("[BL] OTA flag valid, checking VTOR...\r\n");

    /* 2. OTA pending — VALIDATE STAGING FIRST (before touching old app!) */
    {
        volatile uint32_t *sv = (volatile uint32_t *)STAGING_START;
        staging_sp = sv[0];
        staging_pc = sv[1];
    }

    uart_puts("[BL] VTOR: SP="); uart_puthex32(staging_sp);
    uart_puts(" PC="); uart_puthex32(staging_pc);
    uart_puts("\r\n");

    bl_log_word(staging_sp); bl_log_word(staging_pc);

    if (!validate_staging_vtor()) {
        bl_log_set(2, 2, staging_sp, staging_pc);
        erase_staging_and_flag();
        flash_lock();
        goto boot_app;
    }

    bl_log_set(1, 2, staging_sp, staging_pc);

    /* Step 2: VTOR valid — 2 slow blinks */
    led_slow_blink(2);
    uart_puts("[BL] VTOR OK, checking CRC-32...\r\n");

    /* Init hardware CRC peripheral */
    hw_crc32_init();

    /* 2c. CRC-32 verification — use hardware CRC if self-test passed,
     * fall back to software CRC otherwise (guarantees match with gw_ota). */
    {
        uint32_t flag_crc = *(volatile uint32_t *)(FLAG_PAGE + FLAG_OFF_CRC32);
        uint32_t calc_crc;
        uint32_t crc_size;
        int use_hw_crc;  /* 1 = hardware CRC, 0 = software fallback */

        /* Self-test validation — "123456789" expected result */
        {
            const char test[] = "123456789";
            uint32_t test_sw = bl_crc32((const uint8_t *)test, 9);
            uint32_t test_hw = hw_crc32((const uint8_t *)test, 9);
            use_hw_crc = (test_sw == test_hw) ? 1 : 0;
            bl_log_word(test_sw);
            bl_log_word(test_hw);
            bl_log_word(use_hw_crc);  /* log which CRC method is used */
        }

        crc_size = fw_size;
        if (crc_size == 0 || crc_size > (STAGING_PAGES * PAGE_SIZE)) {
            crc_size = scan_firmware_size();
        }
        if (crc_size < 1024) {
            bl_log_set(2, 3, crc_size, 0);
            erase_staging_and_flag();
            flash_lock();
            goto boot_app;
        }

        bl_log_word(crc_size); bl_log_word(flag_crc);
        /* Log first 16 bytes of staging for data integrity check */
        {
            volatile uint32_t *s8 = (volatile uint32_t *)STAGING_START;
            bl_log_word(s8[0]); bl_log_word(s8[1]);
            bl_log_word(s8[2]); bl_log_word(s8[3]);
        }

        /* Compute CRC — prefer hardware for speed, fall back to software */
        if (use_hw_crc) {
            calc_crc = hw_crc32((const uint8_t *)STAGING_START, crc_size);
        } else {
            calc_crc = bl_crc32((const uint8_t *)STAGING_START, crc_size);
        }
        {
            uint32_t calc_sw = bl_crc32((const uint8_t *)STAGING_START, crc_size);
            bl_log_word(calc_crc);  /* chosen CRC method result */
            bl_log_word(calc_sw);   /* software CRC result (for comparison) */
        }
        /* Segment CRCs for narrowing down data divergence */
        {
            uint32_t mid = crc_size / 2;
            if (mid < 1024) mid = 1024;
            if (use_hw_crc) {
                bl_log_word(hw_crc32((const uint8_t *)STAGING_START, 1024));
                bl_log_word(hw_crc32((const uint8_t *)(STAGING_START + mid), 1024));
                bl_log_word(hw_crc32((const uint8_t *)(STAGING_START + crc_size - 1024), 1024));
            } else {
                bl_log_word(bl_crc32((const uint8_t *)STAGING_START, 1024));
                bl_log_word(bl_crc32((const uint8_t *)(STAGING_START + mid), 1024));
                bl_log_word(bl_crc32((const uint8_t *)(STAGING_START + crc_size - 1024), 1024));
            }
        }

        if (calc_crc != flag_crc) {
            bl_log_set(2, 3, flag_crc, calc_crc);
            uart_puts("[BL] CRC MISMATCH flag="); uart_puthex32(flag_crc);
            uart_puts(" calc="); uart_puthex32(calc_crc);
            uart_puts(" method="); uart_putc(use_hw_crc ? 'H' : 'S'); uart_puts("\r\n");
            erase_staging_and_flag();
            flash_lock();
            goto boot_app;
        }
    }

    bl_log_set(1, 3, 0, 0);

    /* Step 3: CRC OK — 3 slow blinks */
    led_slow_blink(3);
    goto bl_erase_app;  /* skip retry label on normal path */

bl_retry_copy:
    /* P2: Boot counter retry — app crashed before confirming boot.
     * FIRST check if APP CRC still matches — if yes, app is intact and
     * crash was a transient issue (or software bug).  Just increment
     * retry and try booting again WITHOUT erasing or re-copying.
     * Only re-copy from staging if APP CRC is actually corrupt. */
    {
        uint32_t flag_crc = *(volatile uint32_t *)(FLAG_PAGE + FLAG_OFF_CRC32);
        uint32_t app_crc;
        int app_ok = 0;

        if (retry_cnt >= MAX_BOOT_RETRIES) {
            uart_puts("[BL] BOOT RETRY EXHAUSTED → SOS\r\n");
            bl_log_set(2, 8, retry_cnt, 0);
            erase_staging_and_flag();
            flash_lock();
            led_sos();
        }

        uart_puts("[BL] STATE=BOOTING retry="); uart_putdec(retry_cnt);
        uart_puts(" → checking APP CRC...\r\n");

        /* Check APP CRC with DCEN/ICEN disabled */
        hw_crc32_init();
        {
            uint32_t _save = FLASH_ACR;
            FLASH_ACR &= ~((1UL << 9) | (1UL << 10));
            __asm { dsb } __asm { isb }
            app_crc = hw_crc32((const uint8_t *)APP_START, fw_size);
            FLASH_ACR = _save; __asm { dsb } __asm { isb }
        }

        bl_log_word(app_crc);
        bl_log_word(flag_crc);

        if (app_crc == flag_crc) {
            /* APP is intact — just increment retry counter and re-boot.
             * No erase, no re-copy.  If the app has a persistent software
             * bug, MAX_BOOT_RETRIES will eventually trigger SOS. */
            app_ok = 1;
            uart_puts("[BL] APP CRC OK → re-boot without re-copy\r\n");
            flash_erase_page(FLAG_PAGE / PAGE_SIZE);
            flag_write_dword(FLAG_OFF_MAGIC, MAGIC_OTA_READY, OTA_STATE_BOOTING);
            flag_write_dword(FLAG_OFF_SIZE, fw_size, retry_cnt + 1);
            flag_write_dword(FLAG_OFF_CRC32, flag_crc, 0);
            flash_lock();
            bl_log_set(1, 8, retry_cnt + 1, 0);
            goto boot_app;
        }

        /* APP CRC mismatch — verify staging is still intact before re-copy.
         * This is the worst case: APP is corrupt but staging should be good. */
        uart_puts("[BL] APP CRC MISMATCH → checking staging...\r\n");
        {
            uint32_t calc_crc = hw_crc32((const uint8_t *)STAGING_START, fw_size);
            if (calc_crc != flag_crc) {
                /* Both corrupt → unrecoverable */
                uart_puts("[BL] STAGING also corrupt → SOS\r\n");
                bl_log_set(2, 9, app_crc, calc_crc);
                erase_staging_and_flag();
                flash_lock();
                led_sos();
            }
        }
        uart_puts("[BL] Staging OK → re-copy\r\n");

        /* Increment retry BEFORE erasing APP (power-loss safety) */
        {
            uint32_t saved_crc = flag_crc;
            flash_erase_page(FLAG_PAGE / PAGE_SIZE);
            flag_write_dword(FLAG_OFF_MAGIC, MAGIC_OTA_READY, OTA_STATE_BOOTING);
            flag_write_dword(FLAG_OFF_SIZE, fw_size, retry_cnt + 1);
            flag_write_dword(FLAG_OFF_CRC32, saved_crc, 0);
        }
        /* Fall through to erase+copy */
    }

bl_erase_app:
    /* 3. Determine firmware size */
    if (fw_size == 0 || fw_size > (STAGING_PAGES * PAGE_SIZE)) {
        fw_size = scan_firmware_size();
    }
    if (fw_size < 1024) {
        uart_puts("[BL] FW size < 1024 → rollback\r\n");
        erase_staging_and_flag();
        flash_lock();
        goto boot_app;
    }

    uart_puts("[BL] FW size="); uart_putdec(fw_size);
    uart_puts(" → erasing app pages...\r\n");

    /* 4. Erase application pages (4 to 31) */
    {
        uint32_t first_page = BOOT_PAGES;
        uint32_t last_page  = (STAGING_START / PAGE_SIZE) - 1;
        int erase_err = 0;
        for (page = first_page; page <= last_page; page++) {
            flash_erase_page(page);
            if (flash_check_errors() != 0) {
                erase_err = 1;
                break;
            }
        }
        if (erase_err) {
            bl_log_set(2, 5, page, 0);
            erase_staging_and_flag();
            flash_lock();
            goto boot_app;
        }

        /* Erase verified by flash_check_errors() above — each page erase
         * completed without hardware error (no WRPERR/PGAERR/SIZERR/PGSERR).
         * We intentionally do NOT read-back verify erased cells here.
         *
         * WHY: On STM32L4, the ART data cache can return ghost data even
         * with DCEN=ICEN=0 (silicon errata).  This caused a FALSE erase-failure
         * detection where check[0] read as 0x00000004 instead of 0xFFFFFFFF.
         * The bootloader then erased staging+flag and tried to boot the (erased)
         * APP → invalid SP/PC → CPU Lockup → brick.
         *
         * The subsequent copy step (flash_program_dword) has rigorous per-dword
         * two-pass read-back verify (immediate + delayed 50µs), and the final
         * full-APP CRC-32 verification catches any actual corruption. */
    }

    /* Step 4: Erase done — 4 slow blinks */
    led_slow_blink(4);

    uart_puts("[BL] App pages erased, copying staging→app...\r\n");
    led_on();  /* LED ON during copy — if it hangs here, LED stays ON permanently */

    /* 5. Copy staging → app, 8 bytes at a time, with read-back verify */
    {
        int copy_ok = 1;
        for (offset = 0; offset < fw_size; offset += 8) {
            uint32_t *src = (uint32_t *)(STAGING_START + offset);
            uint32_t w0 = src[0], w1 = src[1];
            if (flash_program_dword(APP_START + offset, w0, w1) != 0) {
                uart_puts("[BL] COPY FAIL at offset="); uart_putdec(offset); uart_puts("\r\n");
                copy_ok = 0;
                break;
            }
        }

        if (!copy_ok) {
            bl_log_set(2, 6, offset, retry_cnt);
            flash_lock();
            retry_cnt++;
            if (retry_cnt < MAX_COPY_RETRIES) {
                flash_erase_page(FLAG_PAGE / PAGE_SIZE);
                flag_write_dword(FLAG_OFF_MAGIC, MAGIC_OTA_READY, OTA_STATE_READY);
                flag_write_dword(FLAG_OFF_SIZE, fw_size, retry_cnt);
                flash_lock();
                uart_puts("[BL] Retry — reboot\r\n");
                /* Don't use SCB_AIRCR here — Heisenbug (flash ops + reset
                 * from flash = stuck).  Fast-blink LED forever: user power-cycles
                 * to retry.  Bootloader re-reads flag and retries the copy. */
                while (1) {
                    led_on();  delay_ms(100);
                    led_off(); delay_ms(100);
                }
            } else {
                uart_puts("[BL] All retries exhausted → SOS\r\n");
                erase_staging_and_flag();
                flash_lock();
                led_sos();
            }
        }
        uart_puts("[BL] Copy complete\r\n");
    }
    led_off(); /* Copy done — LED OFF */

    /* Step 5: Copy done — 5 slow blinks */
    led_slow_blink(5);

    /* Step 6: Full CRC-32 verification of copied APP against staging.
     * CRC covers the ENTIRE firmware (not just first 256 bytes).
     * Physical flash read (DCEN=ICEN=0) to bypass ART cache and
     * detect marginal cells that pass immediate read-back but fail
     * after the programming charge dissipates (seconds later).
     *
     * LED is ON during CRC — if it stays ON, the device hung here. */
    uart_puts("[BL] Verifying app CRC...\r\n");
    bl_log_set(1, 6, 0, 0);
    {
        uint32_t flag_crc  = *(volatile uint32_t *)(FLAG_PAGE + FLAG_OFF_CRC32);
        uint32_t app_crc;
        int crc_ok = 0;

        led_on();
        {
            uint32_t _save = FLASH_ACR;
            FLASH_ACR &= ~((1UL << 9) | (1UL << 10));  /* DCEN=0, ICEN=0 */
            __asm { dsb } __asm { isb }

            /* Log APP VTOR with cache off */
            {
                volatile uint32_t *_av = (volatile uint32_t *)APP_START;
                bl_log_word(_av[0]); bl_log_word(_av[1]);
            }

            /* Feed IWDG before CRC computation (~2ms for 32KB) */
            IWDG_KR = 0xAAAA;
            app_crc = hw_crc32((const uint8_t *)APP_START, fw_size);

            bl_log_word(app_crc);
            bl_log_word(flag_crc);

            if (app_crc == flag_crc) {
                crc_ok = 1;
            }

            FLASH_ACR = _save; __asm { dsb } __asm { isb }
        }
        led_off();

        if (!crc_ok) {
            uart_puts("[BL] APP CRC MISMATCH app="); uart_puthex32(app_crc);
            uart_puts(" exp="); uart_puthex32(flag_crc);
            uart_puts("\r\n");
            bl_log_set(2, 7, app_crc, flag_crc);
            flash_lock();
            retry_cnt++;
            if (retry_cnt < MAX_COPY_RETRIES) {
                flash_erase_page(FLAG_PAGE / PAGE_SIZE);
                flag_write_dword(FLAG_OFF_MAGIC, MAGIC_OTA_READY, OTA_STATE_READY);
                flag_write_dword(FLAG_OFF_SIZE, fw_size, retry_cnt);
                flash_lock();
                uart_puts("[BL] Retry after CRC mismatch\r\n");
                led_error_blink();
                SCB_AIRCR = (0x5FAUL << 16U) | 0x04U;
                __asm { dsb } __asm { isb }
                while (1) { }
            }
            uart_puts("[BL] All retries exhausted — SOS\r\n");
            erase_staging_and_flag();
            flash_lock();
            led_sos();
        }
    }
    /* Step 6: CRC verify OK — 6 slow blinks */
    led_slow_blink(6);
    uart_puts("[BL] App CRC verified OK\r\n");

    uart_puts("[BL] SUCCESS → boot new app\r\n");

    /* P2: Write STATE=BOOTING — keep staging intact for rollback.
     * App must call gw_ota_confirm_boot() after successful init to clear
     * the flag and erase staging.  If app crashes before confirming,
     * bootloader re-copies from staging (up to MAX_BOOT_RETRIES = 3 times).
     * After 3 failed boot attempts → SOS (firmware is defective). */
    {
        uint32_t saved_crc = *(volatile uint32_t *)(FLAG_PAGE + FLAG_OFF_CRC32);
        flash_erase_page(FLAG_PAGE / PAGE_SIZE);
        flag_write_dword(FLAG_OFF_MAGIC, MAGIC_OTA_READY, OTA_STATE_BOOTING);
        flag_write_dword(FLAG_OFF_SIZE, fw_size, 0);      /* RETRY=0: boot counter start */
        flag_write_dword(FLAG_OFF_CRC32, saved_crc, 0);
    }
    flash_lock();

    /* === ALL 6 STEPS PASSED — 5s solid LED ===
     * After OTA, watch the LED: 1-2-3-4-5-6 slow blinks,
     * then 5 seconds solid ON.  If you see this, the
     * bootloader completed successfully and is about
     * to boot the new app. */
    led_on();  delay_ms(5000);
    led_off(); delay_ms(500);

    /* Fall through to boot_app */

boot_app:
    led_off();  /* Bootloader exiting — LED OFF */

    /* Read APP vector table with DCEN/ICEN disabled.
     * Double duty: (a) get SP/PC for VTOR validation,
     *              (b) flush ART cache by reading through D-bus. */
    {
        uint32_t sp, pc;
        uint32_t _acr = FLASH_ACR;
        FLASH_ACR &= ~((1UL << 9) | (1UL << 10));  /* DCEN=0, ICEN=0 */
        __asm { dsb } __asm { isb }
        {
            volatile uint32_t *_av = (volatile uint32_t *)APP_START;
            sp = _av[0];
            pc = _av[1];
        }
        FLASH_ACR = _acr;  /* restore ART */
        __asm { dsb } __asm { isb }

        /* Validate APP VTOR before jumping.
         * If APP is erased (all 0xFF) or corrupt, jumping to an invalid
         * SP/PC causes CPU Lockup — the HardFault handler cannot push
         * its exception frame with a bad SP, and the core locks up with
         * LED OFF → impossible to diagnose.  Catch it here instead. */
        if (sp < SRAM1_START || sp > SRAM1_END ||
            pc < APP_START || pc > STAGING_START ||
            (pc & 1UL) == 0) {
            uart_puts("[BL] APP VTOR INVALID sp="); uart_puthex32(sp);
            uart_puts(" pc="); uart_puthex32(pc);
            uart_puts(" → SOS\r\n");
            bl_log_set(2, 10, sp, pc);
            led_sos();
        }

        uart_puts("[BL] Booting app at 0x08002000\r\n\r\n");
        SCB_VTOR = APP_START;
        __set_MSP(sp);
        ((void (*)(void))pc)();
        while (1) { }
    }
}
