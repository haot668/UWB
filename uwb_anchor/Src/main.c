/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : UWB Anchor firmware for DW3210 on STM32L412KBU6
  *                   Clock: HSE 8MHz -> PLL -> 80MHz SYSCLK
  *                   Debug: USART2 (PA2/PA3) 921600-8N1
  ******************************************************************************
  */
#include <main.h>
#include <stm32l4xx_hal.h>
#include <port.h>
#include "examples_defines.h"
#include <example_selection.h>

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef  hspi1;
UART_HandleTypeDef huart2;

#define UNIT_TEST 0

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_NVIC_Init(void);
static void MX_RTC_Init(void);

extern example_ptr example_pointer;
extern int unit_test_main(void);

/*! --------------------------------------------------------------------------
 * @brief Simple software LFSR random for DW3000 (no hardware RNG on L412).
 */
static uint32_t lfsr_seed = 0xDEADBEEF;
void sw_rng_init(void)
{
    lfsr_seed = HAL_GetTick() ^ 0x12345678;
}

uint32_t sw_rng_get(void)
{
    uint32_t bit;
    bit = ((lfsr_seed >> 31) ^ (lfsr_seed >> 21) ^ (lfsr_seed >> 1) ^ (lfsr_seed >> 0)) & 1;
    lfsr_seed = (lfsr_seed << 1) | bit;
    return lfsr_seed;
}

void test_run_info(unsigned char *data)
{
    HAL_UART_Transmit(&huart2, data, strlen((const char *)data), 100);
    HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, 100);
}

int main(void)
{
    /* Quick LED blink — proves main() reached after OTA boot */
    {   volatile uint32_t _d;
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
        GPIOA->MODER = (GPIOA->MODER & ~(3UL<<10)) | (1UL<<10);
        GPIOA->BSRR = (1UL<<5);
        for (_d=0;_d<4000000;_d++){__NOP();}
        GPIOA->BSRR = (1UL<<21);
    }

    HAL_Init();
    SystemClock_Config();

    sw_rng_init();  /* LFSR seed for DW3000 random — needs HAL_GetTick() */

    build_examples();

    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_SPI1_Init();
    MX_NVIC_Init();
    MX_RTC_Init();

    /* RTC 时钟精度测试：打印初始化后的时间 */
    {
        char dbg[64];
        /* 直接读 RTC 寄存器，避免 HAL 重复定义问题 */
        uint32_t tr = RTC->TR;  /* Time register */
        uint32_t dr = RTC->DR;  /* Date register */
        uint8_t hour   = (uint8_t)((tr & (RTC_TR_HT | RTC_TR_HU)) >> RTC_TR_HU_Pos);
        uint8_t minute = (uint8_t)((tr & (RTC_TR_MNT | RTC_TR_MNU)) >> RTC_TR_MNU_Pos);
        uint8_t second = (uint8_t)((tr & (RTC_TR_ST | RTC_TR_SU)) >> RTC_TR_SU_Pos);
        uint8_t year   = (uint8_t)((dr & (RTC_DR_YT | RTC_DR_YU)) >> RTC_DR_YU_Pos);
        uint8_t month  = (uint8_t)((dr & (RTC_DR_MT | RTC_DR_MU)) >> RTC_DR_MU_Pos);
        uint8_t day    = (uint8_t)((dr & (RTC_DR_DT | RTC_DR_DU)) >> RTC_DR_DU_Pos);
        sprintf(dbg, "RTC_INIT %02x:%02x:%02x 20%02x/%02x/%02x\r\n",
                hour, minute, second, year, month, day);
        HAL_UART_Transmit(&huart2, (uint8_t*)dbg, strlen(dbg), 100);
    }

    port_DisableEXT_IRQ();
    setup_DWICRSTnIRQ(0);

    Sleep(2000); // 2s delay for DW3210 power-up stabilization

    if (UNIT_TEST)
    {
        unit_test_main();
    }
    else
    {
        example_pointer();
    }

    while (1)
    {
    }
}

/**
  * @brief System Clock Configuration for STM32L412KB
  *        Primary: HSE (8MHz TCXO) -> PLL (x20 /2) -> 80MHz SYSCLK
  *        Fallback: MSI (4MHz) -> PLL (x40 /2) -> 80MHz SYSCLK
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    HAL_StatusTypeDef ret;

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    /*--- Enable LSE (32.768 kHz) for RTC and low-power timers ---*/
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;  /* 32.768kHz 晶振（PC14-PC15） */
    ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
    if (ret != HAL_OK) {
        /* LSE 启动失败，继续用内部 LSI（RTC 走时精度会下降）*/
        /* 警告：板子上 LSE 晶振可能虚焊或损坏 */
    }

    /*--- Try HSE first (8MHz external TCXO) ---*/
    /* 注意：HSE 配置会覆盖 OscillatorType，但 LSE 已经 ON，不会被关闭 */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.MSIState = RCC_MSI_OFF;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 20;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);

    if (ret != HAL_OK)
    {
        /*--- HSE failed, fall back to MSI (4MHz internal) ---*/
        RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
        RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
        RCC_OscInitStruct.MSIState = RCC_MSI_ON;
        RCC_OscInitStruct.MSICalibrationValue = 0;
        RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
        RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
        RCC_OscInitStruct.PLL.PLLM = 1;
        RCC_OscInitStruct.PLL.PLLN = 40;
        RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
        RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
        if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        {
            _Error_Handler(__FILE__, __LINE__);
        }
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /*--- Configure RTC clock source to LSE ---*/
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

static void MX_NVIC_Init(void)
{
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    /* DW3000 IRQ (PA12 → EXTI12 → EXTI15_10_IRQn).
     * Priority 0 = highest (same as SysTick). Must preempt all other
     * interrupts so UWB frame RX is never delayed — a missed SFD leads
     * to frame loss and PREPARE/COORD failures. */
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
}

static void MX_RTC_Init(void)
{
    /* 直接操作 RTC 寄存器，绕过 HAL 的重复定义问题 */
    /* LSE 32.768kHz 已由 SystemClock_Config 配置为 RTC 时钟源 */
    /* STM32L412 RTC: ICSR(0x0C) 有 INIT/INITF/RSF, CR(0x18) 有 INIT 控制 */
    volatile uint32_t count = 0;

    /* 解锁写保护 */
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;

    /* 进入初始化模式 */
    RTC->ICSR |= RTC_ICSR_INIT;  /* SET INIT = 1 */
    /* 等待 INITF=1 */
    while (!(RTC->ICSR & RTC_ICSR_INITF) && ++count < 100000) { }
    if (count >= 100000) { return; }  /* 超时保护 */

    /* 设置预分频: 32768/128/256 = 1Hz */
    /* PRER[31:16] = PREDIV_A-1 = 127, PRER[14:0] = PREDIV_S-1 = 255 */
    RTC->PRER = (127UL << 16) | 255UL;

    /* 24 小时制，无夏令时 */
    RTC->CR &= ~RTC_CR_FMT;  /* FMT=0: 24h */

    /* 退出初始化模式，允许计数 */
    RTC->ICSR &= ~RTC_ICSR_INIT;  /* CLEAR INIT = 0 */

    /* 等待日历同步完成 (RSF=1)，LSE 稳定需要 ~2个 RTCCLK */
    count = 0;
    while (!(RTC->ICSR & RTC_ICSR_RSF) && ++count < 1000000) { }
    /* RSF 会在退出 INIT 后由硬件置 1，表示日历已同步 */

    /* 锁定写保护 */
    RTC->WPR = 0xFF;
}

static void MX_SPI1_Init(void)
{
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;  /* 80MHz/4 = 20MHz (DW3210 max) */
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
}

static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 921600;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(DW_RESET_GPIO_Port, DW_RESET_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DW_WAKEUP_GPIO_Port, DW_WAKEUP_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = DW_IRQn_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DW_IRQn_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = DW_RESET_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(DW_RESET_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = DW_NSS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(DW_NSS_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = DW_WAKEUP_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(DW_WAKEUP_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LED1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LED2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);
}

void HAL_UARTEx_WakeupCallback(UART_HandleTypeDef *huart)
{
    UNUSED(huart);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    UNUSED(htim);
}


void _Error_Handler(char *file, int line)
{
    UNUSED(file);
    UNUSED(line);

    __HAL_RCC_GPIOA_CLK_ENABLE();
    {
        GPIO_InitTypeDef g = {0};
        g.Pin = LED1_Pin;
        g.Mode = GPIO_MODE_OUTPUT_PP;
        g.Pull = GPIO_NOPULL;
        g.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(LED1_GPIO_Port, &g);
    }

    while (1)
    {
        volatile uint32_t i;
        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
        for (i = 0; i < 200000; i++) { __NOP(); }
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{
}
#endif
