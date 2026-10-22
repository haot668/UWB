/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : UWB Tag firmware for DW3210 on STM32L412KBU6
  *                   Clock: HSE 8MHz -> PLL -> 80MHz SYSCLK
  *                   Debug: USART2 (PA2/PA3) 115200-8N1
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

extern example_ptr example_pointer;
extern int unit_test_main(void);

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
    HAL_Init();
    SystemClock_Config();

    sw_rng_init();  /* LFSR seed for DW3000 random — needs HAL_GetTick() */

    build_examples();

    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_SPI1_Init();
    MX_NVIC_Init();

    port_DisableEXT_IRQ();
    setup_DWICRSTnIRQ(0);

    Sleep(2000);

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

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    HAL_StatusTypeDef ret;

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    /*--- Try HSE first (8MHz external TCXO) ---*/
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

    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

static void MX_NVIC_Init(void)
{
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    /* DW3000 IRQ (PA12 → EXTI12 → EXTI15_10_IRQn).
     * Only set priority; the DW3000 driver enables/disables this IRQ
     * via port_EnableEXT_IRQ() / port_DisableEXT_IRQ(). */
    /* DW3000 IRQ (PA12 → EXTI12 → EXTI15_10_IRQn).
     * Priority 0 = highest (same as SysTick). Must preempt all other
     * interrupts so UWB frame RX is never delayed. */
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
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
    huart2.Init.BaudRate = 115200;
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
