/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program for DW3210 on STM32L412KBU6
  *                   Clock: MSI 4MHz → PLL → 80MHz SYSCLK
  *                   Debug: USART2 (PA2/PA3) 115200-8N1
  *                   No USB, no hardware RNG.
  ******************************************************************************
  */
#include <main.h>
#include <stm32l4xx_hal.h>
#include <port.h>
#include "examples_defines.h"
#include <example_selection.h>
#include "examples/ex_09b_fusion_anchor/gateway_cmd.h"

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef  hspi1;
#if defined(TEST_SS_TWR_RESPONDER) || defined(TEST_TDOA_TAG) || defined(TEST_FUSION_TAG) || defined(TEST_DOWNLINK_TAG) || defined(FUSION_USART1)
UART_HandleTypeDef huart1;
#else
UART_HandleTypeDef huart2;
#endif
TIM_HandleTypeDef  htim1;

#define UNIT_TEST 0

#if defined(TEST_FUSION_ANCHOR)
static uint8_t gw_rx_byte;  /* persistent buffer for UART RX interrupt */
#endif

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
#if defined(TEST_SS_TWR_RESPONDER) || defined(TEST_TDOA_TAG) || defined(TEST_FUSION_TAG) || defined(TEST_DOWNLINK_TAG) || defined(FUSION_USART1)
static void MX_USART1_UART_Init(void);
#else
static void MX_USART2_UART_Init(void);
#endif
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_NVIC_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

extern example_ptr example_pointer;
extern int unit_test_main(void);

/*! --------------------------------------------------------------------------
 * @brief Simple software LFSR random for DW3000 (no hardware RNG on L412).
 *        Returns a 32-bit pseudo-random value.
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

/*! --------------------------------------------------------------------------
 * @brief Send debug text via USART2 (replaces USB CDC output).
 */
void test_run_info(unsigned char *data)
{
#if defined(TEST_SS_TWR_RESPONDER) || defined(TEST_TDOA_TAG) || defined(TEST_FUSION_TAG) || defined(TEST_DOWNLINK_TAG) || defined(FUSION_USART1)
    HAL_UART_Transmit(&huart1, data, strlen((const char *)data), 100);
    HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, 100);
#else
    HAL_UART_Transmit(&huart2, data, strlen((const char *)data), 100);
    HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, 100);
#endif
}

int main(void)
{
    build_examples();
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
#if defined(TEST_SS_TWR_RESPONDER) || defined(TEST_TDOA_TAG) || defined(TEST_FUSION_TAG) || defined(TEST_DOWNLINK_TAG) || defined(FUSION_USART1)
    MX_USART1_UART_Init();
#else
    MX_USART2_UART_Init();
#endif
    MX_SPI1_Init();
    MX_TIM1_Init();
    MX_NVIC_Init();

#if defined(TEST_FUSION_ANCHOR)
    /* Start UART interrupt-driven reception for gateway AT commands */
    HAL_UART_Receive_IT(&huart2, &gw_rx_byte, 1);
#endif

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
  *        MSI (4MHz) → PLL (x40 /2) → 80MHz SYSCLK
  *        HCLK = 80MHz, APB1 = 80MHz, APB2 = 80MHz
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* Voltage scaling for 80MHz: Range 1 */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Enable MSI and configure to 4MHz, then PLL: 4*40/2 = 80MHz */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;       /* 4 MHz */
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 40;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;             /* /2 */
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
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
#if defined(TEST_SS_TWR_RESPONDER) || defined(TEST_TDOA_TAG) || defined(TEST_FUSION_TAG) || defined(TEST_DOWNLINK_TAG) || defined(FUSION_USART1)
    /* USART1 interrupt */
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
#else
    /* USART2 interrupt */
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
#endif

    /* TIM1 CC interrupt */
    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
}

/* SPI1: PB3=SCK, PB4=MISO, PB5=MOSI, PA15=NSS (SW)
 * Clock: APB2 = 80MHz, initial prescaler /32 = 2.5MHz */
static void MX_SPI1_Init(void)
{
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
}

/* TIM1 - used for DW3210 timestamp capture */
static void MX_TIM1_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_SlaveConfigTypeDef sSlaveConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 65535;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
    htim1.Init.RepetitionCounter = 0;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
    sSlaveConfig.InputTrigger = TIM_TS_ITR0;
    if (HAL_TIM_SlaveConfigSynchronization(&htim1, &sSlaveConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sConfigOC.OCMode = TIM_OCMODE_TIMING;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    HAL_TIM_MspPostInit(&htim1);
}

/* USART1: PA9=TX, PA10=RX, 115200-8N1 */
#if defined(TEST_SS_TWR_RESPONDER) || defined(TEST_TDOA_TAG) || defined(TEST_FUSION_TAG) || defined(TEST_DOWNLINK_TAG) || defined(FUSION_USART1)
static void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
}
#endif

/* USART2: PA2=TX, PA3=RX, 115200-8N1 */
#if !defined(TEST_SS_TWR_RESPONDER) && !defined(TEST_TDOA_TAG) && !defined(TEST_FUSION_TAG) && !defined(TEST_DOWNLINK_TAG) && !defined(FUSION_USART1)
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
#endif

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* DW_RESET (PB6) output low (reset active) */
    HAL_GPIO_WritePin(DW_RESET_GPIO_Port, DW_RESET_Pin, GPIO_PIN_RESET);

    /* DW_NSS (PA15) output high (deselected) */
    HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_SET);

    /* DW_WAKEUP (PB7) output low */
    HAL_GPIO_WritePin(DW_WAKEUP_GPIO_Port, DW_WAKEUP_Pin, GPIO_PIN_RESET);

    /* LED1 (PA5), LED2 (PA11) output low */
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

    /* DW_IRQn (PA12) EXTI rising with pull-down */
    GPIO_InitStruct.Pin = DW_IRQn_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(DW_IRQn_GPIO_Port, &GPIO_InitStruct);

    /* DW_RESET (PB6) output push-pull */
    GPIO_InitStruct.Pin = DW_RESET_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(DW_RESET_GPIO_Port, &GPIO_InitStruct);

    /* DW_NSS (PA15) output push-pull with pull-up */
    GPIO_InitStruct.Pin = DW_NSS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(DW_NSS_GPIO_Port, &GPIO_InitStruct);

    /* DW_WAKEUP (PB7) output push-pull */
    GPIO_InitStruct.Pin = DW_WAKEUP_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(DW_WAKEUP_GPIO_Port, &GPIO_InitStruct);

    /* LED1 (PA5), LED2 (PA11) output push-pull */
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* DW3210 timestamp capture callback would go here */
    UNUSED(htim);
}

void HAL_UARTEx_WakeupCallback(UART_HandleTypeDef *huart)
{
    UNUSED(huart);
}

#if defined(TEST_FUSION_ANCHOR)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        gateway_uart_rx_byte(gw_rx_byte);
        HAL_UART_Receive_IT(&huart2, &gw_rx_byte, 1);
    }
}
#endif

void _Error_Handler(char *file, int line)
{
    UNUSED(file);
    UNUSED(line);

    /* Rapid blink LED1 to indicate fatal error */
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
