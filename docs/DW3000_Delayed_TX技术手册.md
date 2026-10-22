# DW3000/DW3210 Delayed TX 技术手册

> **芯片**: DW3210 (DW3000变体)  
> **内容**: Delayed TX 使用说明、API、寄存器操作  
> **版本**: V1.0

---

## 一、Delayed TX 概述

Delayed TX 是DW3000系列芯片的重要功能，允许用户在指定时间发送UWB帧。这对于TDOA定位系统中的基站间时间同步至关重要。

### 1.1 核心概念

`
┌─────────────────────────────────────────────────────────────────────┐
│                    Delayed TX 时间基准分类                            │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  1. DWT_START_TX_DELAYED (DX_TIME模式)                             │
│     TX时间 = DX_TIME寄存器的值                                       │
│                                                                     │
│  2. DWT_START_TX_DLY_REF (参考时间模式)                           │
│     TX时间 = DREF_TIME + DX_TIME                                    │
│                                                                     │
│  3. DWT_START_TX_DLY_RS (RX时间戳模式)                            │
│     TX时间 = RX_TIME_0 + DX_TIME                                    │
│                                                                     │
│  4. DWT_START_TX_DLY_TS (TX时间戳模式)                            │
│     TX时间 = TX_TIME_LO + DX_TIME                                   │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
`

---

## 二、关键寄存器说明

### 2.1 时间寄存器一览

| 寄存器 | 地址 | 长度 | 说明 |
|--------|------|------|------|
| **DX_TIME** | 0x2C | 4字节 | 延迟TX时间设置，存储bits[39:8] |
| **DREF_TIME** | 0x30 | 4字节 | 参考时间基准 |
| **RX_TIME_0** | 0x64 | 4字节 | 最近一次RX帧的时间戳 |
| **TX_TIME_LO** | 0x74 | 4字节 | 最近一次TX帧的时间戳 |
| **TX_TIME_RAW** | 0x10000 | 4字节 | TX原始时间戳 |

### 2.2 DX_TIME 寄存器详解

`
寄存器地址: 0x2C (DX_TIME_ID)
长度: 4字节 (32位)
存储格式: bits[39:8]，即40位时间戳的低32位，右移8位

┌─────────────────────────────────────────────────────────────────┐
│ DX_TIME 寄存器位定义                                             │
├─────────────────────────────────────────────────────────────────┤
│ Bit[31:1] │ DX_TIME: 延迟时间 bits[39:8]                       │
│ Bit[0]    │ 保留，始终为0                                        │
└─────────────────────────────────────────────────────────────────┘

重要: 设置DX_TIME时，传入的值应该是 (实际40位时间 >> 8)
`

### 2.3 状态寄存器相关位

| 状态位 | 地址偏移 | 说明 |
|--------|----------|------|
| **HPDWARN** | SYS_STATUS[27] | 半周期警告，延迟超过半个周期 |
| **TXERR** | SYS_STATE_LO | TX错误状态 |
| **SYS_STATE_TXERR** | 0xA0000 | TX错误状态值 |

---

## 三、API 函数说明

### 3.1 设置延迟时间

`c
//============================================================================
// 函数: dwt_setdelayedtrxtime
// 头文件: deca_device_api.h (line 1091)
//============================================================================

/*!
 * @brief This API function configures the delayed transmit time or the delayed RX on time
 *
 * @param starttime - uint32_t - the delayed TX time to load into DX_TIME register
 *                    (bits[39:8] of the 40-bit timestamp, i.e. right-shifted by 8)
 *
 * @return none
 */
void dwt_setdelayedtrxtime(uint32_t starttime);
`

**实现代码** (deca_device.c line 3651):

`c
void dwt_setdelayedtrxtime(uint32_t starttime)
{
    dwt_write32bitoffsetreg(DX_TIME_ID, 0, starttime); // Note: bit 0 of this register is ignored
}
`

**使用示例**:

`c
// 计算期望的TX时间 (40位时间戳)
uint64_t scheduled_tx_time = prev_tx_ts + period_dtu;

// 设置延迟时间 (右移8位，因为DX_TIME存储bits[39:8])
dwt_setdelayedtrxtime((uint32_t)(scheduled_tx_time >> 8));
`

### 3.2 启动发送

`c
//============================================================================
// 函数: dwt_starttx
// 头文件: deca_device_api.h (line 1065)
//============================================================================

/*!
 * @brief This call initiates the transmission
 *
 * @param mode - uint8_t - TX模式组合，可以是以下值的组合:
 *
 *   基础模式:
 *   - DWT_START_TX_IMMEDIATE   (0x00)  立即发送
 *   - DWT_START_TX_DELAYED     (0x01)  延迟发送 (基于DX_TIME)
 *   - DWT_START_TX_DLY_REF     (0x04)  延迟发送 (基于DREF_TIME+DX_TIME)
 *   - DWT_START_TX_DLY_RS      (0x08)  延迟发送 (基于RX_TIME+DX_TIME)
 *   - DWT_START_TX_DLY_TS      (0x10)  延迟发送 (基于TX_TIME+DX_TIME)
 *   - DWT_START_TX_CCA         (0x20)  CCA模式下发送
 *
 *   修饰符:
 *   - DWT_RESPONSE_EXPECTED    (0x02)  TX完成后自动开启RX
 *
 * @return int - DWT_SUCCESS(0)成功, DWT_ERROR(-1)失败
 */
int dwt_starttx(uint8_t mode);
`

### 3.3 快速命令 (Fast Commands)

`c
//============================================================================
// 快速命令定义 (deca_device_api.h line 87-103)
//============================================================================

// Delayed TX 命令
#define CMD_DTX           0x3   // 延迟TX，RMARKER时间=DX_TIME
#define CMD_DTX_W4R      0xD   // 延迟TX，TX完成后开启RX
#define CMD_DTX_REF       0x9   // 延迟TX，RMARKER时间=DREF_TIME+DX_TIME
#define CMD_DTX_REF_W4R  0x10   // 延迟TX，RMARKER时间=DREF_TIME+DX_TIME，TX完成后开启RX
#define CMD_DTX_RS        0x7   // 延迟TX，RMARKER时间=RX_TIME+DX_TIME
#define CMD_DTX_RS_W4R    0xF   // 延迟TX，RMARKER时间=RX_TIME+DX_TIME，TX完成后开启RX
#define CMD_DTX_TS        0x5   // 延迟TX，RMARKER时间=TX_TIME+DX_TIME
#define CMD_DTX_TS_W4R    0xE   // 延迟TX，RMARKER时间=TX_TIME+DX_TIME，TX完成后开启RX

// 立即TX命令
#define CMD_TX            0x1   // 立即TX
#define CMD_TX_W4R        0xC   // 立即TX，TX完成后开启RX

// 控制命令
#define CMD_TXRXOFF       0x0   // 关闭TX/RX，进入IDLE
#define CMD_CCA_TX        0x2   // CCA模式下TX
#define CMD_CCA_TX_W4R    0xB   // CCA模式下TX，TX完成后开启RX
`

---

## 四、Delayed TX 工作流程

### 4.1 标准 Delayed TX (DWT_START_TX_DELAYED)

`
┌─────────────────────────────────────────────────────────────────────┐
│                    Delayed TX 流程图                                 │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  1. 计算期望TX时间                                                  │
│     scheduled = prev_tx_ts + period                                 │
│              ↓                                                       │
│  2. 设置DX_TIME寄存器                                              │
│     dwt_setdelayedtrxtime(scheduled >> 8)                         │
│              ↓                                                       │
│  3. 启动延迟TX                                                     │
│     dwt_starttx(DWT_START_TX_DELAYED)                              │
│              ↓                                                       │
│  4. 检查HPDWARN状态                                                │
│     ├─ HPDWARN=0: 延迟时间有效                                      │
│     └─ HPDWARN=1: 延迟超过半周期，返回DWT_ERROR                     │
│              ↓                                                       │
│  5. 检查TX错误状态                                                  │
│     ├─ SYS_STATE=0xA0000: TX错误，取消                             │
│     └─ 其他: TX正常完成                                             │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
`

### 4.2 代码实现

`c
//============================================================================
// Delayed TX 完整实现示例
//============================================================================

int delayed_tx_with_retry(uint64_t period_dtu)
{
    static uint64_t prev_tx_ts_ext = 0;  // 64位扩展时间戳
    int ret;
    uint8_t dly_mode = DWT_START_TX_DELAYED;

    // 计算期望的TX时间
    uint64_t scheduled_tx = prev_tx_ts_ext + period_dtu;

    // 设置延迟时间 (右移8位)
    dwt_setdelayedtrxtime((uint32_t)(scheduled_tx >> 8));

    // 尝试延迟TX
    ret = dwt_starttx(dly_mode);

    if (ret != DWT_SUCCESS)
    {
        // 延迟失败，fallback到立即发送
        dwt_starttx(DWT_START_TX_IMMEDIATE);
        return -1;
    }

    // 延迟成功，更新prev_tx_ts
    // 注意: 实际TX时间戳在TX完成后通过dwt_readtxtimestamp()读取
    uint64_t actual_ts = get_tx_timestamp_u64();

    // 处理40位时间戳回绕，扩展到64位
    if (actual_ts < (prev_tx_ts_ext & 0xFFFFFFFFFFULL))
    {
        // 发生了回绕，增加高24位
        prev_tx_ts_ext += 0x1000000000ULL;
    }
    prev_tx_ts_ext = (prev_tx_ts_ext & ~0xFFFFFFFFFFULL) | actual_ts;

    return 0;
}
`

---

## 五、Delayed TX 模式详解

### 5.1 四种延迟模式对比

| 模式 | 宏定义 | 时间基准 | 精度 | 适用场景 |
|------|--------|----------|------|---------|
| **DX_TIME** | DWT_START_TX_DELAYED | DX_TIME寄存器 | ±15ps | 基站周期性广播 |
| **DLY_REF** | DWT_START_TX_DLY_REF | DREF_TIME+DX_TIME | ±15ps | 需要参考时间的同步 |
| **DLY_RS** | DWT_START_TX_DLY_RS | RX_TIME+DX_TIME | ±15ps | 响应RX事件 |
| **DLY_TS** | DWT_START_TX_DLY_TS | TX_TIME+DX_TIME | ±15ps | 连续TX序列 |

### 5.2 DX_TIME模式 (推荐用于粮库场景)

**公式**: TX_time = DX_TIME

**使用步骤**:
`c
// 1. 计算下一帧时间
uint64_t next_tx_time = last_tx_time + 50ms_dtu;  // 50ms周期

// 2. 设置DX_TIME (右移8位)
dwt_setdelayedtrxtime((uint32_t)(next_tx_time >> 8));

// 3. 启动延迟TX
int ret = dwt_starttx(DWT_START_TX_DELAYED);

if (ret == DWT_ERROR)
{
    // HPWARN置位，延迟超过半周期
    // Fallback到立即发送
    dwt_starttx(DWT_START_TX_IMMEDIATE);
}
`

### 5.3 DLY_TS模式 (TX时间戳基准)

**公式**: TX_time = TX_TIME_LO + DX_TIME

**特点**: 相对于当前TX时间戳加偏移，适合连续广播

**使用步骤**:
`c
// 1. 读取当前TX时间戳
uint32_t current_tx_lo = dwt_readtxtimestamplo32();

// 2. 计算偏移 (例如50ms = 3,194,900 DTU)
uint32_t offset = 3194900;  // 约50ms @ 63.898MHz

// 3. 设置DX_TIME为偏移值
dwt_setdelayedtrxtime(offset);

// 4. 启动基于TX时间戳的延迟TX
dwt_starttx(DWT_START_TX_DLY_TS);
`

---

## 六、错误处理与状态检测

### 6.1 HPDWARN (半周期警告)

`c
//============================================================================
// HPDWARN检测
//============================================================================

// HPDWARN = bit[27] of SYS_STATUS
// 当延迟时间超过半个周期时置位

// 方法1: 通过SYS_STATUS寄存器
uint32_t sys_status = dwt_read32bitreg(SYS_STATUS_ID);
if (sys_status & SYS_STATUS_HPDWARN_BIT_MASK)
{
    // 延迟超过半周期，TX将被取消或立即执行
}

// 方法2: 在dwt_starttx()中检查
// dwt_starttx()内部会检查HPDWARN，如果置位则返回DWT_ERROR
`

### 6.2 TX错误状态检测

`c
//============================================================================
// TX错误状态检测
//============================================================================

// 检查SYS_STATE_LO寄存器
uint32_t sys_state = dwt_read32bitreg(SYS_STATE_LO_ID);

if (sys_state == DW_SYS_STATE_TXERR)  // 0xA0000
{
    // TX错误发生，可能的原因:
    // 1. 延迟时间已过
    // 2. 设备未处于IDLE状态
    // 3. 帧太长
    
    // 处理: 关闭TX/RX，重新初始化
    dwt_writefastCMD(CMD_TXRXOFF);
}
`

### 6.3 完整的错误处理示例

`c
//============================================================================
// 完整的Delayed TX错误处理
//============================================================================

typedef struct {
    uint32_t hp_warn_count;   // 半周期警告计数
    uint32_t tx_err_count;    // TX错误计数
    uint32_t success_count;   // 成功计数
} DlyTxStats;

int safe_delayed_tx(uint64_t scheduled_time, DlyTxStats *stats)
{
    int ret;
    uint8_t status[4];

    // 1. 设置延迟时间
    dwt_setdelayedtrxtime((uint32_t)(scheduled_time >> 8));

    // 2. 启动延迟TX
    ret = dwt_starttx(DWT_START_TX_DELAYED);

    // 3. 读取状态
    dwt_readfromdevice(SYS_STATUS_ID, 0, 4, status);

    // 4. 检查HPDWARN (bit 3 of byte 3)
    if (status[3] & 0x80)  // HPDWARN = bit[27]
    {
        stats->hp_warn_count++;
        dwt_writefastCMD(CMD_TXRXOFF);
        return -1;  // 延迟时间太远
    }

    // 5. 检查TX错误
    uint32_t sys_state = dwt_read32bitreg(SYS_STATE_LO_ID);
    if (sys_state == 0xA0000)  // TXERR state
    {
        stats->tx_err_count++;
        dwt_writefastCMD(CMD_TXRXOFF);
        return -2;  // TX错误
    }

    // 6. 成功
    stats->success_count++;
    return 0;
}
`

---

## 七、时间戳转换

### 7.1 时间戳格式

`
┌─────────────────────────────────────────────────────────────────────┐
│                    DW3000 时间戳格式                                 │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  内部计数器: 40位 @ 63.898 GHz                                     │
│  周期: 约4.3秒 (2^40 / 63.898e9)                                  │
│                                                                     │
│  寄存器存储:                                                        │
│  - DX_TIME (0x2C): bits[39:8]，32位                              │
│  - TX_TIME_LO (0x74): bits[31:0]，32位                           │
│  - RX_TIME_0 (0x64): bits[31:0]，32位                            │
│                                                                     │
│  API返回:                                                           │
│  - dwt_readtxtimestamp(): 5字节，bits[39:0]                       │
│  - dwt_readtxtimestamplo32(): bits[31:0]                         │
│  - dwt_readtxtimestamphi32(): bits[39:32]                        │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
`

### 7.2 时间戳读写函数

`c
//============================================================================
// 时间戳读取函数 (shared_functions.c)
//============================================================================

// 读取完整40位TX时间戳 (返回64位)
uint64_t get_tx_timestamp_u64(void)
{
    uint8_t ts_tab[5];
    uint64_t ts = 0;
    int8_t i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

// 读取SYS_TIME (40位扩展到64位)
uint64_t get_sys_timestamp_u64(void)
{
    // SYS_TIME寄存器存储bits[39:8]，需要左移8位
    return ((uint64_t)dwt_read32bitreg(SYS_TIME_ID)) << 8;
}

//============================================================================
// 时间戳写入函数 (用于DX_TIME)
//============================================================================

// 设置延迟时间 (输入40位，右移8位后写入)
void set_delayed_tx_time(uint64_t full_40bit_time)
{
    // DX_TIME存储bits[39:8]，即右移8位
    uint32_t dx_value = (uint32_t)(full_40bit_time >> 8);
    dwt_setdelayedtrxtime(dx_value);
}
`

---

## 八、粮库场景应用

### 8.1 基站广播时序

`
┌─────────────────────────────────────────────────────────────────────┐
│  粮库场景: 50ms周期广播 (20Hz)                                     │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  每帧间隔: 50ms = 3,194,900 DTU @ 63.898MHz                       │
│                                                                     │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │                                                              │   │
│  │  prev_tx ──────► next_tx                                    │   │
│  │    │               │                                        │   │
│  │    │    +50ms     │                                        │   │
│  │    │               │                                        │   │
│  │  实际TX时间戳    DX_TIME设置值                              │   │
│  │                                                              │   │
│  └──────────────────────────────────────────────────────────────┘   │
│                                                                     │
│  代码实现:                                                          │
│  prev_tx_ext += 3194900;  // 50ms步进                           │
│  dwt_setdelayedtrxtime(prev_tx_ext >> 8);                        │
│  dwt_starttx(DWT_START_TX_DELAYED);                               │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
`

### 8.2 推荐配置

`c
//============================================================================
// 粮库场景Delayed TX配置
//============================================================================

// 时间常数
#define UUS_TO_DWT_TIME    63898       // 1微秒 = 63898 DTU
#define PERIOD_50MS_DTU    (50 * 1000 * UUS_TO_DWT_TIME)  // 50ms = 3,194,900 DTU

// Delayed TX参数
#define DLY_TX_MARGIN_DTU  1000        // 1ms安全余量
#define DLY_TX_TIMEOUT_MS  10          // 超时检测

// 错误处理策略
typedef enum {
    DLY_OK = 0,
    DLY_HPWARN = -1,      // 半周期警告
    DLY_TXERR = -2,       // TX错误
    DLY_TIMEOUT = -3,     // 超时
} DlyTxRet;

// 安全的延迟发送函数
DlyTxRet anchor_delayed_broadcast(void)
{
    static uint64_t tx_ts_ext = 0;
    uint32_t status;
    int ret;

    // 计算下一帧时间
    tx_ts_ext += PERIOD_50MS_DTU;

    // 设置延迟时间
    dwt_setdelayedtrxtime((uint32_t)(tx_ts_ext >> 8));

    // 启动延迟TX
    ret = dwt_starttx(DWT_START_TX_DELAYED);

    if (ret != DWT_SUCCESS)
    {
        // 检查HPDWARN
        status = dwt_read32bitreg(SYS_STATUS_ID);
        if (status & SYS_STATUS_HPDWARN_BIT_MASK)
        {
            return DLY_HPWARN;
        }
        return DLY_TXERR;
    }

    // 读取实际TX时间戳
    uint64_t actual_ts = get_tx_timestamp_u64();

    // 更新扩展时间戳 (处理40位回绕)
    if (actual_ts < (tx_ts_ext & 0xFFFFFFFFFFULL))
    {
        tx_ts_ext += 0x1000000000ULL;  // 增加高24位
    }
    tx_ts_ext = (tx_ts_ext & ~0xFFFFFFFFFFULL) | actual_ts;

    return DLY_OK;
}
`

---

## 九、寄存器地址速查表

| 名称 | 地址 | 长度 | 说明 |
|------|------|------|------|
| DX_TIME_ID | 0x2C | 4 | 延迟TX时间 (bits[39:8]) |
| DREF_TIME_ID | 0x30 | 4 | 参考时间基准 |
| RX_TIME_0_ID | 0x64 | 4 | RX时间戳 |
| TX_TIME_LO_ID | 0x74 | 5 | TX时间戳 |
| TX_TIME_RAW_ID | 0x10000 | 4 | TX原始时间戳 |
| SYS_STATUS_ID | 0x44 | 4 | 系统状态 |
| SYS_STATE_LO_ID | 0x15 | 4 | 系统状态低 |
| SYS_TIME_ID | 0x1C | 4 | 系统时间 |

### 状态位速查

| 位 | 名称 | 说明 |
|----|------|------|
| SYS_STATUS[27] | HPDWARN | 半周期警告 |
| SYS_STATUS[1] | CP_LOCK | PLL锁定 |
| SYS_STATE_LO[16] | TXERR | TX错误 |

---

## 十、常见问题

### Q1: HPDWARN置位是什么原因？

**A**: 延迟时间超过当前时间半个周期 (~2.15秒)。这通常发生在:
- 计算的调度时间已过期
- 系统时间基准不稳定

**解决**: 检查时间戳计算逻辑，确保调度时间 > 当前时间

### Q2: TX时间戳凝固不更新？

**A**: 可能原因:
- 延迟TX失败，切换到立即TX
- TX时间戳读取时机不对（应在TX完成后立即读取）

**解决**: 在TX完成后立即读取时间戳，参考代码中的 get_tx_timestamp_u64()

### Q3: 如何选择四种延迟模式？

**A**: 
- **DWT_START_TX_DELAYED**: 推荐用于周期性广播
- **DWT_START_TX_DLY_REF**: 需要外部参考时间
- **DWT_START_TX_DLY_RS**: 响应接收事件
- **DWT_START_TX_DLY_TS**: 连续TX序列

### Q4: DX_TIME右移8位的原理？

**A**: DW3000内部时间戳是40位，但DX_TIME寄存器只存储bits[39:8]（32位），最低8位被丢弃。这是DW3000硬件设计，API要求传入右移后的值。

---

**文档版本**: V1.0  
**创建日期**: 2026-09-17  
**参考手册**: DW3000 User Manual, DW3210 API Documentation
