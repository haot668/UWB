# 粮库UWB定位系统完整技术方案

> **项目**: UWB TDOA 定位系统  
> **芯片**: DW3210 (DW3000变体)  
> **晶振**: TG2016SMN 38.4000M TCXO  
> **日期**: 2026-09-17  
> **版本**: V1.0

---

## 一、系统需求分析

### 1.1 使用场景

| 场景 | 基站数量 | 标签数量 | 输出频率 | 定位精度 | 环境特点 |
|------|---------|---------|---------|---------|---------|
| **长方形粮库** | 6个 | ≤16个 | ≥20Hz | ≤20cm | 规则空间，遮挡较少 |
| **圆筒仓** | 最多16个 | ≤16个 | ≥20Hz | ≤20cm | 环形空间，多层粮堆遮挡 |

### 1.2 技术指标要求

`
┌─────────────────────────────────────────────────────┐
│  核心指标                                           │
├─────────────────────────────────────────────────────┤
│  • 坐标输出频率: ≥ 20 Hz (机器人导航需求)          │
│  • 定位精度误差: ≤ 20 cm                           │
│  • 标签数量支持: ≤ 16个                            │
│  • 基站间时间同步方式: TDOA + Delayed TX            │
│  • 时间同步精度: < 1 ns (亚纳秒级)                 │
└─────────────────────────────────────────────────────┘
`

---

## 二、系统架构设计

### 2.1 TDOA + Clock Sync 架构

基于DW3210的TDOA（到达时间差）定位技术，结合基站间时钟同步，实现无需有线连接的无线定位系统。

`
┌─────────────────────────────────────────────────────────────────┐
│                      系统架构                                    │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   ┌─────────┐      Timesync广播        ┌─────────┐              │
│   │  Anchor │◄──────(20Hz)────────────►│  Anchor │              │
│   │   A1    │                          │   A2    │              │
│   └────┬────┘                          └────┬────┘              │
│        │                                     │                     │
│        │          无线时间同步               │                     │
│        │◄──────────────────────────────────►│                     │
│        │   factor + offset (Kalman滤波)      │                     │
│        │                                     │                     │
│        │         TDOA定位                    │                     │
│        │◄──────────────────────────────────►│                     │
│        │                                     │                     │
│   ┌────┴────┐                          ┌────┴────┐              │
│   │  Anchor │                          │  Anchor │              │
│   │   A3    │                          │   A4    │              │
│   └────┬────┘                          └────┬────┘              │
│        │                                     │                     │
│        └──────────┬──────────────────────────┘                     │
│                   │                                                │
│              ┌────▼────┐                                          │
│              │   Tag   │  ← 机器人携带                              │
│              │(标签)   │                                           │
│              └─────────┘                                           │
└─────────────────────────────────────────────────────────────────┘
`

### 2.2 拓扑部署方案

#### 长方形粮库 (6基站)

`
      [A1]────────────────[A2]────────────────[A3]
       │                                       │
       │                                       │
       │           Robot + Tag                 │
       │              ↓                       │
       │                                       │
      [A4]────────────────[A5]────────────────[A6]

  粮库尺寸示例: 40m × 30m × 8m (高)
  基站高度: 3-5m (高于粮堆表面)
  基站间距: ≤15m (保证LOS视距)
`

#### 圆筒仓 (16基站)

`
              [A1]    [A2]    [A3]    [A4]      ← 顶层 (z=9m)
             /                                  \
           [A5]                                [A6]   ← 第三层 (z=7m)
           │    [Robot + Tag]                  │
          [A7]                                  [A8]  ← 第二层 (z=5m)
           \                                  /
            [A9]   [A10]   [A11]   [A12]       ← 底层 (z=3m)

  圆筒仓直径: 20m
  多层部署应对粮堆遮挡问题
  基站均匀环形分布
`

---

## 三、UWB频道配置方案

### 3.1 推荐配置参数

`c
//============================================================================
// DW3210 UWB频道配置 - Channel 9, 128 preamble, 6.8Mbps
// 配置说明: 该配置在粮库环境（粉尘、金属干扰）表现最佳
//============================================================================
static dwt_config_t config = {
    .channel        = 9,              // 信道9，干扰少，绕射能力适中
    .preambleLen   = DWT_PLEN_128,  // 128前导码，平衡速度与距离
    .pacSize       = DWT_PAC8,      // 8-symbol PAC
    .prf           = DWT_PRF_64M,   // 64MHz脉冲重复频率
    .txCode        = 9,             // 发送前导码
    .rxCode        = 9,             // 接收前导码
    .sfdType       = 1,             // SFD类型
    .dataRate      = DWT_BR_6M8,    // 6.8Mbps高速率，减少空中碰撞
    .phrMode       = DWT_PHRMODE_STD,
    .phrRate       = DWT_PHRRATE_STD,
    .sfdTO         = (129 + 8 - 8), // SFD超时
    .stsMode       = DWT_STS_MODE_OFF,  // TDOA模式不需要STS
    .stsLen        = DWT_STS_LEN_64,
    .pdoaMode      = DWT_PDOA_M0
};

// 发射功率配置
extern dwt_txconfig_t txconfig_options = {
    .PGdly        = 0x34,           // 发射增益延迟
    .power        = 0x0F0F0F0F,     // 发射功率 (根据实际调整)
};
`

### 3.2 配置选择依据

| 参数 | 选择 | 理由 |
|------|------|------|
| 信道 | Channel 9 | 粮库环境干扰少，信号衰减适中 |
| 前导码长度 | 128 | 平衡定位速度与抗多径能力 |
| 数据速率 | 6.8Mbps | 高速率减少帧碰撞，支持20Hz输出 |
| STS模式 | 关闭 | TDOA定位不需要STS测距 |
| PRF | 64MHz | 高PRF提高时间分辨率 |

---

## 四、时间同步方案

### 4.1 当前实现分析

项目采用 **Kalman + RC 双滤波器** 架构进行时钟同步：

`
┌─────────────────────────────────────────────────────────────┐
│                  时钟同步数据流                              │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  Anchor TX:                                                 │
│    1. prev_tx_ts_ext (64位扩展，防止40位回绕)                │
│    2. 嵌入Timesync帧广播                                    │
│                                                             │
│  Tag RX:                                                    │
│    3. 接收帧，提取CFO (Carrier Frequency Offset)            │
│    4. factor = 1.0 + cfo_raw / 67108864 (DW3210分频)      │
│    5. offset_dtu = rx_local - factor * tx_remote           │
│    6. Kalman滤波factor, RC滤波offset                         │
│                                                             │
│  定位计算:                                                  │
│    TDOA[i] = to_global(sync[i], rx_local[i])              │
│            - to_global(sync[ref], rx_local[ref])           │
│                                                             │
└─────────────────────────────────────────────────────────────┘
`

### 4.2 关键常数配置

`c
//============================================================================
// 时间同步常数定义 (clock_sync.h)
//============================================================================

// DW3000 CIA clock offset寄存器分频系数
// 手册规定: factor = 1.0 + (cfo_raw / 2^26)
// 2^26 = 67,108,864
#define CS_CFO_TO_FACTOR_DIVISOR  67108864.0

// Kalman滤波器参数 (针对TCXO 38.4MHz调优)
#define CS_DEFAULT_FACTOR_PROC_VAR 5e-12    // 晶振漂移过程噪声
#define CS_DEFAULT_FACTOR_MEAS_VAR 1e-7     // CFO测量噪声
#define CS_DEFAULT_RC_ALPHA        0.30f    // RC平滑因子

// DTU (Device Time Unit) 转换
// 1 DTU = 1 / (499.2MHz × 128) ≈ 15.65 ps
#define DTU_TO_CM  0.4691f   // 时间转距离: 15.65ps × 光速 ≈ 0.4691cm
`

### 4.3 改进的时间同步算法

`c
//============================================================================
// 改进的时钟同步状态 (clock_sync_improved.h)
//============================================================================
#ifndef CLOCK_SYNC_IMPROVED_H
#define CLOCK_SYNC_IMPROVED_H

// 支持场景2的16基站追踪
#define CS_MAX_ANCHORS  16

// Kalman滤波器参数 (针对粮库环境优化)
#define CS_FACTOR_PROC_VAR    1e-11   // TCXO晶振漂移过程噪声
#define CS_FACTOR_MEAS_VAR    5e-8    // CFO测量噪声
#define CS_RC_ALPHA           0.25f   // RC平滑因子

// 异常值检测阈值
#define CS_OUTLIER_THRESHOLD      0.003   // 3ppt异常检测阈值
#define CS_MAX_OUTLIER_COUNT      3        // 连续3次异常则重置

typedef struct {
    double  factor;                  // 滤波后的factor值
    double  factor_var;              // factor方差
    double  offset_dtu;              // 滤波后的offset
    float   quality;                 // 同步质量 0..1
    bool    initialized;             // 初始化标志
    uint8_t consecutive_outliers;    // 连续异常计数
    uint64_t last_remote_ts;         // 上次远端时间戳
    uint64_t last_local_ts;          // 上次本地时间戳
    uint32_t update_count;           // 更新次数
} ImprovedClockSync;

#endif
`

### 4.4 多基站同步策略

`c
//============================================================================
// 基站分组广播调度 (fusion_anchor.c)
// 目的: 避免多基站同时广播造成碰撞
//============================================================================

// 同步周期: 50ms (20Hz)
#define DL_TIMESYNC_PERIOD_MS      50u

// 时间槽分配
typedef enum {
    SLOT_SYNC_0  = 0,    // 0-10ms:   基站组1广播
    SLOT_SYNC_1  = 10,   // 10-20ms:  基站组2广播
    SLOT_SYNC_2  = 20,   // 20-30ms:  基站组3广播
    SLOT_SYNC_3  = 30,   // 30-40ms:  基站组4广播
    SLOT_SOLVE   = 40,   // 40-50ms:  定位解算
} SyncSlot;

// 基站分组 (4组，每组4个基站)
static const uint8_t anchor_group[4][4] = {
    {0,  4,  8,  12},   // 组1: 基站0,4,8,12
    {1,  5,  9,  13},   // 组2: 基站1,5,9,13
    {2,  6,  10, 14},   // 组3: 基站2,6,10,14
    {3,  7,  11, 15},   // 组4: 基站3,7,11,15
};
`

---

## 五、定位解算方案

### 5.1 TDOA数学模型

`
┌─────────────────────────────────────────────────────────────┐
│                    TDOA定位原理                             │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  设标签位置 P(x,y,z)，基站i位置 Ai(xi,yi,zi)                │
│                                                             │
│  TDOA约束:                                                  │
│    toa_i - toa_ref = d_i - d_ref                            │
│    其中 d_i = ||P - Ai||, d_ref = ||P - A_ref||             │
│                                                             │
│  线性化方程:                                                │
│    (xi - xref)*x + (yi - yref)*y + (zi - zref)*z         │
│    = 0.5*[(xi²+yi²+zi²) - (xref²+yref²+zref²)            │
│           - (d_i² - d_ref²)]                               │
│                                                             │
│  高斯牛顿迭代求解                                           │
│                                                             │
└─────────────────────────────────────────────────────────────┘
`

### 5.2 改进的TDOA求解器

`c
//============================================================================
// 改进的TDOA求解器 (tdoa_solver.c)
//============================================================================

#define TDOA_MIN_ANCHORS     4       // 至少需要4个基站
#define TDOA_MAX_ANCHORS     16      // 最多使用16个

// RANSAC异常值剔除参数
#define RANSAC_MAX_ITERATIONS     100
#define INLIER_THRESHOLD_CM       15.0f   // 15cm内点阈值
#define MIN_INLIER_RATIO          0.6f    // 至少60%内点

typedef struct {
    float   x_cm, y_cm, z_cm;    // 求解位置
    float   rms_cm;               // RMS残差
    float   quality;              // 解算质量 0..1
    uint8_t inliers;              // 内点数量
    uint8_t outliers;             // 外点数量
} TDOASolution;

// 求解函数
int improved_tdoa_solve(
    const dl_anchor_entry_t *anchors,  // 基站位置数组
    const uint64_t *toa_dtu,           // TOA时间戳数组
    uint8_t num_anchors,               // 基站数量
    float *x_cm_out, float *y_cm_out, float *z_cm_out,
    float *rms_out,
    float *quality_out
);
`

---

## 六、场景化配置方案

### 6.1 长方形粮库配置 (6基站)

`c
//============================================================================
// 长方形粮库配置 (config_warehouse_6anchor.h)
//============================================================================

#ifndef CONFIG_WAREHOUSE_6ANCHOR_H
#define CONFIG_WAREHOUSE_6ANCHOR_H

#define WAREHOUSE_MAX_ANCHORS    6
#define WAREHOUSE_MAX_TAGS       16
#define WAREHOUSE_SYNC_PERIOD_MS 50      // 20Hz
#define WAREHOUSE_OUTPUT_RATE_HZ 20

// 基站坐标 (cm单位)
// 粮库尺寸: 40m x 30m x 8m
static const dl_anchor_entry_t warehouse_anchors[6] = {
    {.id=0, .x_cm=0,     .y_cm=0,     .z_cm=300},  // 角落A1
    {.id=1, .x_cm=4000,  .y_cm=0,     .z_cm=300},  // 角落A2
    {.id=2, .x_cm=4000,  .y_cm=3000,  .z_cm=300},  // 角落A3
    {.id=3, .x_cm=0,     .y_cm=3000,  .z_cm=300},  // 角落A4
    {.id=4, .x_cm=2000,  .y_cm=0,     .z_cm=400},  // 中间上方A5
    {.id=5, .x_cm=2000,  .y_cm=3000,  .z_cm=400},  // 中间下方A6
};

// 预期定位精度: ≤15cm (LOS环境)
#define WAREHOUSE_EXPECTED_ACCURACY_CM  15

#endif
`

### 6.2 圆筒仓配置 (16基站)

`c
//============================================================================
// 圆筒仓配置 (config_silo_16anchor.h)
//============================================================================

#ifndef CONFIG_SILO_16ANCHOR_H
#define CONFIG_SILO_16ANCHOR_H

#define SILO_MAX_ANCHORS     16
#define SILO_MAX_TAGS        16
#define SILO_SYNC_PERIOD_MS  50      // 20Hz
#define SILO_OUTPUT_RATE_HZ  20

// 基站坐标 (cm单位)
// 圆筒仓直径: 20m, 分4层环形部署
// 每层4个基站，均匀分布 (0°, 90°, 180°, 270°)
static const dl_anchor_entry_t silo_anchors[16] = {
    // 底层 (z=3m) - 基础覆盖
    {.id=0,  .x_cm=-1000, .y_cm=0,     .z_cm=300},
    {.id=1,  .x_cm=0,     .y_cm=1000,  .z_cm=300},
    {.id=2,  .x_cm=1000,  .y_cm=0,     .z_cm=300},
    {.id=3,  .x_cm=0,     .y_cm=-1000, .z_cm=300},
    
    // 第二层 (z=5m) - 应对初期粮堆遮挡
    {.id=4,  .x_cm=-1000, .y_cm=0,     .z_cm=500},
    {.id=5,  .x_cm=0,     .y_cm=1000,  .z_cm=500},
    {.id=6,  .x_cm=1000,  .y_cm=0,     .z_cm=500},
    {.id=7,  .x_cm=0,     .y_cm=-1000, .z_cm=500},
    
    // 第三层 (z=7m) - 高位基站
    {.id=8,  .x_cm=-1000, .y_cm=0,     .z_cm=700},
    {.id=9,  .x_cm=0,     .y_cm=1000,  .z_cm=700},
    {.id=10, .x_cm=1000,  .y_cm=0,     .z_cm=700},
    {.id=11, .x_cm=0,     .y_cm=-1000, .z_cm=700},
    
    // 顶层 (z=9m) - 覆盖最高粮堆
    {.id=12, .x_cm=-1000, .y_cm=0,     .z_cm=900},
    {.id=13, .x_cm=0,     .y_cm=1000,  .z_cm=900},
    {.id=14, .x_cm=1000,  .y_cm=0,     .z_cm=900},
    {.id=15, .x_cm=0,     .y_cm=-1000, .z_cm=900},
};

// 预期定位精度: ≤20cm (多层部署补偿遮挡)
#define SILO_EXPECTED_ACCURACY_CM  20

#endif
`

---

## 七、代码修改清单

### 7.1 必需修改 (P0)

#### 文件1: downlink_protocol.h

`c
// 修改位置: shared/shared_data/downlink_protocol.h
//============================================================================

// 修改前 (第XX行)
#define DL_MAX_TRACKED_ANCHORS      8u     // 仅支持8个基站

// 修改后 - 支持圆筒仓16基站
#define DL_MAX_TRACKED_ANCHORS      16u    // 支持最多16个基站
`

#### 文件2: fusion_tag.c

`c
// 修改位置: shared/fusion_tag/fusion_tag.c
//============================================================================

// 修改前 (第XX行)
// Tag仅追踪8个基站，修改后支持16个
static TagAnchorTracker s_trackers[DL_MAX_TRACKED_ANCHORS];

// 注意: DL_MAX_TRACKED_ANCHORS已改为16，此处自动生效
`

### 7.2 建议修改 (P1)

#### 文件3: clock_sync.h

`c
// 修改位置: shared/shared_data/clock_sync.h
//============================================================================

// 增加动态参考基站选择结构
typedef struct {
    uint8_t ref_anchor_id;           // 参考基站ID
    uint8_t ref_rank;                // 参考基站rank
    float   ref_quality_weight;      // 质量加权因子
    uint8_t num_active;              // 当前活跃基站数
} DynamicReference;

// 增加全局参考选择函数声明
void clock_sync_select_best_reference(DynamicReference *ref, 
                                      TagAnchorTracker *trackers);
`

#### 文件4: clock_sync.c

`c
// 修改位置: shared/shared_data/clock_sync.c
//============================================================================

// 增加异常值自动重置逻辑
bool clock_sync_update(AnchorClockSync *s,
                       int16_t cfo_raw,
                       uint64_t rx_local,
                       uint64_t tx_remote,
                       uint32_t dt_ms)
{
    // ... 现有代码 ...
    
    // 新增: 连续异常处理
    if (outlier) {
        s->consecutive_outliers++;
        if (s->consecutive_outliers >= CS_MAX_OUTLIER_COUNT) {
            // 连续3次异常，重置滤波器
            s->factor = 1.0;
            s->factor_var = 1e-6;
            s->offset_dtu = 0;
            s->consecutive_outliers = 0;
            // 不作为异常处理，继续更新
        }
    } else {
        s->consecutive_outliers = 0;
    }
    
    return !outlier;
}

// 新增: 动态参考选择
void clock_sync_select_best_reference(DynamicReference *ref,
                                      TagAnchorTracker *trackers)
{
    float best_score = 0;
    uint8_t best_idx = 0;
    
    for (int i = 0; i < DL_MAX_TRACKED_ANCHORS; i++) {
        if (!trackers[i].is_active) continue;
        
        // 综合评分: 质量 × 更新频率
        float score = trackers[i].sync.quality 
                    * (trackers[i].sync.update_count / 100.0f);
        
        if (score > best_score) {
            best_score = score;
            best_idx = i;
        }
    }
    
    ref->ref_anchor_id = trackers[best_idx].rank;
    ref->ref_rank = trackers[best_idx].rank;
    ref->ref_quality_weight = best_score;
}
`

### 7.3 可选优化 (P2)

#### 文件5: fusion_anchor.c - 分组广播

`c
// 修改位置: shared/fusion_anchor/fusion_anchor.c
//============================================================================

// 新增: 基站分组调度
static uint8_t get_my_group(void)
{
    uint8_t rank = g_anchor.rank % 4;
    return rank;  // 返回分组编号 0-3
}

static uint32_t compute_next_tx_slot(void)
{
    uint8_t group = get_my_group();
    uint32_t now = HAL_GetTick();
    uint32_t cycle = now / DL_TIMESYNC_PERIOD_MS;
    
    // 计算下一轮广播时间
    return (cycle + 1) * DL_TIMESYNC_PERIOD_MS + group * 10;
}
`

---

## 八、性能保障方案

### 8.1 时间预算分析

`
┌─────────────────────────────────────────────────────────────────────┐
│  50ms周期时间分配                                                    │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  0ms ───────────────────────────────────────────────────────► 50ms  │
│  │                                                                   │
│  ├─ 0-40ms ─┤ 40-45ms ├─ 45-50ms ─┤                              │
│  │           │         │            │                                │
│  │  接收窗口  │  解算   │   UART输出 │                                │
│  │  (连续RX)  │  (5ms)  │  +准备     │                                │
│  │            │         │            │                                │
│  │            │         │            │                                │
│  └────────────┴─────────┴────────────┘                              │
│                                                                     │
│  关键约束:                                                           │
│    • 40ms内必须收到至少4个基站数据                                    │
│    • 解算时间 ≤ 5ms (STM32L412算力足够)                              │
│    • UART输出非阻塞                                                  │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
`

### 8.2 异常处理策略

| 异常场景 | 检测方法 | 处理策略 |
|---------|---------|---------|
| 基站离线 | 5秒无更新 | 标记inactive，切换参考基站 |
| CFO异常 | 连续3次>阈值 | 重置该基站Kalman状态 |
| 定位失败 | 迭代不收敛 | 输出上一帧有效位置 |
| 可见基站不足 | <4个活跃 | 降低输出频率或输出无效 |

---

## 九、验证与测试

### 9.1 定位精度测试方法

`python
# test_localization_accuracy.py
import serial
import time
import numpy as np

def test_position_accuracy():
    \"\"\"在粮库中多点测试定位精度\"\"\"
    
    ser = serial.Serial('COM5', 921600, timeout=1)
    
    test_points = [
        (0, 0),          # 起点
        (1000, 0),       # 1m
        (2000, 0),       # 2m
        (3000, 0),       # 3m
        (4000, 0),       # 4m
        (4000, 1000),    # 4m, 1m
        (4000, 2000),    # 4m, 2m
        (4000, 3000),    # 4m, 3m
        (2000, 1500),    # 中心
    ]
    
    errors = []
    
    for true_x, true_y in test_points:
        print(f"Testing at ({true_x}, {true_y})...")
        input("Press Enter when robot is in position...")
        
        # 采集10个样本
        measurements = []
        for _ in range(10):
            line = ser.readline().decode('utf-8', errors='ignore')
            if 'P,' in line:  # 位置输出格式
                # 解析: P,seq,X=xxx,Y=xxx,Q=xxx
                parts = line.split(',')
                for p in parts:
                    if p.startswith('X='):
                        meas_x = int(p.split('=')[1])
                    if p.startswith('Y='):
                        meas_y = int(p.split('=')[1])
                measurements.append((meas_x, meas_y))
            time.sleep(0.1)
        
        # 统计
        meas_x_arr = [m[0] for m in measurements]
        meas_y_arr = [m[1] for m in measurements]
        
        # 使用中位数减少异常值影响
        med_x = np.median(meas_x_arr)
        med_y = np.median(meas_y_arr)
        
        error = np.sqrt((med_x - true_x)**2 + (med_y - true_y)**2)
        errors.append(error)
        
        print(f"  Measured: ({med_x:.0f}, {med_y:.0f})")
        print(f"  Error: {error:.1f}cm")
        print()
    
    # 汇总
    print("=" * 50)
    print("精度测试结果:")
    print(f"  平均误差: {np.mean(errors):.1f}cm")
    print(f"  最大误差: {np.max(errors):.1f}cm")
    print(f"  最小误差: {np.min(errors):.1f}cm")
    print(f"  90分位: {np.percentile(errors, 90):.1f}cm")
    print(f"  95分位: {np.percentile(errors, 95):.1f}cm")

if __name__ == '__main__':
    test_position_accuracy()
`

### 9.2 时间同步质量监控

`c
// 在fusion_tag.c中添加同步质量输出
void print_sync_quality_report(void)
{
    char report[256];
    int pos = 0;
    pos += sprintf(report + pos, "SYNC[");
    
    int active_count = 0;
    for (int i = 0; i < DL_MAX_TRACKED_ANCHORS; i++) {
        if (!s_trackers[i].is_active) continue;
        active_count++;
        pos += sprintf(report + pos, "R%d:F=%07lu:Q=%02u%% ",
            s_trackers[i].rank,
            (unsigned long)(s_trackers[i].sync.factor * 1e6),
            (unsigned int)(s_trackers[i].sync.quality * 100));
    }
    
    pos += sprintf(report + pos, "] (%d anchors)\r\n", active_count);
    test_run_info((uint8_t*)report);
}
`

### 9.3 测试检查清单

`
┌────────────────────────────────────────────────────────────┐
│                  现场测试检查清单                           │
├────────────────────────────────────────────────────────────┤
│                                                            │
│  □ 基站固件烧录完成                                        │
│  □ 基站坐标配置正确                                       │
│  □ 基站间时间同步正常 (factor≈1.0, quality>80%)           │
│  □ 标签能正常接收≥4个基站广播                             │
│  □ 静态定位精度测试 (≤20cm)                               │
│  □ 动态精度测试 (机器人移动时)                            │
│  □ 20Hz输出频率验证                                       │
│  □ 遮挡场景测试 (圆筒仓粮堆环境)                          │
│  □ 长期稳定性测试 (连续运行24小时)                        │
│                                                            │
└────────────────────────────────────────────────────────────┘
`

---

## 十、已知问题与限制

### 10.1 当前已知问题

| 问题 | 状态 | 说明 |
|------|------|------|
| PLL校准失败 | ⚠️ 已知 | OTP未烧录，使用RC时钟，不影响TX_TIMESTAMP |
| SYS_TIME冻结 | ⚠️ 已知 | IDLE_RC状态下冻结，使用TX_TIMESTAMP代替 |
| 软复位卡住 | ⚠️ 已知 | SOFT_RST位未清零，但功能正常 |

### 10.2 限制说明

1. **OTP未烧录**: DW3210出厂OTP未烧录PLL校准数据，PLL锁定失败但不影响TX_TIMESTAMP准确性
2. **RC时钟模式**: 系统运行在RC时钟模式，factor精度约1e-6，足以满足20cm定位需求
3. **基站分组**: 当前方案采用分组广播避免碰撞，会降低单基站广播频率

---

## 十一、总结

### 11.1 方案对比

| 功能 | 当前实现 | 改进后 | 改进效果 |
|------|---------|--------|---------|
| 追踪基站数 | 8个 | 16个 | 支持圆筒仓场景 |
| 异常恢复 | 简单阈值 | 连续异常重置 | 更稳定 |
| 参考基站 | 固定 | 动态选择 | 更精确 |
| TDOA求解 | 基础GA | RANSAC+加权 | 更抗干扰 |

### 11.2 实施建议

`
优先级排序:
  P0 (立即实施):
    1. 修改DL_MAX_TRACKED_ANCHORS为16
    2. 验证16基站追踪功能
    
  P1 (一周内):
    3. 实现动态参考基站选择
    4. 实现异常值自动重置
    
  P2 (后续优化):
    5. 实现RANSAC TDOA求解
    6. 实现分组广播调度
`

### 11.3 预期效果

| 指标 | 目标 | 预期 |
|------|------|------|
| 定位精度 | ≤20cm | ≤15cm (LOS) |
| 输出频率 | ≥20Hz | 20Hz稳定 |
| 系统容量 | 16标签 | 16标签 |
| 基站容量 | 16基站 | 16基站 |

---

**文档版本**: V1.0  
**创建日期**: 2026-09-17  
**最后更新**: 2026-09-17  
**维护者**: UWB项目组
