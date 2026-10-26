# UWB TDOA 下行定位系统

基于 DW3000 的下行 TDOA（Time Difference of Arrival）室内定位系统，支持多 Anchor 广播 + Tag 自定位。

## 项目状态：🟡 开发中（部分功能待测试）

---

## 功能进度

### ✅ 已完成并测试通过

| 功能 | 状态 | 说明 |
|------|------|------|
| Anchor 广播信标 | ✅ | 100ms 周期，TX_OK，坐标正确广播 |
| Tag 接收信标 | ✅ | 250ms RX 窗口，坐标提取正确 |
| Anchor 时钟同步 (factor) | ✅ | Kalman 滤波，Q=86-87，factor≈0.99994 |
| Tag 时钟同步 (factor) | ✅ | 跟踪每个 Anchor 的 factor |
| Anchor 坐标配置 | ✅ | AT+POS 命令，掉电保存 |
| Staggered TX | ✅ | 250μs/slot，最多 4 个 Anchor |
| Tag 参考锚点选择 | ✅ | beacon_count≥2 且最高质量优先 |
| TDOA slot_offset 补偿 | ✅ | `tdoa_corrected = tdoa - (slot_B - slot_A) × 250μs_DTU` |
| 协议格式 | ✅ | 32B TimeSync 信标，0xEE 功能码 |
| Anchor→Tag 单向定位 | ✅ | 架构正确，等待 TDOA 解算验证 |

### ⚠️ 已开发未测试通过

| 功能 | 状态 | 说明 |
|------|------|------|
| Tag TDOA 解算位置输出 | ⚠️ | `P:X=...,Y=...,Q=...` 待验证 |
| Tag offset 计算 | ⚠️ | `offset = factor × first_tx_ts − rx_at_first` 待验证 |
| TDOA 阈值过滤 | ⚠️ | 200m 阈值，待实际数据验证 |
| Tag 位置滑动滤波 | ⚠️ | POS_FILTER_SIZE=5 待验证 |
| 定位精度 | ⚠️ | 待实际测量 |
| 移动场景稳定性 | ⚠️ | 待动态测试 |

### ❌ 未开发功能

| 功能 | 状态 | 说明 |
|------|------|------|
| Level 0 根时钟选举 | ❌ | 当前 cs_level=1 固定 |
| 层级同步派发 | ❌ | cs_level 字段存在但未启用 |
| 多区域支持 | ❌ | Anchor 共用、跨区域定位 |
| TOA (Time of Arrival) 备选 | ❌ | 作为 TDOA 失败时的 fallback |
| OTA 固件升级 | ❌ | gw_ota 模块存在但未启用 |
| 上位机协议 | ❌ | Gateway 命令解析未完整实现 |

---

## 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                    时钟同步层级 (Anchor ↔ Anchor)                 │
│                                                                   │
│   ┌──────────┐                                                    │
│   │ Level 0  │ ← 根时钟源（当前未选举，cs_level=1 固定）             │
│   │ (Root)   │  定期发送 TimeSync 信标 (0xEE)                     │
│   └────┬─────┘  broadcast, 100ms 周期                             │
│        │                                                          │
│        ▼ 监听 TimeSync                                            │
│   ┌──────────┐    ┌──────────┐    ┌──────────┐                   │
│   │ Level 1  │    │ Level 1  │    │ Level 1  │  ← 当前所有 Anchor │
│   │  Anchor  │    │  Anchor  │    │  Anchor  │    cs_level=1     │
│   └──────────┘    └──────────┘    └──────────┘                   │
└──────────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────────┐
│                    TDOA 定位链路 (Anchor → Tag)                    │
│                                                                   │
│   所有 Anchor（Level 0/1/2）依次广播 TimeSync 信标（staggered）：   │
│                                                                   │
│   Slot 0 (rank%4=0) ────▶ │                                      │
│   Slot 1 (rank%4=1) ─────▶ │ ────▶  ┌────────┐                   │
│   Slot 2 (rank%4=2) ─────▶ │ ────▶  │        │  Tag              │
│   Slot 3 (rank%4=3) ─────▶ │ ────▶  │   Tag  │  TDOA 解算         │
│                             │         │        │  位置输出         │
│                             │         └────────┘  P:X=,Y=,Q=     │
└──────────────────────────────────────────────────────────────────┘
```

---

## 定位流程

```
Anchor 端（100ms 周期）:
─────────────────────────
每 100ms 广播 TimeSync 信标 (32B):
  [tx_timestamp40][cs_level][parent_eui64][x_cm][y_cm][z_cm]

  Slot = rank % 4 (0..3), 每个 slot = 250μs
  Slot 0: T+0μs, Slot 1: T+250μs, Slot 2: T+500μs, Slot 3: T+750μs

  factor = 锚点时钟频率 / 上级锚点时钟频率（Kalman 滤波）
  offset = factor × first_tx_ts − rx_at_first（RC 滤波）
  quality = 同步质量分数（0~1）

Tag 端（250ms RX 窗口）:
─────────────────────────
1. 监听所有 Anchor 信标（250ms 窗口 ≥ 2 个信标/Anchor）
2. 对每个 Anchor 更新 Kalman 滤波（factor, offset）
3. 用 first_rx_ts_local 记录窗口内首次 RX 时间戳
4. 计算两两 TDOA:
   global_A = offset_A + factor_A × rx_A
   global_B = offset_B + factor_B × rx_B
   slot_tdoa = (slot_B - slot_A) × 250μs_DTU
   geo_tdoa = (global_B - global_A) - slot_tdoa
5. Chan-Taylor 算法解算 Tag 位置
```

---

## 时钟同步核心公式

来自 `时间同步原理.txt`（BD8NCF 技术）:

```
deta = (ST2 - ST1) - (RT2 - RT1)
Ratio = deta / (ST2 - ST1)

global_time = offset + factor × local_time
offset = factor × first_tx_ts − rx_at_first
```

---

## 关键文件

| 文件 | 职责 |
|------|------|
| `fusion_anchor.c` | Anchor 端：广播信标、接收上级信标并同步 |
| `fusion_tag.c` | Tag 端：RX 窗口 + 时钟同步 + TDOA 解算 |
| `clock_sync.c` | Kalman 滤波：factor（频率比）+ RC 滤波：offset（相位偏移） |
| `tdoa_solver.c` | Chan-Taylor 混合 TDOA 求解器 |
| `downlink_protocol.h` | 协议：信标格式(32B)、TDOA 常量 |
| `shared_defines.h` | UWB 配置：信道、速率、天线延迟 |

---

## 信标格式（32 字节 TimeSync）

| 偏移 | 字段 | 说明 |
|------|------|------|
| 0-1 | frame_ctrl | `0x41, 0x88` |
| 2 | seq | 序列号 |
| 3-4 | pan_id | `0xCA, 0xDE` |
| 5-6 | dest | `0xFFFF` 广播 |
| 7-8 | source | Anchor rank (LE) |
| 9 | func | `0xEE` (FUNC_DOWNLINK_BEACON) |
| 10-14 | tx_timestamp | 40-bit 预测 TX 时间戳 |
| 15 | cs_level | 同步层级 (0=root, 1+=child) |
| 16-23 | parent_eui64 | 父节点 EUI |
| 24-25 | x_cm | X 坐标 (int16 cm) |
| 26-27 | y_cm | Y 坐标 (int16 cm) |
| 28-29 | z_cm | Z 坐标 (int16 cm) |
| 30-31 | reserved | 保留 |

---

## 硬件配置

- **MCU**: STM32L412KBUx
- **UWB芯片**: DW3000
- **信道**: Channel 9 (4940 MHz)
- **速率**: 6.8 Mbps (DWT_BR_6M8)
- **前导码**: 128 symbols (DWT_PLEN_128)
- **天线延迟**: TX=16385, RX=16385

---

## 开发环境

- **IDE**: Keil MDK-ARM
- **编译**: C99 标准（Keil ARM Compiler）
- **调试**: UART 输出 (921600 8N1；bootloader 引导日志仍为 115200)
- **协议**: 下行 TDOA

---

## 待办事项

### 高优先级
- [ ] Tag TDOA 解算成功输出位置 `P:X=...,Y=...,Q=...`
- [ ] 验证 TDOA 精度（静态定位精度 < 10cm）
- [ ] 验证 TDOA 精度（移动场景 2m/s）

### 中优先级
- [ ] 实现 Level 0 根时钟选举
- [ ] 实现层级派发（cs_level 传播）
- [ ] 多区域 Anchor 共用支持
- [ ] 定位精度优化（算法调参）

### 低优先级
- [ ] OTA 固件升级
- [ ] 上位机协议完善
- [ ] TOA fallback 解算
- [ ] 移动轨迹平滑滤波

---

## 参考资料

- `时间同步原理.txt` - BD8NCF 时钟同步核心原理
- `downlink_protocol.h` - 协议定义
- `clock_sync.h` / `clock_sync.c` - 时钟同步实现
- `tdoa_solver.c` - Chan-Taylor TDOA 求解器
