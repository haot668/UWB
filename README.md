# UWB 实时定位系统 (RTLS)

基于 **Decawave DW3210** UWB 收发器 + **STM32L412KBU6** MCU 的 TDOA+TOF 融合定位系统，支持基站自动组网、标签定位、PC 上位机可视化管理。

---

## 1. 系统架构

```
┌──────────────────────────────────────────────────────────────┐
│                    PC 上位机 (Python + tkinter)                │
│                    uwb_gateway_py/uwb_host.py                 │
│                     UART AT 指令 (115200 8N1)                  │
└────────────────────────────┬─────────────────────────────────┘
                             │
                    ┌────────┴────────┐
                    │   网关 (Gateway) │  ← 主基站 + UART 桥接
                    │   UID: 0000      │    RANK=0, USART2
                    └────────┬────────┘
                             │ UWB (Channel 5, 6.8Mbps)
              ┌──────────────┼──────────────┐
              │              │              │
        ┌─────┴─────┐  ┌────┴────┐  ┌─────┴─────┐
        │ 基站 A01   │  │ 基站 A02 │  │  标签 T1   │
        │ (Anchor)   │  │ (Anchor) │  │  (Tag)     │
        │ RANK=1     │  │ RANK=2   │  │ 本地解算    │
        │ USART2     │  │ USART2   │  │  USART2    │
        └───────────┘  └──────────┘  └───────────┘
```

**定位原理：** 基站下行广播位置+定时 → 标签被动接收并本地 Gauss-Newton 解算 → 标签 UART 直接输出坐标。同时保留原融合协议 (TDOA+TOF) 的 UART 原始数据上报通道。

---

## 2. 目录结构

```
UWB_V0.0.1/
├── README.md                          # 本文件
├── _gen_projects.py                   # Keil 工程自动生成脚本
│
├── shared/                            # === 三工程共享代码 ===
│   ├── Inc/                           # 公共头文件
│   │   ├── main.h                     #   引脚定义 (DW3210→STM32L412KBU6)
│   │   ├── stm32l4xx_it.h             #   中断声明
│   │   └── stm32l4xx_hal_conf.h       #   HAL 模块开关
│   ├── system/                        # 系统初始化
│   │   ├── system_stm32l4xx.c         #   CMSIS SystemInit/SystemCoreClock
│   │   └── stm32l4xx_hal_timebase_TIM.c  # TIM6 时基模板 (备用)
│   ├── startup/
│   │   └── startup_stm32l412xx.s      #   CMSIS 启动文件
│   ├── Drivers/                       # STM32 HAL + CMSIS
│   │   ├── CMSIS/Include/             #   ARM CMSIS 核心头文件
│   │   ├── CMSIS/Device/ST/STM32L4xx/ #   STM32L4xx 设备头文件
│   │   └── STM32L4xx_HAL_Driver/      #   HAL 驱动 (仅用 15 个 .c)
│   ├── decadriver/                    # DW3000 UWB 驱动 (6 文件)
│   │   ├── deca_device.c              #   主驱动 (~4910 行)
│   │   ├── deca_device_api.h          #   公开 API
│   │   ├── deca_regs.h                #   寄存器映射
│   │   ├── deca_types.h / deca_vals.h / deca_version.h
│   ├── platform/                      # HAL 移植层 (6 文件)
│   │   ├── port.c / port.h            #   GPIO/SPI/UART/延时/中断
│   │   ├── deca_spi.c / deca_spi.h    #   SPI 读写
│   │   ├── deca_mutex.c               #   中断互斥
│   │   └── deca_sleep.c               #   休眠封装
│   ├── MAC_802_15_4/                  # IEEE 802.15.4 MAC 帧处理
│   │   ├── mac_802_15_4.c / .h
│   ├── MAC_802_15_8/                  # IEEE 802.15.8 MAC (AES)
│   │   ├── mac_802_15_8.c / .h
│   ├── shared_data/                   # 协议定义 + 共享函数
│   │   ├── shared_defines.h           #   全局常量/错误类型
│   │   ├── shared_functions.c / .h    #   时间戳/延时/错误检查
│   │   ├── fusion_protocol.h          #   TDOA+TOF 融合定位协议
│   │   ├── auto_anchor.h              #   基站自发现协议
│   │   ├── gateway_protocol.h         #   网关 UWB 指令协议
│   │   └── downlink_protocol.h        #   下行 TDOA 协议
│   ├── config/                        # RF 配置
│   │   ├── config_options.c / .h      #   34 种预设 + TX Power
│   ├── examples_info/                 # 测试入口路由
│   │   ├── example_info.c             #   宏→函数指针映射
│   │   └── examples_defines.h         #   类型定义
│   ├── fusion_anchor/                 # 基站固件 (主/从)
│   │   └── fusion_anchor.c            #   ~1560 行，核心逻辑 + 下行 TX + 两帧时钟同步
│   ├── fusion_tag/                    # 标签固件
│   │   └── fusion_tag.c               #   ~270 行，下行接收 + 本地解算
│   └── downlink/                      # 下行 TDOA 解算
│       └── tdoa_solver.c              #   Gauss-Newton 多边定位求解器
│
├── uwb_gateway/                       # === 工程1: 网关 ===
│   ├── DWS3000_Gateway.uvprojx        # Keil MDK 工程
│   ├── DWS3000_Gateway.sct            # 链接脚本
│   └── Src/
│       ├── main.c                     #   入口 + 时钟 + UART RX 中断
│       ├── example_selection.h        #   TEST_FUSION_ANCHOR, RANK=0
│       ├── gateway_cmd.c / .h         #   AT 指令解析 (~782 行)
│       ├── stm32l4xx_hal_msp.c        #   SPI/USART MSP 初始化
│       └── stm32l4xx_it.c             #   中断处理
│
├── uwb_anchor/                        # === 工程2: 从基站 ===
│   ├── DWS3000_Anchor.uvprojx
│   ├── DWS3000_Anchor.sct
│   └── Src/
│       ├── main.c
│       ├── example_selection.h        #   TEST_FUSION_ANCHOR, RANK=1
│       ├── stm32l4xx_hal_msp.c
│       └── stm32l4xx_it.c
│
├── uwb_tag/                           # === 工程3: 标签 ===
│   ├── DWS3000_Tag.uvprojx
│   ├── DWS3000_Tag.sct
│   └── Src/
│       ├── main.c
│       ├── example_selection.h        #   TEST_FUSION_TAG
│       ├── stm32l4xx_hal_msp.c
│       └── stm32l4xx_it.c
│
├── uwb_gateway_py/                    # === Python PC 上位机 ===
│   ├── uwb_host.py                    #   主程序 (~2261 行)
│   ├── anchor_cfg.json                #   基站坐标持久化
│   └── run.bat                        #   启动脚本
│
└── API/                               # 原始 Decawave API 工程 (保留参考)
```

---

## 3. 硬件平台

| 组件 | 型号 | 说明 |
|------|------|------|
| MCU | STM32L412KBU6 | Cortex-M4, 80MHz, 128KB Flash, 40KB SRAM |
| UWB | DW3210 (Decawave/Qorvo) | Channel 5 (6.5GHz), 6.8Mbps, STS 模式 1 |
| 时钟 | HSE 8MHz → PLL 80MHz | 外部有源晶振 ±25ppm |
| UART | USART2 | 115200 8N1, 三设备均通过 USART2 (PA2/PA3) |
| SPI | SPI1 | PB3=SCK, PB4=MISO, PB5=MOSI, PA15=NSS |
| GPIO | PA12=DW_IRQ, PB6=DW_RESET, PB7=DW_WAKEUP | LED: PA5, PA11 |

### DW3210 引脚连接

```
STM32L412          DW3210
─────────          ──────
PA2  (TX)    ←→    UART RX (网关 AT 指令)
PA3  (RX)    ←→    UART TX
PB3  (SCK)   ──→   SCK
PB4  (MISO)  ←──   MISO
PB5  (MOSI)  ──→   MOSI
PA15 (NSS)   ──→   NSS (软件控制)
PA12 (IRQ)   ←──   IRQn (EXTI12, 上升沿)
PB6  (RST)   ──→   RSTn
PB7  (WAKE)  ──→   WAKEUP
PA5           ──→   LED1
PA11          ──→   LED2
```

---

## 4. 编译与烧录

### 环境要求
- **Keil MDK-ARM V5.05** (ARMCC V5.05) 或更高版本
- **Pack:** Keil.STM32L4xx_DFP.3.1.0
- **MicroLIB** 已启用 (减小代码体积)

### 编译步骤

```bash
# 打开对应工程文件:
#   uwb_gateway/DWS3000_Gateway.uvprojx   (网关)
#   uwb_anchor/DWS3000_Anchor.uvprojx     (基站)
#   uwb_tag/DWS3000_Tag.uvprojx           (标签)

# Keil 中: Project → Build Target (F7)
# 输出: Output/DWS3000_*.hex (用于烧录)
```

### 工程预定义宏

| 工程 | 宏定义 |
|------|--------|
| Gateway | `USE_HAL_DRIVER, STM32L412xx, TEST_FUSION_ANCHOR, ENABLE_GATEWAY` |
| Anchor | `USE_HAL_DRIVER, STM32L412xx, TEST_FUSION_ANCHOR` |
| Tag | `USE_HAL_DRIVER, STM32L412xx, TEST_FUSION_TAG` |

### 公共包含路径 (三工程相同)

```
..\shared\Inc
..\shared\Drivers\CMSIS\Include
..\shared\Drivers\CMSIS\Device\ST\STM32L4xx\Include
..\shared\Drivers\STM32L4xx_HAL_Driver\Inc
..\shared\decadriver
..\shared\platform
..\shared\MAC_802_15_4
..\shared\MAC_802_15_8
..\shared\shared_data
..\shared\examples_info
..\shared\config
..\shared\fusion_anchor
..\shared\fusion_tag
..\shared\downlink
..\shared\system
.\Src
```

---

## 5. 已实现功能清单

### 5.1 UWB 定位协议

| 功能 | 状态 | 说明 |
|------|:----:|------|
| TDOA Blink 广播 | 已实现 | 标签周期性发送 Blink 帧 (FC=0xE2)，含序列号和 40-bit 时间戳 |
| 时钟同步 (Clock Sync) | 已实现 | 两帧协议：信标 (0xF0) + 同步帧 (0xF8, ~200us 间隔)，同步帧携带**本次**信标 TX 时间戳 |
| 从基站 ferr 计算 | 已实现 | 从基站比较连续两个同步帧：`ferr = (d_master - d_slave) × 2³⁰ / d_slave`，差值公式防溢出 |
| TOF 双向测距 | 已实现 | Poll→Response→Final 三步测距，单位 UUS → 米 |
| 从基站 TDMA 上报 | 已实现 | 从基站使用 TDMA 时隙 (5ms slots) 上报 Slave Report 给主基站 |
| 主基站 UART 输出 | 已实现 | 格式: `F,SEQ,R=TS,D=DIST,C=CLK_OFF[,Sn:R=TS:C=CO]` |
| 40-bit 时间戳打包 | 已实现 | `pack_40bit_ts()` / `unpack_40bit_ts()` 使用 DW3000 高精度时钟 |
| STS 模式 1 安全测距 | 已实现 | 使用 STS 128-bit，防中继攻击 |
| 下行 TDOA 定位解算 | 已实现 | 基站广播位置+定时 (0xF0/0xF1)，标签 Gauss-Newton 解算 → UART 坐标 |

### 5.2 基站自组网

| 功能 | 状态 | 说明 |
|------|:----:|------|
| 自动发现 (Auto Discovery) | 已实现 | 上电自动 LISTEN→ELECT→JOIN→RUN 状态机 |
| 主基站选举 | 已实现 | 基于 UID (STM32 UID) 优先级选举 |
| Beacon 心跳 | 已实现 | 主基站每 10 轮循环广播 Beacon 帧 |
| 从基站自动加入 | 已实现 | JOIN_REQUEST → JOIN_ACK 交互 |
| 主基站丢失检测 | 已实现 | 连续 3 个 Sync 周期未收到 Beacon → 重发现 |
| 从基站 Node Count | 已实现 | 主基站 Beacon 携带当前在线从基站数量 |

### 5.3 网关 AT 指令

| 指令 | 说明 |
|------|------|
| `AT+PING` | 网关心跳检测，回复 `+PONG` |
| `AT+LIST` | 请求在线设备列表，回复 `+ONLINE,UID,TYPE,RANK` |
| `AT+FIND=<UID>,<0/1>` | 查找设备 (LED 闪烁)，通过 UWB 转发 FIND_REQ |
| `AT+COORD=<UID>,<X>,<Y>,<Z>` | 设置基站坐标，通过 UWB 转发 COORD_WRITE |
| `AT+COORD?=<UID>` | 查询基站坐标 |
| `AT+RESET=<UID>` | 远程复位设备 |
| `AT+OTA_START=<UID>,<SIZE>` | OTA 升级开始 |
| `AT+OTA_DATA=<UID>,<OFFSET>,<HEX>` | OTA 数据块传输 |
| `AT+OTA_END=<UID>` | OTA 升级结束 |

### 5.4 设备追踪 (仅网关)

| 功能 | 状态 | 说明 |
|------|:----:|------|
| 设备检测 | 已实现 | 通过 Blink/POLL/Slave Report 帧自动发现标签和基站 |
| 在线心跳 | 已实现 | 每 5 秒重发 `+ONLINE` 保持上位机同步 |
| 超时离线 | 已实现 | 7 秒未收到设备帧 → 发送 `+OFFLINE` |
| 设备列表 | 已实现 | 最多 16 个设备 (ANCHOR/TAG/GATEWAY 三种类型) |
| 类型修复 | 已修复 | 网关正确识别为 GATEWAY 类型，不再误判为 ANCHOR |
| SysTick 修复 | 已修复 | `HAL_SYSTICK_Callback` 调用 `HAL_IncTick()` 解决 `uwTick=0` |

### 5.5 PC 上位机

| 功能 | 状态 | 说明 |
|------|:----:|------|
| 串口连接 | 已实现 | 支持 COM 口选择、波特率配置、自动重连 |
| 设备列表 | 已实现 | 在线/离线分组显示、自动编号 (T1/T2/A01/A02) |
| 2D 地图 | 已实现 | 支持平移/缩放、设备图标、轨迹线、网格、比例尺 |
| 基站坐标管理 | 已实现 | 添加/编辑/删除基站坐标、JSON 持久化 |
| 设备查找 | 已实现 | 发送 FIND 指令，远程闪烁 LED |
| OTA 升级 | 已实现 | 单设备 + 批量升级、进度跟踪、Hex 文件解析 |
| 在线/离线检测 | 已实现 | TAG 8s/ANCHOR 15s 超时、心跳 + 刷新按钮 |
| 刷新功能 | 已实现 | AT+LIST 主动刷新、双向状态同步 |
| 基站链路质量 | 已实现 | 地图上显示基站颜色标识 |
| 演示模式 | 已实现 | 无硬件时加载示例数据展示 |

### 5.6 下行 TDOA 协议

| 功能 | 状态 |
|------|:----:|
| `downlink_protocol.h` — 协议定义 (含 0xF8 时钟同步帧) | 已实现 |
| `downlink_anchor.c` — 基站下行信标/Blink TX | 已集成到 `fusion_anchor.c` |
| `fusion_tag.c` — 标签下行接收 + 本地解算 | 已实现 |
| `tdoa_solver.c` — Gauss-Newton 解算 | 已实现 (FPU 硬件加速) |
| 编译守卫 `FUSION_DOWNLINK_MODE` | 三工程均已启用 |
| 两帧时钟同步 (0xF0 + 0xF8) | 已实现 — 主站每周期发信标+同步帧，从站用同步帧计算 FERR |
| AT+RATE 动态频率调节 | 已实现 — 20~5000ms，上位机下拉框选择 |

---

## 6. UWB 帧协议一览

### 融合定位协议 (fusion_protocol.h)

| 帧类型 | FC[0] | 方向 | 说明 |
|--------|-------|------|------|
| BLINK | 0xE2 | Tag → All | TDOA 定位请求 |
| CLOCK_SYNC | 0xAA | Main → All | 时钟同步基准 (含 40-bit TS) |
| TOF_POLL | 0xE3 | Tag → Main | TOF 测距请求 |
| TOF_RESP | 0xE4 | Main → Tag | TOF 测距响应 |
| TOF_FINAL | 0xE5 | Tag → Main | TOF 测距确认 |
| SLAVE_REPORT | 0xE6 | Slave → Main | 从基站时间戳上报 |

### 自动发现协议 (auto_anchor.h)

| 帧类型 | FC[0] | 方向 | 说明 |
|--------|-------|------|------|
| BEACON | 0xE7 | Main → All | 主基站在线心跳 |
| ELECTION_CLAIM | 0xE8 | Any → All | 选举声明 |
| JOIN_REQUEST | 0xE9 | Slave → Main | 请求加入网络 |
| JOIN_ACK | 0xEA | Main → Slave | 确认加入 + 分配 RANK |

### 网关指令协议 (gateway_protocol.h)

| 帧类型 | FC[0] | 方向 | 说明 |
|--------|-------|------|------|
| FIND_REQ | 0xF2 | GW → Target | 查找设备 |
| FIND_ACK | 0xF3 | Target → GW | 查找确认 |
| COORD_WRITE | 0xF4 | GW → Anchor | 写入坐标 |
| COORD_ACK | 0xF5 | Anchor → GW | 坐标确认 |
| OTA_DATA | 0xF6 | GW → Target | OTA 数据块 |
| OTA_ACK | 0xF7 | Target → GW | OTA 确认 |

### 下行 TDOA 协议 (downlink_protocol.h)

| 帧类型 | FC[0] | 方向 | 说明 |
|--------|-------|------|------|
| DOWNLINK_BEACON | 0xF0 | Master → Tag | 主基站广播锚点位置表 (N×4 bytes) + 周期序号 |
| DOWNLINK_BLINK | 0xF1 | Slave → Tag | 从基站 TDMA 下行 Blink，源地址编码 rank |
| DL_CLOCK_SYNC | 0xF8 | Master → All | 时钟同步帧，携带**本次**信标 TX 时间戳 (信标后 ~200us) |

---

## 7. 主循环架构

### 7.1 下行模式 (FUSION_DOWNLINK_MODE)

**主基站循环 (~500ms)：**
```
Phase 1: RX 标签 Blink (0xE2) / 选举帧 / JOIN_REQ  ← 500ms 轮询
         └── 标签 Blink → gateway_device_seen() 在线检测
Phase 7: TX 下行信标 (0xF0)  ← 每周期，携带锚点位置表
         TX 时钟同步帧 (0xF8) ← 每周期，携带**本次**信标 TX 时间戳 (信标后 ~200us)
         TX 自发现信标 (0xE7) ← 每 10 周期
         gateway_poll_uart()  ← AT 指令轮询
```

**从基站循环 (~500ms)：**
```
Phase 1: RX 下行信标 (0xF0)  ← 心跳检测
         RX 时钟同步帧 (0xF8) ← FERR 计算 + TDMA Blink TX (0xF1)
         RX 自发现信标 (0xE7) ← 主站在线确认
         未收到 → 打印 "." 心跳丢失
         
FERR 计算: d_master = sync_tx[N] - sync_tx[N-1]  (主站时钟间隔)
           d_slave  = sync_rx[N] - sync_rx[N-1]  (从站时钟间隔)
           ferr = (d_master - d_slave) × 2³⁰ / d_slave
```

### 7.2 融合模式 (!FUSION_DOWNLINK_MODE)

**主基站循环：**
```
Phase 1: RX TDOA Blink    ← 500ms 超时，监听标签 Blink
Phase 2: TX Clock Sync     ← 延迟 300μs 后发送 Sync 帧
Phase 3: RX TOF Poll       ← 200ms 超时，监听标签 Poll
Phase 4: TX TOF Response   ← 延迟发送 Resp (含时间戳)
Phase 5: RX TOF Final      ← 200ms 超时，监听标签 Final
Phase 6: RX Slave Reports  ← TDMA 收集从基站上报
Phase 7: UART 输出 + Beacon + Gateway Poll
```

**标签循环 (下行模式)：** RX Beacon (0xF0) → 发 TDOA Blink (0xE2, 10ms 后) → RX N-1 Blinks (0xF1) → tdoa_solve() → TX 位置报告 (0xF2) → UART 输出坐标

### UART 输出格式

| 设备 | 格式 | 示例 | 说明 |
|------|------|------|------|
| 主基站 (融合) | `F,SEQ,R=TS,D=DIST,C=CLK_OFF[,Sn:R=TS:C=CO]` | `F,0A,R=01234567,D=12.34` | TDOA+TOF 原始数据上报 |
| 标签 (下行) | `P,SEQ,X=<cm>,Y=<cm>,Z=<cm>,Q=<rms>` | `P,1F,X=123.4,Y=567.8,Z=150.0,Q=2.1` | 下行 TDOA 定位解算结果 |
| 从基站 (下行) | `S,<rank>,<ferr_hex>` | `S,01,0000001B` | 从站时钟频率偏移 (8 位 hex，有符号) |
| 基站日志 | `START MASTER` / `START SLAVE` | — | 组网上电状态 |
| 网关 AT | `+PONG` / `+ONLINE,...` / `+OFFLINE,...` | `+ONLINE,0001,A,1` | AT 指令响应 |

---

## 8. 已知问题与修复记录

| 问题 | 状态 | 修复 |
|------|:----:|------|
| `HAL_GetTick()` 始终返回 0 | 已修复 | `stm32l4xx_it.c` 添加 `HAL_SYSTICK_Callback()` 调用 `HAL_IncTick()` |
| 标签仅上电时上位机显示离线 | 已修复 | 网关心跳 + 超时检测 (基于 HAL_GetTick) |
| 网关被识别为 ANCHOR | 已修复 | `gw_dev_t.type` 扩展为三态 'T'/'G'/'A' |
| 设备列表刷新频闪 | 已修复 | `handle_online` 仅在状态变化时刷新 UI |
| 刷新按钮只下线不恢复 | 已修复 | `_finish_refresh` 改为双向同步 |
| Keil link error L6047U | 已修复 | 移除未用的 USB CDC 代码 + HAL 精简 |
| SysTick 中断不触发 | 已修复 | STM32L4 HAL 的 `HAL_SYSTICK_Callback` 是弱空函数，需用户实现 |
| 下行 TDOA 定位解算 | 已实现 | 基站增加下行 TX (0xF0/0xF1)，标签重写为纯接收+本地 Gauss-Newton 解算 |
| FERR int64 溢出 (500ms 间隔 × 2³⁰ > 2⁶³) | 已修复 | 改用差值公式: `(d_master - d_slave) × FERR_SCALE / d_slave` |
| FERR 符号反复跳变 (d_master/d_slave 测相邻区间) | 已修复 | 两帧协议：同步帧 (0xF8) 携带本次信标 TX 时间戳，d_master 和 d_slave 测同一区间 |
| 从站收到信标 0xF0 后 DW3000 停止 RX → 同步帧 0xF8 丢失 | 已修复 | for 循环内每收到帧后立即 `dwt_rxenable()`，确保信标后 200us 的同步帧能被接收 |
| 标签 0xE2 Blink 导致从站聋掉 | 已修复 | 非信标帧处理后 `dwt_rxenable()` 重新使能 RX |
| 从站保存状态 (saved_d_slave) 因丢帧 stale 导致溢出 | 已修复 | 两帧协议无需保存跨周期状态，仅需 prev_sync_tx/rx_ts |

---

## 9. 代码量统计

| 组件 | 文件数 | 代码行数 |
|------|:------:|:--------:|
| 工程特有 Src (3 工程) | 15 | ~2,200 |
| 共享应用代码 | ~14 | ~4,600 |
| DW3000 驱动 | 6 | ~10,680 |
| HAL + CMSIS | ~130 | ~80,000 |
| Python 上位机 | 5 | ~2,690 |
| **应用代码合计** | **~35** | **~6,800** |

三个固件工程通过 `shared/` 目录共享超过 95% 的代码。

---

## 10. 开发环境

| 工具 | 版本 | 用途 |
|------|------|------|
| Keil MDK-ARM | V5.05 (ARMCC V5.05) | 固件编译 |
| STM32L4xx DFP | 3.1.0 | CMSIS/HAL 驱动 |
| Python | 3.x | 上位机运行 |
| pyserial | 最新 | 串口通信 |
| tkinter | 内置 | GUI 框架 |

---

## 11. 快速启动

### 硬件准备
1. 网关板 ×1 (烧录 `uwb_gateway` 固件)
2. 基站板 ×1~N (烧录 `uwb_anchor` 固件)
3. 标签板 ×1~N (烧录 `uwb_tag` 固件)
4. USB-TTL 串口模块连接网关 USART2 (PA2/PA3)

### 启动顺序
1. 网关板先上电 → UART 输出 `START MASTER`
2. 基站板上电 → 自动发现并加入网络 → UART 输出 `START SLAVE`
3. 标签板最后上电 → 接收下行信标 → 本地解算 → UART 输出坐标
4. PC 运行 `uwb_gateway_py/run.bat` → 选择 COM 口 → 连接

### 上位机操作
- **Refresh** — AT+LIST 刷新在线设备
- **Add Anchor** — 设置基站坐标
- **Find** — 查找设备 (LED 闪烁)
- **OTA** — 无线固件升级
- **地图** — 滚轮缩放、拖拽平移
