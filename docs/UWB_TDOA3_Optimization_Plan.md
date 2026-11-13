# UWB 下行 TDOA 定位系统 · 完整优化实施方案

> 参考对象：Bitcraze LPS Node Firmware（`bitcraze/lps-node-firmware`，LGPL-3.0，活跃维护）
> 目标：满足粮库场景（矩形仓 / 圆筒仓，遮挡、多高度、机器人导航），不改硬件
> 状态：可落地实施稿（本文档只描述改什么、怎么改、如何验收；不含代码提交）

---

## 0. 文档约定

| 项 | 说明 |
|---|---|
| 代码根 | `D:\Work\UWB_project\UWB_V0.0.1` |
| 真实编译目录 | `shared/`（`API\Src\` 为过期副本，勿引用/勿同步） |
| 引用格式 | 绝对路径 + 单行号 |
| 术语 | **锚/Anchor**=基站；**Tag**=标签；**roster**=参与组网的锚集合；**α**=本地钟/远端钟频率比 |
| 单位约定 | 距离一律 **cm**；时间一律 **DTU**（1 DTU = 15.65 ps = 0.46907 cm），换算常量 `CM_PER_DTU_F` |

### 0.1 现状基线（已核实）

| 组件 | 规模 | 位置 |
|---|---|---|
| Anchor 业务 | 3111 LOC | `D:\Work\UWB_project\UWB_V0.0.1\shared\fusion_anchor\fusion_anchor.c` |
| Tag 业务 | 1957 LOC | `D:\Work\UWB_project\UWB_V0.0.1\shared\fusion_tag\fusion_tag.c` |
| 下行协议 | 602 LOC | `D:\Work\UWB_project\UWB_V0.0.1\shared\shared_data\downlink_protocol.h` |
| 时钟同步 | 411/243 LOC | `D:\Work\UWB_project\UWB_V0.0.1\shared\shared_data\clock_sync.c` / `clock_sync.h` |
| 位置求解 | 462 LOC | `D:\Work\UWB_project\UWB_V0.0.1\shared\downlink\tdoa_solver.c` |
| 位姿解算 | — | `D:\Work\UWB_project\UWB_V0.0.1\shared\downlink\tdoa_solver.c:432` 决定 2D/3D |

### 0.2 与 bitcraze 的规模对照

| 项 | bitcraze | 本项目 |
|---|---|---|
| 算法核心文件 | `src/` 31 + `inc/` 35 = 66 文件 | 5 个文件（其中 2 个 2000~3000 行） |
| 算法可插拔 | `uwbAlgorithm_t{init,onEvent}` 注册表 | 无，算法与总线/时序/打印焊死 |
| 并发模型 | FreeRTOS + ISR 信号量 + 事件驱动 | 裸 `while(1)`（`fusion_anchor.c:2787`、`fusion_tag.c:1967`） |
| 协议文档 | 独立 md，字段级语义 | 语义散落在 `.c` 注释中 |

---

## 1. 使用场景与量化目标

### 1.1 场景

| 场景 | 锚数量 | 标签数量 | 特征 |
|---|---|---|---|
| 矩形粮库 | 6（基准） | ≤16 | 线缆遮挡、粮堆遮挡 |
| 圆筒仓 | ≤16 | ≤16 | 锚安装在**不同高度**、遮挡严重、空间非凸 |

### 1.2 目标指标（分两级）

| 级别 | 指标 | 目标 | 说明 |
|---|---|---|---|
| **L1 必须达成** | 定位输出可用率 | ≥95% 的采样点给出非退化坐标 | 排除"恒等于质心" |
| L1 | 坐标输出频率 | ≥20 Hz | 机器人导航要求 |
| L1 | 组网规模 | 8 锚 / 16 标签（本期），架构支持 16 锚 | `DL_MAX_TRACKED_ANCHORS 8` |
| L1 | 锚不同高度 | 支持真 3D 解算（Z 不恒定） | 需 ≥5 锚且垂直方向信息量达标 |
| **L2 目标** | 静态定位误差 | 30~50 cm | 现实可达 |
| L2 | 动态定位误差 | ≤1 m @ 1 m/s | 20 cm 在无公共 38.4 MHz 参考下不可达（见 §1.3） |

### 1.3 精度物理下限（必须写进验收文档）

无公共参考源时，各锚晶振独立，典型 20 ppm；Anchor→Anchor 时基偏差随同步龄期增长：

```
误差 ≈ 频率偏差 × 同步龄期
     = 20e-6 × 0.15 s × 3e8 m/s ≈ 0.9 m   （极端）
     = 1e-6  × 0.05 s × 3e8 m/s ≈ 1.5 cm  （理想）
```

**结论**：20 cm 目标必须依赖「公共 38.4 MHz 参考 + 严格同步龄期控制」。本方案为**纯软件路线**，承诺 30~50 cm；20 cm 作为"若硬件后续加参考源的预留接口"处理（§4.8 预留 `common_ref_valid` 标志位）。

---

## 2. 现状诊断：根因清单

### 2.1 已确认的根因（带证据）

| # | 根因 | 证据 | 后果 |
|---|---|---|---|
| R1 | **时间基准被绑死在 Root 绝对网格** | `fusion_tag.c:1511`（`slot_eff = DTU_PER_SLOT × s_root_rate`）、`fusion_tag.c:1510` | Root 挂 / 选举乱 / slot=255 → TDOA 输入退化 |
| R2 | **单值字段争用**：`tdoa_now/tdoa_avg/tdoa_valid` 只有一份，slot 路径与 ToF 路径语义不同 | `fusion_tag.c:164`~`166` | 两条路径互相污染，越调越乱 |
| R3 | **求解器输入退化 → 输出恒等于锚质心** | 种子=锚质心 `tdoa_solver.c:408`；GN 残差对退化输入不敏感 | "坐标一直是质心"是**输入退化的数学指纹**，非求解器 bug |
| R4 | **4 锚强制 2D** | `tdoa_solver.c:432`（`if (n < 5) use_2d = 1`） | 4 锚永远解不出 Z；且 4 锚恰定，5 cm TDOA 噪声 → 94 cm 位置误差（源码注释自述 `tdoa_solver.c:426`） |
| R5 | **物理合理性门限形同虚设** | `fusion_tag.c:591`~`592`（±150 m 固定窗） | 任意离谱 TDOA 都能进解算 |
| R6 | **时钟校正接受带宽过宽** | `clock_sync.h:73`（`CS_FACTOR_CLAMP=2e-6`，即 2 ppm） | 比 bitcraze 的 30 ppb 宽 **60 倍**，离群值污染 α |
| R7 | **锚间选举/层级是主要故障源** | `fusion_anchor.c:711`、`fusion_anchor.c:745` | 单向链路 → 选举失败 → 无 Root → 全网失效 |
| R8 | **巨量函数 + 裸循环** | `fusion_anchor.c:3111` 行 / `fusion_tag.c:1957` 行；`while(1)` @ `fusion_anchor.c:2787` | 无法并行验证两套方案；收发/定时/打印互相阻塞 |
| R9 | **配置持久化空桩** | `gateway_cmd.c:3329`~`3341`；`fusion_anchor.c:43` | 坐标无处落盘，重启丢失 |
| R10 | **权重恒为 1.0**，残差超标一次性 `*=0.5` | `tdoa_solver.c:159`、`tdoa_solver.c:191`、`tdoa_solver.c:250` | 噪声锚与好锚等权 |

### 2.2 尚未排除的候选根因（Phase 0 必须一次性判掉）

| # | 候选 | 吻合度 | 排查方法 | 成本 |
|---|---|---|---|---|
| U1 | delayed TX 帧内 `tx_timestamp40` 是**计划时刻**还是**实际出发时刻** | 高 | 读 `DL_TS_TX_TS_IDX` 与 slot 计划值直接比对 | 低 |
| U2 | `tx_err` 符号/量纲与 slot 补偿重复计入 | 高 | 打印 `tx_err` 原值 + 补偿后差值，看是否随 slot 单调 | 低 |
| U3 | 锚实际摆放 ≠ 配置坐标 | 高 | Phase 0 台架强制对齐坐标 | 零 |
| U4 | Tag 落在锚凸包内 → 法方程病态 | 中 | 凸包内/外各测一次对比 | 低 |
| U5 | Tag 侧锚坐标单位与 `CM_PER_DTU` 不同系 | 已排除 | `shared_defines.h:46` = 0.46907 cm/DTU，与 499.2 MHz×128 一致 | — |

### 2.3 诊断结论（决定方案形态）

> 求解器层（`tdoa_solver.c`）已是成熟实现：Gauss-Newton 12 次迭代、`Z_INFO_MIN` 垂直信息量判据、2D/3D 自适应、多起点（上次解 + 质心）、离群重加权。
> **因此求解器不是当前瓶颈，算法加固必须排在测量域修正之后。**

---

## 3. 目标架构总览

### 3.1 分层图（对标 bitcraze）

```
┌──────────────────────────────────────────────────────────────────────┐
│ L5  工具/客户端   上位机(PyQt) │ 网关固件 │ 地面工具 CLI            │
│     · 在线列表   · 坐标配置   · 组网选站   · 地图/轨迹/质量           │
├──────────────────────────────────────────────────────────────────────┤
│ L4  服务层       roster 分发 │ 坐标分发 │ 日志/诊断变量 │ 持久化      │
│     （与 UWB 算法解耦，对标 bitcraze LPP + cfg）                      │
├──────────────────────────────────────────────────────────────────────┤
│ L3  协议层       dl_proto.c（编解码） │ downlink_protocol.h（字段语义） │
│     · 定位帧 Type A │ 距离帧 Type B │ 组网帧 Type C                  │
├──────────────────────────────────────────────────────────────────────┤
│ L2  算法层       uwb_tdoa_anchor.c │ uwb_tdoa_tag.c │ tdoa_solver.c    │
│     · 事件驱动状态机 · 成员/roster · α 校正 · TDOA 测量域 · 解算     │
├──────────────────────────────────────────────────────────────────────┤
│ L1  无线抽象层   uwb.c（IRQ→事件分发/定时/队列）  ← 新增，Phase 6    │
├──────────────────────────────────────────────────────────────────────┤
│ L0  HAL/驱动     deca_device_api.c │ deca_spi.c │ port.c             │
└──────────────────────────────────────────────────────────────────────┘
```

### 3.2 五个架构原则（不可违反）

| 原则 | 说明 | 对应根因 |
|---|---|---|
| **P-1 无全局时间基准** | 任何测量路径都**不得**引用 Root 网格或 slot 索引 | R1 R2 |
| **P-2 耦合半径 ≤ 1 跳** | 锚只依赖"它直接能听到的锚"；roster 由外部（上位机）下发，不做运行时选举 | R7 |
| **P-3 调度域与测量域分离** | 调度（防碰撞）可以任意，测量（TDOA）只由"自报 TX 时刻 + 逐锚 α"决定 | R1 R2 |
| **P-4 单一数据路径** | 任何时刻只有一条 TDOA 路径喂解算器，切换点在**唯一一处** | R2 |
| **P-5 降级可见** | 每层输出可观测量（α 残差、样本数、丢包率、门限拒绝数），禁止"静默失败" | 全部 |

### 3.3 目标数据流

```
Anchor（每锚独立、无 Root）
  ├─ 周期 TX 定位帧 { id, seq, tx_ts40, tx_err, pos(x,y,z), flags }
  └─ 接收 Tag/锚帧 → 维护 per-peer α、rx 统计
                ↓
Tag（sniffer + 本地解算）
  ├─ 收帧 → 记录 rx_ts40(本地钟) + 解出远端 tx_ts40 + tx_err
  ├─ 逐锚 α：M_k(t) = ref_rx_k + factor_k × (t − ref_tx_k)
  ├─ TDOA(A,B) = (rx_A − TX_A) − (rx_B − TX_B)，TX_A = M_A(tx_A_raw) + tx_err_A
  ├─ 门限：|TDOA| ≤ d(A,B) + margin
  ├─ 解算：≥5 锚 3D / 4 锚 2D / <4 锚不出解
  └─ 输出 (x,y,z,Q) @ ≥20 Hz → 机器人 + 上位机
```

## 4. 核心机制设计

### 4.1 时间基准：放弃 Root 绝对网格

#### 4.1.1 现状

```c
/* fusion_tag.c:1510 ~ 1512  （现状，等价表达） */
toa_eq      = rx_local − slot × slot_eff − tx_err;   /* slot_eff = DTU_PER_SLOT * s_root_rate */
s_root_rate = Root 的 TS 端点比值中位数;               /* fusion_tag.c:1455~1483 */
```

问题：`s_root_rate` 是**一个全局公共因子**，而每个锚的本地钟相对 Root 的偏差是**各自独立**的。公共因子无法消除各锚的独立 skew；同时 slot 索引把测量误差与 slot 大小耦合（slot 越大误差越大）。

#### 4.1.2 目标

```c
/* 目标：每锚独立映射，Tag 本地钟域 */
TX_A = M_A(tx_A_raw) + tx_err_A
     = (ref_rx_A + factor_A × (tx_A_raw − ref_tx_A)) + tx_err_A
```

其中 `factor_A`、`ref_tx_A`、`ref_rx_A` 来自 **Tag 侧 per-anchor clock_sync**（`clock_sync.h:90` 的相对形式已具备，只需收紧门限）。

#### 4.1.3 误差特性对比（为什么更好）

| | slot 网格路径 | per-anchor 映射路径 |
|---|---|---|
| 误差来源 | Root 公共因子 + slot 索引耦合 | 各锚 α 的**参考点年龄** |
| slot 越大 | 误差越大 | **无关** |
| Root 挂掉 | 全网失效 | **不受影响** |
| 需要锚↔锚链路 | **需要** | **不需要** ✅ |
| 锚数增加 | slot 表膨胀 | 每个锚独立，互不影响 ✅ |

#### 4.1.4 保留 slot 表的用途（降级）

slot 表**不删除**，改为：
1. 调度相位表（防碰撞），来源改为**上位机下发的 roster**
2. 自检用（观测某锚是否在预期时隙出现）

标志位 `DL_TS_FLAG_TX_OFFGRID`（`downlink_protocol.h:343`）在 delayed TX 失败时置位，该帧的 TDOA 必须丢弃。

---

### 4.2 协议层：定位帧 Type A（核心，帧长不变）

#### 4.2.1 关键约束

* **帧长保持 67 B**，不触碰 `DL_TS_TAIL_PAD_IDX 65`（DW3210 会破坏最后 2 字节，`downlink_protocol.h:305`、`:313`）
* 不新增 Anchor→Anchor 转发语义（bitcraze TDoA3 的 `remoteCount×{id,seq,rxTs,distance}` 中继格式在本期**不引入**，见 §4.2.5）

#### 4.2.2 定位帧字段表（Type A）

| 偏移 | 字段 | 类型 | 长度 | 说明 | 现状 |
|---|---|---|---|---|---|
| 0 | `frame_type` | u8 | 1 | `0xA1` = 定位帧 | 新增 |
| 1 | `anchor_id` | u16 | 2 | 锚 ID（0..65535） | 已有语义 |
| 3 | `seq` | u8 | 1 | 该锚发送序号（0~127 循环） | 新增 |
| 4 | `tx_ts40` | u40 | 5 | 本帧 TX 时刻（发送方钟域，低 40 bit） | **已有** `DL_TS_TX_TS_IDX:317` |
| 9 | `tx_err` | i16 | 2 | 相对 slot 网格的出发误差 | **已有** `DL_TS_TX_ERR_IDX:321` |
| 11 | `pos_x` | i16 | 2 | cm | 已有 |
| 13 | `pos_y` | i16 | 2 | cm | 已有 |
| 15 | `pos_z` | i16 | 2 | cm | 已有 |
| 17 | `flags` | u8 | 1 | 见 §4.2.3 | 部分已有 |
| 18 | `slot` | u8 | 1 | 调度相位/时隙（**仅调度用，测量不依赖**） | 已有 |
| 19 | `cycle_id` | u16 | 2 | 超帧计数（**仅观测/配对用**） | 已有 `downlink_protocol.h:323` |
| 21 | `tx_rate_ppb` | i16 | 2 | 发送方钟频偏差估计（ppb） | **新增**，用于 Tag 侧 α 先验 |
| 23 | `tx_err_valid`/`offgrid` | u8 | 1 | 复用 flags | 已有 |
| 24..64 | 保留 | — | 41 | 预留 Type B 扩展 | 已有 roster 区 |

#### 4.2.3 flags 位定义（收敛版）

| 位 | 名称 | 语义 | 现有常量 |
|---|---|---|---|
| 0 | `TX_TS_PRECISE` | tx_ts40 是精确发射时刻（非估计） | `DL_TS_FLAG_TX_TS_PRECISE`（`downlink_protocol.h:341`） |
| 1 | `TX_ERR_VALID` | tx_err 有效 | `DL_TS_FLAG_TX_ERR_VALID`（`:342`） |
| 2 | `TX_OFFGRID` | delayed TX 失败/未按计划 → **整帧 TDOA 作废** | `DL_TS_FLAG_TX_OFFGRID`（`:343`） |
| 3 | `POS_VALID` | 坐标有效 | 新增 |
| 4 | `CALIB_VALID` | 天线延迟已标定 | 新增 |
| 5 | `NET_MEMBER` | 属于当前 roster | 新增 |
| 6 | `COMMON_REF` | 外部公共 38.4 MHz 参考有效（预留） | 新增 |
| 7 | 保留 | — | — |

#### 4.2.4 Tag 侧使用规则（严格）

```
if (frame.flags & TX_OFFGRID)        → 丢弃整帧（不参与 TDOA，不更新 α 样本）
if (!(frame.flags & TX_TS_PRECISE))  → 不用于 TDOA，可用于 α 估计
if (!(frame.flags & NET_MEMBER))      → 忽略（不 roster 内的锚）
```

#### 4.2.5 为什么本期不引入 bitcraze 的 ToF 中继

bitcraze TDoA3 用**锚间 ToF（distance 字段）**消除 α 的常数偏置；本方案用 **per-anchor 映射**达到同一目的，且**不需要锚↔锚链路**（当前实测存在单向不可达）。

代价：α 的常数偏置残留，表现为系统性的距离偏置。**缓解手段**（Phase 5）：
* 收紧 α 门限到 30 ppb（bitcraze 量级）
* 物理门限用 `d(A,B) + margin` 吸收残余偏置
* 中值 + Huber 鲁棒均值抑制残余偏置的影响

**终态（Phase 7，预留）**：若后续修复锚↔锚链路，可扩 `dl_anchor_entry_t` 增加 `distance` 字段，把 ToF 中继引入，实现 bitcraze 原生形态，届时 Root 彻底降级为纯配置分发者。

---

### 4.3 时钟校正层：对齐 bitcraze 的三道机制

#### 4.3.1 bitcraze 参照实现

```c
/* bitcraze src/uwb_tdoa_anchor3.c */
#define CLOCK_CORRECTION_ACCEPTED_NOISE 0.03e-6   /* ±30 ppb 接受窗 */
#define CLOCK_CORRECTION_FILTER 0.1d              /* 一阶 LP 系数 */
#define CLOCK_CORRECTION_BUCKET_MAX 4             /* 漏桶上限 */
```

流程（`updateClockCorrection` :434）：
1. 计算 `diff = new_correction − current`
2. `|diff| < 30 ppb` → **接受**：LP 更新 `c = c×0.9 + new×0.1`，漏桶 +1
3. 否则 → **拒绝**：漏桶 −1；漏桶空且新值在规格窗内 → **整体重置**为新值（说明当前值已错）

#### 4.3.2 本项目目标参数

| 参数 | 现值 | 目标 | 位置 |
|---|---|---|---|
| α 接受带宽 | `CS_FACTOR_CLAMP = 2e-6` (2 ppm) | `3e-8` (30 ppb) | `clock_sync.h:73` |
| 异常漏桶上限 | 无 / `CS_GATE_MAX_REJECT=8` | 4（对齐 bitcraze） | `clock_sync.h:125` |
| 参考点最大龄期 | `CS_REF_MAX_AGE_DTU = 3e10` (0.47 s) | 保留，但**龄期需上报** | `clock_sync.h:102` |
| 滤波 | Kalman（`clock_sync.c`） | 保留 Kalman，**加 LP 前置门** | — |

#### 4.3.3 分阶段收敛（避免"收紧即锁死"）

| 阶段 | 接受带宽 | 目的 |
|---|---|---|
| Phase 1 | 2 ppm（保持现值） | 先跑通测量域，收集 α 残差分布 |
| Phase 2 | 500 ppb | 观察收敛速度 |
| Phase 3 | 100 ppb | — |
| Phase 4 | 30 ppb（目标） | 对齐 bitcraze |

每阶段用 `clock_sync` 的残差统计决定是否继续收紧；**若 clamped 率 > 5% 则回退一档**。

#### 4.3.4 可观测性（新增输出）

```
CLOCK k=<anchor> f=<factor> dev=<ppb> n=<samples> age=<dtu> rej=<rejected> reset=<count>
```

---

### 4.4 拓扑与组网：取消运行时选举，改为上位机下发 roster

#### 4.4.1 现状与问题

| 项 | 现状 | 问题 |
|---|---|---|
| Root 选举 | `fusion_anchor.c:404`~`417`（rank 比较）、`fusion_anchor.c:858` | 选举冲突、互覆盖 |
| 层级 | `cs_level` / `parent_rank` | 层级与 slot 表耦合，一处错全网错 |
| 单向链路 | — | 实测 R488↔R209 单向不可达 → 选举必败 |
| slot 表真源 | Anchor 侧 roster（`fusion_anchor.c:711`、`:745`、`:789`、`:794`） | 断链即无 roster |

#### 4.4.2 目标：roster 由上位机/网关统一下发

```
上位机选站 → 网关 → 各锚（Type C 组网帧）
```

| 收益 | 说明 |
|---|---|
| 消除选举冲突 | 不再有运行时选举，rank 概念可从测量路径彻底移除 |
| 消除 Root 依赖 | Root 挂 → 只是少一个 beacon 源，**其余锚照常定位** |
| 适配单向链路 | 只要 Tag 能听到该锚，Tag 就能用；锚↔锚不通不影响测量 |
| 匹配既定业务流程 | 用户流程本就是"上位机显示在线基站 → 配置坐标 → 选站组网" |

#### 4.4.3 组网状态机（上位机侧）

| 状态 | 转移条件 | 超时 | 失败动作 |
|---|---|---|---|
| `NET_IDLE` | 用户点击"组网" | — | — |
| `NET_DISCOVERING` | 收到 ≥N 个锚的在线上报（N=3） | 5 s | 提示"在线基站不足" |
| `NET_CONFIGURING` | 已下发坐标，等待各锚 ACK | 3 s | 重发 ≤3 次 |
| `NET_ACTIVE` | 全部锚 ACK + 至少 4 个成员（3D 需 5） | — | — |
| `NET_PARTIAL` | 部分锚 ACK | 持续重试 | 允许降级运行（成员 <4 则不出解） |

#### 4.4.4 Anchor 侧改动

* 新增 `roster` 接收与持久化（**消除 `fusion_anchor.c:43`、`:225`~`227` 的坐标空桩**）
* **移除**选举/层级逻辑对测量路径的影响；`cs_level`、`parent_rank` 降级为纯诊断字段
* 锚收到 `NET_MEMBER` 集合外锚的帧 → 忽略（不更新 α、不统计）

#### 4.4.5 组网帧 Type C（网关↔锚，走 SPI 通道）

```
组网帧 = { type(0xC1), roster_id, member_count, n×{anchor_id, pos_x, pos_y, pos_z, slot, flags}, crc }
```

---

### 4.5 调度层：伪随机相位（替代 Root 网格）

#### 4.5.1 原则

调度只解决"谁在什么时候发"，**测量完全不看调度**（P-3）。

#### 4.5.2 相位分配算法

每锚在 100 ms 超帧内的发射相位由 **anchor_id 派生的确定性伪随机偏移**决定：

```
phase(anchor_id) = (hash(anchor_id) mod N_PHASE) × (SUPERCYCLE_MS / N_PHASE)
```

| 参数 | 值 | 理由 |
|---|---|---|
| `SUPERCYCLE_MS` | 100 | 保持现值 `downlink_protocol.h:181` |
| `N_PHASE` | 8 | 16 锚时每相位 2 锚，碰撞概率低 |
| 相位抖动 | ±5 ms 随机 | 避免同相位锚每次完全对齐 |

优点：无主（各锚独立算自己的相位，只需 anchor_id）、周期确定性保留、可在上位机显示调度表。

#### 4.5.3 与 delayed TX 的关系

delayed TX 仍然使用，但**目标时刻由本地相位表算出**，不依赖 Root 广播：

```
target = 当前超帧起点 + phase(anchor_id) + jitter
```

若 delayed TX 失败 → 置 `TX_OFFGRID`，该帧作废，**绝不回退到"用 slot_eff 硬补"**（那会重新引入 Root 网格）。

---

### 4.6 TDOA 测量域：核心公式与实现

#### 4.6.1 目标计算式

```
TX_A(tag域) = M_A(tx_A_raw) + tx_err_A
M_A(t)      = ref_rx_A + factor_A × (t − ref_tx_A)
TDoA(A,B)   = (rx_A − TX_A) − (rx_B − TX_B)
```

`M_A` 就是 `clock_sync.h:90` 已有的相对映射形式，**无需新公式**，只需在 Tag 侧对每个锚各维护一份。

#### 4.6.2 关键点：传播延迟自动对消

`rx_A = TX_A + d_A/c`（d_A 为 Tag↔锚 A 距离），代入上式：

```
rx_A − TX_A = M_A(tx_A_raw) + tx_err_A 映射误差 + d_A/c
```

两锚相减后 `d_A/c − d_B/c` **不会**自动对消——这正是 bitcraze 用 ToF 中继解决的问题。本方案用**物理门限 + 鲁棒均值 + 残差诊断**吸收残余（§4.7）。**必须在文档中标注这是本方案的已知精度损失来源。**

#### 4.6.3 实现位置

| 改动 | 位置 | 说明 |
|---|---|---|
| Tag 侧维护 per-anchor α | `fusion_tag.c` 新增 `s_peer_alpha[]`（与 `s_trackers[]` 同生命周期） | 现有 `clock_sync` 已提供更新接口 |
| 计算 `TX_A` | 替换 `fusion_tag.c:1510`~`1530` 的 `toa_eq` | 用 `M_A(tx_raw) + tx_err` |
| 计算 TDOA | 替换 `fusion_tag.c:1536`（`tdoa_now = toa_eq − s_root_toa_eq`） | 改为 `(rx_A−TX_A) − (rx_B−TX_B)` |
| 剥离 slot 补偿 | `fusion_tag.c:605`、`fusion_tag.c:1511` 的 `slot_eff`/`s_root_rate` 从 TDOA 路径移除 | 保留给调度与自检 |
| 单一数据路径开关 | **唯一切换点** `fusion_tag.c:1536`（P-4） | 编译期 `FEATURE_TDOA_PATH_SELFTEST` |

#### 4.6.4 同轮配对

不同锚的帧不在同一时刻发出 → **不能用 `cycle_id` 配对**（现 `fusion_tag.c:1535`）。改用：

* 每锚独立 `seq`（0~127 循环）
* TDOA 计算取**各锚最新一帧**，不要求同时刻
* 有效性判据：两锚帧的 `cycle_id` 差 ≤ 1（或时间差 ≤ `TDOA_MAX_PAIR_AGE_DTU`）

#### 4.6.5 单位与符号校验（Phase 0 必做）

* `CM_PER_DTU_F = 0.46907`（`shared_defines.h:46`）✔ 已核实
* `tx_err` 符号：定义为「实际出发时刻 − 计划时刻」，则 `TX = M(tx_raw) + tx_err` **成立**；若定义相反则取负。**必须在 Phase 0 用实测确认**（对应 U2）

---

### 4.7 解算层：门限、鲁棒、维度、多解

#### 4.7.1 物理合理性门限（最高性价比改造）

替换 `fusion_tag.c:591`~`592` 的 ±150 m 固定窗：

```
|TDOA_cm| ≤ d(A,B)_cm + margin_cm,   margin = 100 cm（可配）
```

`d(A,B)` 由**配置坐标**算出（上位机已下发）。跨锚不可靠时 margin 放宽到 200 cm。

#### 4.7.2 鲁棒平均：EMA → 中值 + Huber

现 `fusion_tag.c:1544` 为简单 EMA。目标：

1. 维护最近 N=8 个 TDOA 样本窗口
2. 取中值作为初值
3. Huber 迭代加权（k=1.345 × MAD）2 次

#### 4.7.3 维度策略

| 有效锚数 | 策略 | 依据 |
|---|---|---|
| < 4 | **不出解**（保持上一解 + Q 恶化） | 3 个 TDOA 方程无法定 4 个未知量（x,y,z + 参考） |
| = 4 | 强制 2D（Z 取上次的 z 或锚平均高度） | `tdoa_solver.c:432` 现有策略；4 锚恰定对噪声极敏感（注释自述 5 cm→94 cm） |
| ≥ 5 | 3D，且用 `z_info ≥ Z_INFO_MIN` 判据 | `tdoa_solver.c:64`、`:75` |

**因此：面向"锚不同高度"的真 3D 场景，台架必须 ≥5 锚。**

#### 4.7.4 权重：恒等权 → IRLS（Phase 4）

现 `tdoa_solver.c:159` 恒 `weight[k]=1.0f`、`:191`/`:250` 残差超标一次性 `*=0.5`。

目标 Huber/Tukey 权重 + 位置一致性自适应门限（对标 bitcraze 第四道防线）：

```
if (|residual| 与当前估计位置不一致) → 该样本剔除
若剔除比例 > 40% → 认为当前估计已错 → 放宽门限重解（bitcraze 机制）
```

#### 4.7.5 多解假设 + 轨迹连续性

圆筒仓/凸包内存在多解。保留最近 K=3 个假设解，用速度门限（≤3 m/s）选择连续者；连续 3 次无新解则冻结输出并置 Q=低。

#### 4.7.6 位置滤波与输出

* α-β 滤波（`fusion_tag.c:1415` 写 `pos.z_cm` 处接入）
* 输出 `(x, y, z, Q)`，`Q` 由 `n_inlier_out`/`rms_out`（`tdoa_solver.c:353`、`:510`）映射

---

### 4.8 预留：公共参考接口

`flags` bit6 `COMMON_REF` 预留。若后续接入公共 38.4 MHz：

* Anchor 上报 `common_ref_valid`
* Tag 侧在 `factor_A` 上叠加公共参考项
* α 接受带宽可从 30 ppb 进一步收紧到 1~3 ppb

此为**不改硬件前提下**为 20 cm 目标预留的唯一路径。

---

## 5. 分层重构（Phase 6，架构解耦）

### 5.1 目标文件结构

```
shared/
  uwb/
    uwb.c                 L1 无线抽象：IRQ→事件、定时队列、收发状态
    uwb_tdoa_anchor.c     L2 Anchor 算法状态机
    uwb_tdoa_tag.c        L2 Tag 算法状态机
    uwb_proto.c           L3 编解码
  shared_data/
    downlink_protocol.h   L3 字段语义（唯一真源）
    clock_sync.h/.c       L4 逐锚 α
  downlink/
    tdoa_solver.c         L4 解算
```

### 5.2 `uwbAlgorithm_t` 接口（对标 bitcraze）

```c
typedef struct {
    void     (*init)(const uwb_config_t *cfg, dw_context_t *dev);
    uint32_t (*on_event)(dw_context_t *dev, uwb_event_t ev);  /* 返回下次超时 ms */
} uwb_algorithm_t;

typedef enum {
    UWB_EV_PACKET_SENT = 0,
    UWB_EV_PACKET_RECEIVED,
    UWB_EV_RX_TIMEOUT,
    UWB_EV_RX_FAILED,
    UWB_EV_TIMER,
} uwb_event_t;
```

### 5.3 事件来源

现有 `deca_device_api.c` 已具备中断回调能力；需新增：

* `EXTI` IRQ → 置 flag + 记录 `rx_timestamp`（**在 ISR 内立刻读时间戳**，这是保证 RX 精度不被中断延迟污染的关键）
* 主循环改为 `while(1) { if (flags) { handle_event(); } else { do_idle(); } }`

### 5.4 为什么 Phase 6 排在最后

重构本身不产生功能收益，且风险高（改错时序 → TX 失败率上升）。必须先用 Phase 0~5 把功能跑通、行为正确，**再**做纯结构重构，且每一步保持可回滚。

---

## 6. 上位机与网关

### 6.1 功能清单（对齐用户既定流程）

| 序 | 功能 | 说明 |
|---|---|---|
| 1 | 显示在线锚/标签 | 基于心跳/统计帧，含 RSSI（若驱动可读） |
| 2 | 配置锚坐标 | **持久化到网关侧 Flash**（消除 `gateway_cmd.c:3329` 空桩） |
| 3 | 选站组网 | 下发 roster + 坐标 + 相位表（§4.4.3 状态机） |
| 4 | find 功能 | 分批闪烁，避免同时闪（当前 bug） |
| 5 | 地图显示 | 锚位置 + 标签轨迹 + 解算质量 Q |

### 6.2 坐标持久化方案

锚坐标是**静态配置**，天然属于网关/上位机侧：

* 网关 Flash：`<anchor_id, x, y, z, crc>` 记录表
* 锚本体：只存**运行时态**（roster 副本、α 状态），不存坐标
* 好处：坐标改了立即生效；锚重启无需重新配置

### 6.3 find 功能去冲突

当前"三个基站同时频闪"的根因是 find 命令被**广播给所有锚**。方案：

* find 命令携带 `token`（16 bit 随机）+ `nonce`
* 锚收到后，若 `token` 与自身 `find_token` 不符 → 忽略
* 上位机逐个发（带 200 ms 间隔）→ 每次只有一站闪

---

## 7. 参数总表

| 参数 | 现值 | 目标 | 位置 | 阶段 |
|---|---|---|---|---|
| `CM_PER_DTU_F` | 0.46907 | 不变 | `shared_defines.h:46` | — |
| `CS_FACTOR_CLAMP` | 2e-6 | 3e-8 | `clock_sync.h:73` | Phase 4 |
| `CS_GATE_MAX_REJECT` | 8 | 4 | `clock_sync.h:125` | Phase 4 |
| `CS_REF_MAX_AGE_DTU` | 3e10 | 不变 + 上报 | `clock_sync.h:102` | Phase 1 |
| 物理门限 margin | 150 m | 1 m | `fusion_tag.c:591` | Phase 2 |
| 鲁棒窗口 N | — | 8 | 新增 | Phase 3 |
| 最少锚数（3D） | 5 | 5 | `tdoa_solver.c:432` | — |
| `Z_INFO_MIN` | 0.02 | 不变 | `tdoa_solver.c:64` | — |
| 超帧 | 100 ms | 不变 | `downlink_protocol.h:181` | — |
| 相位数 | — | 8 | 新增 | Phase 2 |
| 定位帧长度 | 67 B | **67 B（不变）** | `downlink_protocol.h:305` | — |

---

## 8. 实施路线

### Phase 0：黄金台架 + 根因判定（不改变任何行为）

**目标**：把"输出恒等于质心"变成可判定信号，并判掉 U1~U4。

| 项 | 内容 |
|---|---|
| 台架 | **≥5 锚**（真 3D 必需），固定坐标且**物理摆放与坐标一致**；Tag 固定在已知点；手工量出各锚到 Tag 的距离作真值 |
| 新增打印 | `TDOA_RAW A=<id> tx_raw=.. tx_map=.. tx_err=.. rx=.. toa=.. tdoa_cm=.. d_ij=..` |
| 判定 | ① `d_ij` 与手工真值一致 → 几何/坐标无问题；② `tx_map − tx_raw` 随 slot 单调变化 → 证实 slot 网格污染；③ `tx_err` 符号验证 U2；④ 输出恰好等于锚质心 → 输入退化确认 |
| 验收 | 拿到 5 组以上的 `TDOA_RAW` 数据，能指出 R1/R2/U1/U2 中哪一条在起作用 |
| 回滚 | 无（纯诊断） |

### Phase 1：切测量域到 per-anchor 映射（Core）

| 项 | 内容 |
|---|---|
| 改动 | §4.6.3 的 4 处；`FEATURE_TDOA_PATH_SELFTEST` 编译开关；**保留旧路径可回退** |
| 新增 | Tag 侧 `s_peer_alpha[]`；`tx_map` 打印 |
| 验收 | TDOA(dtu) 与手工真值偏差 < 5 cm（= 约 10.7 DTU）；输出不再恒等于质心 |
| 回滚 | 关闭编译开关即可 |

### Phase 2：拓扑解耦 + 调度相位

| 项 | 内容 |
|---|---|
| 改动 | 上位机下发 roster（Type C）；锚侧持久化 roster；伪随机相位；选举降级为诊断字段 |
| 验收 | 拔掉任意一锚，其余锚仍能定位（不再全网失效）；单向链路场景可正常工作 |

### Phase 3：鲁棒与门限

| 项 | 内容 |
|---|---|
| 改动 | 物理门限 `d(A,B)+margin`；中值+Huber 替代 EMA；最小锚数门槛 |
| 验收 | 注入 1 个错误 TDOA 样本，解算位置不跳变；跳变率下降 ≥50% |

### Phase 4：时钟收紧 + IRLS

| 项 | 内容 |
|---|---|
| 改动 | α 门限分档收紧；IRLS 权重；位置一致性自适应门限 |
| 验收 | α 残差 σ 下降；静态点位重复性（10 次）标准差 ≤ 15 cm |

### Phase 5：标定与输出

| 项 | 内容 |
|---|---|
| 改动 | 天线延迟标定（两锚互测残差法）；α-β 滤波；`Q` 映射；`n_inlier_out`/`rms_out` 上报 |
| 验收 | 静态定位误差 ≤ 50 cm |

### Phase 6：分层重构

| 项 | 内容 |
|---|---|
| 改动 | §5 全部；`fusion_*.c` 拆分为 `uwb.c` + `uwb_tdoa_anchor.c` + `uwb_tdoa_tag.c` + `uwb_proto.c` |
| 验收 | **全部功能回归通过**；delayed TX 失败率不劣化；周期抖动不劣化 |
| 风险 | 高。必须逐函数迁移，每步保持可编译可运行 |

### Phase 7（可选）：ToF 中继，终态

扩 `dl_anchor_entry_t` 加 `distance`；锚↔锚 ToF；根除 α 常数偏置；Root 降级为纯配置分发。

---

## 9. 测试与验收清单

| 编号 | 测试 | 通过标准 |
|---|---|---|
| T1 | Phase 0 台架 TDOA 真值比对 | 偏差 < 5 cm |
| T2 | 输出非退化检查 | 连续 100 个采样点不全等于锚质心 |
| T3 | 单锚掉线 | 其余 ≥4 锚仍出解 |
| T4 | 单向链路台架 | 仅 Tag↔锚可达时仍可定位 |
| T5 | 坐标输出频率 | ≥20 Hz |
| T6 | 静态精度（5 点） | 每点误差 ≤50 cm |
| T7 | 动态精度（2 m/s） | 滞后 ≤1 m |
| T8 | 3D 有效性 | Z 随 Tag 高度变化，非恒定 |
| T9 | delayed TX 失败率 | ≤0.5%（现状目标 0） |
| T10 | 扰动鲁棒性 | 遮挡/断链 30 s 内恢复 |
| T11 | 组网全流程 | 上位机：显示在线 → 配坐标 → 选站组网 → 地图轨迹 |
| T12 | 锚数扩展 | 5→8 锚无需改代码 |

---

## 10. 风险登记册

| ID | 风险 | 概率 | 影响 | 缓解 |
|---|---|---|---|---|
| K1 | Phase 1 切换后 TDOA 反而更差 | 中 | 高 | 保留旧路径编译开关；Phase 0 先拿数据 |
| K2 | α 收紧导致不收敛 | 中 | 中 | 分档收紧；clamped 率 >5% 回退 |
| K3 | `tx_err` 符号判反 | 中 | 高 | Phase 0 实测判定 |
| K4 | 4 锚场景用户期望 3D | 高 | 中 | 明确告知需 ≥5 锚；UI 提示 |
| K5 | Phase 6 重构引入时序回归 | 高 | 高 | 逐函数迁移；每步 T9 必测 |
| K6 | 物理门限误杀合法大跨度 TDOA | 低 | 中 | 用 `d(A,B)+margin`；margin 可配 |
| K7 | 圆筒仓多径导致 NLOS 偏差 | 高 | 中 | 引入 NLOS 检测 + 剔除（Phase 4/5） |
| K8 | 上位机↔网关坐标协议不一致 | 中 | 中 | 协议字段集中到 `downlink_protocol.h` |

---

## 11. 未决问题（需业务/硬件确认）

| # | 问题 | 影响 | 建议默认值 |
|---|---|---|---|
| Q1 | 锚间链路单向不可达是**遮挡导致**还是**天线/摆放**导致？ | 决定 Phase 7 是否可行 | 视为遮挡，Phase 7 不纳入本期 |
| Q2 | 驾驶是否接受"单锚掉线即需重新组网"？ | 决定 roster 是否需热更新 | 支持热更新（自动剔除失联成员） |
| Q3 | 上位机是否接受坐标存网关侧（锚不存）？ | 决定 Phase 2 存储设计 | 接受 |
| Q4 | 定位输出协议（Tag→机器人）现状是什么？ | 决定 §4.7.6 接口 | 保持现状，仅补 Q 字段 |
| Q5 | 是否有历史轨迹回放需求？ | 决定是否需要 Tag 侧环形缓冲 | 暂无 |

---

## 12. 附：对标结论摘要

| bitcraze 机制 | 本方案对应 | 采纳 |
|---|---|---|
| 无主、无选举、随机 TX | 上位机下发 roster + 伪随机相位 | ✅ 采纳（形态不同，耦合半径同样 ≤1 跳） |
| 逐远端 α + 30 ppb 门 + LP + 漏桶 | `clock_sync` 收紧对齐 | ✅ 采纳 |
| 物理门限 `\|TDoA\| ≤ d(A,B)` | §4.7.1 | ✅ 采纳（最高性价比） |
| 位置一致性自适应门限 | §4.7.4 | ✅ 采纳 |
| 序列号一致性 | 每锚 `seq` + 配对龄期判据 | ✅ 采纳 |
| 天线延迟显式补偿 | Phase 5 标定 | ✅ 采纳 |
| ToF 中继消偏置 | Phase 7（预留） | ⏳ 延后（需锚↔锚链路） |
| 算法可插拔 + 事件驱动 + RTOS | Phase 6 分层；**不引入 RTOS** | ⚠️ 部分采纳 |
| Tag 定位放客户端 | **不采纳**（Tag 本地解算更适合导航） | ❌ |

**一句话**：借鉴 bitcraze 的**算法层与协议纪律**，不借鉴它的**架构假设**（无 RTOS、锚间中继），并保留本项目**Tag 本地解算**这一更适合机器人导航的差异点。

