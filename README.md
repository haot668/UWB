# UWB TDOA 下行定位系统

基于 DW3210/DW3000 的**下行 TDOA** 室内定位系统：多个 Anchor 在统一 TDMA 栅格上轮流广播信标，Tag 收齐一轮信标后本地解算自身三维坐标，坐标经 UWB 上行报给网关，网关再以 ASCII 转发给上位机（Python/Tkinter）。Tag 同时通过串口把坐标直接输出给机器人控制器。

- 定位算法在 **Tag 本地**完成，UWB 上行只搬运坐标，网关不做解算。
- 锚间时基由 **DS-TWR（双向测距）** 闭合，不是各自自由跑。
- 解算器是 **Gauss-Newton 最小二乘**（不是 Chan-Taylor），带离群剔除、多起点与 z 维可辨识性判定。

---

## ⚠️ 当前验证状态（请先读这一段）

**本仓库的代码尚未编译、尚未烧录、尚未在真实硬件上跑通。**

| 项目 | 状态 |
|------|------|
| 编译 | ❌ 未做（本机无 ARM 工具链，仓库内无 CI） |
| 烧录 / 上机 | ❌ 未做 |
| 真实数据跑通 | ❌ 未做 |
| 语法检查 | ✅ 用 WSL 桩编译脚本做过（仅查语法，不查链接与时序） |
| 上位机 `uwb_host.py` | ✅ `python -m py_compile` 通过 |

仓库里 `uwb_tag/_build_tag.log`、`uwb_anchor/_build_*.log` 是 **8 月 21 日**的历史产物，**早于当前架构**（那时还是 32 B 信标 / 250 µs 槽 / Chan-Taylor），**不能**当作当前代码可编译的证据。

因此本文档描述的是**设计意图与代码现状**，不是**实测结论**。所有精度指标一律以"待上机验证"标注。

---

## 系统组成

| 节点 | 硬件 | 职责 |
|------|------|------|
| **Gateway 网关** | STM32L412 + DW3210 | UWB 嗅探/AT 桥，解析下行帧，向上位机吐 ASCII；承载组网、坐标写入、OTA 广播 |
| **Anchor 基站** | STM32L412 + DW3210 | 同一份固件，运行时扮演 **Root** 或 **Child**；Root 占 slot 0 并发栅格，Child 占分配到的槽 |
| **Tag 标签** | STM32L412 + DW3210 | 被动收信标、时钟对齐、TDOA 解算、串口输出坐标、UWB 上行报告 |
| **Host 上位机** | PC / Python + Tkinter | 地图显示、设备管理、坐标配置、组网向导、固件升级 |

Tag 的消费者是机器人，**Tag 与机器人之间走普通串口**，UWB 上行只用于上位机地图调试。

---

## 分层架构

```
L4 业务/应用层    上位机 uwb_host.py（地图 / 组网 / 配置 / OTA）
      ▲
L3 接入/协议转换   Gateway（UWB 帧 ⇄ ASCII 行，AT 命令解析）
      ▲
L2 节点逻辑层     fusion_anchor.c（Root/Child 角色、栅格、DS-TWR 链路）
                  fusion_tag.c（收信标、槽补偿、解算、上报）
      ▲
L1 算法层         tdoa_solver.c（Gauss-Newton）
                  clock_sync.c（频率比 + 相位偏置卡尔曼/RC 滤波）
      ▲
L0 协议/常量层    downlink_protocol.h / shared_defines.h
      ▲
L-1 平台/驱动层   decadriver（DW3210 驱动）/ platform（Keil 移植层）
```

---

## 运行时序：TDMA 栅格

| 常量 | 值 | 含义 |
|------|-----|------|
| `DL_SLOT_DTU_US` | 6250 | 一个时隙 6.25 ms |
| `DL_TIMESYNC_PERIOD_MS` | 50 | 一轮（round）= 8 槽 × 6.25 ms = 50 ms |
| `DL_SUPERCYCLE_MS` | 100 | 超周期 = 2 轮 |
| `DL_SOLVE_INTERVAL_MS` | 50 | Tag 解算节拍 = 20 Hz |
| `DL_RX_WINDOW_MS` | 100 | Tag 收信标窗口 = 1 个超周期 |
| `DL_MAX_TRACKED_ANCHORS` | 8 | 最多跟踪 8 个 Anchor |
| `NET_MAX_PARTICIPANTS` | 8 | 网络成员上限（Root 含在内） |

- **Root** 固定占 **slot 0**，其余成员按 roster 顺序分配 slot 1..N-1，**确定性分配，不做伪随机相位**。
- 槽序号由**上位机选定**，Root 用 `slot_table`（8 × uint16 rank）广播，全网一致。
- Tag 只用**同一 `cycle_id` 轮次**的信标配对做 TDOA（跨轮配对会引入整轮 50 ms = 3.19 M DTU 的误差，远超接受窗口）。
- **上行保护窗**：Tag 的报告落在 slot 0 的保护窗内，发送时刻 = 收到 Root 信标时刻 + `TAG_REPORT_DELAY_US`(3000 µs) + ALOHA 抖动(0..1200 µs)，即 3.0–4.2 ms，相对 6.25 ms 槽留 2.05 ms 余量。Tag 不占自己的槽。

### 单位换算（务必记住）

| 换算 | 值 |
|------|-----|
| 1 DTU | 15.65 ps = **0.46907 cm** 光程 |
| 1 µs | 63 898 DTU = **299.7 m** 光程 |
| 1 ns | **≈ 30 cm** 光程 |
| `CS_REF_MAX_AGE_DTU` = 3.0e10 | 0.4695 s |

---

## 六阶段主流程

```
① 上电 / 发现      Anchor 广播 0xEC ANCHOR_DISCOVERY → Gateway 记下在线设备
② 配坐标          上位机 AT+COORD=<x>,<y>,<z> → Anchor 存坐标并置 COORD_VALID 位
③ 组网            上位机选 Root + 名单 → AT+NETSTART=<root>,<r1>,<r2>...
                  Gateway 广播 0xE8 NETWORK_START → Anchor 回 0xE9 ACK
                  Root 确认上槽（slot != 255）并开始发栅格
④ 跑栅格          Root slot 0 发起；每 50 ms 一轮，8 槽轮转，20 Hz
⑤ 解算            Tag 收齐一轮 → 槽补偿 → Gauss-Newton → 位置 + 质量
⑥ 上报            Tag 发 0xE3 位置报告（24 B）→ Gateway 转成 +POS 行 → 上位机画地图
```

**组网名单是强制的**：上位机不传 roster 时 Gateway 直接回 `+NETSTART_ERR,NOROSTER`，不再回退到"网关恰好知道的全部 Anchor"。因为没配坐标的 Anchor 会给 TDOA 解算贡献一个假的 `(0,0)` 位置，把结果往原点拽；一个成员不符合预期的网络比没有网络更糟——它看起来是能用的。

---

## 核心算法

### 1. 锚间时钟同步（`shared/shared_data/clock_sync.c`）

模型：`local(remote) = ref_rx + factor × (remote − ref_tx)`

- `factor` 是两钟频率比，用 **CFO + 测量斜率**两种来源，优先用 16 次采样（≈800 ms）的斜率估计。
- `CS_FACTOR_CLAMP` = 2e-6 是**单次校正步长上限**，不是"接受带宽"；`CS_FACTOR_CLAMP_WARMUP` = 50e-6 用于预热期。
- `factor = 1.0 + cfo_raw / 67108864.0`。
- 质量门控：`CS_GATE_MIN_INTERVALS` = 3、`CS_GATE_MAX_REJECT` = 8。

> ⚠️ **这里有一个已被证明的关键物理约束**：参考对 `(ref_tx, ref_rx)` 会把该参考帧的**传播时延**吸收进映射常数项，于是 `rx_A − M_A(tx_A)` 只是路径长度的**变化量**。每帧各自刷新参考时 TDOA ≈ 0。
> ⇒ **绝对 TDOA 不可能由 per-anchor 单程映射得到**，只能来自：① 公共时基 / ② 锚↔锚往返测量 / ③ 有线公共参考。本项目走 ①+②：Root 栅格作公共时基，DS-TWR 闭合锚间往返。

### 2. DS-TWR 锚间时基链路（Root ↔ Child）

双向测距闭合 Root↔Child 的传播时延：

```
(r2 − T1) = prop_R→X + f · (T2 − r1) + prop_X→R
⇒ 2 · prop = (r2 − T1) − f · (T2 − r1)
```

- 信标里回带 `rx_ref_ts40` / `rx_ref_cycle`（本帧作为时钟参考的那一帧的 RX 时刻与轮次）。
- Root 求解后把修正**批量单播**给全体 Child（`FUNC_TWR_CORR` = 0xE7，一帧带全部子站，每轮一次）。
- Child 用 `base_origin = rx_local(root beacon) − s_twr_corr_dtu − s_tx_delay_dtu` 把自己的信标摆到 Root 的栅格上。

### 3. 槽补偿 TDOA（Tag）

```
toa_eq = rx_local − slot × (DTU_PER_SLOT × s_root_rate) − tx_err
```

- `s_root_rate` 是 Tag 侧对 Root 速率的**中值滤波**估计（`TAG_RATE_SAMPLES` = 5，单轮 50 ms，≈1–2 ppb）。
- `tx_err`（信标 `[63-64]`，int16 DTU）= 本帧实际离开发射天线的时刻与计划栅格时刻之差，含①DX_TIME 4 ns 粒度截断 ②器件 TX 管线延迟残差。收发时刻都是**时刻**，两锚的原始 TX 戳不能直接相减（各自时基里含 Root→Child 传播时延，几十米量级），能跨锚发布的只有**相对自身栅格的偏差**。
- 栅格**摆放**误差（自身栅格原点离 Root 栅格多远）不由 `tx_err` 解决，由 Root 自标定解决。

### 4. Gauss-Newton 解算（`shared/downlink/tdoa_solver.c`）

| 参数 | 值 |
|------|-----|
| `MAX_ITER` | 12 |
| `CONV_THRESH` | 0.10 cm |
| `OUTLIER_THRESH` | 3.0（超阈残差权重 ×0.5） |
| `Z_INFO_MIN` | 0.02 |

- **多起点**：上一帧解 + 质心两次尝试，取残差小者。
- **维度分级**：锚点数 < 5 走 2D；2D 路径在**迭代前**就把 z 种子设为锚点平面高 `cz`，保证残差模型与上报的 z 自洽。
- **z 可辨识性**：`z_info = H[2][2] / trace(H)`，低于阈值即判 z 不可辨识，走 2D 并置 `Z_VALID = 0`。共面锚点下 z 本质不可辨识到 20 cm 量级，`Z_VALID` 就是"这个高度是不是测出来的"的诚实答案。
- 返回码：`1` 解算成功且 z 有效 / `0` 解算成功但 z 不可解 / `-1` 奇异 / `-2` 锚点不足。

### 5. 质量分（0..1000，越大越好）

```
q_r = 1000 × clamp(1 − rms_cm/50, 0, 1)      /* 残差：5 cm→900，20 cm→600，50 cm→0 */
q_n = 1000 × n_inlier / n_used               /* 内点率 */
q   = q_r × q_n / 1000                       /* 两者都好才算好 */
```

`65535` = **无解**（不是"质量极差"）。

---

## 协议

### 1. TimeSync 信标（67 字节，func = 0xEE）

| 偏移 | 字段 | 说明 |
|------|------|------|
| `[0-1]` | frame_ctrl | `0x41, 0x88` |
| `[2]` | seq8 | 序号 |
| `[3-4]` | pan_id | `0xCA, 0xDE` |
| `[5-6]` | dest | `0xFFFF` 广播 |
| `[7-8]` | source | Anchor rank（LE） |
| `[9]` | func | `0xEE` |
| `[10-14]` | tx_timestamp40 | 40 bit，**本帧** TX 时刻（不是上一帧） |
| `[15]` | flags | bit0 `TX_TS_PRECISE` / bit1 `TX_ERR_VALID` / bit2 `TX_OFFGRID` / bit3 `COORD_VALID` |
| `[16-17]` | reserved | |
| `[18]` | slot | 自身 TDMA 槽号，255 = 未分配 |
| `[19-26]` | parent_eui64 | 8 字节 |
| `[27]` | cs_level | `0xFF` 未初始化 / 0 = Root / ≥1 = Child |
| `[28-43]` | slot_table | 8 × uint16 rank，0 = 空 |
| `[44-51]` | x/y/z + 2 pad | int16 cm |
| `[52-53]` | cycle_id | TDMA 轮次计数 |
| `[54-55]` | present_mask | roster 在位位图 |
| `[56-60]` | rx_ref_ts40 | 本帧所用参考帧的 RX 时刻 |
| `[61-62]` | rx_ref_cycle | 该参考帧的轮次 |
| `[63-64]` | tx_err | 本帧 TX 离格误差（发送方 DTU，int16 LE） |
| `[65-66]` | tail pad | `0xCC 0xDD` |

> 🐛 **DW3210 铁律**：SPI 读回的帧，**最后 2 字节会被损坏**。所以吸收垫必须放在帧**末尾**，所有 `frame_len` 判断都是**下限**（`DL_TIMESYNC_MIN_LEN` = 63），不能用 `DL_TIMESYNC_LEN`（67）。旧固件发的 63 B 信标仍能被解析，只是 `rx_ref_cycle` 是垃圾值 → 该条 DS-TWR 链路作废，其余功能照常（半刷新的网络会降级而不是全黑）。

### 2. 位置报告（24 字节，func = 0xE3，Tag → Gateway）

| 偏移 | 字段 | 说明 |
|------|------|------|
| `[0-1]` | frame_ctrl | |
| `[2]` | seq8 | 轮次计数（网关据此算丢包） |
| `[16-17]` | quality | 0..1000，越大越好；65535 = 无解 |
| `[18-19]` | fw_version | |
| `[20]` | flags | bit0 `Z_VALID` / bit1 `SOLVE_VALID` / bit2 `3D_SOLVED` / bit3 `DEGRADED` |
| `[21]` | rms_mm | 残差毫米（255 饱和） |
| `[22-23]` | tail pad | `0xCC 0xDD` |

帧长从 22 → 24 是为了让 flags 和残差落在尾部垫**之前**。

`DEGRADED` 置位表示 Tag 活着但**全网静默**（Root 挂了），此时报告会**主动降频**，序列号出现空洞是**故意的**，消费方不要当成丢帧。

### 3. 其他帧

| func | 名称 | 方向 |
|------|------|------|
| `0xE3` | `DL_POS_REPORT` | Tag → Gateway |
| `0xE7` | `FUNC_TWR_CORR` | Root → 广播（批量路径修正） |
| `0xE8` | `FUNC_NETWORK_START` | Gateway → Anchor 广播 |
| `0xE9` | `FUNC_NETWORK_ACK` | Anchor → Gateway 单播 |
| `0xEC` | `FUNC_ANCHOR_DISCOVERY` | Anchor → Gateway 广播 |
| `0xED` | `FUNC_CLOCK_FEEDBACK` | Observer → 单播 |
| `0xEE` | `FUNC_DOWNLINK_BEACON` | Anchor → 全体（TimeSync） |

---

## 接口

### 1. 网关 AT 命令（UART，921600 8N1）

| 命令 | 说明 |
|------|------|
| `AT+PING` | 心跳，回 `+PONG` |
| `AT+LIST` | 列出已知设备 |
| `AT+FIND` | 触发发现扫描 |
| `AT+COORD?=<uid>` / `AT+COORD=<uid>,<x>,<y>,<z>` | 读/写基站坐标（写后置 `COORD_VALID`） |
| `AT+NETWORK=<...>` | 查看网络配置 |
| `AT+NETSTART=<root>[,<r1>,...]` | **起网**，roster 必填 |
| `AT+START` / `AT+STOP` | 起停定位栅格 |
| `AT+RATE=<n>` | 设置上报率 |
| `AT+RESET=<...>` | 复位 |
| `AT+FWV` | 查询固件版本 |
| `AT+OTA_START= / AT+OTA_DATA= / AT+OTA_END=` | 单机 OTA |
| `AT+OTA_BCAST= / ..._DATA= / ..._END` | 广播 OTA |

### 2. 网关 → 上位机 ASCII 行（`+` 前缀）

`+POS`（位置）、`+ONLINE` / `+SEEN` / `+OFFLINE`（设备上下线）、`+DEVTYPE`（同一 ID 类型冲突）、`+FIND`（发现结果）、`+COORDA` / `+COORDERR` / `+COORDSTAT`（坐标写入结果）、`+NETSTART_OK` / `+NETSTART_ERR` / `+NET_OK` / `+NET_ERR`（组网）、`+NETGRID`（已上槽成员）、`+NETMISS`（缺失成员）、`+NETNBR`（邻居）、`+RANKCOLL`（rank 冲突）、`+RATE_OK` / `+RATE_ERR`、`+PONG`、`+OTA_*`、`+START_OK` / `+STOP_OK`。

`+POS` 格式：

```
+POS,<UID>,<X.XX>,<Y.XX>,<Z.ZZ>,<Q>[,<FLAGS>,<RMS_MM>][,<SEQ>]
```

坐标单位米，质量是 0..1000 分（越大越好，65535 = 无解）。`FLAGS`/`RMS_MM`/`SEQ` 是**追加在末尾**的，老解析器按前 6 字段读照样正确。

### 3. Tag 本地串口行（给机器人）

```
P,SEQ,X=<m>,Y=<m>,Z=<m>,Q=<0..1000>,ZV=<0|1>,R=<残差 m>,M=<2D|3D>
```

**每个字段都是从已发布的帧缓冲里读回来的**，所以串口行与空中的 UWB 帧永远不会互相矛盾。`ZV=0` 表示高度不是测出来的（是锚点平面高）。

### 4. 诊断

| 位置 | 内容 |
|------|------|
| Tag 串口 | `P,...` 产品行、`TSUM` 汇总行、`TDOA_DBG` 逐锚残差、`RX_TS` / `CFO` 时钟诊断 |
| Anchor 串口 | `GR,<rank>,<残差 cm>,<cs_count>,<cs_q>`，1 Hz 打印栅格残差与时钟同步状态 |
| Gateway | `+NETGRID` / `+NETMISS` / `+DEVTYPE` / `+RANKCOLL` 等 |

---

## 上位机（`uwb_gateway_py/uwb_host.py`）

- Tkinter 单文件应用，串口连网关，ASCII 行解析。
- **地图视图**：基站按状态着色（运行中 / 降级 / 未组网 = 灰），Tag 实时位置与轨迹，z 无效时显示 `~` 而不是伪精确的平面高。
- **设备列表**：在线 / 见过（`+SEEN`）/ 离线三态，类型冲突（`+DEVTYPE`）单独标记。
- **坐标配置**：单站对话框 + 批量写入。
- **组网向导**：选 Root → 勾选 roster → 选槽序（默认按到 Root 距离排序）→ 下发 `AT+NETSTART`；按钮状态由 `net_phase()` 驱动（"组网中 n/m" → "✓ 组网成功" / "⚠ 降级运行"），名单不全时**绝不**显示成功。
- **固件升级**：单机 OTA 与批量 OTA（基站 / 标签分别选）。
- **轨迹管理**：清除标签轨迹、离群点剔除（按 0..1000 质量刻度，绿 ≥700 / 黄 ≥400 / 红 <400 / 离群 <300）。

---

## 硬件与射频配置

| 项 | 值 |
|----|----|
| MCU | STM32L412KBUx |
| UWB | DW3210 / DW3000 |
| 信道 | Channel 9（4940 MHz） |
| 前导码 | `DWT_PLEN_128` + `DWT_PAC8` |
| 速率 | `DWT_BR_6M8`（6.8 Mbps） |
| 天线延迟 | TX = RX = 16385 |
| 调试串口 | 921600 8N1（bootloader 引导日志仍为 115200） |

> `dwt_configure()` 的返回值**必须检查**：PLL 自校准失败会静默留在 FOSC 上，跨站率差远超 20 ppm 预算。

---

## 目录结构

```
.
├── shared/                  ← 真正的公共代码与构建目录
│   ├── fusion_anchor/       基站固件逻辑（Root/Child 同一份）
│   ├── fusion_tag/          标签固件逻辑
│   ├── downlink/            TDOA 解算器
│   ├── shared_data/         协议头、时钟同步、共享常量
│   ├── platform/            Keil 移植层
│   ├── decadriver/          DW3210 驱动
│   └── config/              配置
├── uwb_tag/                 标签 Keil 工程
├── uwb_anchor/              基站 Keil 工程
├── uwb_gateway/             网关 Keil 工程
├── uwb_gateway_py/          上位机（uwb_host.py）
├── bootloader/              Bootloader
├── docs/                    架构、方案、评审文档
├── test/                    宿主端测试（`solver_host_test.c`：求解器可执行回归）
└── API/                     ⚠️ DW3000 官方示例 + **陈旧副本**，勿改
```

> ⚠️ **`API/Src/examples/ex_09b_fusion_anchor/`、`ex_09a_fusion_tag/`、`ex_10a_downlink_tag/` 里是旧版源码副本**（行数与现行版本差几千行），且 `API/DWS3000_STL412.uvprojx` 真的在编译 ex_09b 那两份。请**不要**在那些文件里改代码——改动不生效。
>
> 真正的代码在 `shared/` 下，`uwb_*/` 里是 Keil 工程与 `main.c` 入口。

---

## 文档

| 文档 | 内容 |
|------|------|
| [`docs/系统架构与算法说明.md`](docs/系统架构与算法说明.md) | **活文档**：流程分层、算法细节、已知偏差（§5）、TDMA 鲁棒性分析（§3.13）、误差预算更正（§3.15） |
| [`docs/UWB_TDOA3_Optimization_Plan_v2.md`](docs/UWB_TDOA3_Optimization_Plan_v2.md) | 优化方案 v2（**当前有效的方案**） |
| [`docs/UWB_TDOA3_Optimization_Plan.md`](docs/UWB_TDOA3_Optimization_Plan.md) | 优化方案 v1，**已降为历史备份**，其三条主要结论已被 v2 推翻 |
| [`docs/粮库UWB定位系统技术方案.md`](docs/粮库UWB定位系统技术方案.md) | 应用侧方案 |
| [`docs/代码审查报告.md`](docs/代码审查报告.md) | 2026-11-20 代码审查轮：11 项已修复 + 16 项未修复清单（含位置、后果、优先级） |
| [`docs/Zone_Mesh_Plan_Review.md`](docs/Zone_Mesh_Plan_Review.md) | 2026-11-24：Zone Mesh 方案评审 —— 5 项待拍板事项的结论 + 方案 3 处前提更正 + 求解器退化保护的宿主端实测 |
| [`docs/DW3000_Delayed_TX技术手册.md`](docs/DW3000_Delayed_TX技术手册.md) | 延迟发送机制 |

**活文档约定**：任何改动影响到流程或算法的提交，必须**同一次提交**更新 `docs/系统架构与算法说明.md`。

---

## 已知约束

- **z 维本质不可辨识**（共面锚点）。`Z_VALID` 标志就是为此存在。真正的三维部署需要"标签安装高度已知 → 固定 z 解 x/y"，目前没有这条配置链路。
- **Tag 侧速率估计是弱环**：用单轮 50 ms 估 Root 速率（≈1–2 ppb），而基站侧用 800 ms 窗口（≈1 ppb）。这是槽补偿的主要误差源之一。
- **`tx_err` 不解决栅格摆放误差**，那需要 Root 侧自标定。
- **锚点坐标的 flash 持久化是桩实现**：重启后 `COORD_VALID` 会清零，需重新下发坐标。
- **上行无调度**：多个 Tag 共享 slot 0 的保护窗，靠 ALOHA 抖动避让，标签一多就会撞。正式方案需要每 Tag 独立上行槽。
- **OTA** 功能存在，但未在真实硬件上验证过。

---

## 贡献约定

- 每个离散改动单独提交，conventional-commit 前缀 + 中文一行标题。
- 落在 `feature/ds-twr-sts`，之后 fast-forward `main` 并把两个分支一起推。
- 涉及流程/算法的改动，同 commit 更新 `docs/系统架构与算法说明.md`。
- 本机无 ARM 工具链，只能做 WSL 桩语法检查；**编译与上机验证的责任在上机侧**。
