# DS-TWR 实施方案路线图

**分支**：`feature/ds-twr-sts`
**回退锚点**：tag `tdoa-v1.0-baseline` @ `49800d9`
**目标**：用 DS-TWR + STS 替代 TDOA，达到 ±10 cm 精度

---

## 已交付

| 文件 | 内容 | Commit |
|---|---|---|
| `shared/shared_data/ds_twr_protocol.h` | 协议头（FUNC codes、帧格式、字段偏移） | `44300b0` |
| `shared/shared_data/shared_functions.c` | `dstwr_compute()` Decawave DS-TWR 公式 | `44300b0` |
| `shared/shared_data/shared_functions.h` | 声明 `dstwr_compute` | `44300b0` |

编译验证：`uwb_anchor/DWS3000_Anchor_Master` 0 error / 16 warning（皆历史遗留）。

---

## 待实施（Stage 1.2 - 1.5）

### Stage 1.2：4-锚对等选举

**目标**：扩展 `auto_discovery()`，使所有 anchor（无论原"主从"）最终获得 rank 1-4。

**改动文件**：`shared/fusion_anchor/fusion_anchor.c` (auto_discovery 后处理)

**步骤**：
1. 在 `auto_discovery()` 完成后（`disc->state == AUTO_STATE_RUN` 时），增加：
   ```c
   // 收集所有 anchor 的 UID（包括"master"自己）
   // 按 UID 升序排序
   // 我的新 rank = 我在升序列表中的位置 (1..4)
   ```
2. 实现方式：**复用现有 `g_joined_uids[]` 表**，但**重新分配 rank**：
   ```c
   // 假设 g_joined_uids[] 已有 4 个 UID
   // 按 UID 升序排列
   for (i = 0; i < 4; i++) {
       if (g_joined_uids[i] == my_uid32) {
           my_ds_twr_rank = i + 1;  // 1..4
           break;
       }
   }
   ```
3. 写入新变量 `g_ds_twr_rank`（全局静态，供主循环读取）

**与 TDOA 兼容**：保留 `g_my_rank` 供 TDOA 路径，DS-TWR 路径用 `g_ds_twr_rank`。

**预计改动**：~50 行

---

### Stage 1.3：Anchor DS-TWR 主循环

**目标**：在 `main_anchor_loop()` 中增加 DS-TWR 分支（`#ifdef PROTOCOL_DS_TWR`）。

**改动文件**：`shared/fusion_anchor/fusion_anchor.c` + `shared/fusion_anchor/fusion_anchor.h`(?) + `shared/shared_data/example_selection.h`

**主循环结构**：
```c
while (1) {
    uint32_t cycle_start = portGetTickCnt();
    
#ifdef PROTOCOL_DS_TWR
    // ============= DS-TWR 角色 =============
    dstwr_anchor_cycle(g_ds_twr_rank);
#else
    // ============= TDOA 角色 (existing) =============
    main_anchor_loop_body();  // existing beacon/blink logic
#endif
    
    // 节流：等 cycle 50ms 完成
    while ((portGetTickCnt() - cycle_start) < DSTWR_CYCLE_MS) Sleep(1);
}
```

**`dstwr_anchor_cycle(rank)` 函数**（新增）：
```c
void dstwr_anchor_cycle(uint8_t my_rank) {
    // Phase 1: RX POLL (timeout ~2ms)
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    uint32_t deadline = portGetTickCnt() + 2;
    while (portGetTickCnt() < deadline) {
        if (poll_rx_done(&ts_poll_rx, &session_id)) break;
    }
    
    // Phase 2: TX RESP in my slot (rank * 3ms after POLL)
    uint32_t slot_time = cycle_start + my_rank * 3;
    while (portGetTickCnt() < slot_time) Sleep(1);
    tx_resp(session_id, ts_poll_rx, get_tx_timestamp_u64());
    
    // Phase 3: RX FINAL (timeout ~3ms, starting at 15ms after cycle_start)
    deadline = cycle_start + DSTWR_FINAL_OFFSET_MS + 3;
    while (portGetTickCnt() < deadline) {
        if (poll_final_rx_done(&ts_final_rx, &tx_t_poll, &tx_t_resp_rx_1to4)) break;
    }
    
    // Phase 4: compute TOF + TX REPORT in slot
    int32_t dist_mm;
    uint32_t tof_dtu;
    dstwr_compute(&ts, &dist_mm, &tof_dtu);
    
    slot_time = cycle_start + DSTWR_FINAL_OFFSET_MS + my_rank * 3;
    while (portGetTickCnt() < slot_time) Sleep(1);
    tx_report(session_id, my_rank, dist_mm);
}
```

**预计改动**：~250 行（含辅助函数）

---

### Stage 1.4：Tag DS-TWR 主循环

**目标**：在 tag 主循环（`fusion_tag.c` 的 `while(1)`）中增加 DS-TWR 分支。

**改动文件**：`shared/fusion_tag/fusion_tag.c` + `shared/shared_data/example_selection.h`

**主循环结构**：
```c
while (1) {
    uint32_t cycle_start = portGetTickCnt();
    
#ifdef PROTOCOL_DS_TWR
    // ============= DS-TWR 角色 =============
    dstwr_tag_cycle();
#else
    // ============= TDOA 角色 (existing) =============
    downlink_tag_cycle();  // existing RX beacon/blinks + solve
#endif
    
    while ((portGetTickCnt() - cycle_start) < DSTWR_CYCLE_MS) Sleep(1);
}
```

**`dstwr_tag_cycle()` 函数**（新增）：
```c
void dstwr_tag_cycle() {
    // Phase 1: TX POLL [T_poll_tx]
    tx_poll(&session_id_t);
    uint64_t T_poll_tx = get_tx_timestamp_u64();
    
    // Phase 2: RX 4 RESP in slots (2..12 ms)
    uint32_t resp_done_mask = 0;
    for (uint_t i = 0; i < 4; i++) {
        // wait until anchor i's slot window opens
        uint32_t slot_open = cycle_start + (i + 1) * 3;
        while (portGetTickCnt() < slot_open) Sleep(1);
        // try RX in slot (3ms window)
        if (rx_resp(&resp[i])) {
            resp_done_mask |= (1 << i);
        }
    }
    
    // Phase 3: TX FINAL [T_poll_tx, T_resp_rx_1..4]
    uint32_t final_time = cycle_start + DSTWR_FINAL_OFFSET_MS;
    while (portGetTickCnt() < final_time) Sleep(1);
    tx_final(session_id_t, resp_done_mask, T_poll_tx, T_resp_rx_1to4);
    
    // Phase 4: RX 4 REPORT in slots (17..28 ms)
    for (uint_t i = 0; i < 4; i++) {
        uint32_t slot_open = cycle_start + DSTWR_FINAL_OFFSET_MS + (i + 1) * 3;
        while (portGetTickCnt() < slot_open) Sleep(1);
        if (rx_report(&report[i])) {
            distances[i] = report[i].distance_mm;
        }
    }
    
    // Phase 5 (Stage 2): 多边定位 → Kalman → POS_REPORT
    // 此处先暂存 distances[]
}
```

**预计改动**：~300 行（含辅助函数）

---

### Stage 1.5：编译验证 + 编译开关

**改动文件**：
- `shared/shared_data/example_selection.h` — 加 `PROTOCOL_DS_TWR` 开关
- `uwb_anchor/DWS3000_Anchor_Master.uvprojx` + `DWS3000_Anchor_Slave.uvprojx` — 不需要改（`fusion_anchor.c` 已链接）
- `uwb_tag/DWS3000_Tag.uvprojx` — 不需要改（`fusion_tag.c` 已链接）

**验证步骤**：
1. 编译 anchor 工程：`UV4.exe -b DWS3000_Anchor_Master.uvprojx -j0`
2. 编译 tag 工程：`UV4.exe -b DWS3000_Tag.uvprojx -j0`
3. 验证 size < 32KB（DS-TWR 路径替代 TDOA 路径，预期减小）
4. 临时切回 `PROTOCOL_TDOA=1, PROTOCOL_DS_TWR=0` 验证 TDOA 仍可编译（互斥保护**）

---

### Phase 2：多边定位（Stage 2）

**新增文件**：`shared/fusion_tag/twr_multilateration.c`

**函数**：
```c
int twr_solve_2d(const dl_anchor_entry_t anchors[4],
                 const int32_t distance_mm[4],
                 float *x_cm, float *y_cm, float *rms_cm);
```

**算法**：3×2 线性最小二乘（i=1 为参考） + leave-one-out（4 选3）。
- **输入**：4 个 anchor 坐标 + 4 个距离
- **输出**：tag (x, y) + 残差 RMS
- **失败**：RMS > 100cm 或解出 (x,y) 超出 anchor 凸包

**预计改动**：~150 行

**复用 Kalman**：tag 现有的 `kalman_update_2d()` 直接可用，无需修改。

---

### Phase 3：STS 启用

**改动**：
1. `uwb_tag/Src/example_selection.h` — `TEST_DS_TWR_STS=1`
2. `uwb_anchor/Src/example_selection.h` — `TEST_DS_TWR_STS=1`
3. `config_options.c` — STS 配置（DW3210 STS mode 设置）
4. 各帧 payload 末尾追加 STS 段（128 符号）
5. RX 端启用 STS 首径检测

**预计改动**：~100 行（DW3000 配置为主）

---

## 关键文件清单

| 文件 | Stage | 预计行数 |
|---|---|---|
| `shared/shared_data/ds_twr_protocol.h` | 1.1 ✓ | 178 |
| `shared/shared_data/shared_functions.c` | 1.1 ✓ | +61 |
| `shared/shared_data/shared_functions.h` | 1.1 ✓ | +6 |
| `shared/fusion_anchor/fusion_anchor.c` | 1.2 + 1.3 | +50 + 250 |
| `shared/fusion_tag/fusion_tag.c` | 1.4 | +300 |
| `shared/fusion_tag/twr_multilateration.c` | 2 (new) | +150 |
| `shared/shared_data/example_selection.h` | 1.5 | +5 |
| STS 配置（多处） | 3 | +100 |
| **总计** | | **~1100 行** |

---

## 验证节点（里程碑）

| 节点 | 验证内容 | 通过标准 |
|---|---|---|
| Stage 1.5 完成 | 编译 + UART banner "DS_TWR V0.1" | 0 error, < 32KB |
| 4 锚开机 | LED 灯指示 rank 1-4 | 实际 rank 与 UID 升序匹配 |
| 单锚测距 | 标签固定1m 处 | 1m ± 0.3m (无 STS) |
| 4 锚同时测距 | 中心 6 点 | 中心 ±15cm, 角落 ±25cm (无 STS) |
| 启用 STS | 同上 | ±5-10cm ✓ |

---

## 风险与缓解

| 风险 | 影响 | 缓解 |
|---|---|---|
| 32KB Keil 上限 | Stage 1.3 + 1.4 同时存在可能超 | 编译开关互斥：DS_TWR=1 时 TDOA=0 |
| anchor slot 冲突 | 2 锚同时 TX → 丢包 | UID 排序保证唯一 rank |
| Tag TX 功耗 | 机器人电池寿命 | 调低 POLL/FINAL TX power |
| FINAL 多锚同时 RX | SPI buffer 溢出 | DW3210 RX buffer 大小足够（128 字节） |
| tag slot 同步 | 4 锚 slot 不对齐 | tag 用绝对时间调度（HAL_GetTick），3ms slot + 0.5ms guard |

---

## 时间估计

| Stage | 工作量 | 备注 |
|---|---|---|
| 1.2 | 0.5 周 | 改动小，主要是后处理逻辑 |
| 1.3 | 1.5 周 | anchor 主循环改写，需要仔细测试 slot 时序 |
| 1.4 | 1.5 周 | tag 主循环改写，4 slot RX 状态机复杂 |
| 1.5 | 0.5 周 | 编译开关 + 验证 |
| 2 | 1 周 | 多边定位 + Kalman 复用 |
| 3 | 1 周 | STS 启用 |
| **总计** | **6 周** | |

---

## 文档版本

- v1.0 — 2026-08-21 初版
- 基于 commit `44300b0` (Stage 1.1)