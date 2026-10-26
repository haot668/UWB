#!/usr/bin/env python3
"""UWB Gateway — Host Application (Python + tkinter)"""

import tkinter as tk
from tkinter import ttk, messagebox
import ctypes
import threading
import queue
import time
import math
import json
import os
import sys
import serial
import serial.tools.list_ports

# ══════════════════════════════════════════════
#  方案参数（必须与固件 downlink_protocol.h 保持一致）
# ══════════════════════════════════════════════
# NET_MAX_PARTICIPANTS：一次组网最多参与的基站总数（1 个 Root + 7 个子站）。
# 固件侧 NET_MAX_PARTICIPANTS 也是 8，超出的成员会被丢弃，所以上位机要拦。
NET_MAX_PARTICIPANTS = 8
# 少于 3 个基站时 TDOA 解算没有几何约束，坐标会退化到质心。
NET_MIN_PARTICIPANTS = 3

# ── High DPI ──
if sys.platform == "win32":
    try:
        ctypes.windll.shcore.SetProcessDpiAwareness(2)  # PerMonitorV2
    except Exception:
        try:
            ctypes.windll.user32.SetProcessDPIAware()
        except Exception:
            pass

# ══════════════════════════════════════════════
#  State
# ══════════════════════════════════════════════
class State:
    def __init__(self):
        self.ser: serial.Serial | None = None
        self.devices: dict[str, dict] = {}
        self.selected_uid: str = ""
        self.find_states: dict[str, bool] = {}
        self.anchor_coords: dict[str, tuple] = {}
        self.anchor_counter: int = 0
        self.anchor_label: dict[str, str] = {}
        self.tag_counter: int = 0
        self.tag_label: dict[str, str] = {}
        self.demo_active: bool = False
        self.ota_active: bool = False
        self.ota_uid: str = ""
        self.ota_data: bytes = b""
        self.ota_offset: int = 0
        self.ota_total: int = 0
        self.ota_chunk: int = 104   # max safe: 127B PHY - overhead, 8-byte aligned for STM32L4 flash

        # Map
        self.scale: float = 1.0
        self.ox: float = 0.0
        self.oy: float = 0.0
        self.drag: bool = False
        self.dx: float = 0
        self.dy: float = 0
        self.moved: float = 0
        self.mx: int = 0
        self.my: int = 0
        self.hover_uid: str = ""

        # Display
        self.show_grid: bool = True
        self.show_trails: bool = True
        self.show_labels: bool = True
        self.show_links: bool = True
        self.link_quality: dict[tuple, int] = {}

        # Batch OTA
        self.batch_ota_active: bool = False
        self.batch_ota_queue: list[str] = []
        self.batch_ota_results: dict[str, str] = {}
        self.batch_ota_dialog: tk.Toplevel | None = None
        self.batch_ota_file_path: str = ""

        # Liveness
        self.last_update: dict[str, float] = {}
        self.refresh_pending: bool = False
        self.refresh_seen: set[str] = set()
        self.timed_out: set[str] = set()  # locally timed-out, don't resurrect via refresh

        # Network formation (P1-1/P1-3)
        self.net_roster: list[int] = []       # ranks taking part in the network
        self.net_missing_ranks: set[int] = set()  # roster members that never joined
        self.net_root_rank: int = 0

        self.grid_w: float = 100.0
        self.grid_h: float = 100.0

S = State()
rx_queue = queue.Queue()
root: tk.Tk | None = None

# ══════════════════════════════════════════════
#  Colors
# ══════════════════════════════════════════════
C = {
    "bg":       "#f6f8fa",
    "surface":  "#ffffff",
    "card":     "#ffffff",
    "border":   "#d0d7de",
    "text":     "#1f2328",
    "muted":    "#656d76",
    "accent":   "#0969da",
    "green":    "#1a7f37",
    "yellow":   "#9a6700",
    "red":      "#cf222e",
    "orange":   "#bc4c00",
    "anchor":   "#0969da",
    "tag":      "#cf222e",
}

# ══════════════════════════════════════════════
#  Serial
# ══════════════════════════════════════════════
def serial_thread():
    buf = b""
    while True:
        if S.ser and S.ser.is_open:
            try:
                if S.ser.in_waiting:
                    data = S.ser.read(S.ser.in_waiting)
                    buf += data
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        line = line.strip()
                        if line:
                            rx_queue.put(line.decode("utf-8", errors="replace"))
            except (serial.SerialException, OSError):
                rx_queue.put(("__ERR__",))
                S.ser = None
                buf = b""
                time.sleep(1)
        else:
            time.sleep(0.1)

def send_cmd(cmd: str):
    if S.ser and S.ser.is_open:
        try:
            S.ser.write((cmd + "\r\n").encode())
            return True
        except serial.SerialException:
            return False
    return False

def process_rx():
    while not rx_queue.empty():
        item = rx_queue.get_nowait()
        if isinstance(item, tuple) and item[0] == "__ERR__":
            on_disconnected()
            continue
        parse_line(str(item).strip())

def parse_line(line: str):
    if not line:
        return
    # Debug: print every received line to terminal (even when host app occupies COM port)
    print(f"[RX] {line}", flush=True)
    parts = line.split(",")
    head = parts[0].strip()
    if head == "+ONLINE" and len(parts) >= 3:
        handle_online(parts)
    elif head == "+SEEN" and len(parts) >= 3:
        handle_seen(parts)
    elif head == "+OFFLINE" and len(parts) >= 2:
        handle_offline(parts)
    elif head == "+NETMISS" and len(parts) >= 3:
        handle_netmiss(parts)
    elif head == "+NETSTATE" and len(parts) >= 3:
        handle_netstate(parts)
    elif head == "+NETNBR" and len(parts) >= 3:
        handle_netnbr(parts)
    elif head == "+RANKCOLL" and len(parts) >= 3:
        handle_rankcoll(parts)
    elif head == "+POS" and len(parts) >= 6:
        handle_pos(parts)
    elif head == "+COORDAck" and len(parts) >= 5:
        handle_coord(parts)
    elif head == "+COORD" and len(parts) >= 5:
        # AT+COORD? read response: raw cm integers (e.g. "+COORD,0001,150,200,0")
        handle_coord(parts, from_read=True)
    elif head == "+COORD_ERR" and len(parts) >= 2:
        uid = parts[1].strip()
        label = get_label(uid)
        # Cancel pending timeout
        if getattr(S, "_coord_pending", None) == uid:
            S._coord_pending = None
            if hasattr(S, "_coord_timeout_id") and S._coord_timeout_id:
                root.after_cancel(S._coord_timeout_id)
                S._coord_timeout_id = None
        set_status(f"坐标写入失败 → {label}: 主基站未确认", "error")
        messagebox.showerror("坐标写入失败",
            f"基站 {label} ({uid})\n\n"
            "网关已发送，但未收到主基站确认。\n请检查:\n"
            "  • 主基站是否上电\n"
            "  • 主基站是否在网关通信范围内",
            parent=root)
    elif head == "+OTA_BCAST_READY" and len(parts) >= 2:
        handle_bcast_ready(parts)
    elif head == "+OTA_BCAST_DONE":
        handle_bcast_done()
    elif head == "+OTA_BCAST_ERROR" and len(parts) >= 2:
        handle_bcast_error(parts)
    elif head == "+OTA_SKIP" and len(parts) >= 3:
        uid = parts[1].strip()
        seq = parts[2].strip()
        set_status(f"OTA → {get_label(uid)}: 块 {seq} 跳过 (网关重试耗尽)", "warning")
    elif head == "+OTA_SKIP_NACK" and len(parts) >= 3:
        uid = parts[1].strip()
        seq = parts[2].strip()
        set_status(f"OTA → {get_label(uid)}: 块 {seq} NACK跳过 (Flash错误)", "warning")
    elif head == "+OTA_READY":
        handle_ota_ready(parts)
    elif head == "+OTA_PROGRESS" and len(parts) >= 3:
        handle_ota_progress(parts)
    elif head == "+OTA_ACK" and len(parts) >= 4:
        handle_ota_ack(parts)
    elif head == "+OTA_DONE":
        handle_ota_done(parts)
    elif head == "+OTA_ERROR" and len(parts) >= 3:
        handle_ota_error(parts)
    elif head == "+LINK" and len(parts) >= 3:
        handle_link(parts)
    elif head == "+FIND" and len(parts) >= 2:
        if len(parts) >= 3:
            uid = parts[1].strip()
            state = parts[2].strip()
            label = get_label(uid)
            set_status(f"寻找命令已发送 → {label} ({uid})", "info")
        else:
            set_status(f"FIND回执: {line.strip()}", "info")
    elif head == "+PONG":
        set_status("网关 Ping 成功 — 收到 +PONG", "success")
        if getattr(S, "_ping_pending", False):
            S._ping_pending = False
            if hasattr(S, "_ping_timeout_id") and S._ping_timeout_id:
                root.after_cancel(S._ping_timeout_id)
                S._ping_timeout_id = None
            # Restore PING button
            if hasattr(S, "_ping_btn") and S._ping_btn:
                S._ping_btn.config(text="发送", bg=C["card"], fg=C["text"], state="normal")
                S._ping_btn = None
            messagebox.showinfo("AT+PING 结果", "网关 Ping 成功 — 收到 +PONG")
    elif head == "+RATE_OK" and len(parts) >= 2:
        ms = int(parts[1].strip())
        hz = 1000.0 / ms if ms > 0 else 0
        set_status(f"标签更新频率已设置: {ms}ms ({hz:.1f} Hz)", "success")
        if getattr(S, "_rate_pending", False):
            S._rate_pending = False
            if hasattr(S, "_rate_timeout_id") and S._rate_timeout_id:
                root.after_cancel(S._rate_timeout_id)
            if hasattr(S, "_rate_btn") and S._rate_btn:
                S._rate_btn.config(text="发送", bg=C["card"], fg=C["text"], state="normal")
                S._rate_btn = None
            messagebox.showinfo("AT+RATE 结果", f"标签更新频率已设置:\n{ms} ms ({hz:.1f} Hz)")
    elif head == "+RATE_ERR":
        set_status("频率被拒绝 (有效范围: 20–5000ms)", "warning")
        if getattr(S, "_rate_pending", False):
            S._rate_pending = False
            if hasattr(S, "_rate_timeout_id") and S._rate_timeout_id:
                root.after_cancel(S._rate_timeout_id)
            S._rate_timeout_id = None
            if hasattr(S, "_rate_btn") and S._rate_btn:
                S._rate_btn.config(text="发送", bg=C["card"], fg=C["text"], state="normal")
                S._rate_btn = None
            messagebox.showwarning("AT+RATE 结果", "频率被拒绝 (有效范围: 20–5000ms)")
    elif head == "+FWV" and len(parts) >= 4:
        handle_fwv(parts)
    elif head == "+START_OK":
        set_status("TDOA 已启动 — Master 开始发送信标", "success")
    elif head == "+START_ERR":
        set_status("启动失败 — Master 未响应 START 命令", "error")
    elif head == "+STOP_OK":
        set_status("TDOA 已停止 — Master 返回配置模式", "info")
    elif head == "+STOP_ERR":
        set_status("停止失败 — Master 未响应 STOP 命令", "error")
    elif head == "+NET_OK" and len(parts) >= 2:
        mask_str = parts[1].strip()
        set_status(f"组网配置已确认 → 掩码 0x{mask_str}", "success")
    elif head == "+NET_ERR":
        set_status("组网配置失败 — Master 未响应", "error")
    elif head == "+NETSTART_OK" and len(parts) >= 3:
        ack_count = int(parts[1].strip())
        root_rank = int(parts[2].strip())
        # P1-1: the gateway appends the roster members that never answered:
        #   +NETSTART_OK,<ack>,<root>,<miss_cnt>,<rank1>,<rank2>,...
        miss_ranks = []
        if len(parts) >= 5:
            try:
                miss_cnt = int(parts[3].strip())
                miss_ranks = [int(p.strip()) for p in parts[4:4 + miss_cnt]
                              if p.strip().isdigit()]
            except (ValueError, IndexError):
                miss_ranks = []
        if miss_ranks:
            miss_txt = ", ".join(f"R{r}" for r in miss_ranks)
            set_status(f"组网启动 → {ack_count} 个基站响应 (Root=R{root_rank})，"
                       f"未加入: {miss_txt}", "warning")
            # Anchors that never joined must be kept out of the TDOA solve.
            S.net_missing_ranks = set(miss_ranks)
            messagebox.showwarning("组网不完整",
                f"以下基站未加入网络：\n\n  {miss_txt}\n\n"
                "它们不会进入 TDOA 解算集合，定位精度会下降。\n"
                "请检查这些基站的供电与射频链路，然后重新组网。",
                parent=root)
        else:
            set_status(f"组网启动成功 → {ack_count} 个基站响应 (Root=R{root_rank})", "success")
            S.net_missing_ranks = set()
        if hasattr(S, "_netstart_btn") and S._netstart_btn:
            S._netstart_btn.config(text="✓ 组网成功", state="disabled", bg=C["green"])
            S._netstart_btn = None
        # P1-3: the anchor-side timing is much slower than the gateway's
        # broadcast (de-bounce 800 ms, ALOHA 500 ms, slot confirmation up to
        # 3 s).  Re-check the network state 2.5 s later so the operator sees
        # the real result instead of the optimistic ACK count.
        root.after(2500, _check_network_states)
    elif head == "+NETSTART_ERR":
        set_status("组网启动失败 — 基站未响应", "error")
        if hasattr(S, "_netstart_btn") and S._netstart_btn:
            S._netstart_btn.config(text="开始组网", state="normal", bg=C["card"])
            S._netstart_btn = None
    elif head == "+FIND_ERR" and len(parts) >= 2:
        set_status(f"查找失败 — 设备 {parts[1].strip()} 未响应", "error")
    else:
        # Show unrecognized lines in status bar for debugging
        if line and not line.startswith("B,") and not line.startswith("T,") \
           and not line.startswith("S,") and not line.startswith("W,") \
           and not line.startswith("K,") and not line.startswith("M,") \
           and not line.startswith("L,") and not line.startswith("R,"):
            set_status(f"[GW] {line.strip()}", "info")

def get_label(uid: str) -> str:
    """Return display label for a device."""
    dtype = S.devices.get(uid, {}).get("type", "")
    if dtype == "GATEWAY":
        return "网关"
    elif dtype == "ANCHOR":
        return S.anchor_label.get(uid, uid)
    elif dtype == "TAG":
        return S.tag_label.get(uid, uid)
    return uid

def handle_online(parts):
    uid = parts[1].strip()
    raw_type = parts[2].strip().upper()
    if raw_type in ("TAG", "T"):
        dtype = "TAG"
    elif raw_type.startswith("GATE"):
        dtype = "GATEWAY"
    else:
        dtype = "ANCHOR"
    # Rank from +ONLINE message (e.g. "+ONLINE,0001,ANCHOR,08" → rank=8)
    rank = 0
    if len(parts) >= 4:
        try:
            rank = int(parts[3].strip(), 10)
        except (ValueError, IndexError):
            rank = 0
    is_new = uid not in S.devices
    if is_new:
        S.devices[uid] = {"trail": []}
    d = S.devices[uid]
    was_offline = not d.get("online")
    old_type = d.get("type")
    d["type"] = dtype
    d["rank"] = rank  # store rank for display
    d["pending"] = False   # confirmed online now (it was only "seen" before)
    S.last_update[uid] = time.time()
    # Persist device roster so reconnect doesn't show empty list
    if not hasattr(S, "_saved_devices"): S._saved_devices = {}
    S._saved_devices[uid] = {"type": dtype}
    if dtype == "GATEWAY":
        d["online"] = True
    elif S.refresh_pending:
        pass  # defer online decision to _finish_refresh
    else:
        d["online"] = True  # non-refresh +ONLINE: authoritative
        S.timed_out.discard(uid)
    if dtype == "GATEWAY":
        pass  # Gateway has no position, online-only
    elif dtype == "ANCHOR" and uid in S.anchor_coords:
        # P2-2: an anchor without configured coordinates must NOT be given a
        # made-up position.  The fake coordinates put the anchor at a wrong
        # place on the map and in the TDOA solve, and the UI then claimed
        # "✓已设坐标" for an anchor that was never configured.  Leave the
        # coordinates empty — the card then shows "⚠ 未设坐标".
        d["ax"], d["ay"], d["az"] = S.anchor_coords[uid]
    if dtype == "ANCHOR" and uid not in S.anchor_label:
        S.anchor_counter += 1
        S.anchor_label[uid] = f"A{S.anchor_counter:02d}"
        save_anchor_config()
    if dtype == "TAG" and uid not in S.tag_label:
        S.tag_counter += 1
        S.tag_label[uid] = f"T{S.tag_counter}"
        save_anchor_config()
    if S.refresh_pending:
        S.refresh_seen.add(uid)
        return  # suppress per-line refresh; _finish_refresh will rebuild once
    # Only refresh UI when state actually changes (new, offline→online, or type change)
    changed = is_new or was_offline or old_type != dtype
    if changed:
        S._dirty = True; refresh_lists()

def handle_offline(parts):
    uid = parts[1].strip()
    if uid in S.devices:
        S.devices[uid]["online"] = False
    S._dirty = True; refresh_lists()

def handle_seen(parts):
    """A device the gateway has sighted ONCE (the gateway needs 2 sightings
    before it reports a device online).

    P2-3: such a device used to be invisible to the host until the second
    sighting arrived, which is exactly the "上位机搜不到基站" symptom — the
    host polled AT+LIST faster than the debounce could complete.  Register it
    right away, marked as "待确认", so the operator sees it in the list.
    """
    uid = parts[1].strip()
    raw_type = parts[2].strip().upper()
    if raw_type in ("TAG", "T"):
        dtype = "TAG"
    elif raw_type.startswith("GATE"):
        dtype = "GATEWAY"
    else:
        dtype = "ANCHOR"
    rank = 0
    if len(parts) >= 4:
        try:
            rank = int(parts[3].strip(), 10)
        except (ValueError, IndexError):
            rank = 0
    if uid not in S.devices:
        S.devices[uid] = {"trail": []}
    d = S.devices[uid]
    d["type"] = dtype
    d["rank"] = rank
    d["pending"] = True      # sighted, not yet confirmed online
    S.last_update[uid] = time.time()
    if dtype == "ANCHOR":
        if uid in S.anchor_coords:
            d["ax"], d["ay"], d["az"] = S.anchor_coords[uid]
        if uid not in S.anchor_label:
            S.anchor_counter += 1
            S.anchor_label[uid] = f"A{S.anchor_counter:02d}"
            save_anchor_config()
    if S.refresh_pending:
        S.refresh_seen.add(uid)
        return
    S._dirty = True; refresh_lists()

def handle_netmiss(parts):
    """+NETMISS,<root>,<cnt>,<rank1>,... — the Root has not heard these
    roster members.  They are in the network by name but not by link."""
    try:
        root_rank = int(parts[1].strip())
        miss_cnt = int(parts[2].strip())
    except (ValueError, IndexError):
        return
    miss_ranks = [int(p.strip()) for p in parts[3:3 + miss_cnt]
                  if p.strip().isdigit()]
    if not miss_ranks:
        return
    miss_txt = ", ".join(f"R{r}" for r in miss_ranks)
    S.net_missing_ranks = set(miss_ranks)
    set_status(f"⚠ Root R{root_rank} 未收到: {miss_txt} — 这些基站不参与 TDOA 解算",
               "warning")

def handle_netstate(parts):
    """+NETSTATE,<uid>,<state> — anchor network state
    (0=idle, 1=starting, 2=wait_confirm, 3=running)."""
    uid = parts[1].strip()
    try:
        state = int(parts[2].strip())
    except (ValueError, IndexError):
        return
    if uid not in S.devices:
        S.devices[uid] = {"trail": [], "type": "ANCHOR"}
    S.devices[uid]["net_state"] = state

def handle_rankcoll(parts):
    """+RANKCOLL,<uid>,<rank> — two different anchors share one 16-bit rank.

    The rank is the 802.15.4 short address AND the identity used by the
    host list, so a collision makes the two anchors look like one device and
    they fight over the same TDMA slot (P1-4)."""
    try:
        uid = parts[1].strip()
        rank = int(parts[2].strip())
    except (ValueError, IndexError):
        return
    label = get_label(uid)
    set_status(f"⚠ rank 冲突: 基站 {label} (R{rank}) 与另一台基站 rank 相同", "warning")
    d = S.devices.get(uid)
    if d is not None:
        d["rank_collision"] = True

def handle_netnbr(parts):
    """+NETNBR,<rank>,<n_neighbors> — how many anchors this one hears.
    The Root's neighbour count is its coverage: a Root that hears almost
    nobody can never complete the network, so the operator should pick a
    better one (P1-1)."""
    try:
        rank = int(parts[1].strip())
        n = int(parts[2].strip())
    except (ValueError, IndexError):
        return
    uid = f"{rank:04X}"
    if uid not in S.devices:
        S.devices[uid] = {"trail": [], "type": "ANCHOR", "rank": rank}
    S.devices[uid]["n_neighbors"] = n

def _check_network_states():
    """P1-3: 2.5 s after NETSTART, verify that every roster member really
    reached RUNNING (state 3).  The gateway's ACK count is optimistic — an
    anchor answers the NETWORK_START long before it has joined the grid."""
    roster = getattr(S, "net_roster", []) or []
    if not roster:
        return
    not_running = []
    for rank in roster:
        uid = f"{rank:04X}"
        d = S.devices.get(uid, {})
        state = d.get("net_state")
        if state is None:
            not_running.append((rank, None))
        elif state != 3:
            not_running.append((rank, state))
    if not_running:
        detail = ", ".join(f"R{r}({'未上报' if st is None else '状态'+str(st)})"
                           for r, st in not_running)
        set_status(f"组网复查 → 尚未进入运行状态: {detail}", "warning")
        messagebox.showwarning("组网复查",
            "以下基站在 2.5 秒后仍未进入运行状态：\n\n"
            f"  {detail}\n\n"
            "它们暂时不参与 TDOA 解算，请检查链路后重新组网。",
            parent=root)
    else:
        set_status(f"组网复查 → 全部 {len(roster)} 个基站已进入运行状态", "success")

    # P1-1: the Root must be able to reach the other anchors.  Its neighbour
    # count is the only coverage measure we have; if the chosen Root hears
    # almost nobody, suggest the anchor with the best coverage instead.
    root_rank = getattr(S, "net_root_rank", 0)
    root_nbr = S.devices.get(f"{root_rank:04X}", {}).get("n_neighbors")
    if root_nbr is not None and root_nbr < 2:
        cands = [(d.get("n_neighbors", 0), int(d.get("rank") or 0))
                 for uid, d in S.devices.items()
                 if d.get("type") == "ANCHOR" and d.get("n_neighbors") is not None]
        best = max(cands) if cands else None
        hint = ""
        if best and best[1] != root_rank and best[0] > root_nbr:
            hint = (f"\n\n覆盖度更好的基站：R{best[1]}（邻居数 {best[0]}）。\n"
                    f"建议改用 R{best[1]} 作为 Root 重新组网。")
        set_status(f"⚠ Root R{root_rank} 覆盖不足（邻居数 {root_nbr}）", "warning")
        messagebox.showwarning("Root 覆盖不足",
            f"Root R{root_rank} 只能听到 {root_nbr} 个邻居基站。\n\n"
            "覆盖不足的 Root 无法把时隙表分发给所有基站，\n"
            "组网可能不完整。\n"
            f"请把 Root 换到覆盖更好的基站后重新组网。{hint}",
            parent=root)

def handle_pos(parts):
    uid = parts[1].strip()
    x, y, z, q = float(parts[2]), float(parts[3]), float(parts[4]), int(parts[5])
    if uid not in S.devices:
        S.devices[uid] = {"trail": [], "type": "TAG"}
    d = S.devices[uid]

    # --- TDOA position smoothing: outlier rejection + exponential filter ---
    OUTLIER_MAX_M  = 2.0      # reject jumps > 2m (x/y are meters)
    SMOOTH_ALPHA   = 0.3      # blend factor (0.3 = 30% new, 70% old)

    prev_x = d.get("x")       # last smoothed position (None on first fix)
    prev_y = d.get("y")

    if prev_x is not None and prev_y is not None:
        dist = math.hypot(x - prev_x, y - prev_y)
        if dist > OUTLIER_MAX_M and q < 300:
            # Outlier: position jumped too far in one cycle → drop it
            d["dropped"] = d.get("dropped", 0) + 1
        else:
            # Accepted: exponential smoothing
            d["x"] = prev_x * (1.0 - SMOOTH_ALPHA) + x * SMOOTH_ALPHA
            d["y"] = prev_y * (1.0 - SMOOTH_ALPHA) + y * SMOOTH_ALPHA
    else:
        # First fix: no smoothing
        d["x"] = x
        d["y"] = y

    d["rx"] = x; d["ry"] = y  # raw (unsmoothed) for diagnostics
    d["online"] = True; d["z"] = z; d["q"] = q
    S.last_update[uid] = time.time()
    S.timed_out.discard(uid)

    # Trail uses smoothed position
    d["trail"].append((d["x"], d["y"]))
    if len(d["trail"]) > 300:
        d["trail"].pop(0)

def handle_coord(parts, from_read=False):
    uid = parts[1].strip()
    x_raw, y_raw, z_raw = float(parts[2]), float(parts[3]), float(parts[4])
    # AT+COORD? read response sends raw cm integers (e.g. "150" = 1.50m) → convert to m
    # AT+COORD write-ACK sends meter values (e.g. "2.00" = 2.00m) → store as-is
    if from_read:
        x, y, z = x_raw / 100.0, y_raw / 100.0, z_raw / 100.0
    else:
        x, y, z = x_raw, y_raw, z_raw
    old = S.devices.get(uid, {})
    old_x, old_y, old_z = old.get("ax"), old.get("ay"), old.get("az")
    if uid in S.devices:
        S.devices[uid]["ax"] = x; S.devices[uid]["ay"] = y; S.devices[uid]["az"] = z
        S.devices[uid]["coord_ok"] = True
    S.anchor_coords[uid] = (x, y, z)
    S._dirty = True
    save_anchor_config()
    # Cancel pending timeout and show success popup
    if getattr(S, "_coord_pending", None) == uid:
        S._coord_pending = None
        if hasattr(S, "_coord_timeout_id") and S._coord_timeout_id:
            root.after_cancel(S._coord_timeout_id)
            S._coord_timeout_id = None
        messagebox.showinfo("坐标写入成功",
            f"基站 {get_label(uid)} ({uid})\n"
            f"X = {x:.2f} m\nY = {y:.2f} m\nZ = {z:.2f} m\n\n"
            "网关已确认发送。\n请检查主基站串口 COORD OK 确认实际写入。",
            parent=root)
    else:
        set_status(f"基站 {uid} 坐标已更新: ({x:.2f}, {y:.2f}, {z:.2f}) m", "success")

def handle_ota_ready(parts):
    """Gateway is ready to receive OTA data."""
    if not S.ota_active:
        return
    _clear_ota_timeout()
    set_status(f"OTA 已开始 → {S.ota_uid}", "info")
    send_ota_chunk()

def handle_ota_progress(parts):
    """Gateway acknowledged a chunk (unicast SW-ARQ)."""
    _clear_ota_timeout()
    S.ota_retry_count = 0
    # Format: "+OTA_PROGRESS,<UID>,<OFFSET>"
    if not S.ota_active:
        return
    try:
        offset = int(parts[2].strip())
    except (ValueError, IndexError):
        return
    S.ota_offset = offset
    pct = min(100, int(offset / S.ota_total * 100))
    set_status(f"OTA → {S.ota_uid}  {pct}%  ({offset}/{S.ota_total})", "info")
    if hasattr(S, "_ota_progress_var"):
        S._ota_progress_var.set(pct)
        S._ota_status_var.set(f"发送中...  {offset:,} / {S.ota_total:,} 字节  ({pct}%)")
    if hasattr(S, "_ota_progress_pct_var"):
        S._ota_progress_pct_var.set(f"{pct}%")
    if S.batch_ota_active and hasattr(S, "_batch_progress_var"):
        S._batch_progress_var.set(pct)
        if hasattr(S, "_batch_progress_pct_var"):
            S._batch_progress_pct_var.set(f"{pct}%")
    if offset < S.ota_total:
        root.after(5, send_ota_chunk)
    else:
        _reset_ota_timeout(5)
        send_cmd(f"AT+OTA_END={S.ota_uid}")

def handle_ota_ack(parts):
    """Device ACK/NACK — informational only (SW-ARQ handles retry at Gateway)."""
    if not S.ota_active:
        return
    try:
        uid = parts[1].strip()
        seq = int(parts[2].strip(), 16)
        status = int(parts[3].strip())
    except (ValueError, IndexError):
        return
    if status == 0:
        return  # ACK OK — Gateway already printed +OTA_PROGRESS
    # NACK (flash error) — Gateway ARQ will retry automatically
    label = get_label(uid)
    set_status(f"OTA → {label}: 块 {seq} NACK → Gateway 重试中...", "warning")

def _batch_ota_next():
    """Start OTA for the next device in the batch queue."""
    if not S.batch_ota_active or not S.batch_ota_queue:
        return
    uid = S.batch_ota_queue.pop(0)
    label = get_label(uid)
    set_status(f"批量升级 → {label}  (剩余 {len(S.batch_ota_queue)})", "info")
    if hasattr(S, "_batch_status_var"):
        done = len(S.batch_ota_results)
        total = done + len(S.batch_ota_queue)
        S._batch_status_var.set(f"设备 {done + 1}/{total + 1}: {label}")
    # Start single-device OTA for this uid
    S.ota_active = True
    S.ota_uid = uid
    S.ota_offset = 0
    S.ota_total = len(S.ota_data)
    S.ota_retry_count = 0
    _reset_ota_timeout(10)
    send_cmd(f"AT+OTA_START={uid},{S.ota_total}")


def handle_ota_done(parts):
    """OTA completed successfully."""
    _clear_ota_timeout()
    S.ota_active = False
    label = get_label(S.ota_uid)
    set_status(f"OTA 完成 → {label}", "success")
    if S.batch_ota_active:
        S.batch_ota_results[S.ota_uid] = "OK"
        _update_batch_log(S.ota_uid, "OK")
        if S.batch_ota_queue:
            root.after(200, _batch_ota_next)
        else:
            _batch_ota_finish()
    else:
        if hasattr(S, "_ota_status_var"):
            S._ota_status_var.set("升级完成！设备将重启。")
        if hasattr(S, "_ota_btn"):
            S._ota_btn.config(text="完成", state="disabled")
        messagebox.showinfo("升级成功", f"{label}\n\n固件升级完成，设备将自动重启。")


def handle_ota_error(parts):
    """OTA failed."""
    _clear_ota_timeout()
    S.ota_active = False
    msg = parts[2].strip() if len(parts) > 2 else "未知错误"
    label = get_label(S.ota_uid)
    set_status(f"OTA 失败 → {label}: {msg}", "error")
    if S.batch_ota_active:
        S.batch_ota_results[S.ota_uid] = f"FAIL: {msg}"
        _update_batch_log(S.ota_uid, f"FAIL: {msg}")
        if S.batch_ota_queue:
            root.after(200, _batch_ota_next)
        else:
            _batch_ota_finish()
    else:
        if hasattr(S, "_ota_status_var"):
            S._ota_status_var.set(f"Error: {msg}")
        if hasattr(S, "_ota_btn"):
            S._ota_btn.config(text="开始升级", state="normal")
        messagebox.showerror("升级失败", f"{label}\n\nOTA 升级失败: {msg}")

# ── Broadcast OTA handlers ──

def handle_bcast_ready(parts):
    """Broadcast OTA: gateway collected PREP_ACKs, ready for data."""
    if not S.batch_ota_active:
        return
    _clear_ota_timeout()
    try:
        count = int(parts[1].strip())
    except ValueError:
        count = 0
    S.bcast_dev_count = count
    S.ota_offset = 0
    S.ota_retry_count = 0
    S.ota_active = True  # reuse ota_active flag for broadcast mode
    S._bcast_mode = True
    set_status(f"广播OTA → {count} 个设备就绪", "info")
    if hasattr(S, "_batch_status_var"):
        S._batch_status_var.set(f"广播模式: {count} 设备同步升级中...")
    _reset_ota_timeout(15)  # generous timeout for first chunk
    send_bcast_chunk()

def handle_bcast_done():
    """Broadcast OTA completed on all devices."""
    _clear_ota_timeout()
    S.ota_active = False
    S._bcast_mode = False
    set_status("广播OTA 完成 → 所有设备升级成功", "success")
    if S.batch_ota_active:
        # Mark all queued devices as OK
        for uid in list(S.batch_ota_queue):
            S.batch_ota_results[uid] = "OK"
            _update_batch_log(uid, "OK")
        S.batch_ota_queue.clear()
        _batch_ota_finish()

def handle_bcast_error(parts):
    """Broadcast OTA error."""
    _clear_ota_timeout()
    S.ota_active = False
    S._bcast_mode = False
    msg = parts[1].strip() if len(parts) > 1 else "未知错误"
    set_status(f"广播OTA 失败: {msg}", "error")
    if S.batch_ota_active:
        for uid in list(S.batch_ota_queue):
            S.batch_ota_results[uid] = f"FAIL: bcast {msg}"
            _update_batch_log(uid, f"FAIL: bcast {msg}")
        S.batch_ota_queue.clear()
        _batch_ota_finish()

def send_bcast_chunk():
    """Send next chunk in broadcast OTA mode — always 104 bytes (padded with 0xFF)."""
    if not S.ota_active or S.ota_offset >= S.ota_total:
        return
    end = min(S.ota_offset + S.ota_chunk, S.ota_total)
    chunk = S.ota_data[S.ota_offset:end]
    if len(chunk) < S.ota_chunk:
        chunk = chunk + b'\xFF' * (S.ota_chunk - len(chunk))
    hex_data = chunk.hex()
    send_cmd(f"AT+OTA_BCAST_DATA={hex_data}")
    _reset_ota_timeout(10)

def _clear_ota_timeout():
    """Cancel any pending OTA timeout."""
    tid = getattr(S, "_ota_timeout_id", None)
    if tid is not None:
        root.after_cancel(tid)
        S._ota_timeout_id = None


def _reset_ota_timeout(seconds=10):
    """Reset the OTA watchdog timer. If no response within `seconds`, OTA is aborted."""
    _clear_ota_timeout()
    S._ota_timeout_id = root.after(int(seconds * 1000), _ota_timeout)


def _ota_timeout():
    """OTA chunk timed out — retry up to 2 times before aborting.
    Gateway ARQ handles up to 15 retries internally (~4.5s).
    Host retry here covers the case where +OTA_PROGRESS was lost on UART."""
    retry = getattr(S, "ota_retry_count", 0) + 1
    S.ota_retry_count = retry
    label = get_label(S.ota_uid)
    if retry < 2:
        set_status(f"OTA 超时 → {label}: 重试 {retry}/2...", "warning")
        send_ota_chunk()
        return
    # 2 retries exhausted → abort
    S.ota_active = False
    S.ota_retry_count = 0
    set_status(f"OTA 超时 → {label}: 2次重试失败，升级中止", "error")
    if hasattr(S, "_ota_status_var"):
        S._ota_status_var.set("超时 — 2次重试失败。请检查网关固件。")
    if hasattr(S, "_ota_btn"):
        S._ota_btn.config(text="开始升级", state="normal")
    if S.batch_ota_active:
        S.batch_ota_results[S.ota_uid] = "FAIL: timeout x2"
        _update_batch_log(S.ota_uid, "FAIL: timeout x2")
        if S.batch_ota_queue:
            root.after(200, _batch_ota_next)
        else:
            _batch_ota_finish()
    else:
        messagebox.showerror("升级失败", f"{label}\n\nOTA 升级超时 — 2次重试均失败。\n请检查:\n"
                             "  • 网关固件是否正常运行\n"
                             "  • 设备是否在线\n"
                             "  • 串口连接是否稳定")

def _update_batch_log(uid, result):
    """Append a result line to the batch OTA log widget."""
    if not hasattr(S, "_batch_log_text"):
        return
    label = get_label(uid)
    tag = "ok" if result == "OK" else "fail"
    S._batch_log_text.config(state="normal")
    S._batch_log_text.insert(tk.END, f"  {label}  {uid}  ", "")
    S._batch_log_text.insert(tk.END, f"{result}\n", tag)
    S._batch_log_text.tag_config("ok", foreground=C["green"])
    S._batch_log_text.tag_config("fail", foreground=C["red"])
    S._batch_log_text.see(tk.END)
    S._batch_log_text.config(state="disabled")


def _batch_ota_finish():
    """Complete the batch OTA operation."""
    S.batch_ota_active = False
    ok = sum(1 for v in S.batch_ota_results.values() if v == "OK")
    fail = len(S.batch_ota_results) - ok
    set_status(f"批量升级完成 → {ok} 成功, {fail} 失败", "success" if fail == 0 else "warning")
    if hasattr(S, "_batch_status_var"):
        S._batch_status_var.set(f"完成: {ok} 成功, {fail} 失败")
    if hasattr(S, "_batch_log_text"):
        S._batch_log_text.config(state="normal")
        S._batch_log_text.insert(tk.END, f"\n  ── 批量升级完成: {ok} 成功, {fail} 失败 ──\n", "summary")
        S._batch_log_text.tag_config("summary", foreground=C["accent"])
        S._batch_log_text.see(tk.END)
        S._batch_log_text.config(state="disabled")
    if hasattr(S, "_batch_start_btn"):
        S._batch_start_btn.config(text="开始批量升级", state="normal")
    if fail == 0:
        messagebox.showinfo("批量升级完成", f"批量固件升级已完成\n\n成功: {ok} 台\n失败: {fail} 台")
    else:
        messagebox.showwarning("批量升级完成", f"批量固件升级已完成（有失败）\n\n成功: {ok} 台\n失败: {fail} 台\n\n请检查失败设备的日志。")

def send_ota_chunk():
    """Send the next chunk of OTA data — always 104 bytes (padded with 0xFF)."""
    if not S.ota_active or S.ota_offset >= S.ota_total:
        return
    end = min(S.ota_offset + S.ota_chunk, S.ota_total)
    chunk = S.ota_data[S.ota_offset:end]
    if len(chunk) < S.ota_chunk:
        chunk = chunk + b'\xFF' * (S.ota_chunk - len(chunk))
    hex_data = chunk.hex()
    send_cmd(f"AT+OTA_DATA={S.ota_uid},{S.ota_offset},{hex_data}")
    _reset_ota_timeout(20)  # 20s — gateway ARQ 20 retries + backoff

def handle_fwv(parts):
    """Store firmware version: +FWV,<type>,<uid>,<version>"""
    try:
        dtype = parts[1].strip()
        uid = parts[2].strip()
        ver = parts[3].strip()
    except (ValueError, IndexError):
        return
    if uid in S.devices:
        S.devices[uid]["fw_ver"] = ver
    # Collect for AT+FWV popup
    if getattr(S, "_fwv_pending", False):
        label = get_label(uid)
        S._fwv_versions[uid] = (label, dtype, ver)
    set_status(f"固件版本: {uid} → {ver}", "info")

def _show_fwv_popup():
    """Show firmware version popup after AT+FWV, then refresh the device list."""
    S._fwv_pending = False
    if hasattr(S, "_fwv_btn") and S._fwv_btn:
        S._fwv_btn.config(text="发送", bg=C["card"], fg=C["text"], state="normal")
        S._fwv_btn = None
    versions = dict(getattr(S, "_fwv_versions", {}))
    # Also include devices that already have fw_ver from auto-detection
    for uid, d in S.devices.items():
        if uid not in versions and d.get("fw_ver"):
            label = get_label(uid)
            dtype = d.get("type", "?")
            versions[uid] = (label, dtype, d["fw_ver"])
    if versions:
        lines = []
        for uid, (label, dtype, ver) in sorted(versions.items()):
            dtype_cn = {"GATEWAY": "网关", "ANCHOR": "基站", "TAG": "标签"}.get(dtype, dtype)
            lines.append(f"  {label}  ({dtype_cn})  →  {ver}")
        messagebox.showinfo("固件版本", "当前设备固件版本:\n\n" + "\n".join(lines))
    else:
        messagebox.showinfo("固件版本", "未收到设备版本响应")
    S._fwv_versions = {}
    S._dirty = True
    refresh_lists()

def handle_link(parts):
    """Store inter-anchor link quality: +LINK,<uid1>,<uid2>,<quality>"""
    try:
        u1 = parts[1].strip()
        u2 = parts[2].strip()
        q = int(parts[3].strip()) if len(parts) > 3 else 5
    except (ValueError, IndexError):
        return
    key = tuple(sorted([u1, u2]))
    S.link_quality[key] = max(0, min(10, q))

def on_disconnected():
    for d in S.devices.values():
        d["online"] = False
    set_status("已断开连接", "warning")
    S._dirty = True; refresh_lists()
    if hasattr(S, "_conn_btn") and S._conn_btn:
        S._conn_btn.config(text="●  连接", style="Accent.TButton")

# ── Device liveness ──
# P2-1: the OFFLINE decision belongs to the gateway only.  The gateway sees
# every UWB frame, so it can tell "device is silent" from "device is gone"
# much better than the host, which only sees the UART stream.  The host used
# to run its own 30 s timer for anchors while the gateway already declared a
# device offline after 15 s — two different standards, and the host was the
# one that wrongly kept showing stale devices as online (and the other way
# round).  The gateway now sends +OFFLINE and that is the single authority.
TAG_TIMEOUT_S   = 0     # 0 = disabled, the gateway decides
ANCHOR_TIMEOUT_S = 0    # 0 = disabled, the gateway decides

def check_device_timeout():
    """Kept for the periodic tick — the offline decision is the gateway's
    (P2-1): it reports +OFFLINE after 15 s of silence.  The host only keeps
    the timestamps fresh for the debug output."""
    now = time.time()
    for uid, d in S.devices.items():
        if not d.get("online"):
            continue
        dtype = d.get("type", "")
        if dtype == "GATEWAY":
            continue
        last = S.last_update.get(uid, 0)
        timeout = TAG_TIMEOUT_S if dtype == "TAG" else ANCHOR_TIMEOUT_S
        if timeout > 0 and now - last > timeout:
            d["online"] = False
            S.timed_out.add(uid)
            print(f"[liveness] {uid} ({dtype}) offline: {now - last:.1f}s (timeout={timeout}s)")
            S._dirty = True
            refresh_lists()

def refresh_device_list():
    """Send AT+LIST and mark non-responding devices offline after a grace period."""
    if not (S.ser and S.ser.is_open):
        return
    if S.refresh_pending:
        return  # already refreshing
    S.refresh_seen.clear()
    S.refresh_pending = True
    send_cmd("AT+LIST")
    # P2-3: the gateway needs 2 sightings before it reports a device online
    # (phantom filter), and the anchors beacon every 2 s while networked.
    # A 2.5 s window was shorter than the debounce needed, so a device could
    # never confirm within one refresh cycle and flickered in and out of the
    # list.  3 s gives the debounce room to complete.
    root.after(3000, _finish_refresh)

def _finish_refresh():
    """Commit refresh: bring back any device the gateway can still see."""
    S.refresh_pending = False
    changed = False
    gateway_online = False
    for uid, d in S.devices.items():
        if d.get("type", "") == "GATEWAY":
            gateway_online = d.get("online", False)
            continue
        if uid in S.refresh_seen:
            if not d.get("online", False):
                d["online"] = True
                S.last_update[uid] = time.time()
                S.timed_out.discard(uid)
                changed = True
    S.refresh_seen.clear()

    # Always refresh UI after AT+LIST — gateway authoritative
    S._dirty = True
    refresh_lists()

    # Restore LIST button
    if hasattr(S, "_list_btn") and S._list_btn:
        S._list_btn.config(text="发送", bg=C["card"], fg=C["text"], state="normal")
        S._list_btn = None

    if changed:
        set_status("设备列表已刷新", "success")

    # Show popup for user-requested AT+LIST
    if getattr(S, "_list_popup_pending", False):
        online = sum(1 for d in S.devices.values() if d.get("online"))
        total = len(S.devices)
        # If nothing came back, auto-retry once after 3 s
        if online == 0 and total == 0:
            set_status("未检测到设备 — 3 秒后重试...", "warning")
            S._list_popup_pending = True
            S._list_retry_count = getattr(S, "_list_retry_count", 0) + 1
            if S._list_retry_count <= 2:
                root.after(3000, refresh_device_list)
                return
            # Retries exhausted — explain what to check instead of a bare count.
            messagebox.showwarning(
                "AT+LIST 结果",
                "未检测到任何设备。\n\n"
                "基站通过 FUNC_ANCHOR_DISCOVERY(0xEC) 主动播报自身，\n"
                "网关收到后才会登记为在线。\n\n"
                "请检查：\n"
                "  1) 基站固件是否已升级（支持 0xEC 发现帧）\n"
                "  2) 基站与网关是否同信道 / 同一 PAN\n"
                "  3) 组网时可先用「开始组网 → 手动 Rank」直接下发 Root 地址")
            S._list_popup_pending = False
            S._list_retry_count = 0
            return
        S._list_retry_count = 0
        S._list_popup_pending = False
        anchor_count = sum(1 for d in S.devices.values()
                          if d.get("type") == "ANCHOR" and d.get("online"))
        tag_count = sum(1 for d in S.devices.values()
                       if d.get("type") == "TAG" and d.get("online"))
        msg = f"在线: {online} / {total} 设备\n\n"
        msg += f"  网关: {'在线' if gateway_online else '离线'}\n"
        msg += f"  基站: {anchor_count}\n"
        msg += f"  标签: {tag_count}"
        messagebox.showinfo("AT+LIST 结果", msg)

# ══════════════════════════════════════════════
#  Config
# ══════════════════════════════════════════════
CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "anchor_cfg.json")

def load_anchor_config():
    try:
        if os.path.exists(CONFIG_FILE):
            with open(CONFIG_FILE, "r") as f:
                data = json.load(f)
                S.anchor_coords = data.get("coords", {})
                S.anchor_label = data.get("labels", {})
                S.anchor_counter = data.get("counter", 0)
                S.tag_label = data.get("tag_labels", {})
                S.tag_counter = data.get("tag_counter", 0)
                # Do NOT restore stale saved devices from disk.
                # Devices are re-discovered live via +ONLINE / +POS messages.
                S._saved_devices = {}
    except Exception:
        pass

def save_anchor_config():
    try:
        with open(CONFIG_FILE, "w") as f:
            json.dump({
                "coords": S.anchor_coords,
                "labels": S.anchor_label,
                "counter": S.anchor_counter,
                "tag_labels": S.tag_label,
                "tag_counter": S.tag_counter,
                "devices": getattr(S, "_saved_devices", {}),
            }, f, indent=2)
    except Exception:
        pass

# ══════════════════════════════════════════════
#  Root Window
# ══════════════════════════════════════════════
def build_ui():
    global root
    root = tk.Tk()
    root.title("UWB 网关 — 定位系统")
    root.configure(bg=C["bg"])
    root.minsize(1100, 650)

    # Start maximized
    try:
        sw = root.winfo_screenwidth()
        sh = root.winfo_screenheight()
        root.geometry(f"{int(sw*0.82)}x{int(sh*0.82)}+{int(sw*0.09)}+{int(sh*0.06)}")
    except Exception:
        root.geometry("1400x900")

    # Style
    style = ttk.Style(root)
    style.theme_use("clam")

    # Base
    style.configure(".", background=C["bg"], foreground=C["text"], font=("Segoe UI", 9))
    style.configure("TFrame", background=C["bg"])
    style.configure("TLabel", background=C["bg"], foreground=C["text"])
    style.configure("TLabelframe", background=C["bg"])
    style.configure("TLabelframe.Label", background=C["bg"], foreground=C["muted"])

    # Buttons
    style.configure("TButton",
        background=C["surface"], foreground=C["text"],
        borderwidth=1, padding=(10, 4), relief="flat",
        font=("Segoe UI", 9))
    style.map("TButton",
        background=[("active", C["border"]), ("pressed", C["accent"])],
        foreground=[("active", C["text"])])

    # Accent button
    style.configure("Accent.TButton",
        background=C["accent"], foreground="#ffffff",
        borderwidth=0, padding=(12, 5), font=("Segoe UI", 9, "bold"))
    style.map("Accent.TButton",
        background=[("active", "#6cb2ff")])

    # Danger button
    style.configure("Danger.TButton",
        background=C["red"], foreground="#ffffff",
        borderwidth=0, padding=(10, 4), font=("Segoe UI", 9))
    style.map("Danger.TButton",
        background=[("active", "#ff6b6b")])

    # Small button
    style.configure("Small.TButton", padding=(6, 2), font=("Segoe UI", 8))

    # Combobox
    style.configure("TCombobox",
        fieldbackground=C["surface"], foreground=C["text"],
        arrowcolor=C["text"], background=C["surface"],
        selectbackground=C["accent"], selectforeground="#ffffff")
    style.map("TCombobox",
        fieldbackground=[("readonly", C["surface"])],
        background=[("active", C["accent"])])

    # Entry
    style.configure("TEntry",
        fieldbackground=C["surface"], foreground=C["text"],
        insertcolor=C["text"], relief="flat", borderwidth=1,
        padding=4)

    # Scrollbar
    style.configure("Vertical.TScrollbar",
        background=C["surface"], troughcolor=C["bg"],
        arrowcolor=C["muted"], relief="flat", borderwidth=0)

    # Green progress bar
    style.configure("green.Horizontal.TProgressbar",
        background="#2da44e", troughcolor=C["border"])

    # Treeview (for potential use)
    style.configure("Treeview",
        background=C["surface"], foreground=C["text"],
        fieldbackground=C["surface"], borderwidth=0)
    style.configure("Treeview.Heading",
        background=C["card"], foreground=C["text"],
        relief="flat", padding=4)

    # Layout
    root.grid_rowconfigure(1, weight=1)
    root.grid_columnconfigure(1, weight=1)

    build_header(root)
    build_sidebar(root)
    build_map(root)
    build_footer(root)

    root.bind("<F11>", toggle_fullscreen)
    root.bind("<Escape>", exit_fullscreen)

    root.after(50, poll_serial)
    root.after(33, render_loop)
    root.after(3000, check_liveness)

    # Set initial sidebar width
    root.after(100, lambda: root.grid_columnconfigure(0, minsize=320))
    return root

def toggle_fullscreen(e=None):
    root.attributes("-fullscreen", True)

def exit_fullscreen(e=None):
    root.attributes("-fullscreen", False)

# ══════════════════════════════════════════════
#  Header
# ══════════════════════════════════════════════
def build_header(parent):
    hdr = tk.Frame(parent, bg=C["surface"], height=52)
    hdr.grid(row=0, column=0, columnspan=2, sticky="ew")
    hdr.grid_propagate(False)

    # Left: brand
    left = tk.Frame(hdr, bg=C["surface"])
    left.pack(side=tk.LEFT, padx=16)
    tk.Label(left, text="◈  UWB 定位系统", bg=C["surface"], fg=C["text"],
             font=("Segoe UI", 15, "bold")).pack(side=tk.LEFT)
    tk.Label(left, text="  v1.0", bg=C["surface"], fg=C["muted"],
             font=("Segoe UI", 9)).pack(side=tk.LEFT)

    # Right: controls
    right = tk.Frame(hdr, bg=C["surface"])
    right.pack(side=tk.RIGHT, padx=14)

    # Baud rate
    tk.Label(right, text="波特率", bg=C["surface"], fg=C["muted"],
             font=("Segoe UI", 9)).pack(side=tk.LEFT, padx=(0, 4))
    baud_var = tk.StringVar(value="921600")
    baud_cb = ttk.Combobox(right, textvariable=baud_var, width=8,
        values=["9600","19200","38400","57600","115200","230400","460800","921600"],
        state="readonly", font=("Segoe UI", 10))
    baud_cb.pack(side=tk.LEFT, padx=(0, 12))
    S._baud_var = baud_var

    # COM port
    tk.Label(right, text="COM", bg=C["surface"], fg=C["muted"],
             font=("Segoe UI", 9)).pack(side=tk.LEFT, padx=(0, 4))
    com_var = tk.StringVar()
    com_cb = ttk.Combobox(right, textvariable=com_var, width=9,
        state="readonly", font=("Segoe UI", 10))
    com_cb.pack(side=tk.LEFT, padx=(0, 6))
    S._com_var = com_var; S._com_cb = com_cb

    # Refresh
    ttk.Button(right, text="⟳", width=3, style="Small.TButton",
               command=refresh_ports).pack(side=tk.LEFT, padx=(0, 10))

    # Connect button
    S._conn_btn = ttk.Button(right, text="●  连接", style="Accent.TButton",
                              command=btn_connect)
    S._conn_btn.pack(side=tk.LEFT)

    # Demo button
    S._demo_btn = ttk.Button(right, text="◉  演示", style="Small.TButton",
                              command=toggle_demo)
    S._demo_btn.pack(side=tk.LEFT, padx=(8, 0))

    refresh_ports()

def refresh_ports():
    ports = [p.device for p in serial.tools.list_ports.comports()]
    S._com_cb["values"] = ports
    if ports and not S._com_var.get():
        S._com_var.set(ports[0])

def btn_connect():
    # Prevent rapid connect/disconnect re-entry
    if getattr(S, "_connecting", False):
        return
    S._connecting = True

    if S.ser and S.ser.is_open:
        S.ser.close(); S.ser = None
        on_disconnected()
        S._conn_btn.config(text="●  连接", style="Accent.TButton")
        set_status("已断开连接", "warning")
        S._connecting = False
        return

    port = S._com_var.get()
    baud = int(S._baud_var.get())
    if not port:
        messagebox.showwarning("未选择端口", "请选择 COM 端口。")
        S._connecting = False
        return

    try:
        if S.demo_active:
            clear_demo()
        S.ser = serial.Serial(port, baud, timeout=0.1)
        S._conn_btn.config(text="◉  断开连接", style="Danger.TButton")
        set_status(f"已连接  →  {port} @ {baud}", "success")
        # Restore saved device roster so list isn't empty on reconnect
        Saved = getattr(S, "_saved_devices", {})
        if Saved:
            for uid, info in Saved.items():
                if uid not in S.devices:
                    S.devices[uid] = {"trail": []}
                    S.devices[uid]["type"] = info.get("type", "?")
                    S.devices[uid]["online"] = False
            S._dirty = True
            refresh_lists()
        # AT+LIST retries — devices appear within ~400ms after initial beacon
        for d in (400, 1200, 3000):
            root.after(d, lambda d=d: send_cmd("AT+LIST"))
    except serial.SerialException as e:
        messagebox.showerror("错误", f"无法打开 {port}:\n{e}")
    finally:
        S._connecting = False

# ══════════════════════════════════════════════
#  Sidebar
# ══════════════════════════════════════════════
def build_sidebar(parent):
    sb = tk.Frame(parent, bg=C["bg"], width=300)
    sb.grid(row=1, column=0, sticky="ns", padx=(4,0), pady=4)
    sb.grid_propagate(False)

    # Tabs
    tabf = tk.Frame(sb, bg=C["surface"])
    tabf.pack(fill=tk.X, padx=6, pady=(6, 2))

    S._tab_dev = tk.Label(tabf, text="设备", bg=C["accent"], fg="#ffffff",
                          font=("Segoe UI", 11, "bold"), padx=22, pady=7,
                          cursor="hand2")
    S._tab_dev.pack(side=tk.LEFT, fill=tk.X, expand=True)
    S._tab_dev.bind("<Button-1>", lambda e: show_tab("devices"))

    S._tab_set = tk.Label(tabf, text="设置", bg=C["surface"], fg=C["muted"],
                          font=("Segoe UI", 11), padx=22, pady=7,
                          cursor="hand2")
    S._tab_set.pack(side=tk.LEFT, fill=tk.X, expand=True)
    S._tab_set.bind("<Button-1>", lambda e: show_tab("settings"))

    # Device panel
    S._dev_frame = tk.Frame(sb, bg=C["bg"])
    S._dev_frame.pack(fill=tk.BOTH, expand=True)

    # Search
    sf = tk.Frame(S._dev_frame, bg=C["surface"])
    sf.pack(fill=tk.X, padx=6, pady=(4, 2))

    search_icon = tk.Label(sf, text="⌕", bg=C["surface"], fg=C["muted"],
                           font=("Segoe UI", 12))
    search_icon.pack(side=tk.LEFT, padx=(6, 0))
    search_var = tk.StringVar()
    search_entry = tk.Entry(sf, textvariable=search_var,
        bg=C["surface"], fg=C["text"], insertbackground=C["text"],
        relief="flat", font=("Segoe UI", 11), width=16)
    search_entry.pack(side=tk.LEFT, padx=4, pady=5, fill=tk.X, expand=True)
    search_var.trace_add("write", lambda *a: refresh_lists())
    S._search_var = search_var

    # Count + Add button
    ctrl_bar = tk.Frame(S._dev_frame, bg=C["bg"])
    ctrl_bar.pack(fill=tk.X, padx=8, pady=(4, 0))

    S._count_label = tk.Label(ctrl_bar, text="0 / 0 在线",
        bg=C["bg"], fg=C["muted"], font=("Segoe UI", 8), anchor="w")
    S._count_label.pack(side=tk.LEFT)

    tk.Button(ctrl_bar, text="⟳ 刷新", bg=C["surface"], fg=C["text"],
              font=("Segoe UI", 8), bd=1, relief="solid",
              padx=8, pady=2, cursor="hand2",
              activebackground=C["border"],
              command=refresh_device_list).pack(side=tk.RIGHT, padx=(0, 6))

    # Hint
    S._hint_label = tk.Label(S._dev_frame,
        text="  基站显示 R=xx (rank) → 未设坐标标黄 ⚠",
        bg=C["bg"], fg=C["yellow"], font=("Segoe UI", 7), anchor="w")
    S._hint_label.pack(fill=tk.X, padx=8, pady=(1, 2))

    # Scrollable card list
    card_canvas = tk.Canvas(S._dev_frame, bg=C["bg"], highlightthickness=0,
                            bd=0, relief="flat")
    scrollbar = tk.Scrollbar(S._dev_frame, orient="vertical",
        command=card_canvas.yview, bg=C["surface"],
        troughcolor=C["bg"], bd=0, relief="flat",
        activebackground=C["border"])
    S._card_container = tk.Frame(card_canvas, bg=C["bg"])
    S._card_container.bind("<Configure>",
        lambda e: card_canvas.configure(scrollregion=card_canvas.bbox("all")))
    win = card_canvas.create_window((0, 0), window=S._card_container,
                                     anchor="nw", tags="inner")

    # Resize inner frame to match canvas width
    def on_card_canvas_resize(e):
        card_canvas.itemconfig("inner", width=e.width)
    card_canvas.bind("<Configure>", on_card_canvas_resize)

    card_canvas.configure(yscrollcommand=scrollbar.set)
    card_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
    scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
    S._card_canvas = card_canvas

    # Mouse wheel scroll for card list
    def on_card_wheel(e):
        card_canvas.yview_scroll(int(-1*(e.delta/120)), "units")
    card_canvas.bind("<Enter>", lambda e: card_canvas.bind_all("<MouseWheel>", on_card_wheel))
    card_canvas.bind("<Leave>", lambda e: card_canvas.unbind_all("<MouseWheel>"))

    # Settings frame (hidden)
    S._set_frame = tk.Frame(sb, bg=C["bg"])

def show_tab(tab):
    if tab == "devices":
        S._set_frame.pack_forget()
        S._dev_frame.pack(fill=tk.BOTH, expand=True)
        S._tab_dev.config(bg=C["accent"], fg="#ffffff", font=("Segoe UI", 11, "bold"))
        S._tab_set.config(bg=C["surface"], fg=C["muted"], font=("Segoe UI", 11))
    else:
        S._dev_frame.pack_forget()
        S._set_frame.pack(fill=tk.BOTH, expand=True)
        S._tab_set.config(bg=C["accent"], fg="#ffffff", font=("Segoe UI", 10, "bold"))
        S._tab_dev.config(bg=C["surface"], fg=C["muted"], font=("Segoe UI", 10))
        build_settings()

def build_settings():
    for w in S._set_frame.winfo_children():
        w.destroy()

    inner = tk.Frame(S._set_frame, bg=C["bg"])
    inner.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

    tk.Label(inner, text="系统", bg=C["bg"], fg=C["text"],
             font=("Segoe UI", 14, "bold")).pack(anchor="w", pady=(0, 10))

    # Status card
    card = tk.Frame(inner, bg=C["surface"], bd=0, relief="flat",
                    highlightthickness=1, highlightbackground=C["border"])
    card.pack(fill=tk.X, pady=3)

    conn = "● 已连接" if (S.ser and S.ser.is_open) else "○ 未连接"
    conn_color = C["green"] if (S.ser and S.ser.is_open) else C["red"]
    tk.Label(card, text=f"  串口:  {conn}  ", bg=C["surface"],
             fg=conn_color, font=("Segoe UI", 11)).pack(anchor="w", padx=12, pady=8)

    port = S.ser.port if S.ser else "--"
    tk.Label(card, text=f"  端口:  {port}", bg=C["surface"],
             fg=C["text"], font=("Segoe UI", 10)).pack(anchor="w", padx=12, pady=3)

    devs = len(S.devices)
    online = sum(1 for d in S.devices.values() if d.get("online"))
    tk.Label(card, text=f"  设备:  {online} / {devs} 在线", bg=C["surface"],
             fg=C["text"], font=("Segoe UI", 10)).pack(anchor="w", padx=12, pady=(3, 10))

    # Commands card
    tk.Label(inner, text="命令", bg=C["bg"], fg=C["text"],
             font=("Segoe UI", 14, "bold")).pack(anchor="w", pady=(18, 10))

    cmds = [
        ("AT+LIST",  "列出所有设备",   "已发送 AT+LIST — 刷新设备列表中...", "info"),
        ("AT+FWV",   "查询固件版本",   "已发送 AT+FWV...", "info"),
    ]
    connected = S.ser and S.ser.is_open
    for cmd, desc, feedback, level in cmds:
        row = tk.Frame(inner, bg=C["surface"], bd=0, relief="flat",
                       highlightthickness=1, highlightbackground=C["border"])
        row.pack(fill=tk.X, pady=3)

        tk.Label(row, text=f"  {cmd}", bg=C["surface"], fg=C["accent"],
                 font=("Consolas", 11, "bold")).pack(side=tk.LEFT, padx=12, pady=8)
        tk.Label(row, text=desc, bg=C["surface"], fg=C["muted"],
                 font=("Segoe UI", 10)).pack(side=tk.LEFT, padx=8)

        def _make_btn_handler(_cmd, _btn, _feedback, _level):
            def _restore_btn():
                _btn.config(text="发送", bg=C["card"], fg=C["text"],
                            state="normal")
            def _handler():
                if not (S.ser and S.ser.is_open):
                    set_status("未连接 — 请先连接串口", "warning")
                    return
                # Set loading state
                _btn.config(text="⏳ 等待中", bg="#e6a800", fg="#1f2328",
                            state="disabled")
                if _cmd == "AT+LIST":
                    if S.refresh_pending:
                        set_status("正在刷新 — 请稍候...", "warning")
                        _restore_btn()
                        return
                    S._list_popup_pending = True
                    S._list_btn = _btn
                    refresh_device_list()
                elif _cmd == "AT+PING":
                    if not send_cmd(_cmd):
                        set_status("命令发送失败 — 串口错误", "error")
                        _restore_btn()
                        return
                    S._ping_pending = True
                    S._ping_btn = _btn
                    def _ping_timeout():
                        if getattr(S, "_ping_pending", False):
                            S._ping_pending = False
                            if hasattr(S, "_ping_btn") and S._ping_btn:
                                S._ping_btn.config(text="发送", bg=C["card"], fg=C["text"], state="normal")
                                S._ping_btn = None
                            messagebox.showwarning("AT+PING 结果",
                                "无响应 — 网关可能离线或忙碌")
                    S._ping_timeout_id = root.after(4000, _ping_timeout)
                elif _cmd == "AT+FWV":
                    if not send_cmd(_cmd):
                        set_status("命令发送失败 — 串口错误", "error")
                        _restore_btn()
                        return
                    S._fwv_pending = True
                    S._fwv_versions = {}
                    S._fwv_btn = _btn
                    root.after(5000, _show_fwv_popup)
                elif _cmd == "AT+RESET":
                    if not send_cmd(_cmd):
                        set_status("命令发送失败 — 串口错误", "error")
                        _restore_btn()
                        return
                    _restore_btn()
                    messagebox.showinfo("AT+RESET 结果",
                        "已发送重置命令。\n\n"
                        "网关将重启。\n"
                        "串口连接将暂时断开。")
                else:
                    if not send_cmd(_cmd):
                        set_status("命令发送失败 — 串口错误", "error")
                        _restore_btn()
                        return
                    # Simple command — flash then restore
                    root.after(800, _restore_btn)
                set_status(_feedback, _level)
            return _handler

        btn_state = "normal" if connected else "disabled"
        btn = tk.Button(row, text="发送", bg=C["card"], fg=C["text"],
                  font=("Segoe UI", 10), bd=1, relief="solid",
                  padx=14, pady=3, cursor="hand2",
                  activebackground=C["accent"], activeforeground="#ffffff",
                  state=btn_state)
        btn.config(command=_make_btn_handler(cmd, btn, feedback, level))
        btn.pack(side=tk.RIGHT, padx=12, pady=8)

    # --- Network config ---
    net_row = tk.Frame(inner, bg=C["surface"], bd=0, relief="flat",
                       highlightthickness=1, highlightbackground=C["border"])
    net_row.pack(fill=tk.X, pady=3)
    tk.Label(net_row, text="  组网配置", bg=C["surface"], fg=C["accent"],
             font=("Consolas", 11, "bold")).pack(side=tk.LEFT, padx=12, pady=8)
    tk.Label(net_row, text="选择参与定位的基站", bg=C["surface"], fg=C["muted"],
             font=("Segoe UI", 10)).pack(side=tk.LEFT, padx=8)
    def _open_network_dialog():
        if not (S.ser and S.ser.is_open):
            set_status("未连接", "warning"); return
        anchors = []
        for uid, d in S.devices.items():
            if d.get("type") == "ANCHOR":
                online = d.get("online", False)
                ax, ay = d.get("ax"), d.get("ay")
                coord_str = f"({ax:.0f}, {ay:.0f})" if ax is not None else "—"
                anchors.append((uid, get_label(uid), online, coord_str))
        if not anchors:
            anchors = [("0000", "A01", False, "—")]
        anchors.sort(key=lambda x: x[0])
        n = len(anchors)
        h = 420 + n * 50
        dlg = tk.Toplevel(root)
        dlg.title("组网配置")
        dlg.geometry(f"750x{h}")
        dlg.minsize(700, 400)
        dlg.configure(bg=C["bg"])
        dlg.transient(root); dlg.grab_set()
        dlg.resizable(True, True)

        # ── Header bar (same style as OTA dialog) ──
        hdr = tk.Frame(dlg, bg=C["accent"], height=52)
        hdr.pack(fill=tk.X)
        hdr.grid_propagate(False)
        tk.Label(hdr, text="  ⚙  组网配置", bg=C["accent"], fg="#ffffff",
                 font=("Segoe UI", 14, "bold")).pack(side=tk.LEFT, padx=18, pady=13)
        online_cnt = sum(1 for _, _, on, _ in anchors if on)
        tk.Label(hdr, text=f"{online_cnt}/{n} 在线  ", bg=C["accent"], fg="#a5d6ff",
                 font=("Segoe UI", 11)).pack(side=tk.RIGHT, padx=18, pady=13)

        # ── Body ──
        body = tk.Frame(dlg, bg=C["bg"])
        body.pack(fill=tk.BOTH, expand=True, padx=16, pady=(12, 6))

        # Hint card
        hint = tk.Frame(body, bg=C["surface"], highlightthickness=1,
                        highlightbackground=C["border"])
        hint.pack(fill=tk.X, pady=(0, 10))
        tk.Label(hint, text="  选择参与 TDOA 定位的基站（至少 2 个，含主基站 0000）",
                 bg=C["surface"], fg=C["text"], font=("Segoe UI", 10)).pack(
                     anchor="w", padx=12, pady=10)

        # Anchor list card
        card = tk.Frame(body, bg=C["surface"], highlightthickness=1,
                        highlightbackground=C["border"])
        card.pack(fill=tk.X, pady=(0, 10))
        # Table header inside card
        thdr = tk.Frame(card, bg=C["surface"])
        thdr.pack(fill=tk.X, padx=10, pady=(10, 4))
        for txt, w in [("基站", 8), ("UID", 6), ("坐标(cm)", 14), ("状态", 7), ("启用", 6)]:
            tk.Label(thdr, text=txt, bg=C["surface"], fg=C["muted"],
                     font=("Segoe UI", 9, "bold"), width=w, anchor="w").pack(side=tk.LEFT, padx=2)
        tk.Frame(card, bg=C["border"], height=1).pack(fill=tk.X, padx=8)

        vars_dict = {}
        for uid, label, online, coord in anchors:
            row = tk.Frame(card, bg=C["surface"])
            row.pack(fill=tk.X, padx=10, pady=2)
            rfg = C["text"] if online else C["muted"]
            tk.Label(row, text=label, bg=C["surface"], fg=rfg,
                     font=("Consolas", 11, "bold"), width=8, anchor="w").pack(side=tk.LEFT, padx=2)
            tk.Label(row, text=uid, bg=C["surface"], fg=rfg,
                     font=("Consolas", 10), width=6, anchor="w").pack(side=tk.LEFT, padx=2)
            tk.Label(row, text=coord, bg=C["surface"], fg=rfg,
                     font=("Consolas", 10), width=14, anchor="w").pack(side=tk.LEFT, padx=2)
            stxt, sfg = ("● 在线", C["green"]) if online else ("○ 离线", C["muted"])
            tk.Label(row, text=stxt, bg=C["surface"], fg=sfg,
                     font=("Segoe UI", 9), width=7, anchor="w").pack(side=tk.LEFT, padx=2)
            v = tk.BooleanVar(value=online)
            vars_dict[uid] = v
            tk.Checkbutton(row, variable=v, bg=C["surface"], fg=C["text"],
                           selectcolor=C["surface"], activebackground=C["surface"],
                           activeforeground=C["accent"]).pack(side=tk.LEFT, padx=14)

        # ── Bottom buttons ──
        bf = tk.Frame(body, bg=C["bg"])
        bf.pack(fill=tk.X, pady=(4, 8))
        def _sel(val): return lambda: [x.set(val) for x in vars_dict.values()]
        tk.Button(bf, text="全选", bg=C["surface"], fg=C["text"],
                  font=("Segoe UI", 10), bd=1, relief="solid",
                  highlightbackground=C["border"],
                  padx=18, pady=5, cursor="hand2",
                  activebackground=C["card"],
                  command=_sel(True)).pack(side=tk.LEFT, padx=3)
        tk.Button(bf, text="全不选", bg=C["surface"], fg=C["text"],
                  font=("Segoe UI", 10), bd=1, relief="solid",
                  highlightbackground=C["border"],
                  padx=18, pady=5, cursor="hand2",
                  activebackground=C["card"],
                  command=_sel(False)).pack(side=tk.LEFT, padx=3)
        def _save():
            mask = 0; cnt = 0
            for uid, v in vars_dict.items():
                if v.get():
                    mask |= (1 << int(uid, 16)); cnt += 1
            if cnt < 2:
                messagebox.showwarning("组网配置", "至少需要选择 2 个基站。", parent=dlg); return
            send_cmd(f"AT+NETWORK={mask:04X}")
            set_status(f"组网已配置: {cnt} 个基站 (掩码 0x{mask:04X})", "success")
            dlg.destroy()
        tk.Button(bf, text="取消", bg=C["surface"], fg=C["muted"],
                  font=("Segoe UI", 11), bd=1, relief="solid",
                  highlightbackground=C["border"],
                  padx=20, pady=5, cursor="hand2",
                  activebackground=C["card"],
                  command=dlg.destroy).pack(side=tk.RIGHT, padx=3)
        tk.Button(bf, text="✓ 保存配置", bg=C["accent"], fg="#ffffff",
                  font=("Segoe UI", 11, "bold"), bd=0, relief="flat",
                  padx=22, pady=7, cursor="hand2",
                  activebackground="#218bff",
                  command=_save).pack(side=tk.RIGHT, padx=6)
    tk.Button(net_row, text="配置", bg=C["card"], fg=C["text"],
              font=("Segoe UI", 10), bd=1, relief="solid",
              padx=14, pady=3, cursor="hand2",
              command=_open_network_dialog,
              state="normal" if connected else "disabled").pack(side=tk.RIGHT, padx=12, pady=8)

    def _open_netstart_dialog():
        """Open Root-selection dialog and send AT+NETSTART=<root_rank>.

        The gateway parses <root_rank> as a DECIMAL number and puts it on the
        air as the anchor's 16-bit short address, so the host must send the
        rank (not the hex UID).  A manual rank entry is provided as a fallback
        for the case where no anchor has been discovered yet.
        """
        if not (S.ser and S.ser.is_open):
            set_status("未连接", "warning"); return
        # Collect online anchors as candidates for Root
        anchors = []
        for uid, d in S.devices.items():
            if d.get("type") == "ANCHOR" and d.get("online"):
                label = get_label(uid)
                coord = ""
                if d.get("ax") is not None:
                    coord = f"({d['ax']:.2f}, {d['ay']:.2f}, {d['az']:.2f})m"
                anchors.append((uid, label, coord, int(d.get("rank") or 0)))
        # Rank 0 is reserved for the gateway — an anchor without a valid rank
        # cannot be designated Root.
        anchors = [a for a in anchors if a[3] > 0]
        anchors.sort(key=lambda x: (x[3], x[0]))

        dlg = tk.Toplevel(root)
        dlg.title("开始组网 — 选择 Root 基站")
        dlg.geometry("550x520")
        dlg.minsize(500, 440)
        dlg.configure(bg=C["bg"])
        dlg.transient(root); dlg.grab_set()

        hdr = tk.Frame(dlg, bg=C["accent"], height=52)
        hdr.pack(fill=tk.X)
        hdr.grid_propagate(False)
        tk.Label(hdr, text="  ⚡  开始组网", bg=C["accent"], fg="#ffffff",
                 font=("Segoe UI", 14, "bold")).pack(side=tk.LEFT, padx=18, pady=13)
        tk.Label(hdr, text=f"{len(anchors)} 在线  ", bg=C["accent"], fg="#a5d6ff",
                 font=("Segoe UI", 11)).pack(side=tk.RIGHT, padx=18, pady=13)

        body = tk.Frame(dlg, bg=C["bg"])
        body.pack(fill=tk.BOTH, expand=True, padx=16, pady=(12, 6))

        hint = tk.Frame(body, bg=C["surface"], highlightthickness=1,
                        highlightbackground=C["border"])
        hint.pack(fill=tk.X, pady=(0, 10))
        tk.Label(hint, text="  选择作为 Root（时钟源）的基站，组网后将由该基站统一广播时钟同步。",
                 bg=C["surface"], fg=C["text"], font=("Segoe UI", 10)).pack(
                     anchor="w", padx=12, pady=10)

        card = tk.Frame(body, bg=C["surface"], highlightthickness=1,
                        highlightbackground=C["border"])
        card.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
        thdr = tk.Frame(card, bg=C["surface"])
        thdr.pack(fill=tk.X, padx=10, pady=(10, 4))
        for txt, w in [("基站", 8), ("UID", 8), ("Rank", 7), ("坐标", 16), ("参与", 5), ("Root", 5)]:
            tk.Label(thdr, text=txt, bg=C["surface"], fg=C["muted"],
                     font=("Segoe UI", 9, "bold"), width=w, anchor="w").pack(side=tk.LEFT, padx=2)
        tk.Frame(card, bg=C["border"], height=1).pack(fill=tk.X, padx=8)

        # Radio value IS the decimal rank — that is what goes on the air.
        selected_root = tk.StringVar(value=str(anchors[0][3]) if anchors else "")
        # P1-1: the participant roster.  Every anchor must know the full list,
        # otherwise a Root that only hears half of the network can never
        # complete it — and nobody can tell which member is missing.
        # All anchors take part by default; the Root is always included.
        participate = {}
        if not anchors:
            tk.Label(card, text="  未发现在线基站 — 请在下方手动输入 Root 的 Rank",
                     bg=C["surface"], fg=C["muted"],
                     font=("Segoe UI", 10)).pack(anchor="w", padx=12, pady=14)
        for uid, label, coord, rank in anchors:
            row = tk.Frame(card, bg=C["surface"])
            row.pack(fill=tk.X, padx=10, pady=2)
            tk.Label(row, text=label, bg=C["surface"], fg=C["text"],
                     font=("Consolas", 11, "bold"), width=8, anchor="w").pack(side=tk.LEFT, padx=2)
            tk.Label(row, text=uid, bg=C["surface"], fg=C["text"],
                     font=("Consolas", 10), width=8, anchor="w").pack(side=tk.LEFT, padx=2)
            tk.Label(row, text=str(rank), bg=C["surface"], fg=C["accent"],
                     font=("Consolas", 10, "bold"), width=7, anchor="w").pack(side=tk.LEFT, padx=2)
            tk.Label(row, text=coord, bg=C["surface"], fg=C["muted"],
                     font=("Consolas", 10), width=16, anchor="w").pack(side=tk.LEFT, padx=2)
            pvar = tk.BooleanVar(value=True)
            participate[rank] = pvar
            tk.Checkbutton(row, variable=pvar, bg=C["surface"], fg=C["accent"],
                           selectcolor=C["surface"], activebackground=C["surface"],
                           activeforeground=C["accent"]).pack(side=tk.LEFT, padx=2)
            tk.Radiobutton(row, variable=selected_root, value=str(rank),
                           bg=C["surface"], fg=C["accent"], selectcolor=C["surface"],
                           activebackground=C["surface"], activeforeground=C["accent"],
                           font=("Segoe UI", 10, "bold")).pack(side=tk.LEFT, padx=8)

        # --- Manual rank entry (fallback when no anchor was discovered) ---
        manual = tk.Frame(body, bg=C["surface"], highlightthickness=1,
                          highlightbackground=C["border"])
        manual.pack(fill=tk.X, pady=(0, 10))
        tk.Label(manual, text="  手动 Rank", bg=C["surface"], fg=C["accent"],
                 font=("Consolas", 11, "bold")).pack(side=tk.LEFT, padx=12, pady=9)
        manual_rank_var = tk.StringVar(value="")
        ment = tk.Entry(manual, textvariable=manual_rank_var, bg=C["bg"], fg=C["text"],
                        font=("Consolas", 11), relief="solid", bd=1, width=8,
                        justify="right", insertbackground=C["text"])
        ment.pack(side=tk.LEFT, padx=6)
        tk.Label(manual, text="十进制地址 (1–65535)，填写后优先使用",
                 bg=C["surface"], fg=C["muted"],
                 font=("Segoe UI", 9)).pack(side=tk.LEFT, padx=4)

        bf = tk.Frame(body, bg=C["bg"])
        bf.pack(fill=tk.X, pady=(4, 8))
        tk.Button(bf, text="取消", bg=C["surface"], fg=C["muted"],
                  font=("Segoe UI", 11), bd=1, relief="solid",
                  highlightbackground=C["border"],
                  padx=20, pady=5, cursor="hand2",
                  activebackground=C["card"],
                  command=dlg.destroy).pack(side=tk.RIGHT, padx=3)
        def _do_netstart():
            manual_txt = manual_rank_var.get().strip()
            if manual_txt:
                if not manual_txt.isdigit() or not (1 <= int(manual_txt) <= 65535):
                    messagebox.showwarning("开始组网",
                        "手动 Rank 必须是 1–65535 之间的十进制整数。", parent=dlg)
                    return
                root_rank = int(manual_txt)
            else:
                sel = selected_root.get()
                if not sel:
                    messagebox.showwarning("开始组网",
                        "请选择一个 Root 基站，或手动输入 Rank。", parent=dlg)
                    return
                root_rank = int(sel)
            # P1-1: build the participant roster — the Root plus every anchor
            # the operator ticked.  The gateway broadcasts it with the
            # NETWORK_START frame so that all anchors know the full network
            # and the Root can report who is missing.
            roster = [root_rank]
            for rank, var in sorted(participate.items()):
                if var.get() and rank != root_rank and rank not in roster:
                    roster.append(rank)
            if len(roster) < NET_MIN_PARTICIPANTS:
                if not messagebox.askyesno("开始组网",
                        f"当前名单只有 {len(roster)} 个基站。\n\n"
                        f"少于 {NET_MIN_PARTICIPANTS} 个基站时 TDOA 解算没有几何约束，"
                        "坐标会退化到质心。\n"
                        "仍然继续组网吗？", parent=dlg):
                    return
            if len(roster) > NET_MAX_PARTICIPANTS:
                if not messagebox.askyesno("开始组网",
                        f"当前名单有 {len(roster)} 个基站。\n\n"
                        f"本方案最多支持 {NET_MAX_PARTICIPANTS} 个基站"
                        f"（1 个 Root + {NET_MAX_PARTICIPANTS - 1} 个子站），"
                        "超出的成员会被固件丢弃。\n"
                        "仍然继续组网吗？", parent=dlg):
                    return
            dlg.destroy()
            S._netstart_btn = netstart_btn
            netstart_btn.config(text="⏳ 组网中...", state="disabled", bg="#e6a800", fg="#1f2328")
            S.net_roster = list(roster)
            S.net_root_rank = root_rank
            S.net_missing_ranks = set()
            send_cmd("AT+NETSTART=" + str(root_rank) + "," + ",".join(str(r) for r in roster[1:]))
            set_status(f"正在组网 — Root=R{root_rank}，参与基站 {len(roster)} 个...", "info")
        tk.Button(bf, text="✓ 开始组网", bg=C["green"], fg="#ffffff",
                  font=("Segoe UI", 11, "bold"), bd=0, relief="flat",
                  padx=22, pady=7, cursor="hand2",
                  activebackground="#218b21",
                  command=_do_netstart).pack(side=tk.RIGHT, padx=6)

    netstart_btn = tk.Button(net_row, text="开始组网", bg=C["card"], fg=C["text"],
                             font=("Segoe UI", 10), bd=1, relief="solid",
                             padx=14, pady=3, cursor="hand2",
                             command=_open_netstart_dialog,
                             state="normal" if connected else "disabled")
    netstart_btn.pack(side=tk.RIGHT, padx=(0, 6), pady=8)

    # --- Uniform Z write (all online anchors) ---
    z_row = tk.Frame(inner, bg=C["surface"], bd=0, relief="flat",
                     highlightthickness=1, highlightbackground=C["border"])
    z_row.pack(fill=tk.X, pady=3)
    tk.Label(z_row, text="  统一写Z", bg=C["surface"], fg=C["accent"],
             font=("Consolas", 11, "bold")).pack(side=tk.LEFT, padx=12, pady=8)
    z_var = tk.StringVar(value="0.00")
    tk.Entry(z_row, textvariable=z_var, bg=C["bg"], fg=C["text"],
             font=("Consolas", 11), relief="solid", bd=1, width=8,
             justify="right", insertbackground=C["text"]).pack(side=tk.LEFT, padx=6)
    tk.Label(z_row, text="米 → 所有在线基站", bg=C["surface"], fg=C["muted"],
             font=("Segoe UI", 9)).pack(side=tk.LEFT, padx=2)
    def _write_uniform_z():
        if not (S.ser and S.ser.is_open):
            set_status("未连接", "warning"); return
        try:
            z = float(z_var.get())
        except ValueError:
            messagebox.showerror("无效输入", "Z 值必须为数字。")
            return
        targets = []
        for uid, d in S.devices.items():
            if d.get("type") == "ANCHOR" and d.get("online"):
                ax, ay = d.get("ax"), d.get("ay")
                if ax is None or ay is None:
                    continue
                targets.append((uid, ax, ay))
        if not targets:
            messagebox.showwarning("统一写入Z", "没有在线且已设置 X/Y 的基站。\n\n请先逐个写入各基站的 X/Y 坐标。")
            return
        for uid, ax, ay in targets:
            send_cmd(f"AT+COORD={uid},{ax:.2f},{ay:.2f},{z:.2f}")
            S.devices[uid]["az"] = z
            S.anchor_coords[uid] = (ax, ay, z)
        save_anchor_config()
        set_status(f"已向 {len(targets)} 个在线基站写入 Z = {z:.2f} m", "success")
        S._dirty = True; refresh_lists()
    tk.Button(z_row, text="写入", bg=C["card"], fg=C["text"],
              font=("Segoe UI", 10), bd=1, relief="solid",
              padx=14, pady=3, cursor="hand2",
              activebackground=C["accent"], activeforeground="#ffffff",
              command=_write_uniform_z,
              state="normal" if connected else "disabled").pack(side=tk.RIGHT, padx=12, pady=8)

    # --- Start TDOA ---
    start_row = tk.Frame(inner, bg=C["surface"], bd=0, relief="flat",
                         highlightthickness=1, highlightbackground=C["green"])
    start_row.pack(fill=tk.X, pady=3)
    tk.Label(start_row, text="  AT+START", bg=C["surface"], fg="#28a745",
             font=("Consolas", 11, "bold")).pack(side=tk.LEFT, padx=12, pady=8)
    tk.Label(start_row, text="启动 TDOA 定位", bg=C["surface"], fg=C["muted"],
             font=("Segoe UI", 10)).pack(side=tk.LEFT, padx=8)
    def _do_start():
        if not (S.ser and S.ser.is_open):
            set_status("未连接", "warning"); return
        send_cmd("AT+START")
        set_status("已发送 AT+START — 等待 Master 启动 TDOA...", "info")
        messagebox.showinfo("启动 TDOA",
            "已发送启动命令。\n\nMaster 将退出配置模式，开始发送信标。\nTag 将开始定位。",
            parent=root)
    def _do_stop():
        if not (S.ser and S.ser.is_open):
            set_status("未连接", "warning"); return
        send_cmd("AT+STOP")
        set_status("已发送 AT+STOP — Master 返回配置模式", "info")
    tk.Button(start_row, text="⏹ 停止", bg="#dc3545", fg="#ffffff",
              font=("Segoe UI", 10, "bold"), bd=1, relief="solid",
              padx=14, pady=3, cursor="hand2",
              activebackground="#c82333", activeforeground="#ffffff",
              command=_do_stop,
              state="normal" if connected else "disabled").pack(side=tk.RIGHT, padx=4, pady=8)
    tk.Button(start_row, text="▶ 启动", bg="#28a745", fg="#ffffff",
              font=("Segoe UI", 10, "bold"), bd=1, relief="solid",
              padx=14, pady=3, cursor="hand2",
              activebackground="#218838", activeforeground="#ffffff",
              command=_do_start,
              state="normal" if connected else "disabled").pack(side=tk.RIGHT, padx=12, pady=8)

    # --- Tag update rate ---
    rate_row = tk.Frame(inner, bg=C["surface"], bd=0, relief="flat",
                        highlightthickness=1, highlightbackground=C["border"])
    rate_row.pack(fill=tk.X, pady=3)

    tk.Label(rate_row, text="  AT+RATE", bg=C["surface"], fg=C["accent"],
             font=("Consolas", 11, "bold")).pack(side=tk.LEFT, padx=12, pady=8)
    tk.Label(rate_row, text="设置标签更新频率", bg=C["surface"], fg=C["muted"],
             font=("Segoe UI", 10)).pack(side=tk.LEFT, padx=8)

    RATE_PRESETS = {"1 Hz": 1000, "2 Hz": 500, "5 Hz": 200, "10 Hz": 100, "20 Hz": 50, "50 Hz": 20}
    rate_var = tk.StringVar(value="10 Hz")
    rate_combo = ttk.Combobox(rate_row, textvariable=rate_var,
                              values=list(RATE_PRESETS.keys()), state="readonly",
                              width=8, font=("Segoe UI", 10))
    rate_combo.pack(side=tk.LEFT, padx=6, pady=8)

    rate_btn = tk.Button(rate_row, text="发送", bg=C["card"], fg=C["text"],
                         font=("Segoe UI", 10), bd=1, relief="solid",
                         padx=14, pady=3, cursor="hand2",
                         activebackground=C["accent"], activeforeground="#ffffff",
                         state=btn_state)
    def _rate_handler(_btn=rate_btn):
        if not (S.ser and S.ser.is_open):
            set_status("未连接 — 请先连接串口", "warning")
            return
        sel = rate_var.get()
        ms = RATE_PRESETS.get(sel, 100)
        cmd = f"AT+RATE={ms}"
        if not send_cmd(cmd):
            set_status("命令发送失败 — 串口错误", "error")
            return
        _btn.config(text="⏳ 等待中", bg="#e6a800", fg="#1f2328", state="disabled")
        S._rate_pending = True
        S._rate_ms = ms
        S._rate_sel = sel
        S._rate_btn = _btn
        set_status(f"已发送 AT+RATE={ms} ({sel}) — 等待响应...", "info")
        def _rate_timeout():
            if getattr(S, "_rate_pending", False):
                S._rate_pending = False
                if hasattr(S, "_rate_btn") and S._rate_btn:
                    S._rate_btn.config(text="发送", bg=C["card"], fg=C["text"], state="normal")
                    S._rate_btn = None
                messagebox.showwarning("AT+RATE 结果", f"无响应 — 网关可能离线或忙碌")
        S._rate_timeout_id = root.after(3500, _rate_timeout)
    rate_btn.config(command=_rate_handler)
    rate_btn.pack(side=tk.RIGHT, padx=12, pady=8)

    # Batch OTA
    tk.Label(inner, text="批量固件升级", bg=C["bg"], fg=C["text"],
             font=("Segoe UI", 14, "bold")).pack(anchor="w", pady=(24, 10))

    batch_row = tk.Frame(inner, bg=C["bg"])
    batch_row.pack(fill=tk.X, pady=(0, 4))

    anchor_cnt = sum(1 for d in S.devices.values()
                     if d.get("type") == "ANCHOR" and d.get("online"))
    tag_cnt = sum(1 for d in S.devices.values()
                  if d.get("type") == "TAG" and d.get("online"))

    tk.Button(batch_row, text=f"⬆ 批量升级基站  ({anchor_cnt} online)",
              bg=C["surface"], fg=C["text"], font=("Segoe UI", 10),
              bd=1, relief="solid", padx=14, pady=8, cursor="hand2",
              activebackground=C["accent"], activeforeground="#ffffff",
              command=lambda: open_batch_ota_dialog("ANCHOR")).pack(
              side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 4))

    tk.Button(batch_row, text=f"⬆ 批量升级标签  ({tag_cnt} online)",
              bg=C["surface"], fg=C["text"], font=("Segoe UI", 10),
              bd=1, relief="solid", padx=14, pady=8, cursor="hand2",
              activebackground=C["accent"], activeforeground="#ffffff",
              command=lambda: open_batch_ota_dialog("TAG")).pack(
              side=tk.LEFT, fill=tk.X, expand=True, padx=(4, 0))

    # Display toggles — large toggle buttons
    tk.Label(inner, text="显示", bg=C["bg"], fg=C["text"],
             font=("Segoe UI", 14, "bold")).pack(anchor="w", pady=(18, 10))

    for key, label in [("show_grid", "网格"), ("show_trails", "轨迹"),
                        ("show_labels", "标签"), ("show_links", "连线")]:
        row = tk.Frame(inner, bg=C["surface"], bd=0, relief="flat",
                       highlightthickness=1, highlightbackground=C["border"])
        row.pack(fill=tk.X, pady=3)
        tk.Label(row, text=f"  {label}", bg=C["surface"], fg=C["text"],
                 font=("Segoe UI", 11), width=14, anchor="w").pack(side=tk.LEFT, padx=12, pady=10)

        # Toggle button instead of tiny checkbox
        def make_toggle(k=key):
            on = getattr(S, k)
            return tk.Button(row, text="◉  ON" if on else "○  OFF",
                bg=C["green"] if on else C["border"], fg="#ffffff" if on else C["muted"],
                font=("Segoe UI", 10, "bold"), bd=0, relief="flat",
                padx=16, pady=5, cursor="hand2",
                activebackground=C["green"] if not on else C["border"],
                activeforeground="#ffffff" if not on else C["muted"],
                command=lambda k=k: toggle_display(k))
        btn = make_toggle()
        btn.pack(side=tk.RIGHT, padx=12, pady=8)
        # Store reference for updates
        if not hasattr(S, "_toggle_btns"):
            S._toggle_btns = {}
        S._toggle_btns[key] = (row, btn)

    def toggle_display(k):
        new_val = not getattr(S, k)
        setattr(S, k, new_val)
        # Update button in-place — no rebuild needed
        if hasattr(S, "_toggle_btns") and k in S._toggle_btns:
            row, btn = S._toggle_btns[k]
            if new_val:
                btn.config(text="◉  ON", bg=C["green"], fg="#ffffff",
                          activebackground=C["border"], activeforeground=C["muted"])
            else:
                btn.config(text="○  OFF", bg=C["border"], fg=C["muted"],
                          activebackground=C["green"], activeforeground="#ffffff")

    # Clear tag trails
    def clear_trails():
        n = 0
        for uid, d in S.devices.items():
            if d.get("type") == "TAG" and d.get("trail"):
                d["trail"] = []
                n += 1
        set_status(f"已清除 {n} 个标签的轨迹" if n else "当前没有标签轨迹", "info")

    tk.Button(inner, text="🗑 清除标签轨迹",
              bg=C["surface"], fg=C["text"], font=("Segoe UI", 10),
              bd=1, relief="solid", padx=14, pady=8, cursor="hand2",
              activebackground=C["accent"], activeforeground="#ffffff",
              command=clear_trails).pack(fill=tk.X, pady=(14, 0))

def refresh_lists(force_rebuild=False):
    search = S._search_var.get().lower() if hasattr(S, "_search_var") else ""

    def sort_key(item):
        uid, d = item
        dtype = d.get("type", "")
        return (not d.get("online"),
                0 if dtype == "GATEWAY" else 1 if dtype == "ANCHOR" else 2,
                uid)

    items = sorted(((k, v) for k, v in S.devices.items()
                    if not search or search in k.lower() or search in v.get("type","").lower()),
                   key=sort_key)

    online_cnt = sum(1 for _, d in items if d.get("online"))
    offline_cnt = len(items) - online_cnt
    S._count_label.config(text=f"● {online_cnt} 在线  ○ {offline_cnt} 离线")

    visible_uids = tuple(uid for uid, _ in items)
    prev_uids = getattr(S, "_prev_visible_uids", ())
    dirty = getattr(S, "_dirty", True)

    if not force_rebuild and not dirty and visible_uids == prev_uids:
        return

    S._prev_visible_uids = visible_uids
    S._dirty = False

    for w in S._card_container.winfo_children():
        w.destroy()

    online_items = [(u, d) for u, d in items if d.get("online")]
    offline_items = [(u, d) for u, d in items if not d.get("online")]

    def render_card(uid, d, is_offline=False):
        online = d.get("online", False)
        dtype = d.get("type", "?")
        is_tag = dtype == "TAG"
        is_gw  = dtype == "GATEWAY"

        border_color = C["red"] if is_offline else C["border"]
        card_bg = "#fafafa" if is_offline else C["card"]

        card = tk.Frame(S._card_container, bg=card_bg,
                        highlightthickness=1, highlightbackground=border_color)
        card.pack(fill=tk.X, padx=8, pady=2)

        top = tk.Frame(card, bg=card_bg)
        top.pack(fill=tk.X, padx=8, pady=(6, 2))

        dot_color = C["green"] if online else C["red"]
        dot = tk.Canvas(top, width=8, height=8, bg=card_bg, highlightthickness=0)
        dot.create_oval(0, 0, 8, 8, fill=dot_color, outline="")
        dot.pack(side=tk.LEFT, padx=(0, 6))

        label_color = C["green"] if online else C["red"]
        label = get_label(uid)
        fw = d.get("fw_ver", "")
        if fw:
            label = f"{label} ({fw})"
        tk.Label(top, text=label, bg=card_bg, fg=label_color,
                 font=("Consolas", 11, "bold")).pack(side=tk.LEFT)

        if is_gw:
            badge_bg, badge_fg = C["green"], "#ffffff"
        elif is_tag:
            badge_bg, badge_fg = "#e6a800", "#1f2328"
        else:
            badge_bg, badge_fg = C["anchor"], "#ffffff"
        tk.Label(top, text=f" {dtype} ", bg=badge_bg, fg=badge_fg,
                 font=("Segoe UI", 8, "bold"), padx=4).pack(side=tk.LEFT, padx=8)

        if uid == S.selected_uid:
            tk.Label(top, text="◀", bg=card_bg, fg=C["accent"],
                     font=("Segoe UI", 10)).pack(side=tk.RIGHT)

        info = ""
        if is_gw:
            info = "● 在线" if online else "○ 离线"
        elif dtype == "ANCHOR":
            rank = d.get("rank", 0)
            info = f"R={rank:02d}  "
            if d.get("ax") is not None:
                info += f"({d['ax']:.2f}, {d['ay']:.2f}, {d['az']:.2f})m  ✓已设坐标"
            else:
                info += "⚠ 未设坐标"
            # P1-1: an anchor that did not join the network must be visible
            # as such — it must not be used in the TDOA solve.
            if rank in (getattr(S, "net_missing_ranks", None) or set()):
                info += "  ⚠ 未加入网络"
            elif d.get("pending"):
                info += "  ○ 待确认"
            if d.get("rank_collision"):
                info += "  ⚠ rank冲突"
        elif d.get("x") is not None:
            info = f"{d['x']:.2f}  {d['y']:.2f}  Q:{d.get('q','?')}"
        else:
            info = "—"
        tk.Label(card, text=info, bg=card_bg, fg=C["muted"],
                 font=("Consolas", 8)).pack(anchor="w", padx=8, pady=(0, 4))

        # Actions only for online non-gateway devices
        if not is_offline and not is_gw:
            act = tk.Frame(card, bg=card_bg)
            act.pack(fill=tk.X, padx=6, pady=(0, 6))

            tk.Button(act, text="📍", bg=C["surface"], fg=C["text"],
                      font=("Segoe UI", 10), bd=0, relief="flat", cursor="hand2",
                      activebackground=C["accent"],
                      command=lambda u=uid: select_device(u)).pack(side=tk.LEFT, padx=1)

            is_master = (uid == "0000")  # Master blinks LED on power-up, no Find needed
            if not is_master and not is_tag:
                find_on = uid in S.find_states and S.find_states[uid]
                find_bg = C["green"] if find_on else C["surface"]
                find_text = "◉ 寻找" if find_on else "○ 寻找"
                tk.Button(act, text=find_text, bg=find_bg, fg=C["text"],
                          font=("Segoe UI", 9), bd=0, relief="flat", cursor="hand2",
                          activebackground=C["green"],
                          command=lambda u=uid: toggle_find(u)).pack(side=tk.LEFT, padx=1)

            if not is_tag:
                tk.Button(act, text="⚙ 坐标", bg=C["surface"], fg=C["text"],
                          font=("Segoe UI", 9), bd=0, relief="flat", cursor="hand2",
                          activebackground=C["accent"],
                          command=lambda u=uid: open_coord_dialog(u)).pack(side=tk.LEFT, padx=1)

            tk.Button(act, text="⬆ OTA", bg=C["surface"], fg=C["text"],
                      font=("Segoe UI", 8), bd=0, relief="flat", cursor="hand2",
                      activebackground=C["orange"],
                      command=lambda u=uid: open_ota_dialog(u)).pack(side=tk.RIGHT, padx=1)

    for uid, d in online_items:
        render_card(uid, d, is_offline=False)

    if offline_items:
        sep = tk.Frame(S._card_container, bg=C["bg"])
        sep.pack(fill=tk.X, padx=8, pady=(10, 4))
        tk.Label(sep, text=f"──  离线设备 ({len(offline_items)})  ──",
                 bg=C["bg"], fg=C["red"], font=("Segoe UI", 8, "bold")).pack()

        for uid, d in offline_items:
            render_card(uid, d, is_offline=True)

def select_device(uid):
    S.selected_uid = uid
    refresh_lists(force_rebuild=True)
    d = S.devices.get(uid, {})
    x, y = get_anchor_xy(d)
    S.ox = x if x is not None else 0
    S.oy = y if y is not None else 0
    show_info(uid)

def toggle_find(uid):
    """Momentary trigger — anchor auto-stops blinking after 5s."""
    send_cmd(f"AT+FIND={uid},1")
    set_status(f"已发送寻找命令 → {uid}", "info")

# ══════════════════════════════════════════════
#  Coord Dialog
# ══════════════════════════════════════════════
def open_coord_dialog(uid):
    d = S.devices.get(uid, {})
    DW, DH = 460, 460
    dialog = tk.Toplevel(root)
    dialog.title(f"基站坐标 — {uid}")
    dialog.configure(bg=C["bg"])
    dialog.transient(root)
    dialog.grab_set()
    dialog.resizable(False, False)

    # ── Header ──
    hdr = tk.Frame(dialog, bg=C["accent"], height=56)
    hdr.pack(fill=tk.X)
    hdr.grid_propagate(False)
    label = get_label(uid)
    header_text = f"  ▲  {label}" + (f"  ({uid})" if label != uid else "")
    tk.Label(hdr, text=header_text, bg=C["accent"], fg="#ffffff",
             font=("Segoe UI", 14, "bold")).pack(side=tk.LEFT, padx=18, pady=14)
    online = d.get("online", False)
    status_text = "● 在线" if online else "○ 离线"
    status_color = "#a5d6ff" if online else "#c8daf5"
    tk.Label(hdr, text=f"{status_text}  ", bg=C["accent"], fg=status_color,
             font=("Segoe UI", 9)).pack(side=tk.RIGHT, padx=18, pady=14)

    # ── Body ──
    body = tk.Frame(dialog, bg=C["bg"])
    body.pack(fill=tk.BOTH, expand=True, padx=16, pady=(14, 6))

    # Current position info
    cur_ax = d.get("ax")
    if cur_ax is not None:
        cur_text = f"当前位置:  ({cur_ax:.2f},  {d.get('ay',0):.2f},  {d.get('az',0):.2f})  m"
    else:
        cur_text = "未设置位置 — 请在下方输入坐标"
    tk.Label(body, text=cur_text, bg=C["bg"], fg=C["muted"],
             font=("Segoe UI", 8)).pack(anchor="w", pady=(0, 10))

    # Coordinate card
    card = tk.Frame(body, bg=C["surface"], highlightthickness=1,
                    highlightbackground=C["border"])
    card.pack(fill=tk.X)

    tk.Label(card, text="  设置位置 (米)", bg=C["surface"], fg=C["muted"],
             font=("Segoe UI", 8, "bold")).pack(anchor="w", padx=14, pady=(12, 10))

    entries = {}
    for label, key, clr in [("X", "ax", "#cf222e"), ("Y", "ay", "#1a7f37"), ("Z", "az", "#0969da")]:
        row = tk.Frame(card, bg=C["surface"])
        row.pack(fill=tk.X, padx=14, pady=6)

        # Colored axis label
        tk.Label(row, text=f"●", bg=C["surface"], fg=clr,
                 font=("Segoe UI", 8)).pack(side=tk.LEFT, padx=(0, 4))
        tk.Label(row, text=label, bg=C["surface"], fg=C["text"],
                 font=("Segoe UI", 13, "bold"), width=2, anchor="w").pack(side=tk.LEFT)

        v = tk.StringVar(value=f"{d.get(key, 0):.2f}")
        e = tk.Entry(row, textvariable=v, bg=C["bg"], fg=C["text"],
            font=("Consolas", 14), relief="solid", bd=1, width=14,
            justify="right", insertbackground=C["text"])
        e.pack(side=tk.RIGHT, padx=(0, 4))
        tk.Label(row, text="m", bg=C["surface"], fg=C["muted"],
                 font=("Segoe UI", 10)).pack(side=tk.RIGHT)
        e.select_range(0, tk.END)
        if key == "ax":
            e.focus_set()
        entries[key] = v

    # ── Buttons ──
    bf = tk.Frame(dialog, bg=C["bg"])
    bf.pack(fill=tk.X, padx=16, pady=(8, 16))

    tk.Button(bf, text="取消", bg=C["bg"], fg=C["muted"],
              font=("Segoe UI", 10), bd=1, relief="solid",
              padx=20, pady=6, cursor="hand2",
              activebackground=C["border"],
              command=dialog.destroy).pack(side=tk.LEFT)

    def save():
        try:
            x = float(entries["ax"].get())
            y = float(entries["ay"].get())
            z = float(entries["az"].get())
        except ValueError:
            messagebox.showerror("无效输入", "坐标必须为数字值。", parent=dialog)
            return
        if not send_cmd(f"AT+COORD={uid},{x:.2f},{y:.2f},{z:.2f}"):
            messagebox.showerror("发送失败", "串口未连接或写入错误。", parent=dialog)
            return
        d["ax"] = x; d["ay"] = y; d["az"] = z
        S.anchor_coords[uid] = (x, y, z)
        save_anchor_config()
        dialog.destroy()
        set_status(f"坐标已发送 → {uid}  等待网关确认...", "info")
        # Set 3s timeout for gateway +COORD response
        S._coord_pending = uid
        S._coord_x, S._coord_y, S._coord_z = x, y, z
        def _coord_timeout():
            if getattr(S, "_coord_pending", None) == uid:
                S._coord_pending = None
                messagebox.showwarning("坐标写入超时",
                    f"基站 {get_label(uid)} ({uid})\n\n"
                    "网关未回复确认。\n可能原因:\n"
                    "  • 网关忙碌中\n"
                    "  • 串口通信异常\n"
                    "  • 基站不在线",
                    parent=root)
        S._coord_timeout_id = root.after(3000, _coord_timeout)
        S._dirty = True; refresh_lists()

    tk.Button(bf, text="保存并发送", bg=C["accent"], fg="#ffffff",
              font=("Segoe UI", 10, "bold"), bd=0, relief="flat",
              padx=20, pady=6, cursor="hand2",
              activebackground="#218bff",
              command=save).pack(side=tk.RIGHT)

    dialog.bind("<Return>", lambda e: save())
    dialog.bind("<Escape>", lambda e: dialog.destroy())

    # Size after all widgets packed — use req size with min floor
    dialog.update_idletasks()
    rw_d, rh_d = dialog.winfo_reqwidth(), dialog.winfo_reqheight()
    w = max(DW, rw_d + 20)
    h = max(DH, rh_d + 20)
    rx, ry = root.winfo_rootx(), root.winfo_rooty()
    rw, rh = root.winfo_width(), root.winfo_height()
    dialog.geometry(f"{w}x{h}+{rx+(rw-w)//2}+{ry+(rh-h)//2}")

# ══════════════════════════════════════════════
#  OTA Dialog
# ══════════════════════════════════════════════
def open_ota_dialog(uid):
    d = S.devices.get(uid, {})
    label = get_label(uid)
    dtype = d.get("type", "?")
    DW, DH = 460, 380
    dialog = tk.Toplevel(root)
    dialog.title(f"固件升级 — {label}")
    dialog.configure(bg=C["bg"])
    dialog.transient(root)
    dialog.grab_set()
    dialog.resizable(False, False)

    # ── Header ──
    hdr = tk.Frame(dialog, bg=C["accent"], height=56)
    hdr.pack(fill=tk.X)
    hdr.grid_propagate(False)
    tk.Label(hdr, text=f"  ⬆  固件升级", bg=C["accent"], fg="#ffffff",
             font=("Segoe UI", 14, "bold")).pack(side=tk.LEFT, padx=18, pady=14)
    tk.Label(hdr, text=f"{label}  ", bg=C["accent"], fg="#a5d6ff",
             font=("Segoe UI", 11, "bold")).pack(side=tk.RIGHT, padx=18, pady=14)

    # ── Body ──
    body = tk.Frame(dialog, bg=C["bg"])
    body.pack(fill=tk.BOTH, expand=True, padx=16, pady=(14, 6))

    # Device info
    info_card = tk.Frame(body, bg=C["surface"], highlightthickness=1,
                         highlightbackground=C["border"])
    info_card.pack(fill=tk.X, pady=(0, 10))

    tk.Label(info_card, text=f"  设备:  {label}  ({uid})   |   类型:  {dtype}",
             bg=C["surface"], fg=C["text"],
             font=("Segoe UI", 10)).pack(anchor="w", padx=12, pady=(8, 0))

    fw_label_text = f"  固件版本:  {S.devices.get(uid, {}).get('fw_ver', '未知')}"
    fw_ver_label = tk.Label(info_card, text=fw_label_text,
                            bg=C["surface"], fg=C["accent"],
                            font=("Segoe UI", 9))
    fw_ver_label.pack(anchor="w", padx=12, pady=(0, 8))

    # Firmware file selection
    file_frame = tk.Frame(body, bg=C["surface"], highlightthickness=1,
                          highlightbackground=C["border"])
    file_frame.pack(fill=tk.X, pady=(0, 10))

    fw_path_var = tk.StringVar()
    fw_size_var = tk.StringVar(value="未选择文件")

    tk.Label(file_frame, text="  固件文件", bg=C["surface"], fg=C["muted"],
             font=("Segoe UI", 8, "bold")).pack(anchor="w", padx=12, pady=(10, 2))

    def select_file():
        from tkinter import filedialog
        path = filedialog.askopenfilename(
            title="选择固件文件",
            filetypes=[("二进制文件", "*.bin"), ("所有文件", "*.*")],
            parent=dialog)
        if path:
            fw_path_var.set(path)
            size = os.path.getsize(path)
            fw_size_var.set(f"{os.path.basename(path)}   —   {size:,} bytes")
            S._ota_file_path = path

    btn_row = tk.Frame(file_frame, bg=C["surface"])
    btn_row.pack(fill=tk.X, padx=10, pady=(4, 2))
    tk.Button(btn_row, text="浏览...", bg=C["accent"], fg="#ffffff",
              font=("Segoe UI", 9, "bold"), bd=0, relief="flat",
              padx=12, pady=3, cursor="hand2",
              activebackground="#218bff",
              command=select_file).pack(side=tk.LEFT)

    tk.Label(file_frame, textvariable=fw_size_var, bg=C["surface"], fg=C["muted"],
             font=("Segoe UI", 9)).pack(anchor="w", padx=14, pady=(0, 8))

    # Progress
    prog_card = tk.Frame(body, bg=C["surface"], highlightthickness=1,
                         highlightbackground=C["border"])
    prog_card.pack(fill=tk.X, pady=(0, 10))

    tk.Label(prog_card, text="  进度", bg=C["surface"], fg=C["muted"],
             font=("Segoe UI", 8, "bold")).pack(anchor="w", padx=12, pady=(10, 2))

    progress_var = tk.IntVar(value=0)
    S._ota_progress_var = progress_var
    progress_pct_var = tk.StringVar(value="0%")
    S._ota_progress_pct_var = progress_pct_var
    prog_row = tk.Frame(prog_card, bg=C["surface"])
    prog_row.pack(fill=tk.X, padx=12, pady=(2, 4))
    progress_bar = ttk.Progressbar(prog_row, variable=progress_var,
                                   maximum=100, length=360,
                                   style="green.Horizontal.TProgressbar")
    progress_bar.pack(side=tk.LEFT, fill=tk.X, expand=True)
    tk.Label(prog_row, textvariable=progress_pct_var, bg=C["surface"], fg=C["accent"],
             font=("Segoe UI", 10, "bold"), width=5, anchor="e").pack(side=tk.RIGHT, padx=(8, 0))

    status_var = tk.StringVar(value="就绪")
    S._ota_status_var = status_var
    tk.Label(prog_card, textvariable=status_var, bg=C["surface"], fg=C["muted"],
             font=("Segoe UI", 9)).pack(anchor="w", padx=12, pady=(0, 10))

    # ── Buttons ──
    bf = tk.Frame(dialog, bg=C["bg"])
    bf.pack(fill=tk.X, padx=16, pady=(4, 14))

    def on_close():
        if S.ota_active:
            S.ota_active = False
            _clear_ota_timeout()
            set_status("OTA 已取消", "warning")
        dialog.destroy()

    tk.Button(bf, text="关闭", bg=C["bg"], fg=C["muted"],
              font=("Segoe UI", 10), bd=1, relief="solid",
              padx=18, pady=5, cursor="hand2",
              activebackground=C["border"],
              command=on_close).pack(side=tk.LEFT)

    def start_ota():
        if S.ota_active:
            messagebox.showwarning("OTA 忙碌", "另一个 OTA 正在进行中。", parent=dialog)
            return
        path = getattr(S, "_ota_file_path", "")
        if not path or not os.path.exists(path):
            messagebox.showerror("未选择文件", "请选择固件二进制文件。", parent=dialog)
            return
        if not S.ser or not S.ser.is_open:
            messagebox.showerror("未连接", "串口未连接。", parent=dialog)
            return

        try:
            with open(path, "rb") as f:
                S.ota_data = f.read()
        except Exception as e:
            messagebox.showerror("读取错误", f"无法读取文件:\n{e}", parent=dialog)
            return

        S.ota_active = True
        S.ota_uid = uid
        S.ota_offset = 0
        S.ota_total = len(S.ota_data)
        S.ota_retry_count = 0
        progress_var.set(0)
        status_var.set(f"开始中...  {S.ota_total:,} 字节")
        S._ota_btn = start_btn
        start_btn.config(text="发送中...", state="disabled")
        set_status(f"OTA 开始 → {label}", "info")
        _reset_ota_timeout(10)  # 10s for gateway to respond with +OTA_READY
        send_cmd(f"AT+OTA_START={uid},{S.ota_total}")

    start_btn = tk.Button(bf, text="开始升级", bg=C["accent"], fg="#ffffff",
              font=("Segoe UI", 10, "bold"), bd=0, relief="flat",
              padx=18, pady=5, cursor="hand2",
              activebackground="#218bff",
              command=start_ota)
    start_btn.pack(side=tk.RIGHT)

    dialog.bind("<Escape>", lambda e: on_close())

    # Size after all packed
    dialog.update_idletasks()
    rw_d, rh_d = dialog.winfo_reqwidth(), dialog.winfo_reqheight()
    w = max(DW, rw_d + 20)
    h = max(DH, rh_d + 20)
    rx, ry = root.winfo_rootx(), root.winfo_rooty()
    rw, rh = root.winfo_width(), root.winfo_height()
    dialog.geometry(f"{w}x{h}+{rx+(rw-w)//2}+{ry+(rh-h)//2}")

# ══════════════════════════════════════════════
#  Batch OTA Dialog
# ══════════════════════════════════════════════
def open_batch_ota_dialog(dtype):
    """Batch firmware upgrade for all anchors or all tags."""
    DW, DH = 520, 520
    dialog = tk.Toplevel(root)
    dialog.title(f"批量升级 — {'基站' if dtype == 'ANCHOR' else '标签'}")
    dialog.configure(bg=C["bg"])
    dialog.transient(root)
    dialog.grab_set()
    dialog.resizable(False, False)
    S.batch_ota_dialog = dialog

    # ── Header ──
    hdr = tk.Frame(dialog, bg=C["accent"], height=56)
    hdr.pack(fill=tk.X)
    hdr.grid_propagate(False)
    dtype_cn = "基站" if dtype == "ANCHOR" else "标签"
    tk.Label(hdr, text=f"  ⬆  批量升级{dtype_cn}", bg=C["accent"], fg="#ffffff",
             font=("Segoe UI", 14, "bold")).pack(side=tk.LEFT, padx=18, pady=14)

    # ── Body ──
    body = tk.Frame(dialog, bg=C["bg"])
    body.pack(fill=tk.BOTH, expand=True, padx=16, pady=(14, 6))

    # Device list
    tk.Label(body, text=f"选择要升级的{dtype_cn}  (仅在线设备)",
             bg=C["bg"], fg=C["text"], font=("Segoe UI", 11, "bold")).pack(anchor="w", pady=(0, 6))

    list_frame = tk.Frame(body, bg=C["surface"], highlightthickness=1,
                           highlightbackground=C["border"])
    list_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 8))

    # Scrollable device list with checkboxes
    list_canvas = tk.Canvas(list_frame, bg=C["surface"], highlightthickness=0, height=150)
    list_scroll = tk.Scrollbar(list_frame, orient="vertical", command=list_canvas.yview)
    list_inner = tk.Frame(list_canvas, bg=C["surface"])
    list_inner.bind("<Configure>",
        lambda e: list_canvas.configure(scrollregion=list_canvas.bbox("all")))
    list_win = list_canvas.create_window((0, 0), window=list_inner, anchor="nw")
    list_canvas.configure(yscrollcommand=list_scroll.set)

    def on_list_resize(e):
        list_canvas.itemconfig(list_win, width=e.width)
    list_canvas.bind("<Configure>", on_list_resize)

    list_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
    list_scroll.pack(side=tk.RIGHT, fill=tk.Y)

    # Mouse wheel
    def on_list_wheel(e):
        list_canvas.yview_scroll(int(-1 * (e.delta / 120)), "units")
    list_canvas.bind("<Enter>", lambda e: list_canvas.bind_all("<MouseWheel>", on_list_wheel))
    list_canvas.bind("<Leave>", lambda e: list_canvas.unbind_all("<MouseWheel>"))

    # Gather online devices of the given type
    device_vars = {}
    devices = [(uid, d) for uid, d in S.devices.items()
               if d.get("type") == dtype and d.get("online")]
    devices.sort(key=lambda x: x[0])

    if not devices:
        tk.Label(list_inner, text="  没有在线设备", bg=C["surface"], fg=C["muted"],
                 font=("Segoe UI", 10)).pack(anchor="w", padx=12, pady=12)

    # Store row frames and indicator labels for visual toggle
    device_rows = {}

    def toggle_device(uid, var, row, indicator):
        var.set(not var.get())
        _refresh_row_style(row, indicator, var.get())

    def _refresh_row_style(row, indicator, selected):
        if selected:
            row.configure(bg="#ebf3ff")
            indicator.configure(text="  ☑", fg=C["accent"], bg="#ebf3ff")
        else:
            row.configure(bg=C["surface"])
            indicator.configure(text="  ☐", fg=C["muted"], bg=C["surface"])

    for uid, d in devices:
        var = tk.BooleanVar(value=True)
        device_vars[uid] = var
        row = tk.Frame(list_inner, bg="#ebf3ff", cursor="hand2")
        row.pack(fill=tk.X, padx=6, pady=2)

        # Large checkbox indicator using unicode
        indicator = tk.Label(row, text="  ☑", bg="#ebf3ff", fg=C["accent"],
                             font=("Segoe UI", 16))
        indicator.pack(side=tk.LEFT, padx=(4, 0), pady=3)

        label = get_label(uid)
        fw = d.get("fw_ver", "?")
        info = f"{label}     {fw}     {'锚点' if dtype == 'ANCHOR' else '标签'}"
        if dtype == "TAG":
            x, y = d.get("x"), d.get("y")
            if x is not None and y is not None:
                info += f"     ({x:.1f}, {y:.1f})"
        info_label = tk.Label(row, text=info, bg="#ebf3ff", fg=C["text"],
                              font=("Consolas", 10))
        info_label.pack(side=tk.LEFT, padx=8, pady=5)

        device_rows[uid] = (row, indicator, info_label)

        # Click entire row to toggle
        def _make_handler(_uid, _var, _row, _ind):
            return lambda e: toggle_device(_uid, _var, _row, _ind)
        for w in (row, indicator, info_label):
            w.bind("<Button-1>", _make_handler(uid, var, row, indicator))

        # Hover effect (only when not selected)
        def _make_enter(_row, _ind, _info, _var):
            def _on_enter(e):
                if not _var.get():
                    _row.configure(bg="#f0f4f8")
                    _ind.configure(bg="#f0f4f8")
                    _info.configure(bg="#f0f4f8")
            return _on_enter

        def _make_leave(_row, _ind, _info, _var):
            def _on_leave(e):
                if not _var.get():
                    _row.configure(bg=C["surface"])
                    _ind.configure(bg=C["surface"])
                    _info.configure(bg=C["surface"])
            return _on_leave

        for w in (row, indicator, info_label):
            w.bind("<Enter>", _make_enter(row, indicator, info_label, var))
            w.bind("<Leave>", _make_leave(row, indicator, info_label, var))

    # Select All / Deselect All
    sel_bar = tk.Frame(body, bg=C["bg"])
    sel_bar.pack(fill=tk.X, pady=(0, 8))

    def select_all():
        for uid, (row, indicator, info_label) in device_rows.items():
            device_vars[uid].set(True)
            _refresh_row_style(row, indicator, True)

    def deselect_all():
        for uid, (row, indicator, info_label) in device_rows.items():
            device_vars[uid].set(False)
            _refresh_row_style(row, indicator, False)

    tk.Button(sel_bar, text="全选", bg=C["surface"], fg=C["text"],
              font=("Segoe UI", 9), bd=1, relief="solid", padx=10, pady=2,
              cursor="hand2", command=select_all).pack(side=tk.LEFT, padx=2)
    tk.Button(sel_bar, text="取消全选", bg=C["surface"], fg=C["text"],
              font=("Segoe UI", 9), bd=1, relief="solid", padx=10, pady=2,
              cursor="hand2", command=deselect_all).pack(side=tk.LEFT, padx=2)

    sel_count_label = tk.Label(sel_bar, text="", bg=C["bg"], fg=C["muted"],
                                font=("Segoe UI", 9))
    sel_count_label.pack(side=tk.RIGHT, padx=4)

    # Firmware file selection
    file_frame = tk.Frame(body, bg=C["surface"], highlightthickness=1,
                           highlightbackground=C["border"])
    file_frame.pack(fill=tk.X, pady=(0, 8))

    fw_size_var = tk.StringVar(value="未选择固件文件")
    S._batch_fw_size_var = fw_size_var

    tk.Label(file_frame, text="  固件文件", bg=C["surface"], fg=C["muted"],
             font=("Segoe UI", 8, "bold")).pack(anchor="w", padx=12, pady=(8, 2))

    file_row = tk.Frame(file_frame, bg=C["surface"])
    file_row.pack(fill=tk.X, padx=10, pady=(0, 6))

    tk.Label(file_row, textvariable=fw_size_var, bg=C["surface"], fg=C["muted"],
             font=("Segoe UI", 9)).pack(side=tk.LEFT, padx=4)

    def select_file():
        from tkinter import filedialog
        path = filedialog.askopenfilename(
            title="选择批量升级固件文件",
            filetypes=[("二进制文件", "*.bin"), ("所有文件", "*.*")],
            parent=dialog)
        if path:
            S.batch_ota_file_path = path
            size = os.path.getsize(path)
            fw_size_var.set(f"  {os.path.basename(path)}   —   {size:,} bytes")

    tk.Button(file_row, text="浏览...", bg=C["accent"], fg="#ffffff",
              font=("Segoe UI", 9, "bold"), bd=0, relief="flat",
              padx=12, pady=3, cursor="hand2",
              activebackground="#218bff",
              command=select_file).pack(side=tk.RIGHT, padx=4)

    # Progress
    prog_frame = tk.Frame(body, bg=C["surface"], highlightthickness=1,
                           highlightbackground=C["border"])
    prog_frame.pack(fill=tk.X, pady=(0, 8))

    batch_status_var = tk.StringVar(value="就绪")
    S._batch_status_var = batch_status_var
    tk.Label(prog_frame, text="  进度", bg=C["surface"], fg=C["muted"],
             font=("Segoe UI", 8, "bold")).pack(anchor="w", padx=12, pady=(8, 2))
    tk.Label(prog_frame, textvariable=batch_status_var, bg=C["surface"], fg=C["text"],
             font=("Segoe UI", 10)).pack(anchor="w", padx=12, pady=(0, 4))

    batch_progress_var = tk.IntVar(value=0)
    S._batch_progress_var = batch_progress_var
    batch_pct_var = tk.StringVar(value="0%")
    S._batch_progress_pct_var = batch_pct_var
    batch_row = tk.Frame(prog_frame, bg=C["surface"])
    batch_row.pack(fill=tk.X, padx=12, pady=(2, 8))
    batch_bar = ttk.Progressbar(batch_row, variable=batch_progress_var,
                                 maximum=100, length=420,
                                 style="green.Horizontal.TProgressbar")
    batch_bar.pack(side=tk.LEFT, fill=tk.X, expand=True)
    tk.Label(batch_row, textvariable=batch_pct_var, bg=C["surface"], fg=C["accent"],
             font=("Segoe UI", 10, "bold"), width=5, anchor="e").pack(side=tk.RIGHT, padx=(8, 0))

    # Log
    log_frame = tk.Frame(body, bg=C["surface"], highlightthickness=1,
                          highlightbackground=C["border"])
    log_frame.pack(fill=tk.BOTH, expand=True)

    tk.Label(log_frame, text="  升级日志", bg=C["surface"], fg=C["muted"],
             font=("Segoe UI", 8, "bold")).pack(anchor="w", padx=12, pady=(8, 2))

    log_text = tk.Text(log_frame, bg=C["bg"], fg=C["text"], font=("Consolas", 8),
                       height=5, relief="flat", bd=0, state="disabled",
                       wrap="word", padx=8, pady=4)
    log_text.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))
    S._batch_log_text = log_text

    # ── Buttons ──
    bf = tk.Frame(dialog, bg=C["bg"])
    bf.pack(fill=tk.X, padx=16, pady=(4, 14))

    def on_close():
        if S.batch_ota_active:
            S.batch_ota_active = False
            S.batch_ota_queue.clear()
            _clear_ota_timeout()
            set_status("批量升级已取消", "warning")
        S.batch_ota_dialog = None
        dialog.destroy()

    tk.Button(bf, text="关闭", bg=C["bg"], fg=C["muted"],
              font=("Segoe UI", 10), bd=1, relief="solid",
              padx=18, pady=5, cursor="hand2",
              activebackground=C["border"],
              command=on_close).pack(side=tk.LEFT)

    def start_batch():
        if S.ota_active or S.batch_ota_active:
            messagebox.showwarning("忙碌", "另一个 OTA 正在进行中。", parent=dialog)
            return

        # Collect selected devices
        selected = [uid for uid, var in device_vars.items() if var.get()]
        if not selected:
            messagebox.showerror("未选择设备", "请至少选择一个设备。", parent=dialog)
            return

        path = S.batch_ota_file_path
        if not path or not os.path.exists(path):
            messagebox.showerror("未选择文件", "请选择固件二进制文件。", parent=dialog)
            return

        if not S.ser or not S.ser.is_open:
            messagebox.showerror("未连接", "串口未连接。", parent=dialog)
            return

        try:
            with open(path, "rb") as f:
                S.ota_data = f.read()
        except Exception as e:
            messagebox.showerror("读取错误", f"无法读取文件:\n{e}", parent=dialog)
            return

        S.ota_total = len(S.ota_data)
        batch_progress_var.set(0)

        # ── Sequential unicast OTA (reliable SW-ARQ, ~15s/device) ──
        S.batch_ota_active = True
        S.batch_ota_queue = list(selected)
        S.batch_ota_results.clear()
        S._bcast_mode = False
        S.bcast_dev_count = 0

        batch_status_var.set(f"顺序升级中... {len(selected)} 个设备")
        S._batch_start_btn = start_btn
        start_btn.config(text="升级中...", state="disabled")

        # Clear log
        log_text.config(state="normal")
        log_text.delete("1.0", tk.END)
        log_text.insert(tk.END, f"  批量OTA升级 — {len(selected)} 个设备 (顺序单播)\n", "summary")
        log_text.tag_config("summary", foreground=C["accent"])
        for uid in selected:
            label = get_label(uid)
            log_text.insert(tk.END, f"    {label}  {uid}\n", "")
        log_text.see(tk.END)
        log_text.config(state="disabled")

        set_status(f"批量OTA → {len(selected)} 设备 准备中...", "info")
        _batch_ota_next()

    start_btn = tk.Button(bf, text="开始批量升级", bg=C["green"], fg="#ffffff",
              font=("Segoe UI", 10, "bold"), bd=0, relief="flat",
              padx=18, pady=5, cursor="hand2",
              activebackground="#1a7f37",
              command=start_batch)
    start_btn.pack(side=tk.RIGHT)
    S._batch_start_btn = start_btn

    # Update selection count on checkbox change
    def update_sel_count(*_):
        n = sum(1 for var in device_vars.values() if var.get())
        sel_count_label.config(text=f"已选 {n}/{len(device_vars)}  ")
    for var in device_vars.values():
        var.trace_add("write", update_sel_count)
    update_sel_count()

    dialog.bind("<Escape>", lambda e: on_close())
    dialog.protocol("WM_DELETE_WINDOW", on_close)

    # Size
    dialog.update_idletasks()
    rw_d, rh_d = dialog.winfo_reqwidth(), dialog.winfo_reqheight()
    w = max(DW, rw_d + 20)
    h = max(DH, rh_d + 20)
    rx, ry = root.winfo_rootx(), root.winfo_rooty()
    rw, rh = root.winfo_width(), root.winfo_height()
    dialog.geometry(f"{w}x{h}+{rx+(rw-w)//2}+{ry+(rh-h)//2}")

# ══════════════════════════════════════════════
#  Map
# ══════════════════════════════════════════════
def build_map(parent):
    mf = tk.Frame(parent, bg=C["bg"])
    mf.grid(row=1, column=1, sticky="nsew", padx=2, pady=4)
    mf.grid_rowconfigure(0, weight=1)
    mf.grid_columnconfigure(0, weight=1)

    canvas = tk.Canvas(mf, bg=C["bg"], highlightthickness=0, bd=0)
    canvas.grid(row=0, column=0, sticky="nsew")
    S._canvas = canvas

    # Tooltip overlay
    tip = tk.Label(mf, text="", bg=C["card"], fg=C["text"],
                   font=("Consolas", 9), padx=6, pady=2,
                   borderwidth=1, relief="solid",
                   highlightbackground=C["border"], highlightthickness=1)
    tip.place(x=0, y=0); tip.place_forget()
    S._tip = tip

    # Info card
    info = tk.Label(mf, text="", bg=C["surface"], fg=C["text"],
                    font=("Consolas", 9), padx=10, pady=6,
                    borderwidth=1, relief="solid",
                    highlightbackground=C["border"], highlightthickness=1,
                    justify=tk.LEFT, anchor="w")
    info.place(x=10, rely=1.0, y=-10, anchor="sw")
    info.place_forget()
    S._info = info

    # Legend overlay
    leg = tk.Label(mf, text="", bg=C["card"], fg=C["text"],
                   font=("Segoe UI", 8), padx=8, pady=5,
                   borderwidth=1, relief="solid",
                   highlightbackground=C["border"], highlightthickness=1,
                   justify=tk.LEFT, anchor="w")
    leg.place(x=10, y=10)
    leg.place_forget()
    S._legend = leg

    # Coordinate display overlay
    coord_disp = tk.Label(mf, text="", bg=C["card"], fg=C["muted"],
                          font=("Consolas", 8), padx=6, pady=2,
                          borderwidth=1, relief="solid",
                          highlightbackground=C["border"], highlightthickness=1)
    coord_disp.place(relx=1.0, y=10, x=-10, anchor="ne")
    coord_disp.place_forget()
    S._coord_disp = coord_disp

    # Mouse bindings
    canvas.bind("<ButtonPress-1>", on_mouse_down)
    canvas.bind("<B1-Motion>", on_mouse_drag)
    canvas.bind("<ButtonRelease-1>", on_mouse_up)
    canvas.bind("<Motion>", on_mouse_move)
    canvas.bind("<MouseWheel>", on_mouse_wheel)
    canvas.bind("<Double-Button-1>", on_dbl_click)

    # Bind mouse enter/leave for hover cursor
    canvas.bind("<Enter>", lambda e: canvas.config(cursor="crosshair" if not S.hover_uid else "hand2"))

# ══════════════════════════════════════════════
#  Map transforms
# ══════════════════════════════════════════════
def w2s(wx, wy):
    c = S._canvas
    return (c.winfo_width()/2  + (wx - S.ox) * S.scale * c.winfo_width()/S.grid_w,
            c.winfo_height()/2 - (wy - S.oy) * S.scale * c.winfo_height()/S.grid_h)

def s2w(sx, sy):
    c = S._canvas
    wx = (sx - c.winfo_width()/2)  / (S.scale * c.winfo_width()/S.grid_w) + S.ox
    wy = -(sy - c.winfo_height()/2) / (S.scale * c.winfo_height()/S.grid_h) + S.oy
    return (wx, wy)

# ══════════════════════════════════════════════
#  Mouse handlers
# ══════════════════════════════════════════════
def on_mouse_down(e):
    S.drag = True; S.dx = e.x; S.dy = e.y; S.moved = 0.0

def on_mouse_drag(e):
    if not S.drag:
        return
    dx, dy = e.x - S.dx, e.y - S.dy
    c = S._canvas
    S.ox -= dx / (S.scale * c.winfo_width() / S.grid_w)
    S.oy += dy / (S.scale * c.winfo_height() / S.grid_h)
    S.dx, S.dy = e.x, e.y
    S.moved += abs(dx) + abs(dy)

def on_mouse_up(e):
    if S.drag and S.moved < 5:
        for uid, d in S.devices.items():
            x, y = get_anchor_xy(d)
            if x is None: continue
            sx, sy = w2s(x, y)
            if math.hypot(e.x - sx, e.y - sy) < 16:
                S.selected_uid = uid; show_info(uid); refresh_lists()
                S.drag = False; return
        hide_info(); S.selected_uid = ""; refresh_lists()
    S.drag = False

def on_mouse_move(e):
    S.mx, S.my = e.x, e.y
    if S.drag:
        S._canvas.config(cursor="fleur")
        return

    found = None
    for uid, d in S.devices.items():
        x, y = get_anchor_xy(d)
        if x is None: continue
        if math.hypot(e.x - w2s(x, y)[0], e.y - w2s(x, y)[1]) < 16:
            found = uid; break

    S.hover_uid = found or ""
    if found:
        S._canvas.config(cursor="hand2")
        show_tip(e.x, e.y, found, S.devices[found])
    else:
        S._canvas.config(cursor="crosshair")
        hide_tip()

def on_mouse_wheel(e):
    z = 1.12 if e.delta > 0 else 1/1.12
    ns = S.scale * z
    if ns < 0.005 or ns > 80:
        return
    c = S._canvas
    cx, cy = e.x - c.winfo_width()/2, c.winfo_height()/2 - e.y
    S.ox += cx/(S.scale*c.winfo_width()/S.grid_w) - cx/(ns*c.winfo_width()/S.grid_w)
    S.oy += cy/(S.scale*c.winfo_height()/S.grid_h) - cy/(ns*c.winfo_height()/S.grid_h)
    S.scale = ns

def on_dbl_click(e):
    anchors = [(d.get("ax",0), d.get("ay",0)) for d in S.devices.values()
               if d.get("ax") is not None and d.get("type") == "ANCHOR"]
    if anchors:
        xs, ys = [a[0] for a in anchors], [a[1] for a in anchors]
        S.ox = (min(xs)+max(xs))/2 if len(xs) > 1 else xs[0]
        S.oy = (min(ys)+max(ys))/2 if len(ys) > 1 else ys[0]
    else:
        S.ox = S.oy = 0; S.scale = 1.0
    hide_info()

def show_tip(sx, sy, uid, d):
    x, y = get_anchor_xy(d); x = x if x is not None else 0; y = y if y is not None else 0
    label = get_label(uid)
    S._tip.config(text=f"  {label}  [{d.get('type','?')}]  {x:.2f}, {y:.2f}  ")
    S._tip.place(x=sx+16, y=sy-10); S._tip.lift()

def hide_tip():
    S._tip.place_forget()

def show_info(uid):
    d = S.devices.get(uid)
    if not d:
        return
    dtype = d.get("type", "?")
    online = d.get("online", False)

    label = get_label(uid)
    lines = []
    lines.append(f"  {label}  " + (f"({uid})" if label != uid else ""))
    lines.append(f"类型:    {dtype}")
    lines.append(f"状态:  {'● 在线' if online else '○ 离线'}")

    if d.get("ax") is not None:
        lines.append(f"坐标:     {d['ax']:.3f}, {d['ay']:.3f}, {d['az']:.3f}")
    if d.get("x") is not None:
        lines.append(f"实时:    {d['x']:.3f}, {d['y']:.3f}, {d.get('z',0):.3f}")
    if d.get("q") is not None:
        lines.append(f"质量: {d['q']}")
    if d.get("fw_ver"):
        lines.append(f"固件: {d['fw_ver']}")

    S._info.config(text="\n".join(lines))
    S._info.place(x=10, rely=1.0, y=-10, anchor="sw"); S._info.lift()

def hide_info():
    S._info.place_forget(); S.selected_uid = ""

# ══════════════════════════════════════════════
#  Render
# ══════════════════════════════════════════════
def render_loop():
    c = S._canvas
    c.delete("all")
    w, h = c.winfo_width(), c.winfo_height()
    if w < 10 or h < 10:
        root.after(33, render_loop)
        return

    # Background
    c.create_rectangle(0, 0, w, h, fill=C["bg"], outline="")

    # Grid
    if S.show_grid:
        mpp = S.grid_w / (S.scale * w)
        # Major grid spacing
        gs_major = 1.0
        while gs_major/mpp < 80: gs_major *= 2
        while gs_major/mpp > 300: gs_major /= 2
        gs_minor = gs_major / 5 if gs_major >= 5 else gs_major / 4

        # Minor grid (dashed/fainter)
        x0 = math.floor((S.ox - S.grid_w/(2*S.scale))/gs_minor)*gs_minor
        x1 = S.ox + S.grid_w/(2*S.scale)
        x = x0
        while x <= x1:
            sx, _ = w2s(x, 0)
            c.create_line(sx, 0, sx, h, fill="#e8eaed", width=1, dash=(2, 6))
            x += gs_minor

        y0 = math.floor((S.oy - S.grid_h/(2*S.scale))/gs_minor)*gs_minor
        y1 = S.oy + S.grid_h/(2*S.scale)
        y = y0
        while y <= y1:
            _, sy = w2s(0, y)
            c.create_line(0, sy, w, sy, fill="#e8eaed", width=1, dash=(2, 6))
            y += gs_minor

        # Major grid (solid, more visible)
        x = math.floor((S.ox - S.grid_w/(2*S.scale))/gs_major)*gs_major
        while x <= x1:
            sx, _ = w2s(x, 0)
            c.create_line(sx, 0, sx, h, fill="#d0d7de", width=1)
            # Label on X axis (bottom)
            _, oy_s = w2s(0, S.oy - S.grid_h/(2*S.scale))
            label_y = min(oy_s + 14, h - 2)
            c.create_text(sx, label_y, text=f"{x:.1f}" if gs_major < 1 else f"{x:.0f}",
                          fill=C["muted"], font=("Consolas", 7), anchor="n")
            x += gs_major

        y = math.floor((S.oy - S.grid_h/(2*S.scale))/gs_major)*gs_major
        while y <= y1:
            _, sy = w2s(0, y)
            c.create_line(0, sy, w, sy, fill="#d0d7de", width=1)
            # Label on Y axis (left)
            ox_s, _ = w2s(S.ox - S.grid_w/(2*S.scale), 0)
            label_x = max(ox_s + 14, 4)
            c.create_text(label_x, sy, text=f"{y:.1f}" if gs_major < 1 else f"{y:.0f}",
                          fill=C["muted"], font=("Consolas", 7), anchor="w")
            y += gs_major

    # Origin axes (stronger, with arrows and labels)
    ox_px, oy_px = w2s(0, 0)
    ax_color = "#7b828b"
    ax_width = 1.8

    # X axis
    c.create_line(0, oy_px, w, oy_px, fill=ax_color, width=ax_width)
    # X arrow (positive end — right side)
    ax_r = 6
    c.create_polygon(w-ax_r, oy_px-ax_r, w, oy_px, w-ax_r, oy_px+ax_r,
                     fill=ax_color, outline="")
    # X label
    c.create_text(w-16, oy_px-12, text="X", fill=ax_color,
                  font=("Consolas", 11, "bold"), anchor="e")

    # Y axis
    c.create_line(ox_px, 0, ox_px, h, fill=ax_color, width=ax_width)
    # Y arrow (positive end — top)
    c.create_polygon(ox_px-ax_r, ax_r, ox_px, 0, ox_px+ax_r, ax_r,
                     fill=ax_color, outline="")
    # Y label
    c.create_text(ox_px+12, 14, text="Y", fill=ax_color,
                  font=("Consolas", 11, "bold"), anchor="w")

    # Origin label
    c.create_text(ox_px+8, oy_px+8, text="O", fill=C["muted"], font=("Consolas", 8, "bold"))

    # Draw devices
    for uid, d in S.devices.items():
        if d.get("type") == "GATEWAY":
            continue  # Gateway has no position on map
        if d.get("type") == "TAG":
            if S.show_trails:
                draw_trail(c, d)
            draw_tag(c, uid, d)
        else:
            draw_anchor(c, uid, d)

    # Inter-anchor links
    if S.show_links:
        draw_links(c)

    # Scale bar
    draw_scale(c, w, h)

    # Legend
    draw_legend()

    # Coord display
    wx, wy = s2w(S.mx, S.my)
    S._coord_disp.config(text=f"  ({wx:.2f}, {wy:.2f})  ×{S.scale:.2f}  ")
    S._coord_disp.place(relx=1.0, y=10, x=-10, anchor="ne")
    S._coord_disp.lift()

    root.after(33, render_loop)

def get_anchor_xy(d):
    """Get anchor position, handling 0 values correctly."""
    x = d.get("ax") if d.get("ax") is not None else d.get("x")
    y = d.get("ay") if d.get("ay") is not None else d.get("y")
    return x, y

def draw_anchor(c, uid, d):
    x, y = get_anchor_xy(d)
    if x is None or y is None: return
    sx, sy = w2s(x, y)
    if sx < -100 or sx > c.winfo_width()+100 or sy < -100 or sy > c.winfo_height()+100: return

    selected = (uid == S.selected_uid)
    hovered = (uid == S.hover_uid)
    online = d.get("online", False)
    highlight = selected or hovered
    icon_s = max(0.4, min(5.0, S.scale))
    r = (14 if highlight else 10) * icon_s
    label = get_label(uid)
    rank = d.get("rank", 0)
    map_label = f"A{rank:02d}"  # e.g. "A08" — rank shown on map icon

    # Colors
    if online:
        body_fill = C["anchor"]     # blue
        body_stroke = "#0550ae"     # darker blue border
        glow_color = "#54aeff"      # light blue glow
    else:
        body_fill = "#c8cdd4"
        body_stroke = "#8b9096"
        glow_color = "#d0d7de"

    # --- Glow ring ---
    if selected:
        gr = r + 8 * icon_s
        c.create_oval(sx-gr, sy-gr, sx+gr, sy+gr,
                      fill="", outline=glow_color, width=max(1, 4*icon_s), dash=(4, 3))
    elif hovered:
        gr = r + 5 * icon_s
        c.create_oval(sx-gr, sy-gr, sx+gr, sy+gr,
                      fill="", outline=glow_color, width=max(1, 2*icon_s))

    # --- Shadow ---
    so = max(1, 2 * icon_s)
    c.create_oval(sx-r+so, sy-r+so, sx+r+so, sy+r+so,
                  fill="#e1e4e8", outline="")

    # --- Outer ring ---
    c.create_oval(sx-r, sy-r, sx+r, sy+r,
                  fill=body_stroke, outline="")

    # --- Inner filled circle ---
    ir = max(1, r - 3 * icon_s)
    c.create_oval(sx-ir, sy-ir, sx+ir, sy+ir,
                  fill=body_fill, outline="")

    # --- Center dot (highlight) ---
    cd = max(2, 3 * icon_s)
    c.create_oval(sx-cd, sy-cd, sx+cd, sy+cd,
                  fill="#ffffff", outline="")

    # --- Ring accent ---
    if online:
        ar = max(r // 2, 4 * icon_s) + icon_s
        c.create_oval(sx-ar, sy-ar, sx+ar, sy+ar,
                      fill="", outline=C["green"], width=max(1, 2*icon_s))

    # --- Label badge ---
    if S.show_labels:
        fs = max(7, int(8 * min(icon_s, 1.8)))
        t = c.create_text(sx, sy+r+14*icon_s, text=map_label, fill=C["text"],
                          font=("Segoe UI", fs, "bold"), anchor="n")
        bbox = c.bbox(t)
        if bbox:
            pad = max(2, 4 * icon_s)
            c.create_rectangle(bbox[0]-pad, bbox[1]-2, bbox[2]+pad, bbox[3]+2,
                             fill=C["surface"], outline=C["border"], width=1)
            c.lift(t)

def draw_tag(c, uid, d):
    x = d.get("x"); y = d.get("y")
    if x is None or y is None: return
    sx, sy = w2s(x, y)
    if sx < -80 or sx > c.winfo_width()+80 or sy < -80 or sy > c.winfo_height()+80: return

    selected = (uid == S.selected_uid)
    hovered = (uid == S.hover_uid)
    online = d.get("online", False)
    highlight = selected or hovered
    r = 32 if highlight else 22      # overall scale (fixed size)

    # Pin geometry: round head above, point at (sx, sy) = actual position
    hx, hy = sx, sy - r * 0.52        # head center
    hr = r * 0.54                      # head radius
    tip_x, tip_y = sx, sy + r * 0.32   # pin point tip
    # triangle side attachment points on the lower part of the head
    lx, ly = sx - hr * 0.80, hy + hr * 0.48
    rx, ry = sx + hr * 0.80, hy + hr * 0.48

    # Colors
    if online:
        body_fill = C["tag"]          # red
        body_stroke = "#8b1620"       # deep dark red
        body_mid = "#d8404f"          # mid red for 3D layering
        glow_color = "#ff7b72"        # light red glow
    else:
        body_fill = "#c8cdd4"
        body_stroke = "#7b8188"
        body_mid = "#b0b5bd"
        glow_color = "#d0d7de"

    # --- Glow ring around head ---
    if selected:
        gr = hr + 10
        c.create_oval(hx-gr, hy-gr, hx+gr, hy+gr,
                      fill="", outline=glow_color, width=5, dash=(5, 4))
        # Second outer glow pulse
        c.create_oval(hx-gr-5, hy-gr-5, hx+gr+5, hy+gr+5,
                      fill="", outline=glow_color, width=2, dash=(2, 4))
    elif hovered:
        gr = hr + 7
        c.create_oval(hx-gr, hy-gr, hx+gr, hy+gr,
                      fill="", outline=glow_color, width=3)

    # --- Shadow (offset dark ghost) ---
    so = 3
    c.create_oval(hx-hr+so, hy-hr+so, hx+hr+so, hy+hr+so,
                  fill="#e1e4e8", outline="")
    c.create_polygon(lx+so, ly+so, rx+so, ry+so, tip_x+so, tip_y+so,
                     fill="#e1e4e8", outline="", joinstyle="round")

    # --- Pin body: triangle (point) + circle (head) merged ---
    c.create_polygon(lx, ly, rx, ry, tip_x, tip_y,
                     fill=body_fill, outline="", joinstyle="round")
    c.create_oval(hx-hr, hy-hr, hx+hr, hy+hr,
                  fill=body_fill, outline="")

    # --- Head border ring ---
    c.create_oval(hx-hr, hy-hr, hx+hr, hy+hr,
                  fill="", outline=body_stroke, width=3)

    # --- 3D inner mid ring ---
    mir = hr - 5
    c.create_oval(hx-mir, hy-mir, hx+mir, hy+mir,
                  fill="", outline=body_mid, width=2)

    # --- Bright inner highlight (reflection dot) ---
    cd = 5
    c.create_oval(hx-cd-2, hy-cd-3, hx+cd-2, hy+cd-3,
                  fill="#ffffff", outline="")
    # Smaller secondary highlight
    sd = 2
    c.create_oval(hx+cd+2, hy-cd-1, hx+cd+2+sd, hy-cd-1+sd,
                  fill="#ffffff", outline="")

    # --- Quality indicator (colored arc inside head) ---
    q = d.get("q")
    if q is not None and q < 65535 and online:
        qc = C["green"] if q >= 5 else (C["yellow"] if q >= 3 else C["red"])
        qr = max(hr // 2, 4) + 1
        c.create_oval(hx-qr, hy-qr, hx+qr, hy+qr,
                      fill="", outline=qc, width=3)
    elif q == 65535 and online:
        # No-fix indicator — gray dashed ring
        qr = max(hr // 2, 4) + 1
        c.create_oval(hx-qr, hy-qr, hx+qr, hy+qr,
                      fill="", outline="#9ba0a8", width=2, dash=(2, 3))

    # --- Label badge above pin ---
    if S.show_labels:
        t = c.create_text(sx, hy-hr-14, text=get_label(uid), fill=C["text"],
                          font=("Segoe UI", 11, "bold"), anchor="s")
        bbox = c.bbox(t)
        if bbox:
            pad = 5
            c.create_rectangle(bbox[0]-pad, bbox[1]-3, bbox[2]+pad, bbox[3]+3,
                             fill=C["surface"], outline=C["border"], width=1)
            c.lift(t)

def draw_trail(c, d):
    trail = d.get("trail", [])
    if len(trail) < 2: return
    n = len(trail)
    for i in range(1, n):
        a = int(30 + 120 * i / n)  # alpha 30-150
        p0, p1 = w2s(trail[i-1][0], trail[i-1][1]), w2s(trail[i][0], trail[i][1])
        color = f"#{a:02x}2040"  # approximate reddish
        c.create_line(p0[0], p0[1], p1[0], p1[1], fill="#f4cdd1", width=2, capstyle="round")

def draw_scale(c, w, h):
    mpp = S.grid_w / (S.scale * w)
    target_px = 120
    scale_m = target_px * mpp
    nice = [0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50, 100, 200, 500]
    best = min(nice, key=lambda n: abs(scale_m - n))
    scale_px = int(best / mpp)
    if scale_px < 35:
        best *= 2; scale_px = int(best / mpp)

    x2 = w - 18; x1 = x2 - scale_px; y = h - 20

    # Background
    c.create_rectangle(x1-6, y-14, x2+6, y+8, fill=C["bg"], outline="")

    c.create_line(x1, y, x2, y, fill=C["text"], width=2, capstyle="round")
    c.create_line(x1, y-5, x1, y+5, fill=C["text"], width=2)
    c.create_line(x2, y-5, x2, y+5, fill=C["text"], width=2)
    unit = f"{best:.0f}m" if best >= 1 else f"{best*100:.0f}cm"
    c.create_text((x1+x2)/2, y-10, text=unit, fill=C["text"],
                  font=("Consolas", 9, "bold"), anchor="s")

def draw_links(c):
    """Draw colored lines between anchors showing link quality."""
    anchors = [(uid, d) for uid, d in S.devices.items()
               if d.get("type") == "ANCHOR" and d.get("ax") is not None]
    drawn = set()
    for i, (u1, d1) in enumerate(anchors):
        for j, (u2, d2) in enumerate(anchors):
            if i >= j:
                continue
            key = tuple(sorted([u1, u2]))
            q = S.link_quality.get(key, -1)
            if q < 0:
                continue
            # Quality → color
            if q >= 7:
                color = "#1a7f37"      # green — good
            elif q >= 4:
                color = "#d29922"      # yellow — medium
            else:
                color = "#cf222e"      # red — poor

            x1, y1 = d1["ax"], d1["ay"]
            x2, y2 = d2["ax"], d2["ay"]
            sx1, sy1 = w2s(x1, y1)
            sx2, sy2 = w2s(x2, y2)

            # Wider line with transparency effect (lighter color underneath)
            c.create_line(sx1, sy1, sx2, sy2, fill=color, width=2.5, capstyle="round")
            # Thinner bright center line
            c.create_line(sx1, sy1, sx2, sy2, fill=color, width=1.0, capstyle="round")

            # Rank-based link label at midpoint: "R=08-R=12 Q=9"
            r1 = d1.get("rank", 0)
            r2 = d2.get("rank", 0)
            mx, my = (sx1 + sx2) / 2, (sy1 + sy2) / 2
            c.create_text(mx, my, text=f"R={r1:02d}\nR={r2:02d} Q={q}", fill=color,
                         font=("Consolas", 7), anchor="c", justify="center")

    # Legend entry
    S._legend.config(text=" ▲ 基站显示R=xx(rank)  ● 在线   ⬤ 标签  ● 在线   ◉ 选中   "
                          "━ 连线: 7+ 5+ 0-3")

def draw_legend():
    if S.show_links and S.link_quality:
        # Keep legend built by draw_links
        S._legend.place(x=10, y=10); S._legend.lift()
    else:
        S._legend.config(text=" ▲ 基站  ● 在线   ⬤ 标签  ● 在线   ◉ 选中")
        S._legend.place(x=10, y=10); S._legend.lift()

# ══════════════════════════════════════════════
#  Demo Data
# ══════════════════════════════════════════════
def toggle_demo():
    """Toggle demo mode on/off."""
    if S.demo_active:
        clear_demo()
    else:
        load_demo_data()

def clear_demo():
    """Remove demo devices and restore real config that was saved before demo."""
    S.devices.clear()
    S.find_states.clear()
    S.link_quality.clear()
    S.selected_uid = ""
    S.demo_active = False
    S._demo_btn.config(text="◉  演示", style="Small.TButton")
    # Restore real config saved before demo
    if hasattr(S, "_saved_config") and S._saved_config:
        S.anchor_coords = S._saved_config.get("coords", {})
        S.anchor_label = S._saved_config.get("labels", {})
        S.anchor_counter = S._saved_config.get("counter", 0)
        S.tag_label = S._saved_config.get("tag_labels", {})
        S.tag_counter = S._saved_config.get("tag_counter", 0)
        S._saved_config = None
    refresh_lists()
    S.ox = 0; S.oy = 0; S.scale = 1.0
    set_status("演示已清除", "info")

def load_demo_data():
    """Load sample devices for map testing without serial connection."""
    import random
    # Save real config before overwriting with demo data
    S._saved_config = {
        "coords": dict(S.anchor_coords),
        "labels": dict(S.anchor_label),
        "counter": S.anchor_counter,
        "tag_labels": dict(S.tag_label),
        "tag_counter": S.tag_counter,
    }
    S.devices.clear()
    S.anchor_coords.clear()
    S.anchor_label.clear()
    S.anchor_counter = 0
    S.find_states.clear()
    S.selected_uid = ""

    # 4 anchors in a rectangle (with rank values)
    anchors = [
        ("0xA001", 0, 0, 2.0, 8),
        ("0xA002", 12, 0, 2.0, 12),
        ("0xA003", 0, 8, 2.0, 24),
        ("0xA004", 12, 8, 2.0, 188),
    ]
    for uid, ax, ay, az, rank in anchors:
        S.devices[uid] = {"type": "ANCHOR", "online": True,
                          "ax": ax, "ay": ay, "az": az, "trail": [], "rank": rank}
        S.anchor_coords[uid] = (ax, ay, az)
        S.anchor_counter += 1
        S.anchor_label[uid] = f"A{rank:02d}"
    # Do NOT save — demo data should not overwrite real anchor_cfg.json

    # 3 tags with positions and trails
    tag_positions = [
        ("T001", 4.5, 3.2),
        ("T002", 7.0, 4.5),
        ("T003", 2.5, 6.0),
    ]
    for uid, x, y in tag_positions:
        d = {"type": "TAG", "online": True, "x": x, "y": y, "z": 0, "q": 5, "trail": []}
        for i in range(20):
            tx = x + (random.random() - 0.5) * 2.0
            ty = y + (random.random() - 0.5) * 2.0
            d["trail"].append((tx, ty))
        S.devices[uid] = d

    # Link quality between anchor pairs (0xA001-0xA004)
    S.link_quality.clear()
    for pair, q in [((0, 1), 9), ((0, 2), 8), ((1, 3), 7), ((2, 3), 8), ((0, 3), 3), ((1, 2), 2)]:
        uids = [anchors[i][0] for i in pair]
        S.link_quality[tuple(sorted(uids))] = q

    S.demo_active = True
    S._demo_btn.config(text="✕  清除", style="Danger.TButton")
    refresh_lists()
    S.ox = 6; S.oy = 4; S.scale = 1.2
    set_status("演示模式 — 已加载 4 基站 + 3 标签", "info")

# ══════════════════════════════════════════════
#  Footer
# ══════════════════════════════════════════════
def build_footer(parent):
    ft = tk.Frame(parent, bg=C["surface"], height=40)
    ft.grid(row=2, column=0, columnspan=2, sticky="ew")
    ft.grid_propagate(False)

    # Color accent bar (left edge indicator)
    S._status_bar = tk.Frame(ft, bg=C["border"], width=4, height=40)
    S._status_bar.pack(side=tk.LEFT, fill=tk.Y)
    S._status_bar.pack_propagate(False)

    S._status_label = tk.Label(ft, text="  就绪", bg=C["surface"], fg=C["muted"],
                                font=("Segoe UI", 10), anchor="w")
    S._status_label.pack(side=tk.LEFT, padx=(6, 0))

    tk.Label(ft, text="F11 全屏  |  双击居中  |  滚轮缩放  |  拖拽平移",
             bg=C["surface"], fg=C["muted"], font=("Segoe UI", 7)).pack(side=tk.RIGHT, padx=10)

    S._status_timer = None  # for auto-reset

def set_status(msg: str, level="info"):
    """Update status bar with color-coded message.
       level: 'info' (blue), 'success' (green), 'warning' (orange), 'error' (red)
       Auto-resets to 'Ready' after 6s for non-error messages."""
    if not hasattr(S, "_status_label") or not S._status_label:
        return

    colors = {
        "info":    C["accent"],
        "success": C["green"],
        "warning": C["orange"],
        "error":   C["red"],
    }
    color = colors.get(level, C["accent"])

    S._status_label.config(text=f"  {msg}", fg=color)
    if hasattr(S, "_status_bar") and S._status_bar:
        S._status_bar.configure(bg=color)

    # Cancel any pending auto-reset
    if S._status_timer:
        root.after_cancel(S._status_timer)
        S._status_timer = None

    # Auto-reset to Ready after 6s (info/success), 10s (warning), never (error)
    if level != "error":
        delay = 10000 if level == "warning" else 6000
        S._status_timer = root.after(delay, _reset_status)

def _reset_status():
    """Reset status bar to default 'Ready' state."""
    if hasattr(S, "_status_label") and S._status_label:
        S._status_label.config(text="  就绪", fg=C["muted"])
    if hasattr(S, "_status_bar") and S._status_bar:
        S._status_bar.configure(bg=C["border"])
    S._status_timer = None

def check_liveness():
    check_device_timeout()
    root.after(3000, check_liveness)

def poll_serial():
    process_rx()
    root.after(30, poll_serial)

# ══════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════
def main():
    global root
    load_anchor_config()
    build_ui()

    def on_closing():
        """Graceful exit: disconnect serial, save config, destroy window."""
        if S.ser and S.ser.is_open:
            try: S.ser.close()
            except: pass
        S.ser = None
        save_anchor_config()
        root.destroy()
    root.protocol("WM_DELETE_WINDOW", on_closing)

    t = threading.Thread(target=serial_thread, daemon=True)
    t.start()

    root.mainloop()

if __name__ == "__main__":
    main()
