"""
UWB Fusion Monitor — Real-time serial data viewer for TDOA+TOF Fusion system.
Reads Anchor UART output and displays distance + clock offset in real time.

Anchor output format: F,SEQ,R=XXXXXXXX,D=XXXX,C=XXXX
  SEQ  = sequence number (hex)
  R    = blink RX timestamp (hex, lower 32-bit)
  D    = TOF distance in cm (hex)
  C    = clock offset (signed 16-bit hex, from carrier integrator)

Usage:
  python uwb_monitor.py [COM port]
  e.g.: python uwb_monitor.py COM3
  If no port specified, lists available ports.

Requires: pyserial, matplotlib
  pip install pyserial matplotlib
"""

import sys
import time
import struct
import threading
from collections import deque
from datetime import datetime

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.ticker import MaxNLocator

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("ERROR: pyserial not installed. Run: pip install pyserial")
    sys.exit(1)

# =====================================================================
# Configuration
# =====================================================================
BAUD_RATE = 115200
MAX_HISTORY = 200  # data points in scrolling window
LOG_FILE = None    # set to a path to enable CSV logging, or None to auto-name

# =====================================================================
# Data storage
# =====================================================================
class FusionData:
    __slots__ = ('seq', 'blink_rx_ts', 'tof_dist_cm', 'clock_offset',
                 'timestamp', 'raw_line')
    def __init__(self, seq, blink_rx_ts, tof_dist_cm, clock_offset, raw_line):
        self.seq = seq
        self.blink_rx_ts = blink_rx_ts
        self.tof_dist_cm = tof_dist_cm
        self.clock_offset = clock_offset
        self.timestamp = time.time()
        self.raw_line = raw_line

history = deque(maxlen=MAX_HISTORY)
data_lock = threading.Lock()
running = True
log_file_handle = None

# =====================================================================
# Serial reader thread
# =====================================================================
def parse_anchor_line(line):
    """Parse 'F,SEQ,R=XXXXXXXX,D=XXXX,C=XXXX' format."""
    try:
        line = line.strip()
        if not line.startswith('F,'):
            return None
        parts = line.split(',')
        if len(parts) != 5:
            return None

        seq = int(parts[1], 16)

        r_part = parts[2]
        if not r_part.startswith('R='):
            return None
        blink_rx_ts = int(r_part[2:], 16)

        d_part = parts[3]
        if not d_part.startswith('D='):
            return None
        tof_dist_cm = int(d_part[2:], 16)

        c_part = parts[4]
        if not c_part.startswith('C='):
            return None
        clock_offset = int(c_part[2:], 16)
        if clock_offset > 0x7FFF:  # sign-extend 16-bit
            clock_offset -= 0x10000

        return FusionData(seq, blink_rx_ts, tof_dist_cm, clock_offset, line)
    except (ValueError, IndexError):
        return None

def parse_tag_line(line):
    """Parse Tag debug lines for display."""
    line = line.strip()
    # Parse TDOA position: P,SEQ,X=x.xx,Y=y.yy,Z=z.zz,Q=q.qq
    if line.startswith('P,'):
        try:
            parts = line.split(',')
            if len(parts) >= 5:
                seq = int(parts[1], 16) if parts[1] else 0
                x = float(parts[2].split('=')[1]) if '=' in parts[2] else 0.0
                y = float(parts[3].split('=')[1]) if '=' in parts[3] else 0.0
                z = float(parts[4].split('=')[1]) if '=' in parts[4] else 0.0
                return ('POS', {'seq': seq, 'x': x, 'y': y, 'z': z, 'raw': line})
        except (ValueError, IndexError):
            pass
        return ('DEBUG', line)
    if line.startswith('D='):
        # Parse distance: D=001B cm → extract hex value
        try:
            hex_str = line[2:6]
            dist_cm = int(hex_str, 16)
            fd = FusionData(0, 0, dist_cm, 0, line)
            fd._tag_dist = True
            return ('DIST', fd)
        except (ValueError, IndexError):
            pass
        return ('DEBUG', line)
    if line.startswith('BLINK OK') or line.startswith('SYNC OK') or \
       line.startswith('POLL OK') or line.startswith('RESP OK') or \
       line.startswith('CYCLE DONE'):
        return ('PHASE', line)
    if line.startswith('SYNC RX FAIL') or line.startswith('RESP RX FAIL'):
        return ('ERROR', line)
    return ('DEBUG', line) if line else None

def serial_reader(port_name):
    global running, log_file_handle

    while running:
        try:
            ser = serial.Serial(port_name, BAUD_RATE, timeout=1)
            print(f"[OK] Connected to {port_name} @ {BAUD_RATE}")
            break
        except serial.SerialException as e:
            print(f"[WAIT] {e} — retrying in 2s...")
            time.sleep(2)

    buf = b''
    while running:
        try:
            if ser.in_waiting:
                buf += ser.read(ser.in_waiting)
                while b'\n' in buf:
                    line_bytes, buf = buf.split(b'\n', 1)
                    try:
                        line = line_bytes.decode('utf-8', errors='replace').strip()
                    except Exception:
                        line = line_bytes.decode('ascii', errors='replace').strip()

                    # Try anchor format first
                    fd = parse_anchor_line(line)
                    if fd:
                        with data_lock:
                            history.append(fd)
                        if log_file_handle:
                            ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                            log_file_handle.write(
                                f"{ts},{fd.seq:02X},{fd.blink_rx_ts:08X},"
                                f"{fd.tof_dist_cm},{fd.clock_offset}\n"
                            )
                            log_file_handle.flush()
                        continue

                    # Otherwise display as debug text
                    result = parse_tag_line(line)
                    if result:
                        tag, val = result
                        if tag == 'DIST':
                            # val is a FusionData with tag distance
                            with data_lock:
                                history.append(val)
                            print(f"  [DIST] D={val.tof_dist_cm} cm (from Tag)")
                        elif tag == 'POS':
                            # val is dict with x, y, z position
                            print(f"  [POS] X={val['x']:.2f}m Y={val['y']:.2f}m Z={val['z']:.2f}m SEQ={val['seq']:02X}")
                        else:
                            print(f"  [{tag}] {val}")
                    elif line:
                        print(f"  [SERIAL] {line}")

            else:
                time.sleep(0.01)
        except serial.SerialException as e:
            print(f"[ERR] Serial error: {e}")
            time.sleep(2)
            try:
                ser.close()
                ser = serial.Serial(port_name, BAUD_RATE, timeout=1)
            except Exception:
                pass
        except Exception as e:
            print(f"[ERR] {e}")
            time.sleep(0.5)

# =====================================================================
# Matplotlib live dashboard
# =====================================================================
def setup_plots():
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8), sharex=False)
    fig.canvas.manager.set_window_title('UWB Fusion Monitor')
    fig.tight_layout(pad=3.0)

    return fig, ax1, ax2, ax3

def animate(frame, axes):
    ax1, ax2, ax3 = axes

    with data_lock:
        if not history:
            return (ax1, ax2, ax3)
        n = len(history)
        xs = list(range(n))
        dists = [h.tof_dist_cm for h in history]
        clocks = [h.clock_offset for h in history]
        seqs = [h.seq for h in history]
        timestamps = [h.blink_rx_ts for h in history]

    # Plot 1: TOF Distance
    ax1.clear()
    ax1.plot(xs, dists, 'b-', linewidth=1.5, marker='.', markersize=4)
    ax1.set_ylabel('Tof Dist (cm)', fontsize=10)
    ax1.set_title('TOF Distance', fontsize=11, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.yaxis.set_major_locator(MaxNLocator(10))
    if dists:
        ax1.set_ylim(max(0, min(dists) - 10), max(dists) + 10)
        avg = sum(dists[-20:]) / min(20, len(dists[-20:]))
        ax1.axhline(y=avg, color='r', linestyle='--', alpha=0.5, label=f'Avg20={avg:.1f}')
        ax1.legend(loc='upper right', fontsize=8)

    # Plot 2: Clock Offset
    ax2.clear()
    ax2.plot(xs, clocks, 'g-', linewidth=1.5, marker='.', markersize=4)
    ax2.set_ylabel('Clock Offset', fontsize=10)
    ax2.set_title('Clock Offset (carrier integrator, signed)', fontsize=11, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.yaxis.set_major_locator(MaxNLocator(10))
    if clocks:
        ax2.axhline(y=0, color='k', linestyle='-', alpha=0.2)
        margin = max(abs(min(clocks)), abs(max(clocks))) + 5
        ax2.set_ylim(-margin, margin)

    # Plot 3: Sequence & Timestamp
    ax3.clear()
    ax3_twin = ax3.twinx()
    ax3.plot(xs, seqs, 'orange', linewidth=1.5, marker='.', markersize=4, label='SEQ')
    ax3.set_ylabel('Seq #', fontsize=10, color='orange')
    ax3.set_title('Sequence & RX Timestamp', fontsize=11, fontweight='bold')
    ax3.tick_params(axis='y', labelcolor='orange')
    ax3.grid(True, alpha=0.3)

    if timestamps:
        ts_ms = [(t - timestamps[0]) / 499.2e6 * 128 * 1000 for t in timestamps]  # ms
        ax3_twin.plot(xs, ts_ms, 'purple', linewidth=1, alpha=0.6, label='Elapsed(ms)')
        ax3_twin.set_ylabel('Elapsed (ms)', fontsize=10, color='purple')
        ax3_twin.tick_params(axis='y', labelcolor='purple')

    lines1 = ax1.get_lines() + [ax1.legend()] if dists else []
    lines2 = ax2.get_lines()
    lines3 = ax3.get_lines() + ax3_twin.get_lines()
    return tuple(lines1 + lines2 + lines3)

# =====================================================================
# Main
# =====================================================================
def main():
    global running, log_file_handle

    # Port selection
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        ports = list(serial.tools.list_ports.comports())
        if not ports:
            print("No serial ports found. Please specify COM port:")
            print("  python uwb_monitor.py COM3")
            sys.exit(1)

        print("Available serial ports:")
        for i, p in enumerate(ports):
            print(f"  [{i}] {p.device} — {p.description}")
        print(f"  [q] Quit")

        choice = input("\nSelect port number: ").strip()
        if choice.lower() == 'q':
            sys.exit(0)
        try:
            port = ports[int(choice)].device
        except (ValueError, IndexError):
            print("Invalid selection.")
            sys.exit(1)

    # Setup logging
    log_filename = f"uwb_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    log_file_handle = open(log_filename, 'w')
    log_file_handle.write("timestamp,seq,blink_rx_ts,tof_dist_cm,clock_offset\n")
    print(f"[LOG] Writing to {log_filename}")

    # Start serial thread
    thread = threading.Thread(target=serial_reader, args=(port,), daemon=True)
    thread.start()

    # Start matplotlib GUI
    print("[GUI] Starting dashboard... (close window to exit)")
    fig, ax1, ax2, ax3 = setup_plots()
    ani = animation.FuncAnimation(fig, animate, fargs=([ax1, ax2, ax3],),
                                  interval=200, cache_frame_data=False, blit=False)
    plt.tight_layout()

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        running = False
        if log_file_handle:
            log_file_handle.close()
        print("[EXIT] Done.")

if __name__ == '__main__':
    main()
