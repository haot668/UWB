"""Console-based UWB monitor — no GUI dependencies, pure terminal output."""
import sys
import time
import serial
from collections import deque
from datetime import datetime

PORT = sys.argv[1] if len(sys.argv) > 1 else 'COM7'
BAUD = 115200
MAX_HISTORY = 50

history = deque(maxlen=MAX_HISTORY)

def parse_anchor_line(line):
    """Parse 'F,SEQ,R=XXXXXXXX,D=XXXX,C=XXXX'."""
    try:
        line = line.strip()
        if not line.startswith('F,'):
            return None
        parts = line.split(',')
        if len(parts) != 5:
            return None
        seq = int(parts[1], 16)
        blink_rx_ts = int(parts[2][2:], 16)
        tof_dist_cm = int(parts[3][2:], 16)
        clock_offset = int(parts[4][2:], 16)
        if clock_offset > 0x7FFF:
            clock_offset -= 0x10000
        return {'seq': seq, 'ts': blink_rx_ts, 'dist': tof_dist_cm, 'co': clock_offset}
    except (ValueError, IndexError):
        return None

def bar(value, max_val, width=40):
    """ASCII bar chart."""
    n = int(value / max_val * width) if max_val > 0 else 0
    return '#' * n + '-' * (width - n)

def main():
    # Open serial
    while True:
        try:
            ser = serial.Serial(PORT, BAUD, timeout=0.1)
            print(f"\n{'='*60}")
            print(f"  UWB Fusion Monitor — {PORT} @ {BAUD}")
            print(f"  {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
            print(f"{'='*60}")
            break
        except serial.SerialException as e:
            print(f"\r[WAIT] {PORT}: {e} — retrying...", end='')
            time.sleep(2)

    # Log file
    logname = f"uwb_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    logf = open(logname, 'w')
    logf.write("timestamp,seq,blink_rx_ts,tof_dist_cm,clock_offset\n")

    buf = b''
    last_stats = 0

    print(f"  {'SEQ':>4s}  {'Dist(cm)':>8s}  {'Dist':40s}  {'ClkOff':>7s}")
    print(f"  {'---':>4s}  {'-------':>8s}  {'----':40s}  {'------':>7s}")

    try:
        while True:
            if ser.in_waiting:
                buf += ser.read(ser.in_waiting)
                while b'\n' in buf:
                    line_bytes, buf = buf.split(b'\n', 1)
                    line = line_bytes.decode('utf-8', errors='replace').strip()

                    d = parse_anchor_line(line)
                    if d:
                        history.append(d)
                        ts = datetime.now().strftime('%H:%M:%S')

                        # Log
                        logf.write(f"{ts},{d['seq']:02X},{d['ts']:08X},{d['dist']},{d['co']}\n")

                        # Dist bar (max 100cm for scaling)
                        dist_bar = bar(d['dist'], 100, 30)
                        print(f"  {d['seq']:4d}  {d['dist']:5d} cm  {dist_bar} {d['co']:+5d}")

                        # Periodic stats
                        now = time.time()
                        if now - last_stats > 5.0 and len(history) >= 5:
                            dists = [h['dist'] for h in history]
                            cos = [h['co'] for h in history]
                            print(f"  {'───':>4s}  {'───────':>8s}  {'────':}  {'──────':>7s}")
                            print(f"  STAT  avg={sum(dists)/len(dists):.0f}cm  "
                                  f"min={min(dists)} max={max(dists)}  "
                                  f"co_avg={sum(cos)/len(cos):.0f}")
                            print(f"  {'───':>4s}  {'───────':>8s}  {'────':}  {'──────':>7s}")
                            logf.flush()
                            last_stats = now

                    elif line:
                        # Show tag debug lines
                        if any(line.startswith(p) for p in ['BLINK', 'SYNC', 'POLL', 'RESP', 'CYCLE', 'D=']):
                            print(f"  [TAG] {line}")
            else:
                time.sleep(0.02)

    except KeyboardInterrupt:
        print(f"\n{'='*60}")
        print(f"  EXIT — Log saved to {logname}")
        print(f"{'='*60}")
    finally:
        ser.close()
        logf.close()

if __name__ == '__main__':
    main()
