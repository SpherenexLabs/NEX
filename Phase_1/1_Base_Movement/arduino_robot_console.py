#!/usr/bin/env python3
import sys, os, time, csv, argparse, threading, queue, re
from datetime import datetime

try:
    import serial
    import serial.tools.list_ports as list_ports
except ImportError:
    print("Missing pyserial. Install with: sudo apt install python3-serial")
    sys.exit(1)

HEADER_LINE = "Idx CMD dur(s) Lsteps Rsteps avgSPS avgRPM dist(cm)"
HEADER_TSV  = "Idx\tCMD\tdur(s)\tLsteps\tRsteps\tavgSPS\tavgRPM\tDist(cm)\t|Dist|\tTurn(deg)"

def auto_detect_port():
    """Try to find an Arduino-like port."""
    candidates = []
    for p in list_ports.comports():
        name = (p.manufacturer or "") + " " + (p.description or "")
        if any(k in name for k in ["Arduino", "wchusbserial", "ttyACM", "ttyUSB", "USB-SERIAL"]):
            candidates.append(p.device)
    # Fallback: common device names
    if not candidates:
        for guess in ["/dev/ttyACM0", "/dev/ttyUSB0"]:
            if os.path.exists(guess):
                candidates.append(guess)
    return candidates[0] if candidates else None

def parse_args():
    ap = argparse.ArgumentParser(description="Arduino motion console for Raspberry Pi")
    ap.add_argument("--port", "-p", default=None, help="Serial port (e.g., /dev/ttyACM0). Auto-detect if omitted.")
    ap.add_argument("--baud", "-b", type=int, default=9600, help="Baud rate (match your sketch). Default 9600.")
    ap.add_argument("--csv", default=None, help="Optional CSV path to append session reports.")
    ap.add_argument("--echo", action="store_true", help="Echo typed commands to the screen.")
    return ap.parse_args()

def make_serial(port, baud):
    ser = serial.Serial(port, baudrate=baud, timeout=0.05)
    # Give the board a moment after opening (some Arduinos reset)
    time.sleep(2.0)
    return ser

def reader_thread(ser, lines_q):
    buf = b""
    while True:
        try:
            chunk = ser.read(4096)
            if not chunk:
                # allow thread to exit if port is closed
                if not ser.is_open:
                    break
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                try:
                    text = line.decode(errors="replace").rstrip("\r")
                except Exception:
                    text = str(line)
                lines_q.put(text)
        except Exception as e:
            lines_q.put(f"[ERR reader] {e}")
            break

REPORT_ROW_RE_SPACE = re.compile(
    r"^\s*(\d+)\s+([FBRLS])\s+([0-9.]+)\s+(-?\d+)\s+(-?\d+)\s+([0-9.]+)\s+([0-9.]+)\s+(-?[0-9.]+)"
)
REPORT_ROW_RE_TSV = re.compile(
    r"^\s*(\d+)\t([FBRLS])\t([0-9.]+)\t(-?\d+)\t(-?\d+)\t([0-9.]+)\t([0-9.]+)\t(-?[0-9.]+)"
)
REPORT_START_RE = re.compile(r"^-{5,}\s*SESSION REPORT\s*-{5,}$")
REPORT_HEADER_RE = re.compile(r"^(Idx\s+CMD|Idx\tCMD)")

def maybe_parse_report_line(line):
    """
    Returns a dict if line looks like a report data row (either space or tab version).
    """
    m = REPORT_ROW_RE_SPACE.match(line)
    tsv = False
    if not m:
        m = REPORT_ROW_RE_TSV.match(line)
        tsv = bool(m)
    if not m:
        return None
    idx, cmd, dur, lsteps, rsteps, avgsps, avgrpm, dist = m.groups()
    return {
        "idx": int(idx),
        "cmd": cmd,
        "dur_s": float(dur),
        "Lsteps": int(lsteps),
        "Rsteps": int(rsteps),
        "avgSPS": float(avgsps),
        "avgRPM": float(avgrpm),
        "dist_cm": float(dist)
    }

def writer_loop(ser, lines_q, csv_path=None, echo=True):
    """
    Print all Arduino lines to terminal, and when a session report is detected,
    capture rows and append to CSV if requested.
    """
    in_report = False
    collected = []
    header_seen = False

    # If CSV requested, ensure header
    if csv_path and not os.path.exists(csv_path):
        with open(csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["timestamp","idx","cmd","dur_s","Lsteps","Rsteps","avgSPS","avgRPM","dist_cm"])

    while True:
        try:
            line = lines_q.get()
            if line is None:
                break

            # Detect report blocks
            if REPORT_START_RE.search(line):
                in_report = True
                header_seen = False
                collected.clear()
                # also print to console
                if echo: print(line)
                continue

            if in_report and REPORT_HEADER_RE.search(line):
                header_seen = True
                if echo: print(line)
                continue

            if in_report and line.strip().startswith("-") and "-----" in line:
                # end divider
                if echo: print(line)
                # flush collected rows to CSV
                if csv_path and collected:
                    ts = datetime.now().isoformat(timespec="seconds")
                    with open(csv_path, "a", newline="") as f:
                        w = csv.writer(f)
                        for row in collected:
                            w.writerow([ts, row["idx"], row["cmd"], row["dur_s"], row["Lsteps"],
                                        row["Rsteps"], row["avgSPS"], row["avgRPM"], row["dist_cm"]])
                    print(f"[saved] {len(collected)} rows -> {csv_path}")
                in_report = False
                collected.clear()
                header_seen = False
                continue

            if in_report and header_seen:
                parsed = maybe_parse_report_line(line)
                if parsed:
                    collected.append(parsed)
                # always show line
                if echo: print(line)
                continue

            # Normal line
            if echo: print(line)

        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"[ERR writer] {e}")
            break

def send_cmd(ser, cmd):
    # Write a single character + newline to be nice (Arduino reads char only, newline is ignored)
    ser.write((cmd[0]).encode())
    ser.flush()

def main():
    args = parse_args()
    port = args.port or auto_detect_port()
    if not port:
        print("Could not find a serial port. Plug in the Arduino and try again, or pass --port /dev/ttyACM0")
        sys.exit(2)
    print(f"[info] Using port={port} baud={args.baud}")

    try:
        ser = make_serial(port, args.baud)
    except Exception as e:
        print(f"Failed to open {port} @ {args.baud}: {e}")
        sys.exit(3)

    # Print initial banner from Arduino (it may reset)
    time.sleep(0.5)

    lines_q = queue.Queue()
    t = threading.Thread(target=reader_thread, args=(ser, lines_q), daemon=True)
    t.start()

    w = threading.Thread(target=writer_loop, args=(ser, lines_q, args.csv, args.echo), daemon=True)
    w.start()

    print("\nCommands: 1=start log, 0=stop+report, F/B/L/R moves, S=stop, +/- speed, ?=status")
    print("Type command letters and press ENTER; Ctrl+C to quit.\n")

    try:
        while True:
            user = input("> ").strip()
            if not user:
                continue
            # allow multi-char like "FRS0" to be sent one by one
            for ch in user:
                if ch in "10FfBbRrLlSs+-?":
                    send_cmd(ser, ch)
                    # tiny delay to avoid mashing serial
                    time.sleep(0.02)
                else:
                    print(f"[warn] ignored: '{ch}'")
    except KeyboardInterrupt:
        pass
    finally:
        try:
            ser.close()
        except Exception:
            pass
        # stop writer
        lines_q.put(None)
        time.sleep(0.2)
        print("\nBye.")

if __name__ == "__main__":
    main()
