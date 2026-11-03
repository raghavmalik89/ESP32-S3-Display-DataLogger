#!/usr/bin/env python3
"""
capture.py - ESP32-S3 Display DataLogger host capture tool.

Reads the logger text protocol from the USB-serial-JTAG CDC port and writes a
wide-format CSV plus a JSON sidecar with per-stream continuity stats.

Firmware frames (see docs/DATA_FORMAT.md):
    #H,<name>,<ver>                 firmware hello
    #D,bmp581=...,qmi8658c=...      device status
    #C,p_hz=...,i_hz=...,...        config
    #R,<reason>                     meta re-emit marker (seq counters reset)
    B,<seq>,<t_us>,<p_raw>,<t_raw>        baro (40 Hz)
    M,<seq>,<t_us>,<sid>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>   IMU (200 Hz)
    V,<seq>,<t_us>,<filter>,<vs_mm_s>,<hs_mm>            vario (20 Hz)
    S,<seq>,<t_us>,<start>,<session_id>                  record start/stop
    F,<seq>,<t_us>,<filter>                              filter change

Handshake: on open the tool sends "G,1" (host grabbing); on exit "G,0".

Usage (use the Espressif IDF python, which has pyserial):
    python tools/capture.py --port COM6 --duration 60 --label bench1
    python tools/capture.py --auto --duration 0          # run until Ctrl+C

Output: data/<YYYYMMDD_HHMMSS>_<label>.csv (+ .json sidecar)
Ctrl+C stops cleanly and still writes the sidecar with stats.
"""
import argparse
import csv
import json
import re
import sys
import time
from datetime import datetime
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial not found. Run with the Espressif IDF python env.")

COLS = ["seq", "t_us", "type", "sid", "p_raw", "t_raw",
        "ax", "ay", "az", "gx", "gy", "gz",
        "filter", "vs", "hs", "start", "session", "note"]

RE_B = re.compile(r"^B,(\d+),(\d+),(\d+),(\d+)$")
RE_M = re.compile(r"^M,(\d+),(\d+),(\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+)$")
RE_V = re.compile(r"^V,(\d+),(\d+),([A-Za-z0-9]+),(-?\d+),(-?\d+)$")
RE_S = re.compile(r"^S,(\d+),(\d+),(\d+),(\d+)$")
RE_F = re.compile(r"^F,(\d+),(\d+),([A-Za-z0-9]+)$")


def auto_port() -> str:
    prefs = []
    for p in list_ports.comports():
        desc = f"{p.description} {p.hwid}".lower()
        if any(k in desc for k in ("usb serial", "jtag", "esp32", "usb cdc", "acm")):
            prefs.append(p.device)
    if not prefs:
        avail = [p.device for p in list_ports.comports()]
        sys.exit(f"No USB-serial port found. Available: {avail or '(none)'}")
    return sorted(prefs)[0]


def parse_line(line: str):
    """Return ('row', dict) | ('meta', key, value) | ('bad', line)."""
    line = line.strip()
    if not line:
        return None
    if line.startswith("#"):
        body = line[1:]
        key, _, val = body.partition(",")
        return ("meta", key.strip(), val.strip())
    for typ, rx, fields in (
        ("B", RE_B, ["seq", "t_us", "p_raw", "t_raw"]),
        ("M", RE_M, ["seq", "t_us", "sid", "ax", "ay", "az", "gx", "gy", "gz"]),
        ("V", RE_V, ["seq", "t_us", "filter", "vs", "hs"]),
        ("S", RE_S, ["seq", "t_us", "start", "session"]),
        ("F", RE_F, ["seq", "t_us", "filter"]),
    ):
        m = rx.match(line)
        if m:
            row = {c: "" for c in COLS}
            row["type"] = typ
            for f, v in zip(fields, m.groups()):
                row[f] = v
            return ("row", row)
    return ("bad", line)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=None)
    ap.add_argument("--auto", action="store_true", help="auto-pick COM port")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--duration", type=float, default=0, help="seconds; 0 = until Ctrl+C")
    ap.add_argument("--label", default="run")
    ap.add_argument("--outdir", default="data")
    args = ap.parse_args()

    port = args.port or (auto_port() if args.auto else None)
    if not port:
        sys.exit("Give --port COMn or use --auto")

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = outdir / f"{stamp}_{args.label}.csv"
    json_path = csv_path.with_suffix(".json")

    last_seq = {}          # stream -> last seq
    stats = {}             # stream -> dict(count, gaps, dups, resets, t0, t1)
    meta = {}
    bad_lines = []
    counts = {"rows": 0}

    def note_gap(stream: str, seq: int):
        st = stats.setdefault(stream, dict(count=0, gaps=0, dups=0, resets=0,
                                           t0=None, t1=None))
        st["count"] += 1
        prev = last_seq.get(stream)
        if prev is not None:
            if seq == prev:
                st["dups"] += 1
            elif seq < prev:            # seq counters reset on #R re-emit
                st["resets"] += 1
            elif seq > prev + 1:
                st["gaps"] += seq - prev - 1
        last_seq[stream] = seq

    print(f"[capture] opening {port} @ {args.baud}, csv -> {csv_path}")
    ser = serial.Serial(port, args.baud, timeout=0.2)
    ser.write(b"G,1\n")    # tell firmware the host is grabbing
    ser.flush()

    t_start = time.monotonic()
    writer = None
    try:
        with open(csv_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=COLS)
            writer.writeheader()
            while True:
                if args.duration and time.monotonic() - t_start >= args.duration:
                    break
                raw = ser.readline()
                if not raw:
                    continue
                try:
                    line = raw.decode("utf-8", "replace")
                except Exception:
                    continue
                parsed = parse_line(line)
                if not parsed:
                    continue
                kind = parsed[0]
                if kind == "meta":
                    _, key, val = parsed
                    meta[key] = val
                    print(f"[meta] #{key} {val}")
                elif kind == "row":
                    _, row = parsed
                    writer.writerow(row)
                    counts["rows"] += 1
                    st = stats.setdefault(row["type"], dict(
                        count=0, gaps=0, dups=0, resets=0, t0=None, t1=None))
                    note_gap(row["type"], int(row["seq"]))
                    tu = int(row["t_us"])
                    st["t0"] = st["t0"] if st["t0"] is not None else tu
                    st["t1"] = tu
                    if counts["rows"] % 2000 == 0:
                        el = time.monotonic() - t_start
                        print(f"[capture] {counts['rows']} rows, {el:.1f}s")
                else:
                    bad_lines.append(line.strip()[:80])
    except KeyboardInterrupt:
        print("\n[capture] Ctrl+C, finishing")
    finally:
        try:
            ser.write(b"G,0\n")
            ser.flush()
            time.sleep(0.1)
            ser.close()
        except Exception:
            pass

    el = time.monotonic() - t_start
    sidecar = {
        "port": port,
        "baud": args.baud,
        "label": args.label,
        "wall_seconds": round(el, 3),
        "csv": str(csv_path),
        "meta": meta,
        "streams": {
            s: {
                **{k: v for k, v in st.items() if k not in ("t0", "t1")},
                "rate_hz": (round(st["count"] / max((st["t1"] - st["t0"]) / 1e6, 1e-9), 2)
                            if st["t1"] and st["t0"] and st["t1"] > st["t0"] else None),
            }
            for s, st in sorted(stats.items())
        },
        "bad_lines": len(bad_lines),
        "bad_sample": bad_lines[:5],
    }
    with open(json_path, "w") as f:
        json.dump(sidecar, f, indent=2)

    print(f"[capture] done: {counts['rows']} rows in {el:.1f}s "
          f"({counts['rows']/max(el,1e-9):.0f} rows/s)")
    for s, st in sorted(stats.items()):
        print(f"  {s}: n={st['count']} gaps={st['gaps']} dups={st['dups']} "
              f"resets={st['resets']}")
    if bad_lines:
        print(f"  bad lines: {len(bad_lines)} (sample: {bad_lines[:3]})")
    print(f"[capture] sidecar -> {json_path}")


if __name__ == "__main__":
    main()
