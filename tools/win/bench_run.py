"""bench_run.py — scripted Task 2 timing bench (Firmware Reliability Rev 1).

Resets the board via RTS, then runs a timeline of host commands while
capturing the stream (B/M/V/#I frames) to a file.

Usage: python bench_run.py COM6 --label bmp_only --cmds "C,1@5;C,0@20" --dur 30
Commands are "CMD@SECONDS" separated by ';'.  Saves _bench_<label>.log.
"""
import argparse
import sys
import time

import serial

ap = argparse.ArgumentParser()
ap.add_argument("port", nargs="?", default="COM6")
ap.add_argument("--label", default="run")
ap.add_argument("--dur", type=float, default=30.0)
ap.add_argument("--cmds", default="")     # "C,1@5;C,0@20"
a = ap.parse_args()

timeline = []
for item in a.cmds.split(";"):
    if not item.strip():
        continue
    cmd, at = item.split("@")
    timeline.append((float(at), cmd.strip()))
timeline.sort()

ser = serial.Serial(a.port, 115200, timeout=0.2)
ser.setDTR(False)
ser.setRTS(True)
time.sleep(0.15)
ser.setRTS(False)
time.sleep(0.5)
ser.reset_input_buffer()

t0 = time.time()
buf = b""
fout = open(f"_bench_{a.label}.log", "wb")
try:
    while time.time() - t0 < a.dur:
        el = time.time() - t0
        while timeline and el >= timeline[0][0]:
            _, cmd = timeline.pop(0)
            ser.write(cmd.encode() + b"\n")
            print(f"[{el:6.1f}s] >> {cmd}", flush=True)
        chunk = ser.read(8192)
        if chunk:
            buf += chunk
            fout.write(chunk)
finally:
    ser.close()
    fout.close()
print(f"-- saved _bench_{a.label}.log ({len(buf)} bytes) --")
