"""rec_test.py — scripted SD recording validation (Firmware Reliability Rev 1).

Usage: python rec_test.py COM6 [--start T] [--stop T] [--out FILE]
Timeline: at --start s sends "C,1", at --stop s sends "C,0" (absolute
seconds from script start).  Everything received is echoed to stdout and
saved to --out for later analysis.  Ctrl-C safe.
"""
import argparse
import sys
import threading
import time

import serial

ap = argparse.ArgumentParser()
ap.add_argument("port", nargs="?", default="COM6")
ap.add_argument("--start", type=float, default=2.0)
ap.add_argument("--stop", type=float, default=12.0)
ap.add_argument("--dur", type=float, default=16.0)
ap.add_argument("--out", default="_rec_test.log")
a = ap.parse_args()

ser = serial.Serial(a.port, 115200, timeout=0.2)
ser.setRTS(False)
time.sleep(0.3)
ser.reset_input_buffer()
stop = time.time() + a.dur
start_sent = stop_sent = False
fout = open(a.out, "wb")

def send(cmd):
    ser.write(cmd.encode() + b"\n")
    print(">>", cmd, flush=True)

try:
    while time.time() < stop:
        t = time.time()
        if not start_sent and t >= stop - a.dur + a.start:
            send("C,1")
            start_sent = True
        if not stop_sent and t >= stop - a.dur + a.stop:
            send("C,0")
            stop_sent = True
        chunk = ser.read(4096)
        if chunk:
            fout.write(chunk)
            sys.stdout.write(chunk.decode("utf-8", "replace"))
            sys.stdout.flush()
finally:
    ser.close()
    fout.close()
print("\n-- saved to", a.out, "--")
