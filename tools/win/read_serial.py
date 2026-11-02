"""One-shot serial capture for bench verification (run from Git Bash).

Usage: python read_serial.py [port] [seconds]
Prints everything the device sends; exits after N seconds.
"""
import os
import sys
import time

import serial

port = sys.argv[1] if len(sys.argv) > 1 else "COM6"
dur = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0

ser = serial.Serial(port, 115200, timeout=0.2)
if os.environ.get("NO_RESET") != "1":
    # USB-Serial/JTAG: pulse RTS to release the chip from reset (esptool may
    # leave it asserted after flashing), then give it time to boot.
    ser.setRTS(True)
    time.sleep(0.2)
    ser.setRTS(False)
    time.sleep(0.8)
ser.reset_input_buffer()
deadline = time.time() + dur
buf = b""
try:
    while time.time() < deadline:
        chunk = ser.read(4096)
        if chunk:
            buf += chunk
            sys.stdout.write(chunk.decode("utf-8", "replace"))
            sys.stdout.flush()
finally:
    ser.close()
print("\n--- capture end (%d bytes) ---" % len(buf))
