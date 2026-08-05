#!/usr/bin/env python3
"""serial_log.py - tee the Tab5's serial output to a file for a fixed window.

    python serial_log.py COM6 120 out.log

Used to catch intermittent faults: run it, reproduce the fault on the device,
then read the log. The scanner prints a 1Hz UART health line
("[scan] uart frames=.. detects=.. crcErr=.. resync=..") which distinguishes a
link problem (crcErr/resync climbing) from a starved-drain problem (frames
flatlining while the app keeps running) from an app hang (line stops entirely).
"""
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM6"
SECS = float(sys.argv[2]) if len(sys.argv) > 2 else 120
OUT = sys.argv[3] if len(sys.argv) > 3 else "tab5.log"

s = serial.Serial()
s.port = PORT
s.baudrate = 115200
s.timeout = 0.3
s.dtr = True
s.rts = False
# wait for the port to exist (cable may not be in yet) — up to 3 minutes
wait_end = time.time() + 180
while True:
    try:
        s.open()
        print(f"port {PORT} open, logging...", flush=True)
        break
    except serial.SerialException:
        if time.time() > wait_end:
            raise
        time.sleep(1.0)

end = time.time() + SECS
n = 0
with open(OUT, "wb") as f:
    while time.time() < end:
        data = s.read(4096)
        if data:
            f.write(data)
            f.flush()
            n += len(data)
s.close()
print(f"captured {n} bytes over {SECS:.0f}s -> {OUT}")
