#!/usr/bin/env python3
"""unitv_fix_sd_shadow.py - Remove stale firmware copies from the Unit V's SD.

WHY: MaixPy's import path puts /sd BEFORE /flash, so old .py files left on the
SD card silently shadow the real firmware in /flash. That is why the camera
kept "reverting" to 115200 baud / the old COCO config after the SD was
reseated: /sd/config.py (old) was winning over /flash/config.py (current).

This deletes config.py / main.py / detector.py / uart_protocol.py / boot.py
from /sd only (never touches /flash), soft-reboots, and reports the boot line
so you can confirm the camera streams at the intended baud (460800).

Usage:  python unitv_fix_sd_shadow.py [COM10]
(Camera USB-C plugged into the PC; Grove side may stay on the Tab5.)
"""
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM10"
SHADOWS = ("config.py", "main.py", "detector.py", "uart_protocol.py", "boot.py")


def open_repl(port):
    s = serial.Serial()
    s.port = port
    s.baudrate = 115200
    s.timeout = 0.4
    s.dtr = False   # do not reset the K210 on open
    s.rts = False
    s.open()
    time.sleep(0.5)
    # Break whatever is running (the stream loop) and enter raw REPL.
    s.write(b"\x03")
    time.sleep(0.5)
    s.write(b"\x03")
    time.sleep(0.5)
    s.read(s.in_waiting or 1)
    s.write(b"\x01")
    time.sleep(0.5)
    s.read(s.in_waiting or 1)
    return s


def raw_exec(s, code, wait=2.0):
    s.write(code.encode() + b"\x04")
    time.sleep(wait)
    out = s.read(16000).decode("ascii", "replace")
    return "".join(c if 32 <= ord(c) < 127 or c == "\n" else "?" for c in out)


def main():
    print(f"[1/3] opening {PORT} ...")
    s = open_repl(PORT)

    print("[2/3] removing SD shadow files ...")
    code = "import os\r\n"
    for f in SHADOWS:
        code += (
            f"try:\r\n os.remove('/sd/{f}'); print('removed /sd/{f}')\r\n"
            f"except Exception as e:\r\n print('skip /sd/{f}:', e)\r\n"
        )
    code += "print('SD NOW=', os.listdir('/sd'))\r\n"
    out = raw_exec(s, code, wait=3.0)
    for line in out.splitlines():
        if any(k in line for k in ("removed", "skip", "SD NOW", "Error")):
            print("   ", line.strip())

    print("[3/3] soft reboot, watching for the stream banner ...")
    s.write(b"\x02")            # back to friendly REPL
    time.sleep(0.3)
    s.write(b"\x04")            # soft reboot -> boot.py -> main.py from /flash
    buf = b""
    end = time.time() + 8
    while time.time() < end:
        buf += s.read(s.in_waiting or 1)
    s.close()
    txt = "".join(c if 32 <= ord(c) < 127 or c == "\n" else "?" for c in
                  buf.decode("ascii", "replace"))
    ok = False
    for line in txt.splitlines():
        if "[main]" in line or "[detector]" in line or "[boot]" in line:
            print("   ", line.strip())
            if "460800" in line:
                ok = True
    print()
    print("RESULT:", "OK - streaming at 460800, shadow gone"
          if ok else "check output above (expected '[main] streaming ... @ 460800')")


if __name__ == "__main__":
    main()
