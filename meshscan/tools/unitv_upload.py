#!/usr/bin/env python3
"""unitv_upload.py - Fast file uploader for the Unit V (K210) over raw REPL.

Replaces ampy for large files: ampy pushes tiny repr-encoded chunks (~1-2 KB/s,
1.35 MB model = 15-35+ min and it hangs silently if the SD drops). This sends
2 KB base64 chunks through the raw REPL with the target file held open across
chunks: ~6-8 KB/s effective (1.35 MB in ~3-4 min), prints progress, verifies
the final size on-device, and fails loudly instead of hanging.

Usage:
    python unitv_upload.py <local_file> <remote_path> [COM10]
    python unitv_upload.py ..\\unitv\\model\\20class.kmodel /sd/20class.kmodel
"""
import binascii
import os
import sys
import time

import serial

CHUNK = 2048


def fail(msg):
    print("FAIL:", msg)
    sys.exit(1)


def raw_exec(s, code, wait=1.2, tag=""):
    """Execute code in raw REPL, return decoded output; raise on Traceback."""
    s.write(code.encode() + b"\x04")
    end = time.time() + wait
    buf = b""
    while time.time() < end:
        buf += s.read(s.in_waiting or 1)
        if buf.endswith(b"\x04>") or buf.endswith(b"\x04\x04>"):
            break
    out = buf.decode("ascii", "replace")
    if "Traceback" in out:
        fail(f"device error during {tag or 'exec'}: {out[-300:]}")
    return out


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(2)
    local, remote = sys.argv[1], sys.argv[2]
    port = sys.argv[3] if len(sys.argv) > 3 else "COM10"
    data = open(local, "rb").read()
    total = len(data)
    print(f"uploading {local} ({total} bytes) -> {remote} on {port}")

    s = serial.Serial()
    s.port = port
    s.baudrate = 115200
    s.timeout = 0.4
    s.dtr = False
    s.rts = False
    s.open()
    time.sleep(0.6)
    s.write(b"\x03")
    time.sleep(0.5)
    s.write(b"\x03")
    time.sleep(0.5)
    s.read(s.in_waiting or 1)
    s.write(b"\x01")                       # raw REPL
    time.sleep(0.5)
    s.read(s.in_waiting or 1)

    # sanity: target FS present + open file once, keep handle in globals
    out = raw_exec(
        s,
        "import os, ubinascii\r\n"
        f"_d='{os.path.dirname(remote) or '/'}'\r\n"
        "print('DIR-OK', os.listdir(_d) is not None)\r\n"
        f"_f=open('{remote}','wb')\r\n"
        "print('OPEN-OK')\r\n",
        wait=2.5, tag="open",
    )
    if "OPEN-OK" not in out:
        fail(f"could not open {remote} (SD missing/unmounted?): {out[-200:]}")

    t0 = time.time()
    sent = 0
    for off in range(0, total, CHUNK):
        b64 = binascii.b2a_base64(data[off:off + CHUNK]).strip().decode()
        raw_exec(s, f"_f.write(ubinascii.a2b_base64('{b64}'))\r\n",
                 wait=3.0, tag=f"chunk@{off}")
        sent = min(off + CHUNK, total)
        if sent % (64 * CHUNK) < CHUNK or sent == total:
            rate = sent / max(time.time() - t0, 0.01) / 1024
            print(f"  {sent}/{total}  ({100*sent//total}%)  {rate:.1f} KB/s")

    out = raw_exec(
        s,
        f"_f.close()\r\nimport os\r\nprint('FINAL=', os.stat('{remote}')[6])\r\n",
        wait=3.0, tag="close",
    )
    s.write(b"\x02")
    s.close()

    for line in out.splitlines():
        if "FINAL=" in line:
            final = int(line.split("FINAL=")[1].strip().split()[0])
            if final == total:
                print(f"OK: {remote} = {final} bytes (exact match) "
                      f"in {time.time()-t0:.0f}s")
                return
            fail(f"size mismatch: device={final} local={total}")
    fail(f"no size confirmation: {out[-200:]}")


if __name__ == "__main__":
    main()
