#!/usr/bin/env python3
"""tab5_screenshot.py - Pull a pixel-perfect screenshot from the Tab5 over USB.

Sends "SS" on the Tab5's USB-CDC port; the firmware replies
    "SCRB" | u16 width | u16 height | width*height RGB565 little-endian
and this saves it as a 24-bit BMP (no third-party deps needed).

Usage:  python tab5_screenshot.py [COM6] [out.bmp]
"""
import struct
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM6"
OUT  = sys.argv[2] if len(sys.argv) > 2 else time.strftime("tab5_%H%M%S.bmp")


def read_exact(s, n, timeout_s=20):
    buf = b""
    end = time.time() + timeout_s
    while len(buf) < n and time.time() < end:
        buf += s.read(n - len(buf))
    if len(buf) != n:
        raise SystemExit(f"short read: {len(buf)}/{n} bytes")
    return buf


def main():
    s = serial.Serial()
    s.port = PORT
    s.baudrate = 115200          # USB-CDC ignores baud; runs at bus speed
    s.timeout = 0.5
    s.dtr = True
    s.rts = False
    s.open()
    time.sleep(0.3)
    s.reset_input_buffer()
    s.write(b"SS")

    # sync on the SCRB header (log lines may precede it)
    window = b""
    end = time.time() + 10
    while time.time() < end:
        b = s.read(1)
        if not b:
            continue
        window = (window + b)[-4:]
        if window == b"SCRB":
            break
    else:
        raise SystemExit("no SCRB header - is the new firmware flashed?")

    w, h = struct.unpack("<HH", read_exact(s, 4))
    print(f"receiving {w}x{h} ({w*h*2} bytes)...")
    px = read_exact(s, w * h * 2, timeout_s=60)
    s.close()

    # RGB565 -> BMP (bottom-up, BGR, rows padded to 4 bytes)
    row_pad = (4 - (w * 3) % 4) % 4
    img_size = (w * 3 + row_pad) * h
    with open(OUT, "wb") as f:
        f.write(b"BM")
        f.write(struct.pack("<IHHI", 54 + img_size, 0, 0, 54))
        f.write(struct.pack("<IiiHHIIiiII", 40, w, h, 1, 24, 0, img_size,
                            2835, 2835, 0, 0))
        for y in range(h - 1, -1, -1):
            row = bytearray()
            base = y * w * 2
            for x in range(w):
                # framebuffer bytes arrive big-endian (swapped) RGB565
                p = (px[base + 2*x] << 8) | px[base + 2*x + 1]
                r = (p >> 11) & 0x1F
                g = (p >> 5)  & 0x3F
                b = p & 0x1F
                row += bytes(((b * 255) // 31, (g * 255) // 63, (r * 255) // 31))
            row += b"\x00" * row_pad
            f.write(row)
    print(f"saved {OUT}")


if __name__ == "__main__":
    main()
