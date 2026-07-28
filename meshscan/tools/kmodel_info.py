#!/usr/bin/env python3
"""kmodel_info.py - Inspect a Kendryte K210 .kmodel file for the Unit V.

Prints the model version and, for the known container formats, a summary of the
layers/outputs so you can confirm input resolution and class count before
flashing. Supports KModel V3 ("KMDL"/0x4B4D444C era) and the V4/V5 ("KMODELV2"
style) magic. Layouts are partially documented, so unknown versions still get a
hexdump of the header.

Usage:
    python kmodel_info.py model.kmodel
"""
import struct
import sys


def hexdump(b, n=64):
    out = []
    for i in range(0, min(len(b), n), 16):
        chunk = b[i:i+16]
        hexs = " ".join("%02x" % c for c in chunk)
        text = "".join(chr(c) if 32 <= c < 127 else "." for c in chunk)
        out.append("%04x  %-47s  %s" % (i, hexs, text))
    return "\n".join(out)


def inspect(path):
    with open(path, "rb") as f:
        data = f.read()
    print("file: %s  (%d bytes)" % (path, len(data)))
    if len(data) < 8:
        print("too small to be a kmodel")
        return

    magic4 = data[:4]
    (u32,) = struct.unpack("<I", data[:4])

    # V3 kmodel: first u32 is version == 3, followed by flags, arch, layers_len...
    if u32 == 3:
        version, flags, arch, layers = struct.unpack("<IIII", data[:16])
        print("format: KModel V3")
        print("  flags=0x%08x arch=%d layer_count=%d" % (flags, arch, layers))
        print("  (V3 stores layer headers next; load on-device with KPU.load)")
        return

    # V4/V5: ASCII-ish magic "KMDL" or "KMODEL"
    if magic4 in (b"KMDL", b"KMD2") or data[:6] == b"KMODEL":
        print("format: KModel V4/V5 (magic=%r)" % magic4)
        # version commonly at offset 4
        try:
            (ver,) = struct.unpack("<I", data[4:8])
            print("  version=%d" % ver)
        except struct.error:
            pass
        print("  (full V4/V5 graph parsing not implemented — header shown below)")
    else:
        print("format: UNKNOWN (magic=%r u32=0x%08x)" % (magic4, u32))

    print("header:")
    print(hexdump(data, 96))


def main(argv):
    if len(argv) < 2:
        sys.stderr.write(__doc__)
        return 2
    inspect(argv[1])
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
