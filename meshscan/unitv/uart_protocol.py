# uart_protocol.py - MeshScan UART framing (Unit V side)
#
# Two packet types are sent from the Unit V to the M5Tab, back to back, with the
# DETECT packet always following its paired FRAME packet:
#
#   FRAME   0xF1 | payload_len:u16 | jpeg_bytes... | crc8
#   DETECT  0xD1 | frame_id:u16 | object_count:u8 |
#                  [ class_id:u8 conf:u8 x:u16 y:u16 w:u16 h:u16 ] * count | crc8
#
# All multi-byte fields are LITTLE-ENDIAN. The crc8 covers every byte of the
# packet AFTER the magic byte and BEFORE the crc itself (i.e. the header fields
# plus the payload). The C++ receiver on the M5Tab implements the identical CRC.
#
# This module avoids numpy and keeps allocations small. `ustruct` is the MaixPy
# name for struct; we fall back to `struct` so the file is also importable on a
# desktop Python for testing.

try:
    import ustruct as struct
except ImportError:
    import struct

MAGIC_FRAME  = 0xF1
MAGIC_DETECT = 0xD1

# --- CRC-8 (poly 0x07, init 0x00, no reflection) ---------------------------
# Precompute the table once at import. Matches the table in uart_receiver.cpp.
_CRC8_TABLE = []
for _b in range(256):
    _c = _b
    for _ in range(8):
        _c = ((_c << 1) ^ 0x07) & 0xFF if (_c & 0x80) else ((_c << 1) & 0xFF)
    _CRC8_TABLE.append(_c)


def crc8(data, crc=0x00):
    t = _CRC8_TABLE
    for byte in data:
        crc = t[(crc ^ byte) & 0xFF]
    return crc & 0xFF


def pack_frame(jpeg_bytes):
    """Build a FRAME packet. Returns a bytes object ready to write to UART."""
    n = len(jpeg_bytes)
    if n > 0xFFFF:
        raise ValueError("JPEG too large for u16 length: %d" % n)
    header = struct.pack("<H", n)              # payload_len
    body = header + bytes(jpeg_bytes)          # crc covers len + jpeg
    c = crc8(body)
    return bytes((MAGIC_FRAME,)) + body + bytes((c,))


def pack_detect(frame_id, detections):
    """Build a DETECT packet.

    detections: iterable of (class_id, confidence_0_100, x, y, w, h) with pixel
    coords. Truncated to 255 objects (object_count is a u8).
    """
    dets = list(detections)[:255]
    body = struct.pack("<HB", frame_id & 0xFFFF, len(dets))
    for (class_id, conf, x, y, w, h) in dets:
        body += struct.pack("<BBHHHH",
                            class_id & 0xFF,
                            max(0, min(100, int(conf))),
                            x & 0xFFFF, y & 0xFFFF, w & 0xFFFF, h & 0xFFFF)
    c = crc8(body)
    return bytes((MAGIC_DETECT,)) + body + bytes((c,))
