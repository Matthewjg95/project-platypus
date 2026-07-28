// frame_buffer.h - Ring buffer of recent JPEG frames + their detections.
//
// The UART receiver hands us a frame's JPEG (in receiver-owned memory) and the
// paired DETECT packet. We copy the JPEG into a PSRAM slot so it survives past
// the callback, and stash the detections alongside. The scan view reads the
// latest slot for live preview; the scan collector samples slots around the
// 360 sweep.

#pragma once
#include <stdint.h>
#include "uart_receiver.h"

#ifndef FB_SLOTS
#define FB_SLOTS 4
#endif
#ifndef FB_SLOT_BYTES
#define FB_SLOT_BYTES (24 * 1024)
#endif

struct FrameSlot {
    uint8_t*  jpeg;          // PSRAM, FB_SLOT_BYTES capacity
    uint16_t  jpeg_len;
    uint16_t  frame_id;
    uint8_t   det_count;
    Detection dets[UART_MAX_DETECTIONS];
    uint32_t  seq;           // monotonically increasing fill order
    bool      valid;
};

class FrameBuffer {
public:
    bool begin();                       // allocate slots in PSRAM
    void pushFrame(const uint8_t* jpeg, uint16_t len);   // starts a new slot
    void attachDetections(const DetectPacket& det);      // fills current slot

    const FrameSlot* latest() const;    // most recently completed slot or null
    uint32_t latestSeq() const { return _seq; }

private:
    FrameSlot _slots[FB_SLOTS];
    int       _cur = -1;                // index being filled
    uint32_t  _seq = 0;
};
