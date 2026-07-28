// frame_buffer.cpp
#include "frame_buffer.h"
#include <esp_heap_caps.h>
#include <string.h>

bool FrameBuffer::begin() {
    for (int i = 0; i < FB_SLOTS; ++i) {
        _slots[i].jpeg = (uint8_t*)heap_caps_malloc(FB_SLOT_BYTES, MALLOC_CAP_SPIRAM);
        if (!_slots[i].jpeg) _slots[i].jpeg = (uint8_t*)malloc(FB_SLOT_BYTES);
        if (!_slots[i].jpeg) return false;
        _slots[i].valid = false;
        _slots[i].seq = 0;
    }
    _cur = -1;
    return true;
}

void FrameBuffer::pushFrame(const uint8_t* jpeg, uint16_t len) {
    int idx = (_cur + 1) % FB_SLOTS;
    FrameSlot& s = _slots[idx];
    uint16_t n = len <= FB_SLOT_BYTES ? len : FB_SLOT_BYTES;
    memcpy(s.jpeg, jpeg, n);
    s.jpeg_len = n;
    s.det_count = 0;
    s.frame_id = 0;
    s.valid = false;            // becomes valid once detections attach (or next frame)
    s.seq = ++_seq;
    _cur = idx;
}

void FrameBuffer::attachDetections(const DetectPacket& det) {
    if (_cur < 0) return;
    FrameSlot& s = _slots[_cur];
    s.frame_id = det.frame_id;
    s.det_count = det.count;
    memcpy(s.dets, det.items, sizeof(Detection) * det.count);
    s.valid = true;             // frame + detections both present
}

const FrameSlot* FrameBuffer::latest() const {
    const FrameSlot* best = nullptr;
    for (int i = 0; i < FB_SLOTS; ++i) {
        if (_slots[i].valid && (!best || _slots[i].seq > best->seq))
            best = &_slots[i];
    }
    return best;
}
