// object_labels.cpp
#include "object_labels.h"

namespace object_labels {

// Pascal VOC 20-class order — MUST match unitv/config.py CLASS_NAMES and the
// 20class.kmodel the camera runs.
static const char* const kNames[20] = {
    "aeroplane","bicycle","bird","boat","bottle",
    "bus","car","cat","chair","cow",
    "diningtable","dog","horse","motorbike","person",
    "pottedplant","sheep","sofa","train","tvmonitor",
};

const char* name(uint8_t class_id) {
    return (class_id < 20) ? kNames[class_id] : "obj";
}

// 12-entry palette; class ids map by modulo for stable, distinct colours.
static const uint32_t kPalette[12] = {
    0xE6194B, 0x3CB44B, 0xFFE119, 0x4363D8, 0xF58231, 0x911EB4,
    0x42D4F4, 0xF032E6, 0xBFEF45, 0xFABED4, 0x469990, 0xDCBEFF,
};

uint32_t color(uint8_t class_id) {
    return kPalette[class_id % 12];
}

} // namespace object_labels
