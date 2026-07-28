// object_labels.h - COCO-80 class names + per-class colours for 3D billboards.
//
// The Unit V sends class_id (COCO index); the M5Tab maps it to a display name
// and a stable colour so each object type renders consistently. Mirrors
// unitv/config.py CLASS_NAMES.

#pragma once
#include <stdint.h>

namespace object_labels {

// Display name for a COCO class_id, or "obj" if out of range.
const char* name(uint8_t class_id);

// Stable 0x00RRGGBB colour for a class_id (hashed into a pleasant palette).
uint32_t    color(uint8_t class_id);

inline uint8_t red  (uint32_t c) { return (c >> 16) & 0xFF; }
inline uint8_t green(uint32_t c) { return (c >>  8) & 0xFF; }
inline uint8_t blue (uint32_t c) { return  c        & 0xFF; }

} // namespace object_labels
