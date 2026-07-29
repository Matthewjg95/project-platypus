// ui_feedback.h - Pseudo-haptic feedback via the Tab5 speaker.
//
// The Tab5 has no vibration motor (confirmed against the hardware maps), so
// interactions get iPhone-ish feedback the speaker way: a barely-audible
// high tick for taps, a low felt-in-the-hand thump for weighty events
// (long-press, scan complete, delete). Non-blocking — M5.Speaker plays async.

#pragma once
#include <M5Unified.h>

namespace ui_feedback {

// light tap acknowledgement
inline void tick() {
    M5.Speaker.tone(2000, 12);
}

// weighty event: long-press recognized, scan complete, destructive action
inline void buzz() {
    M5.Speaker.tone(160, 45);
}

// double-thump for completions worth celebrating (scan finished, OTA, etc.)
inline void buzz2() {
    M5.Speaker.tone(160, 40);
    // second pulse queued slightly later via tone's internal channel timing
    M5.Speaker.tone(200, 40, 1, false);
}

} // namespace ui_feedback
