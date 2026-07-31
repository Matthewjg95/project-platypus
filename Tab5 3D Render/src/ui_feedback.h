// ui_feedback.h - Pseudo-haptic feedback via the Tab5 speaker.
//
// The Tab5 has no vibration motor (confirmed against the hardware maps), so
// interactions get iPhone-ish feedback the speaker way: a barely-audible
// high tick for taps, a low felt-in-the-hand thump for weighty events
// (long-press, scan complete, delete). Non-blocking — M5.Speaker plays async.

#pragma once
#include <M5Unified.h>
#include <math.h>

namespace ui_feedback {

// light tap acknowledgement
inline void tick() {
    M5.Speaker.tone(4000, 25);
}

// Weighty event "thunk". NOTE: the raw-PCM percussive version played nothing
// on this speaker path (playRaw silent where tone() works) — reverted to the
// tone that audibly worked, on a dedicated channel at HALF volume per user
// preference. Percussive PCM can be revisited with the device on the desk.
inline void buzz() {
    M5.Speaker.setChannelVolume(2, 128);
    M5.Speaker.tone(500, 80, 2, true);
}

// double thunk for completions worth celebrating (scan finished, etc.)
inline void buzz2() {
    buzz();
    M5.Speaker.tone(900, 50, 1, false);   // light confirm note on top
}

// call once at boot: speaker bring-up + audible proof-of-life
inline void begin() {
    M5.Speaker.begin();
    M5.Speaker.setVolume(255);
    M5.Speaker.tone(1200, 60);      // boot beep — if you can't hear this,
    M5.Speaker.tone(1800, 60, 1, false);  // the speaker path is the problem
}

} // namespace ui_feedback
