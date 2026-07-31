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

// Weighty event "thunk". HISTORY: raw-PCM playRaw was silent; explicit
// channel + setChannelVolume was ALSO silent. Only the plain default-channel
// tone() has ever produced sound on this path — keep it verbatim and control
// loudness via master volume in begin(). Do not get clever here again.
inline void buzz() {
    M5.Speaker.tone(500, 80);
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
