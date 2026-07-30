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

// Percussive "thunk" — modeled on the power-button reset sound the user
// liked: a ~280Hz sine burst with a fast exponential decay, played as raw
// PCM (a square-wave tone() can't sound percussive). Amplitude baked at
// roughly half of full scale per user preference.
inline void buzz() {
    static int16_t pcm[280];
    static bool ready = false;
    if (!ready) {
        for (int i = 0; i < 280; ++i) {
            float t = i / 16000.0f;                     // 17.5ms total
            pcm[i] = (int16_t)(sinf(2.0f * 3.14159f * 280.0f * t)
                               * expf(-t * 260.0f) * 16000.0f);
        }
        ready = true;
    }
    M5.Speaker.playRaw(pcm, 280, 16000, false, 1, 0);
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
