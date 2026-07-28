// ============================================================
// input.h / input.cpp
//
// Touch input handler for the M5Stack Tab5.
//
// Responsibilities:
//   - Detect single-finger drag → rotate model
//   - Detect two-finger pinch → zoom (future)
//   - Detect tap → reset view (future)
//   - Expose clean delta values each frame
//
// Usage:
//   InputState inp;
//   inp.begin(&M5.Touch);
//
//   // In loop():
//   inp.update();
//   state.rot_y += inp.delta_x * DRAG_SENSITIVITY;
//   state.rot_x += inp.delta_y * DRAG_SENSITIVITY;
// ============================================================

#pragma once
#include <Arduino.h>
#include <M5Unified.h>
#include <math.h>

constexpr float DRAG_SENSITIVITY = 0.005f;
constexpr int   DRAG_DEADZONE    = 3;
constexpr float ZOOM_SENSITIVITY = 0.005f;  // cam_z change per pixel of pinch
constexpr float ZOOM_MIN         = 0.5f;    // closest zoom
constexpr float ZOOM_MAX         = 20.0f;   // furthest zoom

struct InputDelta {
    float dx      = 0.0f;   // rotation drag X
    float dy      = 0.0f;   // rotation drag Y
    float zoom    = 0.0f;   // pinch zoom delta (negative = zoom in)
    bool  active  = false;  // single finger drag active
    bool  pinching = false; // two finger pinch active
};

class InputHandler {
public:
    void begin() {
        _prev_x        = -1;
        _prev_y        = -1;
        _prev_dist     = -1.0f;
        _touching      = false;
        _was_pinching  = false;
    }

    InputDelta update() {
        InputDelta d;

        int count = M5.Touch.getCount();

        if (count >= 2) {
            // Two fingers — pinch zoom
            auto t0 = M5.Touch.getDetail(0);
            auto t1 = M5.Touch.getDetail(1);

            float dx   = (float)(t0.x - t1.x);
            float dy   = (float)(t0.y - t1.y);
            float dist = sqrtf(dx*dx + dy*dy);

            if (_was_pinching && _prev_dist > 0.0f) {
                float delta = dist - _prev_dist;
                // Pinch in (dist shrinks) = zoom out (increase cam_z)
                // Pinch out (dist grows)  = zoom in  (decrease cam_z)
                if (fabsf(delta) > 1.0f) {
                    d.zoom     = -delta * ZOOM_SENSITIVITY;
                    d.pinching = true;
                }
            }

            _prev_dist    = dist;
            _was_pinching = true;
            _touching     = false;
            _prev_x       = -1;
            _prev_y       = -1;

        } else if (count == 1) {
            // Single finger — rotation drag
            _was_pinching = false;
            _prev_dist    = -1.0f;

            auto t = M5.Touch.getDetail(0);

            if (_touching && _prev_x >= 0) {
                int raw_dx = t.x - _prev_x;
                int raw_dy = t.y - _prev_y;

                if (abs(raw_dx) < 150 && abs(raw_dy) < 150) {
                    if (abs(raw_dx) > DRAG_DEADZONE) d.dx = (float)raw_dx;
                    if (abs(raw_dy) > DRAG_DEADZONE) d.dy = (float)raw_dy;
                    d.active = (d.dx != 0.0f || d.dy != 0.0f);
                }
            }

            _prev_x   = t.x;
            _prev_y   = t.y;
            _touching = true;

        } else {
            // No fingers
            _touching     = false;
            _was_pinching = false;
            _prev_x       = -1;
            _prev_y       = -1;
            _prev_dist    = -1.0f;
        }

        return d;
    }

private:
    int   _prev_x       = -1;
    int   _prev_y       = -1;
    float _prev_dist    = -1.0f;
    bool  _touching     = false;
    bool  _was_pinching = false;
};