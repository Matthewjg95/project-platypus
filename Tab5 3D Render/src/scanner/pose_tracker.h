// pose_tracker.h - Dead-reckoned walking pose from IMU step detection.
//
// PORTABILITY: pure logic. Feed it accel magnitude + heading each tick and it
// produces step events and an XZ pose. Any IMU (or real odometry) can drive
// it; nothing here knows about M5/BMI270.
//
// Method: walking bounces the device vertically at step cadence (~1.5-2.5Hz).
// A slow envelope follows |accel| (≈1g at rest); upward deviations crossing
// STEP_THRESH_G with hysteresis + a refractory window count as steps. Each
// step advances the pose STEP_LEN_M along the current heading. Dead reckoning
// drifts, but over a one-room walk (<30 steps) the object-landmark
// registration and the carve tolerances absorb it. Heading convention matches
// the scanner: yaw 0 = +Z, x = sin, z = cos.

#pragma once
#include <stdint.h>
#include <math.h>

class PoseTracker {
public:
    static constexpr float    STEP_LEN_M      = 0.70f;  // average indoor stride
    static constexpr float    STEP_THRESH_G   = 0.12f;  // envelope deviation
    static const     uint32_t STEP_REFRACT_MS = 350;    // max ~2.8 steps/s

    void reset() {
        _x = _z = 0.0f; _dist = 0.0f; _steps = 0;
        _avg = 1.0f; _last_step_ms = 0; _above = false;
    }

    // accel_mag in g. Returns true when this tick registered a step.
    bool tick(float accel_mag, float heading_rad, uint32_t now_ms) {
        _avg += (accel_mag - _avg) * 0.02f;             // slow gravity follower
        float dev = accel_mag - _avg;
        bool step = false;
        if (!_above && dev > STEP_THRESH_G &&
            now_ms - _last_step_ms >= STEP_REFRACT_MS) {
            _above = true;
            _last_step_ms = now_ms;
            _x += STEP_LEN_M * sinf(heading_rad);
            _z += STEP_LEN_M * cosf(heading_rad);
            _dist += STEP_LEN_M;
            ++_steps;
            step = true;
        } else if (_above && dev < STEP_THRESH_G * 0.5f) {
            _above = false;                             // hysteresis re-arm
        }
        return step;
    }

    float x() const        { return _x; }
    float z() const        { return _z; }
    int   steps() const    { return _steps; }
    float distance() const { return _dist; }

private:
    float    _x = 0, _z = 0, _avg = 1.0f, _dist = 0;
    int      _steps = 0;
    uint32_t _last_step_ms = 0;
    bool     _above = false;
};
