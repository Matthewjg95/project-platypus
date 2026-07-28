#pragma once
#include <M5Unified.h>
#include <Wire.h>
#include <stdint.h>

// ============================================================
// joystick_input.h — M5Stack Joystick2 Unit (STM32G030)
//
// Register map (from M5Stack I2C protocol doc V1, 2024/1/10):
//   0x00-0x03 : X_L X_H Y_L Y_H  (16-bit ADC, 0-65535)
//   0x20      : Button  (1=no press, 0=press)
//
// Uses Wire rerouted to Tab5 Grove port (SDA=53, SCL=54).
// Call begin() after M5.begin().
// ============================================================

class JoystickInput {
public:

    bool begin(uint8_t addr = 0x63) {
        _addr = addr;
        M5.Ex_I2C.begin();
        _sda = M5.Ex_I2C.getSDA();
        _scl = M5.Ex_I2C.getSCL();

        Wire.end();
        Wire.begin(_sda, _scl, 400000);
        delay(50);

        Serial.printf("[joystick] Wire SDA=%d SCL=%d addr=0x%02X\n",
                      _sda, _scl, addr);
        _scan_bus();

        // Retry first read — bus may not be settled immediately
        _connected = false;
        for (int attempt = 0; attempt < 5 && !_connected; attempt++) {
            delay(20);
            _connected = _read_raw();
        }
        if (_connected) {
            _centre_x = _raw_x;
            _centre_y = _raw_y;
            Serial.printf("[joystick] CONNECTED  centre x=%u y=%u btn=%u\n",
                          _centre_x, _centre_y, _raw_btn);
        } else {
            Serial.println("[joystick] NOT FOUND");
        }
        return _connected;
    }

    void update() {
        _btn_pressed = false;
        _btn_double  = false;

        if (!_connected) {
            _btn_held = false;
            if (millis() - _last_retry > 2000) {
                _last_retry = millis();
                if (_read_raw()) {
                    _connected = true;
                    _centre_x  = _raw_x;
                    _centre_y  = _raw_y;
                    Serial.println("[joystick] Reconnected");
                }
            }
            return;
        }

        if (!_read_raw()) {
            _read_errors++;
            _consecutive_failures++;
            if (_consecutive_failures >= 3) {
                _connected = false;
                _btn_held  = false;
            }
            return;
        }
        _consecutive_failures = 0;
        _read_count++;

        // Protocol: 1=no press, 0=press (active low, register 0x20)
        bool btn_down = (_raw_btn == 0);
        bool was_down = _btn_held;
        _btn_held    = btn_down;
        _btn_pressed = (btn_down && !was_down);

        if (_btn_pressed) {
            uint32_t now = millis();
            if (_last_press_time > 0 && (now - _last_press_time) < 400) {
                _btn_double      = true;
                _last_press_time = 0;
            } else {
                _last_press_time = now;
            }
        }
    }

    // -1.0 (left) to +1.0 (right)
    float x() const {
        if (!_connected) return 0.0f;
        return _dz(((float)_raw_x - _centre_x) / 32767.0f);
    }

    // -1.0 to +1.0 on Y axis
    float y() const {
        if (!_connected) return 0.0f;
        return _dz(((float)_raw_y - _centre_y) / 32767.0f);
    }

    bool connected()             const { return _connected; }
    bool button_held()           const { return _btn_held; }
    bool button_pressed()        const { return _btn_pressed; }
    bool button_double_pressed() const { return _btn_double; }

    uint16_t raw_x()   const { return _raw_x; }
    uint16_t raw_y()   const { return _raw_y; }
    uint8_t  raw_btn() const { return _raw_btn; }
    int      sda()     const { return _sda; }
    int      scl()     const { return _scl; }
    uint32_t read_count()  const { return _read_count; }
    uint32_t read_errors() const { return _read_errors; }
    const char* scan_result() const { return _scan_result; }

private:
    static constexpr float DZ = 0.08f;

    uint8_t  _addr            = 0x63;
    int      _sda             = -1, _scl = -1;
    bool     _connected       = false;
    uint32_t _last_retry      = 0;
    uint16_t _raw_x           = 32767, _raw_y = 32767;
    uint8_t  _raw_btn         = 1;
    float    _centre_x        = 32767, _centre_y = 32767;
    bool     _btn_held        = false;
    bool     _btn_pressed     = false, _btn_double = false;
    uint32_t _last_press_time = 0;
    uint32_t _read_count      = 0, _read_errors = 0;
    uint8_t  _consecutive_failures = 0;
    char     _scan_result[64] = "not scanned";

    void _scan_bus() {
        char buf[64]; buf[0] = '\0';
        int found = 0;
        for (uint8_t a = 1; a < 127; a++) {
            Wire.beginTransmission(a);
            if (Wire.endTransmission() == 0) {
                char tmp[8];
                snprintf(tmp, sizeof(tmp), "0x%02X ", a);
                strncat(buf, tmp, sizeof(buf) - strlen(buf) - 1);
                Serial.printf("[joystick] scan: 0x%02X\n", a);
                found++;
            }
        }
        if (!found) { strcpy(buf, "none"); Serial.println("[joystick] scan: none"); }
        strncpy(_scan_result, buf, sizeof(_scan_result) - 1);
    }

    bool _read_raw() {
        // Register 0x00: 4 bytes = x_l, x_h, y_l, y_h
        Wire.beginTransmission(_addr);
        Wire.write(0x00);
        if (Wire.endTransmission(false) != 0) return false;
        uint8_t n = Wire.requestFrom((int)_addr, 4);
        if (n < 4) {
            while (Wire.available()) Wire.read();
            return false;
        }
        uint8_t xl = Wire.read();
        uint8_t xh = Wire.read();
        uint8_t yl = Wire.read();
        uint8_t yh = Wire.read();
        uint16_t rx = ((uint16_t)xh << 8) | xl;
        uint16_t ry = ((uint16_t)yh << 8) | yl;
        if (rx == 0 && ry == 0) return false;
        _raw_x = rx;
        _raw_y = ry;

        // Register 0x20: button (1=no press, 0=press)
        Wire.beginTransmission(_addr);
        Wire.write(0x20);
        if (Wire.endTransmission(false) != 0) return false;
        uint8_t nb = Wire.requestFrom((int)_addr, 1);
        if (nb < 1) {
            while (Wire.available()) Wire.read();
            return false;
        }
        _raw_btn = Wire.read();
        return true;
    }

    float _dz(float v) const {
        if (fabsf(v) < DZ) return 0.0f;
        return (v > 0 ? 1.f : -1.f) * (fabsf(v) - DZ) / (1.f - DZ);
    }
};