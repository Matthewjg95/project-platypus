// uart_receiver.cpp - byte-fed state machine for the Unit V UART protocol.
#include "uart_receiver.h"
#include <esp_heap_caps.h>

static const uint8_t MAGIC_FRAME  = 0xF1;
static const uint8_t MAGIC_DETECT = 0xD1;

// CRC-8 poly 0x07, init 0x00 — table-driven, matches uart_protocol.py.
static uint8_t crc8_table[256];
static bool    crc8_ready = false;

static void crc8_init() {
    for (int b = 0; b < 256; ++b) {
        uint8_t c = (uint8_t)b;
        for (int i = 0; i < 8; ++i)
            c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
        crc8_table[b] = c;
    }
    crc8_ready = true;
}
static inline uint8_t crc8_step(uint8_t crc, uint8_t b) {
    return crc8_table[(crc ^ b) & 0xFF];
}

bool UartReceiver::begin(HardwareSerial& serial, uint32_t baud, int rxPin, int txPin) {
    if (!crc8_ready) crc8_init();
    _serial = &serial;
    _serial->begin(baud, SERIAL_8N1, rxPin, txPin);
    _serial->setRxBufferSize(4096);   // absorb bursts between update() calls

    _jpeg = (uint8_t*)heap_caps_malloc(UART_MAX_JPEG, MALLOC_CAP_SPIRAM);
    if (!_jpeg) _jpeg = (uint8_t*)malloc(UART_MAX_JPEG);  // fallback to internal
    return _jpeg != nullptr;
}

void UartReceiver::resync() {
    _state = State::SeekMagic;
    ++_resyncs;
}

void UartReceiver::update() {
    if (!_serial) return;
    // Bound work per call so we never starve the rest of loop().
    int budget = 8192;
    while (budget-- > 0 && _serial->available() > 0) {
        feed((uint8_t)_serial->read());
    }
}

void UartReceiver::feed(uint8_t b) {
    switch (_state) {
    case State::SeekMagic:
        if (b == MAGIC_FRAME)       { _crc = 0; _state = State::FrameLenLo; }
        else if (b == MAGIC_DETECT) { _crc = 0; _state = State::DetIdLo; }
        // else: stay seeking (silently skip junk between packets)
        break;

    // ---- FRAME ----
    case State::FrameLenLo:
        _frameLen = b; _crc = crc8_step(_crc, b);
        _state = State::FrameLenHi;
        break;
    case State::FrameLenHi:
        _frameLen |= (uint16_t)b << 8; _crc = crc8_step(_crc, b);
        if (_frameLen == 0 || _frameLen > UART_MAX_JPEG) { resync(); break; }
        _framePos = 0;
        _state = State::FrameData;
        break;
    case State::FrameData:
        _jpeg[_framePos++] = b; _crc = crc8_step(_crc, b);
        if (_framePos >= _frameLen) _state = State::FrameCrc;
        break;
    case State::FrameCrc:
        if (b == _crc) {
            ++_framesOk;
            if (_frameCb) _frameCb(_jpeg, _frameLen, _frameUser);
        } else {
            ++_crcErrors;
        }
        _state = State::SeekMagic;
        break;

    // ---- DETECT ----
    case State::DetIdLo:
        _det.frame_id = b; _crc = crc8_step(_crc, b);
        _state = State::DetIdHi;
        break;
    case State::DetIdHi:
        _det.frame_id |= (uint16_t)b << 8; _crc = crc8_step(_crc, b);
        _state = State::DetCount;
        break;
    case State::DetCount:
        _det.count = b; _crc = crc8_step(_crc, b);
        _detItemBytes = (uint16_t)b * 10;     // 10 bytes per object on the wire
        _detItemPos = 0;
        if (_det.count > UART_MAX_DETECTIONS) {
            // Too many to store — read+CRC them but we'll drop overflow items.
        }
        _state = (_det.count == 0) ? State::DetCrc : State::DetItems;
        break;
    case State::DetItems:
        if (_detItemPos < sizeof(_detRaw)) _detRaw[_detItemPos] = b;
        ++_detItemPos; _crc = crc8_step(_crc, b);
        if (_detItemPos >= _detItemBytes) _state = State::DetCrc;
        break;
    case State::DetCrc:
        if (b == _crc) {
            ++_detectsOk;
            uint8_t n = _det.count;
            if (n > UART_MAX_DETECTIONS) n = UART_MAX_DETECTIONS;
            for (uint8_t i = 0; i < n; ++i) {
                const uint8_t* p = &_detRaw[i * 10];
                Detection& d = _det.items[i];
                d.class_id   = p[0];
                d.confidence = p[1];
                d.x = (uint16_t)p[2] | ((uint16_t)p[3] << 8);
                d.y = (uint16_t)p[4] | ((uint16_t)p[5] << 8);
                d.w = (uint16_t)p[6] | ((uint16_t)p[7] << 8);
                d.h = (uint16_t)p[8] | ((uint16_t)p[9] << 8);
            }
            _det.count = n;
            if (_detectCb) _detectCb(_det, _detectUser);
        } else {
            ++_crcErrors;
        }
        _state = State::SeekMagic;
        break;
    }
}
