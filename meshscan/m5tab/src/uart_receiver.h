// uart_receiver.h - Parse FRAME + DETECT packets streamed from the Unit V.
//
// Wire protocol (little-endian, must match unitv/uart_protocol.py):
//   FRAME   0xF1 | payload_len:u16 | jpeg... | crc8
//   DETECT  0xD1 | frame_id:u16 | object_count:u8 |
//                  [class_id:u8 conf:u8 x:u16 y:u16 w:u16 h:u16]*count | crc8
//
// CRC-8 (poly 0x07, init 0x00) covers all bytes after the magic and before the
// crc. The receiver is a byte-fed state machine so it never blocks waiting for
// a whole packet; call update() often from loop(). Completed packets are
// delivered through callbacks. The JPEG payload is assembled in a PSRAM buffer
// owned by the receiver and is only valid for the duration of the callback.

#pragma once
#include <Arduino.h>
#include <stdint.h>
#include "../../shared/mesh_format.h"

#ifndef UART_MAX_JPEG
#define UART_MAX_JPEG (24 * 1024)   // generous upper bound for a QVGA JPEG
#endif
#ifndef UART_MAX_DETECTIONS
#define UART_MAX_DETECTIONS 32
#endif

struct Detection {
    uint8_t  class_id;
    uint8_t  confidence;   // 0..100
    uint16_t x, y, w, h;   // pixels, top-left origin
};

struct DetectPacket {
    uint16_t  frame_id;
    uint8_t   count;
    Detection items[UART_MAX_DETECTIONS];
};

// Callbacks: frame jpeg pointer is into receiver-owned PSRAM, valid only during
// the call — copy it (e.g. into the frame buffer) if you need to keep it.
typedef void (*FrameCallback)(const uint8_t* jpeg, uint16_t len, void* user);
typedef void (*DetectCallback)(const DetectPacket& det, void* user);

class UartReceiver {
public:
    // Allocates the JPEG assembly buffer in PSRAM. Call once in setup().
    bool begin(HardwareSerial& serial, uint32_t baud, int rxPin, int txPin);

    void onFrame(FrameCallback cb, void* user)   { _frameCb = cb; _frameUser = user; }
    void onDetect(DetectCallback cb, void* user) { _detectCb = cb; _detectUser = user; }

    // Drain the UART RX FIFO and advance the parser. Non-blocking.
    void update();

    // Diagnostics
    uint32_t crcErrors()   const { return _crcErrors; }
    uint32_t framesOk()    const { return _framesOk; }
    uint32_t detectsOk()   const { return _detectsOk; }
    uint32_t resyncs()     const { return _resyncs; }

private:
    enum class State : uint8_t {
        SeekMagic,
        FrameLenLo, FrameLenHi, FrameData, FrameCrc,
        DetIdLo, DetIdHi, DetCount, DetItems, DetCrc,
    };

    void feed(uint8_t b);
    void resync();

    HardwareSerial* _serial = nullptr;

    State    _state = State::SeekMagic;
    uint8_t  _crc = 0;

    // FRAME assembly
    uint8_t* _jpeg = nullptr;        // PSRAM
    uint16_t _frameLen = 0;
    uint16_t _framePos = 0;

    // DETECT assembly
    DetectPacket _det{};
    uint16_t _detItemBytes = 0;      // total expected item bytes
    uint16_t _detItemPos = 0;
    uint8_t  _detRaw[UART_MAX_DETECTIONS * 10];  // 10 bytes per object on wire

    // callbacks
    FrameCallback  _frameCb = nullptr;  void* _frameUser = nullptr;
    DetectCallback _detectCb = nullptr; void* _detectUser = nullptr;

    // stats
    uint32_t _crcErrors = 0, _framesOk = 0, _detectsOk = 0, _resyncs = 0;
};
