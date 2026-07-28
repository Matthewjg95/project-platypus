// room_fitter.cpp
#include "room_fitter.h"
#include <esp_heap_caps.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

static void* psram_alloc(size_t n) {
    void* p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM);
    return p ? p : malloc(n);
}

RoomFitter::RoomFitter(int gray_w, int gray_h) : _w(gray_w), _h(gray_h) {
    _numTheta = 90;                                   // 2-degree steps, 0..180
    int diag = (int)ceilf(sqrtf((float)(_w*_w + _h*_h)));
    _rhoOffset = diag;
    _numRho = 2 * diag + 1;
}

RoomFitter::~RoomFitter() {
    free(_cosT); free(_sinT); free(_acc); free(_edge); free(_thetaHist);
}

bool RoomFitter::begin() {
    _cosT = (float*)malloc(sizeof(float) * _numTheta);
    _sinT = (float*)malloc(sizeof(float) * _numTheta);
    _acc  = (uint16_t*)psram_alloc(sizeof(uint16_t) * _numTheta * _numRho);
    _edge = (uint8_t*)psram_alloc(_w * _h);
    _thetaHist = (uint32_t*)calloc(_numTheta, sizeof(uint32_t));
    if (!_cosT || !_sinT || !_acc || !_edge || !_thetaHist) return false;

    for (int t = 0; t < _numTheta; ++t) {
        float a = (float)M_PI * (float)t / (float)_numTheta;  // 0..pi
        _cosT[t] = cosf(a);
        _sinT[t] = sinf(a);
    }
    reset();
    return true;
}

void RoomFitter::reset() {
    if (_thetaHist) memset(_thetaHist, 0, sizeof(uint32_t) * _numTheta);
    _haveExtent = false;
    _minX = _maxX = _minZ = _maxZ = 0.0f;
    _frames = 0;
}

// 3x3 Sobel magnitude with a fixed threshold -> binary edge map.
void RoomFitter::sobelEdges(const uint8_t* g) {
    const int W = _w, H = _h;
    memset(_edge, 0, W * H);
    const int THRESH = 96;   // gradient magnitude cutoff (tune per scene)
    for (int y = 1; y < H - 1; ++y) {
        const uint8_t* r0 = g + (y - 1) * W;
        const uint8_t* r1 = g + (y) * W;
        const uint8_t* r2 = g + (y + 1) * W;
        for (int x = 1; x < W - 1; ++x) {
            int gx = (r0[x+1] + 2*r1[x+1] + r2[x+1])
                   - (r0[x-1] + 2*r1[x-1] + r2[x-1]);
            int gy = (r2[x-1] + 2*r2[x] + r2[x+1])
                   - (r0[x-1] + 2*r0[x] + r0[x+1]);
            int mag = abs(gx) + abs(gy);     // L1 approx, cheap
            if (mag >= THRESH) _edge[y*W + x] = 1;
        }
    }
}

// Vote every edge pixel into the Hough accumulator, then add this frame's peak
// strength per theta into the global orientation histogram.
void RoomFitter::houghVote() {
    const int W = _w, H = _h;
    memset(_acc, 0, sizeof(uint16_t) * _numTheta * _numRho);

    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            if (!_edge[y*W + x]) continue;
            for (int t = 0; t < _numTheta; ++t) {
                int rho = (int)lrintf(x * _cosT[t] + y * _sinT[t]) + _rhoOffset;
                uint16_t& cell = _acc[t * _numRho + rho];
                if (cell < 0xFFFF) cell++;
            }
        }
    }

    // For each theta, take its strongest rho bin as that orientation's support.
    for (int t = 0; t < _numTheta; ++t) {
        uint16_t best = 0;
        const uint16_t* row = &_acc[t * _numRho];
        for (int r = 0; r < _numRho; ++r) if (row[r] > best) best = row[r];
        _thetaHist[t] += best;
    }
}

void RoomFitter::addFrame(const uint8_t* gray, float /*yaw_rad*/) {
    if (!_edge || !_acc) return;
    sobelEdges(gray);
    houghVote();
    _frames++;
}

void RoomFitter::addObjectExtent(float x, float z) {
    if (!_haveExtent) {
        _minX = _maxX = x; _minZ = _maxZ = z; _haveExtent = true;
    } else {
        if (x < _minX) _minX = x; if (x > _maxX) _maxX = x;
        if (z < _minZ) _minZ = z; if (z > _maxZ) _maxZ = z;
    }
}

RoomBox RoomFitter::fit() const {
    RoomBox box{};

    // Footprint from object extent (+margin), falling back to a default room.
    if (_haveExtent) {
        box.cx    = 0.5f * (_minX + _maxX);
        box.cz    = 0.5f * (_minZ + _maxZ);
        box.width = (_maxX - _minX) + 2.0f * FOOTPRINT_MARGIN;
        box.depth = (_maxZ - _minZ) + 2.0f * FOOTPRINT_MARGIN;
        if (box.width < DEFAULT_SIDE * 0.5f) box.width = DEFAULT_SIDE * 0.5f;
        if (box.depth < DEFAULT_SIDE * 0.5f) box.depth = DEFAULT_SIDE * 0.5f;
    } else {
        box.cx = box.cz = 0.0f;
        box.width = box.depth = DEFAULT_SIDE;
    }
    box.height = DEFAULT_CEILING;

    // Confidence: Manhattan rooms concentrate Hough support in two orientation
    // clusters ~90 deg apart (vertical + horizontal walls). Measure how much of
    // the total orientation support falls near 0/180 (vertical lines) and 90
    // (horizontal lines) versus diagonal noise.
    if (_thetaHist && _frames > 0) {
        uint64_t total = 0, axis = 0;
        for (int t = 0; t < _numTheta; ++t) {
            total += _thetaHist[t];
            // theta near 0 or pi -> vertical image lines; near pi/2 -> horizontal
            int degx2 = (180 * t) / _numTheta;   // 0..180
            int dV = (degx2 < 90) ? degx2 : (180 - degx2);
            int dH = abs(degx2 - 90);
            if (dV <= 12 || dH <= 12) axis += _thetaHist[t];
        }
        box.confidence = (total > 0) ? (float)((double)axis / (double)total) : 0.0f;
    } else {
        box.confidence = 0.0f;
    }
    return box;
}
