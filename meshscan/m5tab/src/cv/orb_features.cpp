// orb_features.cpp
#include "orb_features.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

namespace cv {

// Bresenham radius-3 circle (16 px), clockwise from top.
static const int CIRC_X[16] = { 0, 1, 2, 3, 3, 3, 2, 1, 0,-1,-2,-3,-3,-3,-2,-1};
static const int CIRC_Y[16] = {-3,-3,-2,-1, 0, 1, 2, 3, 3, 3, 2, 1, 0,-1,-2,-3};

static const int PATCH_R = 13;     // BRIEF sampling radius
static const int BORDER  = 16;     // keep keypoints this far from the edge
static const int ORI_R   = 15;     // orientation patch radius

ORBExtractor::ORBExtractor(int fast_threshold, int nms_radius)
    : _t(fast_threshold), _nms(nms_radius) {
    // Deterministic BRIEF pattern: point pairs in [-PATCH_R, PATCH_R].
    uint32_t s = 0x9E3779B9u;
    auto rnd = [&s]() { s ^= s << 13; s ^= s >> 17; s ^= s << 5; return s; };
    auto coord = [&](void) -> int {
        return (int)(rnd() % (2*PATCH_R + 1)) - PATCH_R;
    };
    for (int i = 0; i < 256; ++i) {
        _pattern[i][0] = (int8_t)coord();
        _pattern[i][1] = (int8_t)coord();
        _pattern[i][2] = (int8_t)coord();
        _pattern[i][3] = (int8_t)coord();
    }
}

bool ORBExtractor::isCorner(const uint8_t* g, int w, int x, int y, int& score) const {
    int Ip = g[y*w + x];
    int hi = Ip + _t, lo = Ip - _t;

    // quick reject on the 4 compass points (indices 0,4,8,12)
    int c0 = g[(y+CIRC_Y[0])*w + x+CIRC_X[0]];
    int c4 = g[(y+CIRC_Y[4])*w + x+CIRC_X[4]];
    int c8 = g[(y+CIRC_Y[8])*w + x+CIRC_X[8]];
    int c12= g[(y+CIRC_Y[12])*w+ x+CIRC_X[12]];
    int brighter = (c0>hi)+(c4>hi)+(c8>hi)+(c12>hi);
    int darker   = (c0<lo)+(c4<lo)+(c8<lo)+(c12<lo);
    if (brighter < 3 && darker < 3) return false;

    // classify all 16
    int8_t cls[16];
    int sum = 0;
    for (int i = 0; i < 16; ++i) {
        int v = g[(y+CIRC_Y[i])*w + x+CIRC_X[i]];
        cls[i] = (v > hi) ? 1 : (v < lo) ? -1 : 0;
        sum += abs(v - Ip);
    }
    // contiguous arc of >= 9 of the same sign (wrap-around)
    for (int sign = -1; sign <= 1; sign += 2) {
        int run = 0, best = 0;
        for (int i = 0; i < 16 + 9; ++i) {
            if (cls[i % 16] == sign) { if (++run > best) best = run; }
            else run = 0;
        }
        if (best >= 9) { score = sum; return true; }
    }
    return false;
}

float ORBExtractor::orientation(const uint8_t* g, int w, int h, int x, int y) const {
    float m01 = 0.0f, m10 = 0.0f;
    int r2 = ORI_R * ORI_R;
    for (int dy = -ORI_R; dy <= ORI_R; ++dy) {
        int yy = y + dy; if (yy < 0 || yy >= h) continue;
        for (int dx = -ORI_R; dx <= ORI_R; ++dx) {
            if (dx*dx + dy*dy > r2) continue;
            int xx = x + dx; if (xx < 0 || xx >= w) continue;
            int v = g[yy*w + xx];
            m10 += dx * v; m01 += dy * v;
        }
    }
    return atan2f(m01, m10);
}

void ORBExtractor::describe(const uint8_t* g, int w, int h, float x, float y,
                            float angle, uint8_t* desc) const {
    float ca = cosf(angle), sa = sinf(angle);
    memset(desc, 0, Keypoints::DESC_BYTES);
    for (int i = 0; i < 256; ++i) {
        float ax = _pattern[i][0], ay = _pattern[i][1];
        float bx = _pattern[i][2], by = _pattern[i][3];
        int rax = (int)lrintf(x + ca*ax - sa*ay);
        int ray = (int)lrintf(y + sa*ax + ca*ay);
        int rbx = (int)lrintf(x + ca*bx - sa*by);
        int rby = (int)lrintf(y + sa*bx + ca*by);
        if (rax<0||rax>=w||ray<0||ray>=h||rbx<0||rbx>=w||rby<0||rby>=h) continue;
        if (g[ray*w + rax] < g[rby*w + rbx]) desc[i>>3] |= (uint8_t)(1u << (i & 7));
    }
}

void ORBExtractor::extract(const uint8_t* gray, int w, int h, Keypoints& out) const {
    out.count = 0;
    // pass 1: collect raw corners with scores into a temporary grid score map.
    // To bound memory we keep a small candidate list and NMS it.
    struct Cand { int x, y, s; };
    static const int CMAX = 2048;
    Cand* cand = (Cand*)malloc(sizeof(Cand) * CMAX);
    if (!cand) return;
    int cn = 0;
    for (int y = BORDER; y < h - BORDER; ++y) {
        for (int x = BORDER; x < w - BORDER; ++x) {
            int sc;
            if (isCorner(gray, w, x, y, sc)) {
                if (cn < CMAX) cand[cn++] = { x, y, sc };
            }
        }
    }
    // pass 2: greedy NMS — sort by score desc, accept if no stronger neighbour kept.
    // simple O(n^2) over candidates (n bounded by CMAX, usually far fewer).
    // sort indices by score
    for (int i = 0; i < cn; ++i)
        for (int j = i+1; j < cn; ++j)
            if (cand[j].s > cand[i].s) { Cand t = cand[i]; cand[i] = cand[j]; cand[j] = t; }

    int r2 = _nms * _nms;
    for (int i = 0; i < cn && out.count < Keypoints::MAX; ++i) {
        bool keep = true;
        for (int k = 0; k < out.count; ++k) {
            float dx = out.x[k] - cand[i].x, dy = out.y[k] - cand[i].y;
            if (dx*dx + dy*dy <= r2) { keep = false; break; }
        }
        if (!keep) continue;
        int idx = out.count++;
        out.x[idx] = (float)cand[i].x;
        out.y[idx] = (float)cand[i].y;
        out.score[idx] = cand[i].s;
        out.angle[idx] = orientation(gray, w, h, cand[i].x, cand[i].y);
        describe(gray, w, h, out.x[idx], out.y[idx], out.angle[idx], out.desc[idx]);
    }
    free(cand);
}

} // namespace cv
