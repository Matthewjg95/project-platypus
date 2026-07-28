// orb_features.h - FAST-9 corners + intensity-centroid orientation + a rotated
// BRIEF-256 descriptor (ORB). Portable C++ operating on an 8-bit grayscale
// buffer; no Arduino deps so it is desktop-testable.
//
// Runs on the half-scale (160x120) gray frames the scan path already produces.
// Capacity is bounded; the strongest corners are kept after non-max suppression.

#pragma once
#include <stdint.h>

namespace cv {

struct Keypoints {
    static const int MAX = 384;
    static const int DESC_BYTES = 32;     // 256-bit BRIEF
    int     count = 0;
    float   x[MAX];
    float   y[MAX];
    float   angle[MAX];                   // radians
    int     score[MAX];
    uint8_t desc[MAX][DESC_BYTES];
};

class ORBExtractor {
public:
    // fast_threshold: intensity delta for the FAST test (typ. 20).
    // nms_radius: suppress weaker corners within this pixel radius.
    explicit ORBExtractor(int fast_threshold = 20, int nms_radius = 4);

    // Detect + describe. `gray` is w*h bytes, row-major. Fills `out`.
    void extract(const uint8_t* gray, int w, int h, Keypoints& out) const;

    void setThreshold(int t) { _t = t; }

private:
    int _t;
    int _nms;
    // BRIEF sampling pattern: 256 point pairs (ax,ay,bx,by) in the patch frame.
    int8_t _pattern[256][4];

    bool isCorner(const uint8_t* g, int w, int x, int y, int& score) const;
    float orientation(const uint8_t* g, int w, int h, int x, int y) const;
    void  describe(const uint8_t* g, int w, int h, float x, float y,
                   float angle, uint8_t* desc) const;
};

} // namespace cv
