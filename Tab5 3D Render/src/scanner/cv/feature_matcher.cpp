// feature_matcher.cpp
#include "feature_matcher.h"
#include <string.h>

namespace cv {

int hamming256(const uint8_t* a, const uint8_t* b) {
    const uint32_t* wa = (const uint32_t*)a;
    const uint32_t* wb = (const uint32_t*)b;
    int d = 0;
    for (int i = 0; i < 8; ++i) d += __builtin_popcount(wa[i] ^ wb[i]);
    return d;
}

// best + second-best match of A[ai] against all of B
static void best_two(const Keypoints& A, int ai, const Keypoints& B,
                     int& bi, int& d1, int& d2) {
    bi = -1; d1 = 1 << 30; d2 = 1 << 30;
    for (int j = 0; j < B.count; ++j) {
        int d = hamming256(A.desc[ai], B.desc[j]);
        if (d < d1) { d2 = d1; d1 = d; bi = j; }
        else if (d < d2) { d2 = d; }
    }
}

int FeatureMatcher::match(const Keypoints& A, const Keypoints& B,
                          Match* out, int max_out,
                          float ratio, bool cross_check, int max_dist) const {
    int n = 0;
    for (int i = 0; i < A.count && n < max_out; ++i) {
        int bi, d1, d2;
        best_two(A, i, B, bi, d1, d2);
        if (bi < 0 || d1 > max_dist) continue;
        if (d2 < (1 << 29) && (float)d1 > ratio * (float)d2) continue;  // ratio test

        if (cross_check) {
            // B[bi]'s best back-match must be i
            int back = -1, bd1 = 1 << 30, bd2 = 1 << 30;
            for (int k = 0; k < A.count; ++k) {
                int d = hamming256(B.desc[bi], A.desc[k]);
                if (d < bd1) { bd2 = bd1; bd1 = d; back = k; }
                else if (d < bd2) { bd2 = d; }
            }
            if (back != i) continue;
        }
        out[n++] = { i, bi, d1 };
    }
    return n;
}

} // namespace cv
