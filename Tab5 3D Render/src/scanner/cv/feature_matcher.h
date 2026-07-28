// feature_matcher.h - Brute-force Hamming matcher for ORB descriptors with a
// Lowe ratio test and optional mutual (cross-check) filtering.

#pragma once
#include <stdint.h>
#include "orb_features.h"

namespace cv {

struct Match {
    int a;        // index into set A
    int b;        // index into set B
    int dist;     // Hamming distance
};

class FeatureMatcher {
public:
    // ratio: Lowe's test (best/second-best < ratio). cross_check: keep a match
    // only if B's best back-match is A. Returns number of matches written.
    int match(const Keypoints& A, const Keypoints& B,
              Match* out, int max_out,
              float ratio = 0.8f, bool cross_check = true,
              int max_dist = 64) const;
};

// Hamming distance between two 256-bit descriptors (32 bytes).
int hamming256(const uint8_t* a, const uint8_t* b);

} // namespace cv
