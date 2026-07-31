// ss_silhouette.h - silhouette extraction from a grayscale frame, on the P4.
//
// The standalone ShadowScan project ran this pipeline on the Unit V in
// MicroPython; here the Unit V keeps its existing MeshScan firmware (JPEG
// stream, untouched) and the Tab5 does the vision work on the decoded frame:
//
//   Otsu (or manual) threshold
//   -> polarity pick (border-majority = background)
//   -> largest 4-connected component (iterative flood fill)
//   -> Moore boundary trace
//   -> RDP simplify (adaptive epsilon, capped point count)

#pragma once
#include <stdint.h>
#include "ss_geometry.h"

namespace ss {

struct Silhouette {
    Poly    pts;            // pixel coords in the source frame, boundary order
    int     threshold = 0;  // threshold actually used
    bool    dark_object = true;
    int     blob_pixels = 0;
    const char* error = nullptr;   // set when extraction failed
};

// gray: w*h 8-bit buffer. manual_thresh: -1 = auto (Otsu), else 0..255.
// Needs ~2*w*h bytes of scratch (mask + flood stack), taken from PSRAM.
// Returns false with sil.error set on failure.
bool extract_silhouette(const uint8_t* gray, int w, int h,
                        int manual_thresh, Silhouette& sil,
                        size_t max_points = 220);

}  // namespace ss
