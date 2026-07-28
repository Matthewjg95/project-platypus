// triangulator.cpp
#include "triangulator.h"
#include "linalg.h"
#include <math.h>

namespace cv {

bool triangulate_dlt(const float P1[12], const float P2[12],
                     float x1, float y1, float x2, float y2,
                     float X[3]) {
    // Each view contributes two rows: x * P_row2 - P_row0, y * P_row2 - P_row1.
    float A[16];
    for (int k = 0; k < 4; ++k) {
        A[0*4 + k] = x1 * P1[2*4 + k] - P1[0*4 + k];
        A[1*4 + k] = y1 * P1[2*4 + k] - P1[1*4 + k];
        A[2*4 + k] = x2 * P2[2*4 + k] - P2[0*4 + k];
        A[3*4 + k] = y2 * P2[2*4 + k] - P2[1*4 + k];
    }
    float h[4];
    if (!smallest_singular_vector(A, 4, 4, h)) return false;
    if (fabsf(h[3]) < 1e-9f) return false;
    X[0] = h[0] / h[3];
    X[1] = h[1] / h[3];
    X[2] = h[2] / h[3];
    return true;
}

} // namespace cv
