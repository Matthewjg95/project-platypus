// triangulator.h - Linear (DLT) triangulation of a point seen in two views.
//
// Given two 3x4 projection matrices (row-major) and the point's image
// coordinates in each view, solve for the homogeneous 3D point as the null
// space of the 4x4 DLT system, then dehomogenise.

#pragma once
#include <stdint.h>

namespace cv {

// P1,P2 are 3x4 row-major. (x1,y1),(x2,y2) are image coords in the same units
// the projection matrices map to. Returns false if the point is at infinity.
bool triangulate_dlt(const float P1[12], const float P2[12],
                     float x1, float y1, float x2, float y2,
                     float X[3]);

} // namespace cv
