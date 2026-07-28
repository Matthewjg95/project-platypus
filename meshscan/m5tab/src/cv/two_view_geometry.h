// two_view_geometry.h - Relative pose between two frames from ORB matches.
//
// Pipeline (calibrated camera):
//   1. RANSAC homography (4-point DLT)  -> measures scene planarity / detects
//      the pure-rotation degeneracy where triangulation is ill-posed.
//   2. RANSAC essential matrix (normalised 8-point) -> the model used for pose.
//   3. recoverPose: decompose E, pick the (R,t) with the most points in front
//      of both cameras (cheirality).
//
// Translation t is recovered only up to scale (monocular). The pipeline anchors
// absolute scale later using known object sizes (depth_estimator).
//
// IMPORTANT capture-motion note: a camera rotating in place has no baseline, so
// the essential matrix degenerates and triangulated depth is meaningless. For
// usable structure the camera must TRANSLATE between frames (strafe along a
// wall, or orbit around objects). `planarity` near 1.0 flags the degenerate
// case so the pipeline can skip triangulating that pair.

#pragma once
#include <stdint.h>

namespace cv {

struct CameraIntrinsics {
    float fx, fy, cx, cy;
    void normalize(float px, float py, float& xn, float& yn) const {
        xn = (px - cx) / fx;  yn = (py - cy) / fy;
    }
};

struct PoseResult {
    bool  valid;
    float R[9];        // rotation, view1 -> view2
    float t[3];        // unit translation (scale ambiguous)
    int   inliers;     // essential-matrix inlier count
    float planarity;   // homography_inliers / essential_inliers, ~1 => planar/rotation
};

// pts are interleaved pixel coords: p1[2i],p1[2i+1] matches p2[2i],p2[2i+1].
// Returns a PoseResult; valid=false if pose could not be recovered.
PoseResult estimate_relative_pose(const float* p1, const float* p2, int n,
                                  const CameraIntrinsics& K,
                                  float pixel_thresh = 1.5f,
                                  int ransac_iters = 200);

// Lower-level pieces (exposed for testing):
int  ransac_homography(const float* p1, const float* p2, int n,
                       float H[9], uint8_t* inlier, int iters, float thresh);
int  ransac_essential(const float* p1, const float* p2, int n,
                      const CameraIntrinsics& K, float E[9], uint8_t* inlier,
                      int iters, float thresh_norm);
bool essential_8point(const float* xn1, const float* xn2, int n, float E[9]);
bool recover_pose(const float E[9], const float* p1, const float* p2,
                  const uint8_t* inlier, int n, const CameraIntrinsics& K,
                  float R[9], float t[3]);

} // namespace cv
