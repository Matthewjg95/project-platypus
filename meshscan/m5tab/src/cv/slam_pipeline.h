// slam_pipeline.h - Incremental monocular structure-from-motion over the scan
// frames. Wires together orb_features -> feature_matcher -> two_view_geometry
// -> triangulator, accumulating a world-frame point cloud.
//
// Per frame: detect ORB, match to the previous frame, estimate relative pose,
// reject the pure-rotation/planar degeneracy, triangulate inlier matches, and
// transform the points into the world frame using the running global pose.
//
// SCALE: monocular translation is recovered only up to scale, so without an
// anchor the per-pair scale is fixed at 1 and the reconstruction drifts in
// absolute size. setScale()/anchorScaleFromDistance() let the caller pin it
// using a known object distance (depth_estimator). Full metric consistency
// needs bundle adjustment + loop closure — TODO for a later pass.

#pragma once
#include <stdint.h>
#include "orb_features.h"
#include "feature_matcher.h"
#include "two_view_geometry.h"
#include "point_cloud.h"

namespace cv {

class SlamPipeline {
public:
    bool begin(const CameraIntrinsics& K, uint32_t cloud_capacity = 20000);
    void reset();
    void freeMem();

    // Feed one grayscale frame (w*h). Returns true if it triangulated new points.
    bool addFrame(const uint8_t* gray, int w, int h);

    void  setScale(float s) { _scale = s; }
    float scale() const { return _scale; }

    const PointCloud& cloud() const { return _cloud; }
    int   frames() const { return _frames; }
    int   lastMatches() const { return _lastMatches; }
    int   lastInliers() const { return _lastInliers; }

    // Tuning
    int   min_matches = 20;
    float planarity_reject = 0.7f;   // skip pair if homography ~ as good as E
    float max_point_depth = 30.0f;   // metres (in unit-baseline*scale terms)

private:
    CameraIntrinsics _K{};
    ORBExtractor   _orb{20, 4};
    FeatureMatcher _matcher{};
    PointCloud     _cloud{};

    Keypoints* _prev = nullptr;      // PSRAM
    Keypoints* _cur  = nullptr;      // PSRAM
    bool _hasPrev = false;

    float _Rg[9];                    // global pose of the previous frame (R)
    float _tg[3];                    // global pose of the previous frame (t)
    float _scale = 1.0f;

    Match* _matchBuf = nullptr;      // PSRAM
    float* _p1 = nullptr;            // PSRAM, interleaved
    float* _p2 = nullptr;

    int _frames = 0, _lastMatches = 0, _lastInliers = 0;

    void triangulatePair(int nmatch, const float R[9], const float t[3]);
};

} // namespace cv
