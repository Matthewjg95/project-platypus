// depth_estimator.h - Monocular distance + world-position estimate from a
// detection's pixel bounding box and a table of known real-world object heights.
//
// Pinhole model:   distance = (real_height_m * focal_px) / bbox_pixel_height
//   focal_px = (image_width/2) / tan(HFOV/2)
//
// Horizontal placement uses the bbox centre offset from the optical axis plus
// the camera yaw at capture time (the scan azimuth), giving a world XZ position
// on a Y-up floor plane (camera at origin, floor y = 0).
//
// This is a Phase-1 heuristic: a single monocular frame cannot recover true
// scale, so accuracy depends entirely on the reference-height table matching
// reality. Unknown classes return valid == false.

#pragma once
#include <stdint.h>

struct ObjectEstimate {
    bool  valid;
    float distance;     // metres from camera
    float x, y, z;      // world position, metres (Y up, floor = 0)
    float real_height;  // reference height used, metres
};

class DepthEstimator {
public:
    // image_width in pixels, hfov in degrees (see config / OV2640 calibration).
    DepthEstimator(int image_width = 320, int image_height = 240,
                   float hfov_deg = 60.0f);

    // Reference height for a COCO class_id, or 0 if unknown.
    float referenceHeight(uint8_t class_id) const;
    bool  isKnown(uint8_t class_id) const { return referenceHeight(class_id) > 0.0f; }

    // Distance only, from bounding-box pixel height.
    float estimateDistance(uint8_t class_id, uint16_t bbox_h_px) const;

    // Full world position. camera_yaw_rad is the scan azimuth at capture.
    // bbox_x is the box's left edge, bbox_w its width (pixels).
    ObjectEstimate estimate(uint8_t class_id,
                            uint16_t bbox_x, uint16_t bbox_w,
                            uint16_t bbox_h_px,
                            float camera_yaw_rad) const;

    float focalPx() const { return _focal_px; }

    // ---- sanity gates (see estimate() for rationale) --------------------
    static const int       MIN_BOX_PX  = 8;      // in the downscaled frame
    static constexpr float ASPECT_TOL  = 1.8f;   // allowed w/h deviation factor
    static constexpr float MAX_RANGE_M = 8.0f;   // indoor plausibility ceiling

private:
    int   _w, _h;
    float _focal_px;
    float _height_table[20];   // metres, indexed by VOC-20 class_id; 0 = unknown
    float _width_table[20];    // metres; >0 = width-based ranging preferred
                               // (bbox width is the stabler scale cue for
                               // wide furniture: sofa, table, monitor)
};
