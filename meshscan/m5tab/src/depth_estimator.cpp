// depth_estimator.cpp
#include "depth_estimator.h"
#include <math.h>

// COCO-80 class ids for the objects we know real heights for. Heights in metres
// come from the project's reference table. Classes not listed stay 0 (unknown).
//   0 person | 56 chair | 57 couch(sofa) | 59 bed | 60 dining table
//  61 toilet | 62 tv | 63 laptop | 71 sink | 72 refrigerator
//
// "monitor" and "door" are not COCO classes, so they never arrive over UART
// from a stock COCO model. If you train a custom model that emits them, add the
// ids here. Reference values kept as named constants for documentation:
//   monitor 0.50, door 2.10.

DepthEstimator::DepthEstimator(int image_width, int image_height, float hfov_deg)
    : _w(image_width), _h(image_height) {
    float hfov_rad = hfov_deg * (float)M_PI / 180.0f;
    _focal_px = (image_width * 0.5f) / tanf(hfov_rad * 0.5f);

    for (int i = 0; i < 80; ++i) _height_table[i] = 0.0f;
    _height_table[0]  = 1.70f;  // person
    _height_table[56] = 0.85f;  // chair
    _height_table[57] = 0.90f;  // couch / sofa
    _height_table[59] = 0.60f;  // bed (mattress top height)
    _height_table[60] = 0.75f;  // dining table
    _height_table[61] = 0.80f;  // toilet
    _height_table[62] = 0.65f;  // tv
    _height_table[63] = 0.35f;  // laptop
    _height_table[71] = 0.90f;  // sink
    _height_table[72] = 1.80f;  // refrigerator
}

float DepthEstimator::referenceHeight(uint8_t class_id) const {
    return (class_id < 80) ? _height_table[class_id] : 0.0f;
}

float DepthEstimator::estimateDistance(uint8_t class_id, uint16_t bbox_h_px) const {
    float h = referenceHeight(class_id);
    if (h <= 0.0f || bbox_h_px == 0) return 0.0f;
    return (h * _focal_px) / (float)bbox_h_px;
}

ObjectEstimate DepthEstimator::estimate(uint8_t class_id,
                                        uint16_t bbox_x, uint16_t bbox_w,
                                        uint16_t bbox_h_px,
                                        float camera_yaw_rad) const {
    ObjectEstimate e{};
    e.real_height = referenceHeight(class_id);
    if (e.real_height <= 0.0f || bbox_h_px == 0) { e.valid = false; return e; }

    e.distance = (e.real_height * _focal_px) / (float)bbox_h_px;

    // Horizontal angle of the bbox centre off the optical axis.
    float cx = (float)bbox_x + (float)bbox_w * 0.5f;
    float dx = cx - (float)_w * 0.5f;
    float angle_off = atanf(dx / _focal_px);
    float world_yaw = camera_yaw_rad + angle_off;

    e.x = e.distance * sinf(world_yaw);
    e.z = e.distance * cosf(world_yaw);
    e.y = e.real_height * 0.5f;   // object centre, assumed standing on the floor
    e.valid = true;
    return e;
}
