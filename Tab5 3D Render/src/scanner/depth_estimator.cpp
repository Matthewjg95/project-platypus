// depth_estimator.cpp
#include "depth_estimator.h"
#include <math.h>

// Pascal VOC 20-class ids for the room objects we know real heights for (metres).
// Order matches unitv/config.py CLASS_NAMES + object_labels. Classes not set
// (vehicles, animals, etc.) stay 0 = unknown and are skipped during placement.
//   4 bottle | 7 cat | 8 chair | 10 diningtable | 11 dog
//  14 person | 15 pottedplant | 17 sofa | 19 tvmonitor

DepthEstimator::DepthEstimator(int image_width, int image_height, float hfov_deg)
    : _w(image_width), _h(image_height) {
    float hfov_rad = hfov_deg * (float)M_PI / 180.0f;
    _focal_px = (image_width * 0.5f) / tanf(hfov_rad * 0.5f);

    for (int i = 0; i < 20; ++i) _height_table[i] = 0.0f;
    _height_table[4]  = 0.25f;  // bottle
    _height_table[7]  = 0.30f;  // cat
    _height_table[8]  = 0.85f;  // chair
    _height_table[10] = 0.75f;  // diningtable
    _height_table[11] = 0.50f;  // dog
    _height_table[14] = 1.70f;  // person
    _height_table[15] = 0.40f;  // pottedplant
    _height_table[17] = 0.90f;  // sofa
    _height_table[19] = 0.50f;  // tvmonitor
}

float DepthEstimator::referenceHeight(uint8_t class_id) const {
    return (class_id < 20) ? _height_table[class_id] : 0.0f;
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
