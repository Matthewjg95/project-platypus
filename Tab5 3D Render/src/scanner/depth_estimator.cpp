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

    // Width references for wide furniture — bbox width varies with viewing
    // angle less than you'd fear for these (frontal viewing dominates) and
    // is far less corrupted by partial vertical framing than height.
    for (int i = 0; i < 20; ++i) _width_table[i] = 0.0f;
    _width_table[10] = 1.40f;   // diningtable
    _width_table[17] = 1.90f;   // sofa
    _width_table[19] = 0.55f;   // tvmonitor
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

    // A box touching the left/right frame edge is truncated: its width reads
    // short, which would range the object FARTHER than it is. Fall back to
    // height ranging (unaffected by horizontal clipping) and skip the aspect
    // check (the truncated aspect is meaningless).
    bool clipped = (bbox_x <= 1) || ((int)bbox_x + (int)bbox_w >= _w - 2);

    float ref_w = (class_id < 20) ? _width_table[class_id] : 0.0f;
    bool  use_w = ref_w > 0.0f && bbox_w > 0 && !clipped;

    // Tiny boxes: at MIN_BOX_PX the per-pixel quantisation error alone is
    // >10% of the range — anything smaller is noise, not measurement.
    if ((use_w ? bbox_w : bbox_h_px) < MIN_BOX_PX) { e.valid = false; return e; }

    // Aspect sanity for classes with both size references: a "sofa" box
    // taller than it is wide is a bad detection or a heavy occlusion, and
    // ranging off either dimension of it produces garbage.
    if (ref_w > 0.0f && !clipped) {
        float expect = ref_w / e.real_height;
        float got    = (float)bbox_w / (float)bbox_h_px;
        float r      = got / expect;
        if (r > ASPECT_TOL || r < 1.0f / ASPECT_TOL) { e.valid = false; return e; }
    }

    // wide-furniture classes range by WIDTH (the stabler scale cue);
    // everything else by height
    if (use_w)
        e.distance = (ref_w * _focal_px) / (float)bbox_w;
    else
        e.distance = (e.real_height * _focal_px) / (float)bbox_h_px;

    // Indoor scans: a range beyond MAX_RANGE_M is a mis-sized box, not a
    // 12-metre living room.
    if (e.distance > MAX_RANGE_M) { e.valid = false; return e; }

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
