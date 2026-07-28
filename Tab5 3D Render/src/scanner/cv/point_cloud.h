// point_cloud.h - PSRAM-backed accumulating 3D point cloud for Phase-2 SfM.
//
// Triangulated points from each frame pair are added here (already transformed
// into the world frame by slam_pipeline). Tracks bounds so the mesh writer can
// emit a correct header. Portable C++ except the allocator, which prefers PSRAM
// when ESP-IDF is present and falls back to malloc otherwise.

#pragma once
#include <stdint.h>

namespace cv {

struct PointCloud {
    uint32_t count = 0;
    uint32_t cap   = 0;
    float*   xyz = nullptr;     // cap*3
    uint8_t* rgb = nullptr;     // cap*3
    float    bounds[6];         // xmin,xmax,ymin,ymax,zmin,zmax

    bool begin(uint32_t capacity);
    void clear();
    void add(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b);
    void freeMem();
};

} // namespace cv
