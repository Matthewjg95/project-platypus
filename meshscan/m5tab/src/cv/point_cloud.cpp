// point_cloud.cpp
#include "point_cloud.h"
#include <stdlib.h>

#if defined(ESP_PLATFORM) || defined(ARDUINO)
  #include <esp_heap_caps.h>
  static void* pc_alloc(size_t n) {
      void* p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM);
      return p ? p : malloc(n);
  }
#else
  static void* pc_alloc(size_t n) { return malloc(n); }
#endif

namespace cv {

bool PointCloud::begin(uint32_t capacity) {
    cap = capacity;
    xyz = (float*)pc_alloc(sizeof(float) * cap * 3);
    rgb = (uint8_t*)pc_alloc(sizeof(uint8_t) * cap * 3);
    if (!xyz || !rgb) return false;
    clear();
    return true;
}

void PointCloud::clear() {
    count = 0;
    bounds[0] = bounds[2] = bounds[4] = 1e30f;
    bounds[1] = bounds[3] = bounds[5] = -1e30f;
}

void PointCloud::add(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b) {
    if (count >= cap) return;
    uint32_t i = count++;
    xyz[i*3+0] = x; xyz[i*3+1] = y; xyz[i*3+2] = z;
    rgb[i*3+0] = r; rgb[i*3+1] = g; rgb[i*3+2] = b;
    if (x < bounds[0]) bounds[0] = x;  if (x > bounds[1]) bounds[1] = x;
    if (y < bounds[2]) bounds[2] = y;  if (y > bounds[3]) bounds[3] = y;
    if (z < bounds[4]) bounds[4] = z;  if (z > bounds[5]) bounds[5] = z;
}

void PointCloud::freeMem() {
    free(xyz); free(rgb);
    xyz = nullptr; rgb = nullptr; cap = count = 0;
}

} // namespace cv
