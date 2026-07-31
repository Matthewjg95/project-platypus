// ss_export.h - ShadowScan file output.
//
//   /shadowscan/<name>.stl    binary STL, millimetres (slicer-ready)
//   /shadowscan/<name>.obj    OBJ triangle soup (optional)
//   /models/<name>.mesh       renderer-native mesh via ScanMeshWriter, so a
//                             scan shows up in the 3D Viewer applet's list
//
// All writes go to SD_MMC (mounted by main.cpp).

#pragma once
#include <vector>
#include "ss_geometry.h"

namespace ss {

// Ensures /shadowscan exists and returns a free base name (shadow_001, ...).
// Returns false when SD isn't available.
bool next_export_name(char* out, size_t out_len);

bool export_stl(const char* base_name, const std::vector<Tri>& tris);
bool export_obj(const char* base_name, const std::vector<Tri>& tris);
bool export_viewer_mesh(const char* base_name, const std::vector<Tri>& tris);

}  // namespace ss
