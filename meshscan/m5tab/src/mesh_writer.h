// mesh_writer.h - Serialize a MeshBuilder result to SD in the .mesh format.
//
// Writes strictly in 4KB chunks straight from the builder's PSRAM arrays, so we
// never allocate a second full-size copy of the mesh. Layout is defined in
// shared/mesh_format.h. The ESP32-S3 is little-endian and the format is
// little-endian, so multi-byte values are written as raw memory.

#pragma once
#include <stdint.h>
#include "mesh_builder.h"

class MeshWriter {
public:
    // path must be an absolute SD path, e.g. "/meshscan/buildings/home/kitchen.mesh".
    // Parent directories must already exist (project_manager handles that).
    static bool writeMesh(const char* path, const MeshBuilder& mb,
                          uint32_t room_id, uint32_t unix_timestamp);
};
