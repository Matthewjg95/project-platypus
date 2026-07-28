// surface_recon.h - Greedy k-nearest-neighbour triangulation of a sparse point
// cloud into a triangle surface.
//
// For each point we find its k nearest neighbours and form triangles from
// neighbour pairs whose three edges are all under a length threshold,
// de-duplicating shared triangles. Faces are streamed to a sink callback so the
// caller (mesh_stream_writer) can write them to SD incrementally without ever
// holding the full face array in RAM.
//
// This is a pragmatic Phase-2 surface: it produces real local geometry but is
// not guaranteed watertight or manifold. TODOs mark the upgrades (oriented
// normals, ball-pivoting, hole filling, a spatial grid for kNN).

#pragma once
#include <stdint.h>
#include "point_cloud.h"

namespace cv {

typedef void (*FaceSink)(uint16_t a, uint16_t b, uint16_t c, void* user);

struct SurfaceReconParams {
    float max_edge = 0.5f;   // metres; reject triangles with any longer edge
    int   k        = 8;      // neighbours considered per point
    uint32_t max_faces = 20000;
};

// Triangulate `pc`, calling `sink` once per accepted face. Returns face count.
uint32_t greedy_triangulate(const PointCloud& pc, const SurfaceReconParams& params,
                            FaceSink sink, void* user);

} // namespace cv
