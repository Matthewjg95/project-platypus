#pragma once
#include <stdint.h>
#include <stddef.h>
#include <math.h>

// ============================================================
// mesh.h — Core mesh data structures
//
// Vertex    : 6 bytes  (int16 x,y,z)
// Face      : 6 bytes  (uint16 a,b,c)
// Vec3f     : 12 bytes (float x,y,z) — used for face normals
//
// face_normals is allocated at load time (PSRAM).
// It stores one precomputed world-space unit normal per face,
// eliminating all per-frame cross products and sqrtf calls.
// ============================================================

// ---- Shared 3D float vector --------------------------------
// Defined here so both mesh.h and renderer.h can use it
// without conflicts.
struct Vec3f {
    float x, y, z;
};

inline Vec3f vec3_cross(Vec3f a, Vec3f b) {
    return { a.y*b.z - a.z*b.y,
             a.z*b.x - a.x*b.z,
             a.x*b.y - a.y*b.x };
}
inline float vec3_dot(Vec3f a, Vec3f b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}
inline Vec3f vec3_normalize(Vec3f v) {
    float len = sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
    if (len < 0.0001f) return {0, 0, 1};
    return {v.x/len, v.y/len, v.z/len};
}

// ---- Compact vertex (on-disk format) -----------------------
struct Vertex {
    int16_t x, y, z;
};

// ---- Face (triangle index triple) --------------------------
struct Face {
    uint16_t a, b, c;
};

// ---- Mesh container ----------------------------------------
struct Mesh {
    uint32_t vertex_count   = 0;
    uint32_t face_count     = 0;

    Vertex*  vertices       = nullptr;  // PSRAM
    Face*    faces          = nullptr;  // PSRAM
    Vec3f*   face_normals   = nullptr;  // PSRAM — precomputed at load time

    bool valid() const {
        return vertices     != nullptr &&
               faces        != nullptr &&
               face_normals != nullptr &&
               vertex_count  > 0       &&
               face_count    > 0;
    }

    // Releases all heap/PSRAM allocations.
    void free_data();
};

// ---- Projected (screen-space) vertex -----------------------
struct ProjectedVertex {
    float sx, sy;
    float z;        // camera-space Z
};

// ---- Transform state ---------------------------------------
struct RenderState {
    float rot_x = 0.0f;
    float rot_y = 0.0f;
    float rot_z = 0.0f;
    float scale = 1.0f;
    float cam_z = 5.0f;
    float fov   = 300.0f;
};

// ---- Auto-fit: compute RenderState for any mesh ------------
inline RenderState fit_to_canvas(const Mesh& mesh,
                                  int canvas_h,
                                  float fill_ratio = 0.7f) {
    RenderState s;
    s.fov = 300.0f;

    // Find longest half-axis in int16 units
    float extent = 1.0f;
    if (mesh.vertices && mesh.vertex_count > 0) {
        int16_t mn[3] = { 32767,  32767,  32767};
        int16_t mx[3] = {-32767, -32767, -32767};
        for (uint32_t i = 0; i < mesh.vertex_count; i++) {
            if (mesh.vertices[i].x < mn[0]) mn[0] = mesh.vertices[i].x;
            if (mesh.vertices[i].y < mn[1]) mn[1] = mesh.vertices[i].y;
            if (mesh.vertices[i].z < mn[2]) mn[2] = mesh.vertices[i].z;
            if (mesh.vertices[i].x > mx[0]) mx[0] = mesh.vertices[i].x;
            if (mesh.vertices[i].y > mx[1]) mx[1] = mesh.vertices[i].y;
            if (mesh.vertices[i].z > mx[2]) mx[2] = mesh.vertices[i].z;
        }
        float ex = (mx[0] - mn[0]) * 0.5f;
        float ey = (mx[1] - mn[1]) * 0.5f;
        float ez = (mx[2] - mn[2]) * 0.5f;
        extent = fmaxf(ex, fmaxf(ey, ez));
        if (extent < 1.0f) extent = 1.0f;
    }

    s.scale = 1.0f / extent;
    float target_px = (canvas_h * 0.5f) * fill_ratio;
    s.cam_z = (s.fov / target_px) * 1.2f;

    return s;
}