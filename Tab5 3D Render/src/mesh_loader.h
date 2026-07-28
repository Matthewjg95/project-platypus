#pragma once
#include "mesh.h"
#include <Arduino.h>
#include <SD_MMC.h>

// ============================================================
// mesh_loader.h — Binary .mesh loader with normal precomputation
//
// After loading vertices and faces from SD card, computes one
// world-space unit normal per face and stores in face_normals.
// This eliminates all per-frame cross products and sqrtf calls
// in the renderer — the single highest-ROI precomputation.
//
// Memory (example: 14k faces):
//   vertices     =  5625 × 6  =  33 KB PSRAM
//   faces        = 14000 × 6  =  84 KB PSRAM
//   face_normals = 14000 × 12 = 168 KB PSRAM
//   Total                     = 285 KB PSRAM
// ============================================================

bool load_mesh(const char* path, Mesh& mesh);

// ============================================================
#ifdef MESH_LOADER_IMPL

#include <esp_heap_caps.h>

// Mesh::free_data — defined here (mesh.h is header-only)
void Mesh::free_data() {
    if (vertices)     { heap_caps_free(vertices);     vertices     = nullptr; }
    if (faces)        { heap_caps_free(faces);        faces        = nullptr; }
    if (face_normals) { heap_caps_free(face_normals); face_normals = nullptr; }
    vertex_count = face_count = 0;
}

bool load_mesh(const char* path, Mesh& mesh) {
    mesh.free_data();

    File f = SD_MMC.open(path, FILE_READ);
    if (!f) {
        Serial.printf("[mesh_loader] Cannot open: %s\n", path);
        return false;
    }

    // ---- Read header ----------------------------------------
    uint32_t header[2] = {0, 0};
    if (f.read(reinterpret_cast<uint8_t*>(header), 8) != 8) {
        Serial.println("[mesh_loader] Header read failed");
        f.close(); return false;
    }
    mesh.vertex_count = header[0];
    mesh.face_count   = header[1];

    Serial.printf("[mesh_loader] %s — verts:%u faces:%u\n",
                  path, mesh.vertex_count, mesh.face_count);

    // Sanity limits
    if (mesh.vertex_count == 0 || mesh.vertex_count > 65536 ||
        mesh.face_count   == 0 || mesh.face_count   > 131072) {
        Serial.println("[mesh_loader] Dimensions out of range");
        f.close(); return false;
    }

    // ---- Allocate in PSRAM ----------------------------------
    size_t vbytes = mesh.vertex_count * sizeof(Vertex);
    size_t fbytes = mesh.face_count   * sizeof(Face);
    size_t nbytes = mesh.face_count   * sizeof(Vec3f);

    mesh.vertices = reinterpret_cast<Vertex*>(
        heap_caps_malloc(vbytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    mesh.faces = reinterpret_cast<Face*>(
        heap_caps_malloc(fbytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    mesh.face_normals = reinterpret_cast<Vec3f*>(
        heap_caps_malloc(nbytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));

    if (!mesh.vertices || !mesh.faces || !mesh.face_normals) {
        Serial.printf("[mesh_loader] PSRAM alloc failed "
                      "(need %u + %u + %u bytes)\n",
                      vbytes, fbytes, nbytes);
        mesh.free_data(); f.close(); return false;
    }

    // ---- Read vertex block ----------------------------------
    if (f.read(reinterpret_cast<uint8_t*>(mesh.vertices), vbytes)
            != (int)vbytes) {
        Serial.println("[mesh_loader] Vertex read failed");
        mesh.free_data(); f.close(); return false;
    }

    // ---- Read face block ------------------------------------
    if (f.read(reinterpret_cast<uint8_t*>(mesh.faces), fbytes)
            != (int)fbytes) {
        Serial.println("[mesh_loader] Face read failed");
        mesh.free_data(); f.close(); return false;
    }

    f.close();

    // ---- Precompute face normals ----------------------------
    // Done once at load time. Eliminates per-frame cross product
    // + sqrtf for every face — ~2ms saved per frame at 14k faces.
    //
    // Normals are computed in int16 vertex space (before scale).
    // The renderer rotates them with the same matrix as vertices,
    // so scale cancels out and the result is correct.

    uint32_t degenerate = 0;

    for (uint32_t i = 0; i < mesh.face_count; i++) {
        const Vertex& va = mesh.vertices[mesh.faces[i].a];
        const Vertex& vb = mesh.vertices[mesh.faces[i].b];
        const Vertex& vc = mesh.vertices[mesh.faces[i].c];

        // Edge vectors in int16 space, promoted to float
        Vec3f e1 = { (float)(vb.x - va.x),
                     (float)(vb.y - va.y),
                     (float)(vb.z - va.z) };
        Vec3f e2 = { (float)(vc.x - va.x),
                     (float)(vc.y - va.y),
                     (float)(vc.z - va.z) };

        Vec3f n = vec3_cross(e1, e2);
        float len = sqrtf(n.x*n.x + n.y*n.y + n.z*n.z);

        if (len < 0.0001f) {
            // Degenerate triangle — use up vector as fallback
            mesh.face_normals[i] = {0.0f, 1.0f, 0.0f};
            degenerate++;
        } else {
            mesh.face_normals[i] = {n.x/len, n.y/len, n.z/len};
        }
    }

    Serial.printf("[mesh_loader] Loaded OK  "
                  "vbuf=%u  fbuf=%u  nbuf=%u bytes  "
                  "degenerate:%u\n",
                  vbytes, fbytes, nbytes, degenerate);
    return true;
}

#endif // MESH_LOADER_IMPL