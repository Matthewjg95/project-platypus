// mesh_renderer.cpp
#include "mesh_renderer.h"
#include "../lib/MeshRenderer/MeshRenderer.h"
#include <SD.h>
#include <esp_heap_caps.h>
#include <math.h>
#include <string.h>

// Define to pass billboard positions in raw metres instead of scaled int16
// space (see lib/MeshRenderer/README.md).
// #define MESH_BILLBOARD_IN_METRES

static void* psram_alloc(size_t n) {
    void* p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM);
    return p ? p : malloc(n);
}

template <typename T>
static bool readLE(File& f, T& v) {
    return f.read((uint8_t*)&v, sizeof(T)) == (int)sizeof(T);
}

void MeshRenderer::unload() {
    free(_mesh.vertices);     _mesh.vertices = nullptr;
    free(_mesh.faces);        _mesh.faces = nullptr;
    free(_mesh.face_normals); _mesh.face_normals = nullptr;
    _mesh.vertex_count = _mesh.face_count = 0;
}

bool MeshRenderer::loadFromFile(const char* path) {
    File f = SD.open(path, FILE_READ);
    if (!f) return false;
    unload();

    bool ok = true;

    // ---- HEADER ----
    char magic[4]; ok &= (f.read((uint8_t*)magic, 4) == 4);
    if (!ok || memcmp(magic, "MESH", 4) != 0) { f.close(); return false; }
    uint8_t version, flags; uint16_t pad; uint32_t room_id, ts;
    ok &= readLE(f, version) && readLE(f, flags) && readLE(f, pad);
    if (version != MESH_VERSION) { f.close(); return false; }
    ok &= readLE(f, room_id) && readLE(f, ts);
    float bounds[6];
    ok &= (f.read((uint8_t*)bounds, sizeof(bounds)) == (int)sizeof(bounds));
    if (!ok) { f.close(); return false; }

    // Uniform scale: centre on bounds midpoint, map largest half-extent to range.
    float cx = 0.5f*(bounds[0]+bounds[1]);
    float cy = 0.5f*(bounds[2]+bounds[3]);
    float cz = 0.5f*(bounds[4]+bounds[5]);
    float ex = bounds[1]-bounds[0], ey = bounds[3]-bounds[2], ez = bounds[5]-bounds[4];
    float maxdim = fmaxf(ex, fmaxf(ey, ez));
    float halfext = 0.5f * maxdim;
    float scale = (halfext > 1e-4f) ? ((float)MESH_INT16_RANGE / halfext) : 1.0f;

    // ---- VERTICES (float -> int16) ----
    uint32_t vcount; ok &= readLE(f, vcount);
    if (!ok || vcount == 0 || vcount > 65535) { f.close(); return false; }
    _mesh.vertices = (Vertex*)psram_alloc(sizeof(Vertex) * vcount);
    if (!_mesh.vertices) { f.close(); return false; }
    _mesh.vertex_count = vcount;
    // Stream-convert in blocks so we never hold the full float array.
    const uint32_t BLK = 256;
    float tmp[BLK * 3];
    for (uint32_t base = 0; base < vcount && ok; base += BLK) {
        uint32_t n = (vcount - base < BLK) ? (vcount - base) : BLK;
        ok &= (f.read((uint8_t*)tmp, sizeof(float)*n*3) == (int)(sizeof(float)*n*3));
        for (uint32_t i = 0; i < n; ++i) {
            float x = (tmp[i*3+0]-cx)*scale, y = (tmp[i*3+1]-cy)*scale, z = (tmp[i*3+2]-cz)*scale;
            Vertex& V = _mesh.vertices[base+i];
            V.x = (int16_t)lrintf(fmaxf(-32767.f, fminf(32767.f, x)));
            V.y = (int16_t)lrintf(fmaxf(-32767.f, fminf(32767.f, y)));
            V.z = (int16_t)lrintf(fmaxf(-32767.f, fminf(32767.f, z)));
        }
    }

    // ---- FACES (uint16, copied verbatim) ----
    uint32_t fcount; ok &= readLE(f, fcount);
    if (ok && fcount > 0) {
        _mesh.faces = (Face*)psram_alloc(sizeof(Face) * fcount);
        if (!_mesh.faces) { f.close(); unload(); return false; }
        _mesh.face_count = fcount;
        ok &= (f.read((uint8_t*)_mesh.faces, sizeof(Face)*fcount) == (int)(sizeof(Face)*fcount));
    }

    // ---- NORMALS (per-vertex int8) — skipped; we recompute per-face. ----
    uint32_t ncount; if (ok && readLE(f, ncount)) f.seek(f.position() + (size_t)ncount * 3);

    // ---- OBJECTS -> billboards ----
    uint8_t ocount = 0;
    if (ok) readLE(f, ocount);
    for (uint8_t i = 0; i < ocount && ok; ++i) {
        uint8_t len = 0; readLE(f, len);
        char label[MESH_MAX_LABEL + 1] = {0};
        uint8_t keep = len > MESH_MAX_LABEL ? MESH_MAX_LABEL : len;
        f.read((uint8_t*)label, keep); label[keep] = '\0';
        if (len > keep) f.seek(f.position() + (len - keep));   // skip overflow
        float bb[6]; f.read((uint8_t*)bb, sizeof(bb));
        uint8_t rgb[3]; f.read(rgb, 3);
        float ox = 0.5f*(bb[0]+bb[1]), oy = 0.5f*(bb[2]+bb[3]), oz = 0.5f*(bb[4]+bb[5]);
#ifndef MESH_BILLBOARD_IN_METRES
        ox = (ox-cx)*scale; oy = (oy-cy)*scale; oz = (oz-cz)*scale;
#endif
        uint32_t color = ((uint32_t)rgb[0] << 16) | ((uint32_t)rgb[1] << 8) | rgb[2];
        renderer_add_billboard(label, ox, oy, oz, color);
    }

    f.close();
    if (!ok) { unload(); return false; }

    computeFaceNormals();
    renderer_load_mesh(&_mesh);
    return true;
}

void MeshRenderer::computeFaceNormals() {
    if (_mesh.face_count == 0) return;
    _mesh.face_normals = (Vec3f*)psram_alloc(sizeof(Vec3f) * _mesh.face_count);
    if (!_mesh.face_normals) return;
    for (uint32_t i = 0; i < _mesh.face_count; ++i) {
        const Vertex& A = _mesh.vertices[_mesh.faces[i].a];
        const Vertex& B = _mesh.vertices[_mesh.faces[i].b];
        const Vertex& C = _mesh.vertices[_mesh.faces[i].c];
        float ux=B.x-A.x, uy=B.y-A.y, uz=B.z-A.z;
        float vx=C.x-A.x, vy=C.y-A.y, vz=C.z-A.z;
        float nx=uy*vz-uz*vy, ny=uz*vx-ux*vz, nz=ux*vy-uy*vx;
        float len = sqrtf(nx*nx+ny*ny+nz*nz);
        if (len < 1e-6f) { _mesh.face_normals[i] = {0,0,1}; continue; }
        _mesh.face_normals[i] = { nx/len, ny/len, nz/len };
    }
}

void MeshRenderer::setCamera(float az, float el, float dist) {
    _az = az; _el = el; _dist = dist;
}
void MeshRenderer::orbit(float daz, float del) {
    _az += daz; _el += del;
    const float lim = 1.5533f;   // ~89 deg
    if (_el >  lim) _el =  lim;
    if (_el < -lim) _el = -lim;
}
void MeshRenderer::zoom(float dd) {
    _dist += dd;
    float lo = (float)MESH_INT16_RANGE * 0.2f, hi = (float)MESH_INT16_RANGE * 6.0f;
    if (_dist < lo) _dist = lo;
    if (_dist > hi) _dist = hi;
}

void MeshRenderer::draw(uint16_t* fb) {
    renderer_set_camera(_az, _el, _dist);
    renderer_draw_frame(fb);
}
