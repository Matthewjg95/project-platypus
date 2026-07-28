// scan_mesh_writer.cpp
#include "scan_mesh_writer.h"
#include <SD_MMC.h>
#include <math.h>
#include <string.h>

static inline File* asFile(void* p) { return reinterpret_cast<File*>(p); }

void ScanMeshWriter::quantisation(const float b[6], float center[3], float& scale) {
    center[0] = 0.5f * (b[0] + b[1]);
    center[1] = 0.5f * (b[2] + b[3]);
    center[2] = 0.5f * (b[4] + b[5]);
    float ex = b[1]-b[0], ey = b[3]-b[2], ez = b[5]-b[4];
    float half = 0.5f * fmaxf(ex, fmaxf(ey, ez));
    scale = (half > 1e-4f) ? ((float)TARGET_RANGE / half) : 1.0f;
}

void ScanMeshWriter::flush() {
    if (_buflen == 0) return;
    asFile(_file)->write(_buf, _buflen);
    _buflen = 0;
}

void ScanMeshWriter::bufWrite(const void* data, uint32_t n) {
    const uint8_t* p = (const uint8_t*)data;
    if (n >= sizeof(_buf)) { flush(); asFile(_file)->write(p, n); _pos += n; return; }
    if (_buflen + n > sizeof(_buf)) flush();
    memcpy(_buf + _buflen, p, n);
    _buflen += n; _pos += n;
}

void ScanMeshWriter::patchU32(uint32_t at, uint32_t value) {
    flush();
    File* f = asFile(_file);
    f->seek(at);
    f->write((const uint8_t*)&value, 4);
    f->seek(_pos);
}

bool ScanMeshWriter::begin(const char* path, const float center[3], float scale) {
    static File s_file;
    s_file = SD_MMC.open(path, FILE_WRITE);
    if (!s_file) return false;
    _file = &s_file;
    _buflen = 0; _pos = 0; _vcount = _fcount = 0;
    _center[0]=center[0]; _center[1]=center[1]; _center[2]=center[2];
    _scale = scale;

    uint32_t counts[2] = { 0, 0 };       // placeholders, patched in finish()
    bufWrite(counts, 8);
    return true;
}

void ScanMeshWriter::addVertexModel(float x, float y, float z) {
    // Scan world is Y-UP (floor=0, objects above); the renderer projects +Y
    // DOWN the screen. Mirror Y about the centre so the floor renders at the
    // bottom. (Without this the floor plate appeared upside-down above the
    // markers.) Label quantisation in the scanner mirrors identically.
    float qx = (x - _center[0]) * _scale;
    float qy = -(y - _center[1]) * _scale;
    float qz = (z - _center[2]) * _scale;
    if (qx >  32767.f) qx =  32767.f; if (qx < -32767.f) qx = -32767.f;
    if (qy >  32767.f) qy =  32767.f; if (qy < -32767.f) qy = -32767.f;
    if (qz >  32767.f) qz =  32767.f; if (qz < -32767.f) qz = -32767.f;
    int16_t v[3] = { (int16_t)lrintf(qx), (int16_t)lrintf(qy), (int16_t)lrintf(qz) };
    bufWrite(v, sizeof(v));
    ++_vcount;
}

void ScanMeshWriter::addFace(uint16_t a, uint16_t b, uint16_t c) {
    // The Y-mirror in addVertexModel flips triangle winding, which inverted
    // the renderer's load-time normals (faces lit from inside -> boxes looked
    // concave). Swap b/c here to restore outward winding after the mirror.
    uint16_t f[3] = { a, c, b };
    bufWrite(f, sizeof(f));
    ++_fcount;
}

bool ScanMeshWriter::finish() {
    flush();
    patchU32(0, _vcount);      // header[0] = vertex_count
    patchU32(4, _fcount);      // header[1] = face_count
    asFile(_file)->close();
    _file = nullptr;
    return true;
}
