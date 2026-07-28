// mesh_writer.cpp
#include "mesh_writer.h"
#include <SD.h>
#include <string.h>

static const size_t CHUNK = 4096;

static bool writeChunked(File& f, const void* data, size_t n) {
    const uint8_t* p = (const uint8_t*)data;
    while (n > 0) {
        size_t take = n < CHUNK ? n : CHUNK;
        if (f.write(p, take) != take) return false;
        p += take; n -= take;
    }
    return true;
}

template <typename T>
static bool writeLE(File& f, T v) {           // ESP32-S3 is little-endian
    return f.write((const uint8_t*)&v, sizeof(T)) == sizeof(T);
}

bool MeshWriter::writeMesh(const char* path, const MeshBuilder& mb,
                           uint32_t room_id, uint32_t unix_timestamp) {
    File f = SD.open(path, FILE_WRITE);
    if (!f) return false;

    bool ok = true;

    // ---- HEADER (40 bytes: 16 fixed + 24 bounds) ----
    uint8_t hdr[16];
    memset(hdr, 0, sizeof(hdr));
    hdr[0]=MESH_MAGIC0; hdr[1]=MESH_MAGIC1; hdr[2]=MESH_MAGIC2; hdr[3]=MESH_MAGIC3;
    hdr[4]=(uint8_t)MESH_VERSION;
    hdr[5]=0;                 // flags
    // hdr[6..7] = pad (0)
    memcpy(&hdr[8],  &room_id, 4);
    memcpy(&hdr[12], &unix_timestamp, 4);
    ok &= (f.write(hdr, 16) == 16);
    ok &= writeChunked(f, mb.bounds(), sizeof(float) * 6);   // bounds[6]

    // ---- VERTICES ----
    ok &= writeLE<uint32_t>(f, mb.vertexCount());
    ok &= writeChunked(f, mb.vertices(), sizeof(float) * mb.vertexCount() * 3);

    // ---- FACES ----
    ok &= writeLE<uint32_t>(f, mb.faceCount());
    ok &= writeChunked(f, mb.faces(), sizeof(uint16_t) * mb.faceCount() * 3);

    // ---- NORMALS (per-vertex int8) ----
    ok &= writeLE<uint32_t>(f, mb.vertexCount());
    ok &= writeChunked(f, mb.normals(), sizeof(int8_t) * mb.vertexCount() * 3);

    // ---- OBJECTS ----
    ok &= writeLE<uint8_t>(f, mb.objectCount());
    const MeshObject* objs = mb.objects();
    for (uint8_t i = 0; i < mb.objectCount() && ok; ++i) {
        const MeshObject& o = objs[i];
        uint8_t len = (uint8_t)strnlen(o.label, MESH_MAX_LABEL);
        ok &= writeLE<uint8_t>(f, len);
        ok &= (f.write((const uint8_t*)o.label, len) == len);
        ok &= writeChunked(f, o.bbox, sizeof(float) * 6);
        uint8_t rgb[3] = { o.r, o.g, o.b };
        ok &= (f.write(rgb, 3) == 3);
    }

    // ---- EOF ----
    uint8_t eof[4] = { MESH_EOF0, MESH_EOF1, MESH_EOF2, MESH_EOF3 };
    ok &= (f.write(eof, 4) == 4);

    f.close();
    return ok;
}
