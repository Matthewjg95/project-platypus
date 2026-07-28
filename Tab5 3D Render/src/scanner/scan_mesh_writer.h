// scan_mesh_writer.h - Write the renderer's native .mesh format from MeshScan.
//
// Renderer format (see ../mesh_loader.h), little-endian:
//   uint32 vertex_count
//   uint32 face_count
//   int16  x,y,z   * vertex_count       (quantised model space)
//   uint16 a,b,c   * face_count
//
// Model-space metres are quantised to int16 exactly like tools/stl_to_mesh.py:
// centre on the bounds midpoint, scale the longest half-extent to TARGET_RANGE.
// Counts come first but a streamed producer doesn't know them up front, so we
// write placeholder counts, stream the data through a 4KB buffer, then seek back
// and patch the real counts. Nothing is held in a second full-size RAM copy.
//
// Writes to SD_MMC (the renderer's storage), not SPI SD.

#pragma once
#include <stdint.h>
#include <stddef.h>

class ScanMeshWriter {
public:
    static const int TARGET_RANGE = 30000;   // matches stl_to_mesh.py / mesh.h

    // Compute quantisation from a model-space AABB (xmin,xmax,ymin,ymax,zmin,zmax).
    static void quantisation(const float bounds[6], float center[3], float& scale);

    // center/scale map metres -> int16: q = clip((v-center)*scale, -32767, 32767)
    bool begin(const char* path, const float center[3], float scale);

    void addVertexModel(float x, float y, float z);   // quantises internally
    void addFace(uint16_t a, uint16_t b, uint16_t c);
    bool finish();

    uint32_t vertexCount() const { return _vcount; }
    uint32_t faceCount()   const { return _fcount; }

private:
    void*    _file = nullptr;      // SD_MMC File*
    uint8_t  _buf[4096];
    uint32_t _buflen = 0;
    uint32_t _pos = 0;
    uint32_t _vcount = 0, _fcount = 0;
    float    _center[3] = {0,0,0};
    float    _scale = 1.0f;

    void bufWrite(const void* data, uint32_t n);
    void flush();
    void patchU32(uint32_t at, uint32_t value);
};
