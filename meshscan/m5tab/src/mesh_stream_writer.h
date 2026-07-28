// mesh_stream_writer.h - Incremental .mesh writer for Phase-2 meshes that are
// too large to assemble in RAM.
//
// Sections store their element count BEFORE the data, but a streamed producer
// doesn't know the count up front. So we write a placeholder count, stream the
// data through a 4KB buffer, then seek back and patch the real count. Vertices
// and faces are never held in a full in-RAM copy.
//
// Usage:
//   MeshStreamWriter w;
//   w.begin(path, room_id, ts, bounds);
//   w.beginVertices(); for (...) w.addVertex(x,y,z); w.endVertices();
//   w.beginFaces();    for (...) w.addFace(a,b,c);   w.endFaces();
//   w.writeObjects(objs, n);    // optional
//   w.finish();

#pragma once
#include <stdint.h>
#include "../../shared/mesh_format.h"

class MeshStreamWriter {
public:
    bool begin(const char* path, uint32_t room_id, uint32_t unix_ts, const float bounds[6]);

    bool beginVertices();
    void addVertex(float x, float y, float z);
    bool endVertices();

    bool beginFaces();
    void addFace(uint16_t a, uint16_t b, uint16_t c);
    bool endFaces();

    bool writeObjects(const MeshObject* objs, uint8_t count);   // optional
    bool finish();

    uint32_t vertexCount() const { return _vcount; }
    uint32_t faceCount()   const { return _fcount; }

private:
    void*    _file = nullptr;        // SD File* (opaque to keep header light)
    uint8_t  _buf[4096];
    uint32_t _buflen = 0;
    uint32_t _pos = 0;               // logical bytes written (buffered + flushed)

    uint32_t _vcount = 0, _fcount = 0;
    uint32_t _vcountPos = 0, _fcountPos = 0;
    bool     _normalsWritten = false;
    bool     _objectsWritten = false;

    void bufWrite(const void* data, uint32_t n);
    void flush();
    void patchU32(uint32_t at, uint32_t value);
    void writeNormalsSection();      // count 0; renderer recomputes per-face
};
