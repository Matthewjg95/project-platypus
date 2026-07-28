// mesh_stream_writer.cpp
#include "mesh_stream_writer.h"
#include <SD.h>
#include <string.h>

// Stash the File object behind the void* in the header.
static inline File* asFile(void* p) { return reinterpret_cast<File*>(p); }

void MeshStreamWriter::flush() {
    if (_buflen == 0) return;
    asFile(_file)->write(_buf, _buflen);
    _buflen = 0;
}

void MeshStreamWriter::bufWrite(const void* data, uint32_t n) {
    const uint8_t* p = (const uint8_t*)data;
    if (n >= sizeof(_buf)) {            // large block: flush then write straight
        flush();
        asFile(_file)->write(p, n);
        _pos += n;
        return;
    }
    if (_buflen + n > sizeof(_buf)) flush();
    memcpy(_buf + _buflen, p, n);
    _buflen += n;
    _pos += n;
}

void MeshStreamWriter::patchU32(uint32_t at, uint32_t value) {
    flush();                            // make file end == _pos
    File* f = asFile(_file);
    f->seek(at);
    f->write((const uint8_t*)&value, 4);
    f->seek(_pos);                      // back to append position
}

bool MeshStreamWriter::begin(const char* path, uint32_t room_id, uint32_t ts,
                             const float bounds[6]) {
    static File s_file;                 // single active writer at a time
    s_file = SD.open(path, FILE_WRITE);
    if (!s_file) return false;
    _file = &s_file;
    _buflen = 0; _pos = 0; _vcount = _fcount = 0; _normalsWritten = false;

    uint8_t hdr[16];
    memset(hdr, 0, sizeof(hdr));
    hdr[0]=MESH_MAGIC0; hdr[1]=MESH_MAGIC1; hdr[2]=MESH_MAGIC2; hdr[3]=MESH_MAGIC3;
    hdr[4]=(uint8_t)MESH_VERSION;
    memcpy(&hdr[8],  &room_id, 4);
    memcpy(&hdr[12], &ts, 4);
    bufWrite(hdr, 16);
    bufWrite(bounds, sizeof(float)*6);
    return true;
}

bool MeshStreamWriter::beginVertices() {
    _vcountPos = _pos;                  // remember where the count lives
    uint32_t placeholder = 0;
    bufWrite(&placeholder, 4);
    _vcount = 0;
    return true;
}
void MeshStreamWriter::addVertex(float x, float y, float z) {
    float v[3] = { x, y, z };
    bufWrite(v, sizeof(v));
    ++_vcount;
}
bool MeshStreamWriter::endVertices() {
    patchU32(_vcountPos, _vcount);
    return true;
}

bool MeshStreamWriter::beginFaces() {
    _fcountPos = _pos;
    uint32_t placeholder = 0;
    bufWrite(&placeholder, 4);
    _fcount = 0;
    return true;
}
void MeshStreamWriter::addFace(uint16_t a, uint16_t b, uint16_t c) {
    uint16_t f[3] = { a, b, c };
    bufWrite(f, sizeof(f));
    ++_fcount;
}
bool MeshStreamWriter::endFaces() {
    patchU32(_fcountPos, _fcount);
    return true;
}

void MeshStreamWriter::writeNormalsSection() {
    uint32_t zero = 0;                  // 0 per-vertex normals; renderer derives
    bufWrite(&zero, 4);
    _normalsWritten = true;
}

bool MeshStreamWriter::writeObjects(const MeshObject* objs, uint8_t count) {
    if (!_normalsWritten) writeNormalsSection();
    bufWrite(&count, 1);
    _objectsWritten = true;
    for (uint8_t i = 0; i < count; ++i) {
        const MeshObject& o = objs[i];
        uint8_t len = (uint8_t)strnlen(o.label, MESH_MAX_LABEL);
        bufWrite(&len, 1);
        bufWrite(o.label, len);
        bufWrite(o.bbox, sizeof(float)*6);
        uint8_t rgb[3] = { o.r, o.g, o.b };
        bufWrite(rgb, 3);
    }
    return true;
}

bool MeshStreamWriter::finish() {
    if (!_normalsWritten) writeNormalsSection();
    if (!_objectsWritten) {                 // emit an empty OBJECTS section
        uint8_t zero = 0;
        bufWrite(&zero, 1);
        _objectsWritten = true;
    }
    uint8_t eof[4] = { MESH_EOF0, MESH_EOF1, MESH_EOF2, MESH_EOF3 };
    bufWrite(eof, 4);
    flush();
    asFile(_file)->close();
    _file = nullptr;
    return true;
}
