// mesh_builder.h - Assemble an in-RAM mesh (float vertices, uint16 faces,
// per-vertex int8 normals, object list) from the room box + placed objects.
//
// This is the float-space staging representation written to disk by mesh_writer
// in the .mesh format. mesh_renderer later loads that file back as the int16
// render Mesh. Phase 1 geometry is tiny (a box plus a marker box per object),
// so it fits comfortably in RAM; capacities are bounded at begin().

#pragma once
#include <stdint.h>
#include "../../shared/mesh_format.h"
#include "room_fitter.h"

class MeshBuilder {
public:
    bool begin(uint32_t max_vertices = 4096, uint32_t max_faces = 8192);
    void reset();

    // Add the rectangular room shell (6 faces) with inward-facing normals.
    void addBoxRoom(const RoomBox& room);

    // Add a small box marker for a detected object and record it as a labelled
    // MeshObject (used by the renderer for billboards). half_* are half-extents.
    void addObjectMarker(const char* label,
                         float cx, float cy, float cz,
                         float half_x, float half_y, float half_z,
                         uint8_t r, uint8_t g, uint8_t b);

    // Finalise: compute per-vertex normals. Call before reading out / writing.
    void finalize();

    // ----- accessors for mesh_writer -----
    uint32_t       vertexCount() const { return _vcount; }
    uint32_t       faceCount()   const { return _fcount; }
    const float*   vertices()    const { return _vx; }     // [vcount*3]
    const uint16_t*faces()       const { return _fi; }     // [fcount*3]
    const int8_t*  normals()     const { return _nz; }     // [vcount*3]
    const float*   bounds()      const { return _bounds; } // [6]
    uint8_t           objectCount() const { return _ocount; }
    const MeshObject* objects()     const { return _objs; }

private:
    float*    _vx = nullptr;    uint32_t _vcap = 0, _vcount = 0;
    uint16_t* _fi = nullptr;    uint32_t _fcap = 0, _fcount = 0;
    int8_t*   _nz = nullptr;    // per-vertex normals, allocated with _vx
    float     _bounds[6];       // xmin,xmax,ymin,ymax,zmin,zmax
    MeshObject _objs[MESH_MAX_OBJECTS];
    uint8_t   _ocount = 0;

    uint32_t addVertex(float x, float y, float z);
    // Add a triangle, flipping winding so its normal points toward (towardRef)
    // or away from the reference point.
    void addOrientedTri(uint32_t a, uint32_t b, uint32_t c,
                        float rx, float ry, float rz, bool towardRef);
    void growBounds(float x, float y, float z);
};
