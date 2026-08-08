// scan_geometry.h - Assemble Phase-1 room geometry as plain float arrays.
//
// Deliberately uses local types (no Vertex/Face/Mesh) so it never collides with
// the renderer's mesh.h. Produces float vertices + uint16 faces (quantised to
// int16 later by ScanMeshWriter) plus a list of labelled objects the applet
// draws as 2D billboards over the rendered canvas (the renderer format has no
// object section).

#pragma once
#include <stdint.h>

struct ScanObject {
    char    label[24];
    float   cx, cy, cz;     // world-space centre (metres)
    uint8_t r, g, b;
};

class ScanGeometry {
public:
    bool begin(uint32_t max_vertices = 4096, uint32_t max_faces = 8192);
    void reset();

    void addBoxRoom(float cx, float cz, float width, float depth, float height);
    void addFloorPlate(float cx, float cz, float width, float depth,
                       float thickness = 0.06f);
    // Carved floor from an occupancy grid (visibility-carved rooms): top
    // surface as merged runs, and boundary edges extruded into WALLS rising
    // wall_h above the floor (both faces drawn — the renderer backface-culls
    // and rooms are viewed from inside and outside). wall_h = 0 gives the
    // old thin-skirt look. occ[gz*stride + gx] != 0 = floor cell; cell =
    // cell size in metres; (minx, minz) = world of cell (0,0)'s corner.
    void addFloorCells(const uint8_t* occ, int stride, int nx, int nz,
                       float minx, float minz, float cell,
                       float thickness = 0.06f, float wall_h = 0.45f);
    // Flat arrow at the scan origin (0,0,0) pointing +Z — marks where the
    // scan began and the direction faced (the corner, per convention).
    void addOriginArrow(float half_w = 0.18f, float len = 0.45f);
    void addObjectMarker(const char* label, float cx, float cy, float cz,
                         float hx, float hy, float hz,
                         uint8_t r, uint8_t g, uint8_t b);

    uint32_t        vertexCount() const { return _vcount; }
    uint32_t        faceCount()   const { return _fcount; }
    const float*    vertices()    const { return _vx; }   // [vcount*3]
    const uint16_t* faces()       const { return _fi; }   // [fcount*3]
    const float*    bounds()      const { return _bounds; }
    uint8_t            objectCount() const { return _ocount; }
    const ScanObject*  objects()     const { return _objs; }

    static const int MAX_OBJECTS = 64;

private:
    float*    _vx = nullptr;  uint32_t _vcap = 0, _vcount = 0;
    uint16_t* _fi = nullptr;  uint32_t _fcap = 0, _fcount = 0;
    float     _bounds[6];
    ScanObject _objs[MAX_OBJECTS];
    uint8_t   _ocount = 0;

    uint32_t addVertex(float x, float y, float z);
    void addOrientedTri(uint32_t a, uint32_t b, uint32_t c,
                        float rx, float ry, float rz, bool towardRef);
    void growBounds(float x, float y, float z);
};
