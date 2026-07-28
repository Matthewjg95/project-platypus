// mesh_builder.cpp
#include "mesh_builder.h"
#include <esp_heap_caps.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

static void* psram_alloc(size_t n) {
    void* p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM);
    return p ? p : malloc(n);
}

bool MeshBuilder::begin(uint32_t max_vertices, uint32_t max_faces) {
    _vcap = max_vertices; _fcap = max_faces;
    _vx = (float*)   psram_alloc(sizeof(float)    * _vcap * 3);
    _nz = (int8_t*)  psram_alloc(sizeof(int8_t)   * _vcap * 3);
    _fi = (uint16_t*)psram_alloc(sizeof(uint16_t) * _fcap * 3);
    if (!_vx || !_nz || !_fi) return false;
    reset();
    return true;
}

void MeshBuilder::reset() {
    _vcount = _fcount = 0;
    _ocount = 0;
    _bounds[0] = _bounds[2] = _bounds[4] = 1e30f;
    _bounds[1] = _bounds[3] = _bounds[5] = -1e30f;
}

void MeshBuilder::growBounds(float x, float y, float z) {
    if (x < _bounds[0]) _bounds[0] = x;  if (x > _bounds[1]) _bounds[1] = x;
    if (y < _bounds[2]) _bounds[2] = y;  if (y > _bounds[3]) _bounds[3] = y;
    if (z < _bounds[4]) _bounds[4] = z;  if (z > _bounds[5]) _bounds[5] = z;
}

uint32_t MeshBuilder::addVertex(float x, float y, float z) {
    if (_vcount >= _vcap) return _vcount - 1;   // clamp; caller bounded capacity
    uint32_t i = _vcount++;
    _vx[i*3+0] = x; _vx[i*3+1] = y; _vx[i*3+2] = z;
    growBounds(x, y, z);
    return i;
}

void MeshBuilder::addOrientedTri(uint32_t a, uint32_t b, uint32_t c,
                                 float rx, float ry, float rz, bool towardRef) {
    if (_fcount >= _fcap) return;
    const float* A = &_vx[a*3]; const float* B = &_vx[b*3]; const float* C = &_vx[c*3];
    float ux = B[0]-A[0], uy = B[1]-A[1], uz = B[2]-A[2];
    float vx = C[0]-A[0], vy = C[1]-A[1], vz = C[2]-A[2];
    float nx = uy*vz - uz*vy, ny = uz*vx - ux*vz, nz = ux*vy - uy*vx;
    float mx = (A[0]+B[0]+C[0])/3.0f, my=(A[1]+B[1]+C[1])/3.0f, mz=(A[2]+B[2]+C[2])/3.0f;
    float dx = rx-mx, dy = ry-my, dz = rz-mz;     // mid -> reference point
    float dot = nx*dx + ny*dy + nz*dz;
    bool pointsToRef = dot > 0.0f;
    if (pointsToRef != towardRef) { uint32_t t = b; b = c; c = t; }  // flip winding
    uint32_t f = _fcount++;
    _fi[f*3+0] = (uint16_t)a; _fi[f*3+1] = (uint16_t)b; _fi[f*3+2] = (uint16_t)c;
}

void MeshBuilder::addBoxRoom(const RoomBox& room) {
    float x0 = room.cx - room.width*0.5f, x1 = room.cx + room.width*0.5f;
    float z0 = room.cz - room.depth*0.5f, z1 = room.cz + room.depth*0.5f;
    float y0 = 0.0f, y1 = room.height;
    float ccx = room.cx, ccy = room.height*0.5f, ccz = room.cz;  // interior ref

    uint32_t v0 = addVertex(x0,y0,z0), v1 = addVertex(x1,y0,z0),
             v2 = addVertex(x1,y0,z1), v3 = addVertex(x0,y0,z1),
             v4 = addVertex(x0,y1,z0), v5 = addVertex(x1,y1,z0),
             v6 = addVertex(x1,y1,z1), v7 = addVertex(x0,y1,z1);

    // Inside-of-room: every face normal points toward the room centre.
    // floor
    addOrientedTri(v0,v1,v2, ccx,ccy,ccz, true); addOrientedTri(v0,v2,v3, ccx,ccy,ccz, true);
    // ceiling
    addOrientedTri(v4,v5,v6, ccx,ccy,ccz, true); addOrientedTri(v4,v6,v7, ccx,ccy,ccz, true);
    // wall z0
    addOrientedTri(v0,v1,v5, ccx,ccy,ccz, true); addOrientedTri(v0,v5,v4, ccx,ccy,ccz, true);
    // wall z1
    addOrientedTri(v3,v2,v6, ccx,ccy,ccz, true); addOrientedTri(v3,v6,v7, ccx,ccy,ccz, true);
    // wall x0
    addOrientedTri(v0,v3,v7, ccx,ccy,ccz, true); addOrientedTri(v0,v7,v4, ccx,ccy,ccz, true);
    // wall x1
    addOrientedTri(v1,v2,v6, ccx,ccy,ccz, true); addOrientedTri(v1,v6,v5, ccx,ccy,ccz, true);
}

void MeshBuilder::addObjectMarker(const char* label,
                                  float cx, float cy, float cz,
                                  float hx, float hy, float hz,
                                  uint8_t r, uint8_t g, uint8_t b) {
    float x0=cx-hx, x1=cx+hx, y0=cy-hy, y1=cy+hy, z0=cz-hz, z1=cz+hz;
    uint32_t v0=addVertex(x0,y0,z0), v1=addVertex(x1,y0,z0),
             v2=addVertex(x1,y0,z1), v3=addVertex(x0,y0,z1),
             v4=addVertex(x0,y1,z0), v5=addVertex(x1,y1,z0),
             v6=addVertex(x1,y1,z1), v7=addVertex(x0,y1,z1);
    // Marker boxes are viewed from outside -> normals point away from centre.
    addOrientedTri(v0,v1,v2, cx,cy,cz, false); addOrientedTri(v0,v2,v3, cx,cy,cz, false);
    addOrientedTri(v4,v5,v6, cx,cy,cz, false); addOrientedTri(v4,v6,v7, cx,cy,cz, false);
    addOrientedTri(v0,v1,v5, cx,cy,cz, false); addOrientedTri(v0,v5,v4, cx,cy,cz, false);
    addOrientedTri(v3,v2,v6, cx,cy,cz, false); addOrientedTri(v3,v6,v7, cx,cy,cz, false);
    addOrientedTri(v0,v3,v7, cx,cy,cz, false); addOrientedTri(v0,v7,v4, cx,cy,cz, false);
    addOrientedTri(v1,v2,v6, cx,cy,cz, false); addOrientedTri(v1,v6,v5, cx,cy,cz, false);

    if (_ocount < MESH_MAX_OBJECTS) {
        MeshObject& o = _objs[_ocount++];
        strncpy(o.label, label ? label : "?", MESH_MAX_LABEL);
        o.label[MESH_MAX_LABEL] = '\0';
        o.bbox[0]=x0; o.bbox[1]=x1; o.bbox[2]=y0; o.bbox[3]=y1; o.bbox[4]=z0; o.bbox[5]=z1;
        o.r=r; o.g=g; o.b=b;
    }
}

void MeshBuilder::finalize() {
    // Per-vertex normals = normalised sum of incident face normals.
    if (_vcount == 0) return;
    float* acc = (float*)calloc(_vcount * 3, sizeof(float));
    if (!acc) { memset(_nz, 0, _vcount * 3); return; }

    for (uint32_t f = 0; f < _fcount; ++f) {
        uint16_t a=_fi[f*3], b=_fi[f*3+1], c=_fi[f*3+2];
        const float* A=&_vx[a*3]; const float* B=&_vx[b*3]; const float* C=&_vx[c*3];
        float ux=B[0]-A[0],uy=B[1]-A[1],uz=B[2]-A[2];
        float vx=C[0]-A[0],vy=C[1]-A[1],vz=C[2]-A[2];
        float nx=uy*vz-uz*vy, ny=uz*vx-ux*vz, nz=ux*vy-uy*vx;
        for (uint16_t idx : {a,b,c}) { acc[idx*3]+=nx; acc[idx*3+1]+=ny; acc[idx*3+2]+=nz; }
    }
    for (uint32_t v = 0; v < _vcount; ++v) {
        float nx=acc[v*3], ny=acc[v*3+1], nz=acc[v*3+2];
        float len = sqrtf(nx*nx+ny*ny+nz*nz);
        if (len < 1e-6f) { _nz[v*3]=0; _nz[v*3+1]=0; _nz[v*3+2]=127; continue; }
        float s = 127.0f / len;
        _nz[v*3]   = (int8_t)lrintf(nx*s);
        _nz[v*3+1] = (int8_t)lrintf(ny*s);
        _nz[v*3+2] = (int8_t)lrintf(nz*s);
    }
    free(acc);
}
