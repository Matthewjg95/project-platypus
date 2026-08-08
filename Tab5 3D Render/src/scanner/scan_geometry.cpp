// scan_geometry.cpp
#include "scan_geometry.h"
#include <esp_heap_caps.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

static void* ps_alloc(size_t n) {
    void* p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM);
    return p ? p : malloc(n);
}

bool ScanGeometry::begin(uint32_t max_vertices, uint32_t max_faces) {
    _vcap = max_vertices; _fcap = max_faces;
    _vx = (float*)   ps_alloc(sizeof(float)    * _vcap * 3);
    _fi = (uint16_t*)ps_alloc(sizeof(uint16_t) * _fcap * 3);
    if (!_vx || !_fi) return false;
    reset();
    return true;
}

void ScanGeometry::reset() {
    _vcount = _fcount = 0; _ocount = 0;
    _bounds[0]=_bounds[2]=_bounds[4]= 1e30f;
    _bounds[1]=_bounds[3]=_bounds[5]=-1e30f;
}

void ScanGeometry::growBounds(float x, float y, float z) {
    if (x<_bounds[0])_bounds[0]=x; if (x>_bounds[1])_bounds[1]=x;
    if (y<_bounds[2])_bounds[2]=y; if (y>_bounds[3])_bounds[3]=y;
    if (z<_bounds[4])_bounds[4]=z; if (z>_bounds[5])_bounds[5]=z;
}

uint32_t ScanGeometry::addVertex(float x, float y, float z) {
    if (_vcount >= _vcap) return _vcount ? _vcount-1 : 0;
    uint32_t i = _vcount++;
    _vx[i*3]=x; _vx[i*3+1]=y; _vx[i*3+2]=z;
    growBounds(x,y,z);
    return i;
}

void ScanGeometry::addOrientedTri(uint32_t a, uint32_t b, uint32_t c,
                                  float rx, float ry, float rz, bool towardRef) {
    if (_fcount >= _fcap) return;
    const float* A=&_vx[a*3]; const float* B=&_vx[b*3]; const float* C=&_vx[c*3];
    float ux=B[0]-A[0],uy=B[1]-A[1],uz=B[2]-A[2];
    float vx=C[0]-A[0],vy=C[1]-A[1],vz=C[2]-A[2];
    float nx=uy*vz-uz*vy, ny=uz*vx-ux*vz, nz=ux*vy-uy*vx;
    float mx=(A[0]+B[0]+C[0])/3, my=(A[1]+B[1]+C[1])/3, mz=(A[2]+B[2]+C[2])/3;
    float dot = nx*(rx-mx)+ny*(ry-my)+nz*(rz-mz);
    if ((dot>0.0f) != towardRef) { uint32_t t=b; b=c; c=t; }
    uint32_t f=_fcount++;
    _fi[f*3]=(uint16_t)a; _fi[f*3+1]=(uint16_t)b; _fi[f*3+2]=(uint16_t)c;
}

void ScanGeometry::addBoxRoom(float cx, float cz, float width, float depth, float height) {
    float x0=cx-width*0.5f, x1=cx+width*0.5f;
    float z0=cz-depth*0.5f, z1=cz+depth*0.5f;
    float y0=0.0f, y1=height;
    float rcx=cx, rcy=height*0.5f, rcz=cz;     // interior reference
    uint32_t v0=addVertex(x0,y0,z0), v1=addVertex(x1,y0,z0),
             v2=addVertex(x1,y0,z1), v3=addVertex(x0,y0,z1),
             v4=addVertex(x0,y1,z0), v5=addVertex(x1,y1,z0),
             v6=addVertex(x1,y1,z1), v7=addVertex(x0,y1,z1);
    // inward-facing normals (viewer is inside the room)
    addOrientedTri(v0,v1,v2,rcx,rcy,rcz,true); addOrientedTri(v0,v2,v3,rcx,rcy,rcz,true);
    addOrientedTri(v4,v5,v6,rcx,rcy,rcz,true); addOrientedTri(v4,v6,v7,rcx,rcy,rcz,true);
    addOrientedTri(v0,v1,v5,rcx,rcy,rcz,true); addOrientedTri(v0,v5,v4,rcx,rcy,rcz,true);
    addOrientedTri(v3,v2,v6,rcx,rcy,rcz,true); addOrientedTri(v3,v6,v7,rcx,rcy,rcz,true);
    addOrientedTri(v0,v3,v7,rcx,rcy,rcz,true); addOrientedTri(v0,v7,v4,rcx,rcy,rcz,true);
    addOrientedTri(v1,v2,v6,rcx,rcy,rcz,true); addOrientedTri(v1,v6,v5,rcx,rcy,rcz,true);
}

// Thin floor slab: renders only what the scan actually knows (the footprint).
// Walls are deliberately NOT drawn — the Phase-1 fitter has orientation
// evidence but cannot localize walls; evidence-based thin walls arrive with
// Phase-2 positional data.
void ScanGeometry::addFloorPlate(float cx, float cz, float width, float depth,
                                 float thickness) {
    float x0=cx-width*0.5f, x1=cx+width*0.5f;
    float z0=cz-depth*0.5f, z1=cz+depth*0.5f;
    float y0=-thickness, y1=0.0f;              // top surface = floor level y=0
    float rcx=cx, rcy=-thickness*0.5f, rcz=cz; // slab centre reference
    uint32_t v0=addVertex(x0,y0,z0), v1=addVertex(x1,y0,z0),
             v2=addVertex(x1,y0,z1), v3=addVertex(x0,y0,z1),
             v4=addVertex(x0,y1,z0), v5=addVertex(x1,y1,z0),
             v6=addVertex(x1,y1,z1), v7=addVertex(x0,y1,z1);
    // outward-facing normals (slab is viewed from outside/above)
    addOrientedTri(v0,v1,v2,rcx,rcy,rcz,false); addOrientedTri(v0,v2,v3,rcx,rcy,rcz,false);
    addOrientedTri(v4,v5,v6,rcx,rcy,rcz,false); addOrientedTri(v4,v6,v7,rcx,rcy,rcz,false);
    addOrientedTri(v0,v1,v5,rcx,rcy,rcz,false); addOrientedTri(v0,v5,v4,rcx,rcy,rcz,false);
    addOrientedTri(v3,v2,v6,rcx,rcy,rcz,false); addOrientedTri(v3,v6,v7,rcx,rcy,rcz,false);
    addOrientedTri(v0,v3,v7,rcx,rcy,rcz,false); addOrientedTri(v0,v7,v4,rcx,rcy,rcz,false);
    addOrientedTri(v1,v2,v6,rcx,rcy,rcz,false); addOrientedTri(v1,v6,v5,rcx,rcy,rcz,false);
}

// Carved floor from an occupancy grid. Surfaces are emitted as merged
// horizontal runs; skirt walls appear only where an occupied cell borders an
// empty one (or the grid edge), so the interior is seam-free — the naive
// slab-per-row version drew every row's side walls as dark corduroy stripes.
void ScanGeometry::addFloorCells(const uint8_t* occ, int stride, int nx, int nz,
                                 float minx, float minz, float cell,
                                 float thickness, float wall_h) {
    const float y0 = -thickness, y1 = 0.0f;    // top surface = floor level y=0
    const float yw = wall_h;                   // boundary walls rise to here
    auto at = [&](int gx, int gz) -> bool {
        return gx >= 0 && gz >= 0 && gx < nx && gz < nz &&
               occ[gz * stride + gx] != 0;
    };
    // TOP surface only — no underside. The slab-style bottom faces tied with
    // the top in the painter's depth sort and bled through as dark streaks
    // from oblique angles. A floor is never viewed from below; the boundary
    // skirts keep the thickness illusion.
    // Greedy maximal-rectangle cover: grow each uncovered cell to the widest
    // row run, then extend that full width down as many rows as stay
    // occupied. Large unified quads instead of per-row strips = the floor
    // reads as one surface, not stacked planks.
    static uint8_t used[64 * 64];
    memset(used, 0, (size_t)stride * nz);
    for (int gz = 0; gz < nz; ++gz) {
        for (int gx = 0; gx < nx; ++gx) {
            if (!at(gx, gz) || used[gz * stride + gx]) continue;
            int w = 1;
            while (gx + w < nx && at(gx + w, gz) && !used[gz * stride + gx + w]) ++w;
            int h = 1;
            while (gz + h < nz) {
                bool ok = true;
                for (int k = 0; k < w; ++k)
                    if (!at(gx + k, gz + h) || used[(gz + h) * stride + gx + k]) { ok = false; break; }
                if (!ok) break;
                ++h;
            }
            for (int r = 0; r < h; ++r)
                for (int k = 0; k < w; ++k)
                    used[(gz + r) * stride + gx + k] = 1;
            float x0 = minx + gx * cell, x1 = x0 + w * cell;
            float z0 = minz + gz * cell, z1 = z0 + h * cell;
            float rx = (x0 + x1) * 0.5f, rz = (z0 + z1) * 0.5f;
            uint32_t a=addVertex(x0,y1,z0), b=addVertex(x1,y1,z0),
                     c=addVertex(x1,y1,z1), d=addVertex(x0,y1,z1);
            addOrientedTri(a,b,c, rx, 1.0f, rz, true);
            addOrientedTri(a,c,d, rx, 1.0f, rz, true);
        }
    }
    // Z-facing skirts: cells whose north/south neighbour is empty
    for (int gz = 0; gz < nz; ++gz) {
        for (int side = 0; side < 2; ++side) {         // 0 = -Z edge, 1 = +Z edge
            int run = -1;
            for (int gx = 0; gx <= nx; ++gx) {
                bool edge = at(gx, gz) && !at(gx, side ? gz + 1 : gz - 1);
                if (edge && run < 0) run = gx;
                else if (!edge && run >= 0) {
                    float x0 = minx + run * cell, x1 = minx + gx * cell;
                    float z  = minz + (gz + side) * cell;
                    float rx = (x0 + x1) * 0.5f;
                    float rz = side ? z + 1.0f : z - 1.0f;
                    uint32_t a=addVertex(x0,y0,z), b=addVertex(x1,y0,z),
                             c=addVertex(x1,yw,z), d=addVertex(x0,yw,z);
                    addOrientedTri(a,b,c, rx, yw*0.5f, rz, true);   // outer face
                    addOrientedTri(a,c,d, rx, yw*0.5f, rz, true);
                    addOrientedTri(a,b,c, rx, yw*0.5f, rz, false);  // inner face
                    addOrientedTri(a,c,d, rx, yw*0.5f, rz, false);
                    run = -1;
                }
            }
        }
    }
    // X-facing skirts: cells whose east/west neighbour is empty
    for (int gx = 0; gx < nx; ++gx) {
        for (int side = 0; side < 2; ++side) {         // 0 = -X edge, 1 = +X edge
            int run = -1;
            for (int gz = 0; gz <= nz; ++gz) {
                bool edge = at(gx, gz) && !at(side ? gx + 1 : gx - 1, gz);
                if (edge && run < 0) run = gz;
                else if (!edge && run >= 0) {
                    float z0 = minz + run * cell, z1 = minz + gz * cell;
                    float x  = minx + (gx + side) * cell;
                    float rz = (z0 + z1) * 0.5f;
                    float rx = side ? x + 1.0f : x - 1.0f;
                    uint32_t a=addVertex(x,y0,z0), b=addVertex(x,y0,z1),
                             c=addVertex(x,yw,z1), d=addVertex(x,yw,z0);
                    addOrientedTri(a,b,c, rx, yw*0.5f, rz, true);   // outer face
                    addOrientedTri(a,c,d, rx, yw*0.5f, rz, true);
                    addOrientedTri(a,b,c, rx, yw*0.5f, rz, false);  // inner face
                    addOrientedTri(a,c,d, rx, yw*0.5f, rz, false);
                    run = -1;
                }
            }
        }
    }
}

// Slim arrow lying just above the floor at the origin, tip toward +Z (the
// direction faced at scan start). Triangles added in both windings so it
// renders regardless of view side.
void ScanGeometry::addOriginArrow(float half_w, float len) {
    const float y = 0.035f;                 // float above the plate, no z-fight
    uint32_t tip = addVertex(0.0f,  y, len);
    uint32_t bl  = addVertex(-half_w, y, 0.0f);
    uint32_t br  = addVertex( half_w, y, 0.0f);
    uint32_t tail= addVertex(0.0f,  y, -0.12f);
    if (_fcount + 4 <= _fcap) {
        _fi[_fcount*3+0]=tip; _fi[_fcount*3+1]=bl;  _fi[_fcount*3+2]=br;  ++_fcount;
        _fi[_fcount*3+0]=tip; _fi[_fcount*3+1]=br;  _fi[_fcount*3+2]=bl;  ++_fcount;
        _fi[_fcount*3+0]=bl;  _fi[_fcount*3+1]=tail;_fi[_fcount*3+2]=br;  ++_fcount;
        _fi[_fcount*3+0]=bl;  _fi[_fcount*3+1]=br;  _fi[_fcount*3+2]=tail;++_fcount;
    }
}

void ScanGeometry::addObjectMarker(const char* label, float cx, float cy, float cz,
                                   float hx, float hy, float hz,
                                   uint8_t r, uint8_t g, uint8_t b) {
    float x0=cx-hx,x1=cx+hx,y0=cy-hy,y1=cy+hy,z0=cz-hz,z1=cz+hz;
    uint32_t v0=addVertex(x0,y0,z0), v1=addVertex(x1,y0,z0),
             v2=addVertex(x1,y0,z1), v3=addVertex(x0,y0,z1),
             v4=addVertex(x0,y1,z0), v5=addVertex(x1,y1,z0),
             v6=addVertex(x1,y1,z1), v7=addVertex(x0,y1,z1);
    // outward-facing normals (viewed from outside)
    addOrientedTri(v0,v1,v2,cx,cy,cz,false); addOrientedTri(v0,v2,v3,cx,cy,cz,false);
    addOrientedTri(v4,v5,v6,cx,cy,cz,false); addOrientedTri(v4,v6,v7,cx,cy,cz,false);
    addOrientedTri(v0,v1,v5,cx,cy,cz,false); addOrientedTri(v0,v5,v4,cx,cy,cz,false);
    addOrientedTri(v3,v2,v6,cx,cy,cz,false); addOrientedTri(v3,v6,v7,cx,cy,cz,false);
    addOrientedTri(v0,v3,v7,cx,cy,cz,false); addOrientedTri(v0,v7,v4,cx,cy,cz,false);
    addOrientedTri(v1,v2,v6,cx,cy,cz,false); addOrientedTri(v1,v6,v5,cx,cy,cz,false);

    if (_ocount < MAX_OBJECTS) {
        ScanObject& o = _objs[_ocount++];
        strncpy(o.label, label ? label : "?", sizeof(o.label)-1);
        o.label[sizeof(o.label)-1]='\0';
        o.cx=cx; o.cy=cy; o.cz=cz; o.r=r; o.g=g; o.b=b;
    }
}
