// room_carve.h - Shared visibility-carve occupancy stamping.
//
// One implementation of "what floor does this room's evidence prove?" used by
// BOTH the mesh rebuild (fine grid -> addFloorCells) and the building map
// (coarse grid -> footprint thumbnails). Evidence, strongest first:
//   - the walked path (you were physically there)
//   - where you stood at the origin
//   - each displayed object's footprint
//   - the sight-line corridor from the nearest path point (or origin) to
//     each object: the camera saw it from there, so the line between is open
//
// PORTABILITY: pure logic over RoomObjDB/RoomPath; grid size and cell pitch
// are caller-chosen so any resolution works.

#pragma once
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "room_objdb.h"
#include "room_path.h"

namespace room_carve {

static const float R_ORIGIN = 0.9f;    // open floor where you stood
static const float R_PATH   = 0.55f;   // corridor around the walked path
static const float R_OBJ    = 0.7f;    // open floor around an object
static const float R_LOS    = 0.45f;   // sight-line corridor half-width

// Fill occ[gn*gn] (row-major, gz*gn+gx) from the room's evidence.
// shown[i] gates db.objs[i]; path may be null/empty. Outputs the grid's
// world placement via minx/minz/nx/nz.
inline void carve(uint8_t* occ, int gn, float cell,
                  const RoomObjDB& db, const uint8_t* shown,
                  const RoomPath* path,
                  float& minx, float& minz, int& nx, int& nz) {
    memset(occ, 0, (size_t)gn * gn);

    // ---- bounds over origin + shown objects + path ----------------------
    minx = -1.2f; minz = -1.2f;
    float maxx = 1.2f, maxz = 1.2f;
    for (int i = 0; i < db.count; ++i) {
        if (!shown[i]) continue;
        const RoomObj& o = db.objs[i];
        if (o.x - 1.0f < minx) minx = o.x - 1.0f;
        if (o.x + 1.0f > maxx) maxx = o.x + 1.0f;
        if (o.z - 1.0f < minz) minz = o.z - 1.0f;
        if (o.z + 1.0f > maxz) maxz = o.z + 1.0f;
    }
    if (path)
        for (int i = 0; i < path->count; ++i) {
            float px = path->x(i), pz = path->z(i);
            if (px - 0.8f < minx) minx = px - 0.8f;
            if (px + 0.8f > maxx) maxx = px + 0.8f;
            if (pz - 0.8f < minz) minz = pz - 0.8f;
            if (pz + 0.8f > maxz) maxz = pz + 0.8f;
        }
    nx = (int)((maxx - minx) / cell) + 1; if (nx > gn) nx = gn;
    nz = (int)((maxz - minz) / cell) + 1; if (nz > gn) nz = gn;

    auto stamp = [&](float sx, float sz, float r) {
        int x0 = (int)((sx - r - minx) / cell), x1 = (int)((sx + r - minx) / cell);
        int z0 = (int)((sz - r - minz) / cell), z1 = (int)((sz + r - minz) / cell);
        for (int gz = z0 < 0 ? 0 : z0; gz <= z1 && gz < nz; ++gz)
            for (int gx = x0 < 0 ? 0 : x0; gx <= x1 && gx < nx; ++gx) {
                float dx = minx + (gx + 0.5f) * cell - sx;
                float dz = minz + (gz + 0.5f) * cell - sz;
                if (dx*dx + dz*dz <= r*r) occ[gz * gn + gx] = 1;
            }
    };
    auto stamp_seg = [&](float ax, float az, float bx, float bz, float r) {
        float dx = bx - ax, dz = bz - az;
        float d = sqrtf(dx*dx + dz*dz);
        int steps = (int)(d / cell) + 1;
        for (int s = 0; s <= steps; ++s) {
            float t = (float)s / (float)steps;
            stamp(ax + dx * t, az + dz * t, r);
        }
    };

    // ---- evidence -------------------------------------------------------
    stamp(0.0f, 0.0f, R_ORIGIN);
    if (path && path->count > 0) {
        stamp_seg(0.0f, 0.0f, path->x(0), path->z(0), R_PATH);
        for (int i = 1; i < path->count; ++i)
            stamp_seg(path->x(i-1), path->z(i-1), path->x(i), path->z(i), R_PATH);
    }
    for (int i = 0; i < db.count; ++i) {
        if (!shown[i]) continue;
        const RoomObj& o = db.objs[i];
        stamp(o.x, o.z, R_OBJ);
        // sight-line from the nearest anchor (origin or a path point): the
        // camera observed this object from roughly there
        float ax = 0.0f, az = 0.0f, best = o.x*o.x + o.z*o.z;
        if (path)
            for (int p = 0; p < path->count; ++p) {
                float dx = path->x(p) - o.x, dz = path->z(p) - o.z;
                float d2 = dx*dx + dz*dz;
                if (d2 < best) { best = d2; ax = path->x(p); az = path->z(p); }
            }
        stamp_seg(ax, az, o.x, o.z, R_LOS);
    }

    // ---- smoothing: rooms, not amoebas ------------------------------
    // The raw union of disc stamps has scalloped edges and pinholes; with
    // walls extruded at the boundary that reads as melted wax. Two passes
    // of a majority filter: lone spurs (<3 of 8 neighbours) erode, notches
    // (>=5 of 8 neighbours) fill. Outlines become clean runs that mesh into
    // straight wall segments, and the evidence-driven overall SHAPE stays.
    static uint8_t src[64 * 64];
    if (gn <= 64) {
        for (int pass = 0; pass < 2; ++pass) {
            memcpy(src, occ, (size_t)gn * gn);
            for (int z = 0; z < nz; ++z)
                for (int x = 0; x < nx; ++x) {
                    int nbr = 0;
                    for (int dz = -1; dz <= 1; ++dz)
                        for (int dx = -1; dx <= 1; ++dx) {
                            if (!dx && !dz) continue;
                            int xx = x + dx, zz = z + dz;
                            if (xx >= 0 && zz >= 0 && xx < nx && zz < nz &&
                                src[zz * gn + xx]) ++nbr;
                        }
                    if (src[z * gn + x]) occ[z * gn + x] = (nbr >= 3) ? 1 : 0;
                    else                 occ[z * gn + x] = (nbr >= 5) ? 1 : 0;
                }
        }
    }
}

} // namespace room_carve
