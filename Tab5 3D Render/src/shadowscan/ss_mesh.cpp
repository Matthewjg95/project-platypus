// ss_mesh.cpp - ear-clip triangulation + extrusion for ShadowScan.
// The algorithm mirrors the standalone project's MeshBuilder, whose output
// was verified watertight (every directed edge paired) on the desktop.
#include "ss_geometry.h"

namespace ss {

static bool point_in_tri(const V2& p, const V2& a, const V2& b, const V2& c) {
    float d1 = (p - a).cross(b - a);
    float d2 = (p - b).cross(c - b);
    float d3 = (p - c).cross(a - c);
    bool has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
    bool has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);
    return !(has_neg && has_pos);
}

static bool triangulate(const Poly& poly, std::vector<uint16_t>& out) {
    size_t n = poly.size();
    if (n < 3 || n > 0xFFFF) return false;
    std::vector<uint16_t> V(n);
    for (size_t i = 0; i < n; ++i) V[i] = (uint16_t)i;

    size_t guard = n * n + 16;
    while (V.size() > 3 && guard--) {
        bool clipped = false;
        for (size_t i = 0; i < V.size(); ++i) {
            uint16_t ia = V[(i + V.size() - 1) % V.size()];
            uint16_t ib = V[i];
            uint16_t ic = V[(i + 1) % V.size()];
            const V2& a = poly[ia];
            const V2& b = poly[ib];
            const V2& c = poly[ic];
            if ((b - a).cross(c - a) <= 1e-9f) continue;      // reflex/collinear
            bool contains = false;
            for (uint16_t vi : V) {
                if (vi == ia || vi == ib || vi == ic) continue;
                if (point_in_tri(poly[vi], a, b, c)) { contains = true; break; }
            }
            if (contains) continue;
            out.push_back(ia); out.push_back(ib); out.push_back(ic);
            V.erase(V.begin() + i);
            clipped = true;
            break;
        }
        if (!clipped) {
            // Numerically stuck (sliver runs). Force-clip so we still close.
            out.push_back(V[V.size() - 1]); out.push_back(V[0]); out.push_back(V[1]);
            V.erase(V.begin());
        }
    }
    if (V.size() == 3) { out.push_back(V[0]); out.push_back(V[1]); out.push_back(V[2]); }
    return !out.empty();
}

std::vector<Tri> extrude(const Poly& contour_in, float thickness_mm) {
    std::vector<Tri> tris;
    if (contour_in.size() < 3 || thickness_mm <= 0) return tris;

    Poly poly = contour_in;
    ensure_ccw(poly);
    if (fabsf(signed_area(poly)) < 1e-3f) return tris;

    std::vector<uint16_t> idx;
    if (!triangulate(poly, idx)) return tris;

    const float zt = thickness_mm, zb = 0.0f;
    size_t n = poly.size();
    tris.reserve(idx.size() / 3 * 2 + n * 2);

    for (size_t i = 0; i + 2 < idx.size(); i += 3) {
        // top cap (+Z, CCW from above)
        tris.push_back({V3(poly[idx[i]].x,     poly[idx[i]].y,     zt),
                        V3(poly[idx[i + 1]].x, poly[idx[i + 1]].y, zt),
                        V3(poly[idx[i + 2]].x, poly[idx[i + 2]].y, zt)});
        // bottom cap (-Z, reversed winding)
        tris.push_back({V3(poly[idx[i]].x,     poly[idx[i]].y,     zb),
                        V3(poly[idx[i + 2]].x, poly[idx[i + 2]].y, zb),
                        V3(poly[idx[i + 1]].x, poly[idx[i + 1]].y, zb)});
    }
    for (size_t i = 0; i < n; ++i) {
        const V2& p0 = poly[i];
        const V2& p1 = poly[(i + 1) % n];
        V3 b0(p0.x, p0.y, zb), b1(p1.x, p1.y, zb);
        V3 t0(p0.x, p0.y, zt), t1(p1.x, p1.y, zt);
        tris.push_back({b0, b1, t1});
        tris.push_back({b0, t1, t0});
    }
    return tris;
}

}  // namespace ss
