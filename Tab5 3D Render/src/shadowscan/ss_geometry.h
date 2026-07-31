// ss_geometry.h - 2D polygon math for the ShadowScan applet.
//
// Self-contained in namespace ss so nothing collides with vm3.h / mesh.h /
// the scanner's cv:: types. Ported from the standalone ShadowScan project
// (UnitV Outline Capture) — RDP simplify, Chaikin smoothing, centroid
// transforms, and ear-clip extrusion into a watertight triangle soup.

#pragma once
#include <stdint.h>
#include <math.h>
#include <vector>
#include <algorithm>

namespace ss {

struct V2 {
    float x = 0, y = 0;
    V2() {}
    V2(float x_, float y_) : x(x_), y(y_) {}
    V2 operator+(const V2& o) const { return V2(x + o.x, y + o.y); }
    V2 operator-(const V2& o) const { return V2(x - o.x, y - o.y); }
    V2 operator*(float s) const { return V2(x * s, y * s); }
    float cross(const V2& o) const { return x * o.y - y * o.x; }
    float dot(const V2& o) const { return x * o.x + y * o.y; }
    float len() const { return sqrtf(x * x + y * y); }
};

struct V3 {
    float x = 0, y = 0, z = 0;
    V3() {}
    V3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    V3 operator-(const V3& o) const { return V3(x - o.x, y - o.y, z - o.z); }
    V3 cross(const V3& o) const {
        return V3(y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x);
    }
    V3 normed() const {
        float l = sqrtf(x * x + y * y + z * z);
        return (l < 1e-12f) ? V3(0, 0, 1) : V3(x / l, y / l, z / l);
    }
};

struct Tri { V3 a, b, c; V3 normal() const { return (b - a).cross(c - a).normed(); } };

using Poly = std::vector<V2>;

inline float signed_area(const Poly& p) {
    float a = 0;
    for (size_t i = 0, n = p.size(); i < n; ++i) a += p[i].cross(p[(i + 1) % n]);
    return a * 0.5f;
}

inline V2 centroid(const Poly& p) {
    V2 c;
    if (p.empty()) return c;
    for (const auto& v : p) { c.x += v.x; c.y += v.y; }
    return V2(c.x / p.size(), c.y / p.size());
}

inline void ensure_ccw(Poly& p) {
    if (signed_area(p) < 0) std::reverse(p.begin(), p.end());
}

inline void bounds(const Poly& p, V2& mn, V2& mx) {
    if (p.empty()) { mn = mx = V2(); return; }
    mn = mx = p[0];
    for (const auto& v : p) {
        mn.x = std::min(mn.x, v.x); mn.y = std::min(mn.y, v.y);
        mx.x = std::max(mx.x, v.x); mx.y = std::max(mx.y, v.y);
    }
}

// Iterative RDP (explicit stack — the deep recursion bit us on the K210 port
// and there's no reason to gamble with the loop-task stack here either).
inline Poly simplify(const Poly& pts, float eps) {
    size_t n = pts.size();
    if (n < 3) return pts;
    std::vector<uint8_t> keep(n, 0);
    keep[0] = keep[n - 1] = 1;
    std::vector<std::pair<size_t, size_t>> stack;
    stack.push_back({0, n - 1});
    while (!stack.empty()) {
        auto seg = stack.back(); stack.pop_back();
        size_t lo = seg.first, hi = seg.second;
        if (hi <= lo + 1) continue;
        V2 a = pts[lo], b = pts[hi], ab = b - a;
        float l2 = ab.dot(ab);
        float maxD2 = -1; size_t idx = lo;
        for (size_t i = lo + 1; i < hi; ++i) {
            float d2;
            if (l2 < 1e-12f) { V2 d = pts[i] - a; d2 = d.dot(d); }
            else { float cr = (pts[i] - a).cross(ab); d2 = cr * cr / l2; }
            if (d2 > maxD2) { maxD2 = d2; idx = i; }
        }
        if (maxD2 > eps * eps) {
            keep[idx] = 1;
            stack.push_back({lo, idx});
            stack.push_back({idx, hi});
        }
    }
    Poly out;
    for (size_t i = 0; i < n; ++i) if (keep[i]) out.push_back(pts[i]);
    return out;
}

// One Chaikin corner-cutting pass (closed polygon), point count re-capped.
inline Poly chaikin(const Poly& src, size_t max_points) {
    if (src.size() < 3) return src;
    Poly out;
    out.reserve(src.size() * 2);
    for (size_t i = 0, n = src.size(); i < n; ++i) {
        const V2& p0 = src[i];
        const V2& p1 = src[(i + 1) % n];
        out.push_back(p0 * 0.75f + p1 * 0.25f);
        out.push_back(p0 * 0.25f + p1 * 0.75f);
    }
    if (out.size() > max_points) out = simplify(out, 0.4f);
    return out;
}

// Rotate (radians) + uniform scale about the centroid.
inline Poly transform(const Poly& p, float ang, float scale) {
    Poly out;
    out.reserve(p.size());
    V2 c = centroid(p);
    float s = sinf(ang), co = cosf(ang);
    for (const auto& v : p) {
        V2 d = v - c;
        out.push_back(V2(c.x + (d.x * co - d.y * s) * scale,
                         c.y + (d.x * s + d.y * co) * scale));
    }
    return out;
}

// Ear-clip + extrude a simple CCW polygon (mm) into a watertight triangle
// soup: top cap at z=thickness, bottom cap at z=0, outward side walls.
// Empty result on degenerate input. Implemented in ss_mesh.cpp.
std::vector<Tri> extrude(const Poly& contour_mm, float thickness_mm);

}  // namespace ss
