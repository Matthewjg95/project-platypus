// ss_export.cpp - STL / OBJ / renderer-.mesh writers for ShadowScan.
#include "ss_export.h"
#include "../scanner/scan_mesh_writer.h"
#include <SD_MMC.h>
#include <string.h>
#include <stdio.h>

namespace ss {

static const char* SS_DIR = "/shadowscan";

bool next_export_name(char* out, size_t out_len) {
    if (!SD_MMC.exists(SS_DIR) && !SD_MMC.mkdir(SS_DIR)) return false;
    for (int i = 1; i < 1000; ++i) {
        snprintf(out, out_len, "shadow_%03d", i);
        char path[64];
        snprintf(path, sizeof(path), "%s/%s.stl", SS_DIR, out);
        if (!SD_MMC.exists(path)) return true;
    }
    return false;
}

bool export_stl(const char* base, const std::vector<Tri>& tris) {
    if (tris.empty()) return false;
    char path[64];
    snprintf(path, sizeof(path), "%s/%s.stl", SS_DIR, base);
    File f = SD_MMC.open(path, FILE_WRITE);
    if (!f) return false;

    uint8_t hdr[80] = {0};
    strncpy((char*)hdr, "ShadowScan binary STL (mm)", 79);
    f.write(hdr, 80);
    uint32_t count = (uint32_t)tris.size();
    f.write((uint8_t*)&count, 4);

    uint8_t rec[50];
    for (const Tri& t : tris) {
        V3 n = t.normal();
        float vals[12] = {n.x,   n.y,   n.z,
                          t.a.x, t.a.y, t.a.z,
                          t.b.x, t.b.y, t.b.z,
                          t.c.x, t.c.y, t.c.z};
        memcpy(rec, vals, 48);
        rec[48] = rec[49] = 0;
        if (f.write(rec, 50) != 50) { f.close(); return false; }
    }
    f.close();
    return true;
}

bool export_obj(const char* base, const std::vector<Tri>& tris) {
    if (tris.empty()) return false;
    char path[64];
    snprintf(path, sizeof(path), "%s/%s.obj", SS_DIR, base);
    File f = SD_MMC.open(path, FILE_WRITE);
    if (!f) return false;
    f.println("# ShadowScan OBJ export (units: mm)");
    f.println("o shadowscan_part");
    char line[96];
    for (const Tri& t : tris) {
        const V3* vs[3] = {&t.a, &t.b, &t.c};
        for (int i = 0; i < 3; ++i) {
            snprintf(line, sizeof(line), "v %.4f %.4f %.4f", vs[i]->x, vs[i]->y, vs[i]->z);
            f.println(line);
        }
    }
    for (size_t i = 0; i < tris.size(); ++i) {
        size_t b = i * 3 + 1;
        snprintf(line, sizeof(line), "f %u %u %u",
                 (unsigned)b, (unsigned)(b + 1), (unsigned)(b + 2));
        f.println(line);
    }
    f.close();
    return true;
}

bool export_viewer_mesh(const char* base, const std::vector<Tri>& tris) {
    if (tris.empty()) return false;
    // ScanMeshWriter's uint16 face indices cap vertices at 65535; we write a
    // 3-verts-per-triangle soup, so the extrusion's <= ~1500 tris is fine.
    if (tris.size() * 3 > 0xFFFF) return false;

    float bx[6] = {1e9f, -1e9f, 1e9f, -1e9f, 1e9f, -1e9f};
    for (const Tri& t : tris) {
        const V3* vs[3] = {&t.a, &t.b, &t.c};
        for (int i = 0; i < 3; ++i) {
            bx[0] = std::min(bx[0], vs[i]->x); bx[1] = std::max(bx[1], vs[i]->x);
            bx[2] = std::min(bx[2], vs[i]->y); bx[3] = std::max(bx[3], vs[i]->y);
            bx[4] = std::min(bx[4], vs[i]->z); bx[5] = std::max(bx[5], vs[i]->z);
        }
    }
    float center[3]; float scale;
    ScanMeshWriter::quantisation(bx, center, scale);

    char path[64];
    snprintf(path, sizeof(path), "/models/%s.mesh", base);
    if (!SD_MMC.exists("/models") && !SD_MMC.mkdir("/models")) return false;

    ScanMeshWriter w;
    if (!w.begin(path, center, scale)) return false;
    for (const Tri& t : tris) {
        w.addVertexModel(t.a.x, t.a.y, t.a.z);
        w.addVertexModel(t.b.x, t.b.y, t.b.z);
        w.addVertexModel(t.c.x, t.c.y, t.c.z);
    }
    for (uint32_t i = 0; i < tris.size(); ++i)
        w.addFace((uint16_t)(i * 3), (uint16_t)(i * 3 + 1), (uint16_t)(i * 3 + 2));
    return w.finish();
}

}  // namespace ss
