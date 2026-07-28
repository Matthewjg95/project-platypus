// room_objdb.h - Persistent per-room object database for additive scanning.
//
// Each room keeps a "<room>.objs" sidecar: every confirmed object in WORLD
// coordinates (floats, metres — unlike the .lbl sidecar, which is quantised
// mesh space regenerated on every mesh write). Rescans load this database,
// register the new scan's observations against it (see below), merge, and
// write everything back — so a room's model *improves* with every walk-around
// instead of being replaced.
//
// REGISTRATION: scan #2 starts at an arbitrary heading and standing spot, so
// its coordinate frame differs from the database by an unknown yaw rotation
// and XZ translation. Objects themselves are the landmarks: we search yaw in
// 10-degree steps x same-class pair hypotheses (each pair fixes the
// translation), score by how many new objects land within MATCH_R of a
// same-class database object, and apply the best transform when at least
// MIN_MATCHES agree. With too few landmarks we fall back to identity and
// plain proximity merging.

#pragma once
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <SD_MMC.h>

struct RoomObj {
    uint8_t cls;
    float   x, y, z;     // world, metres
    float   h;           // reference height
    uint16_t n;          // total observations across all scans
};

class RoomObjDB {
public:
    static const int MAX_OBJS = 64;

    RoomObj objs[MAX_OBJS];
    int     count = 0;

    void clear() { count = 0; }

    // Merge one observation (same rule the live accumulator uses: same class
    // within MERGE_R merges by weighted average; otherwise append).
    static constexpr float MERGE_R = 0.6f;
    void merge(uint8_t cls, float x, float y, float z, float h, uint16_t n_obs = 1) {
        for (int i = 0; i < count; ++i) {
            if (objs[i].cls != cls) continue;
            float dx = objs[i].x - x, dz = objs[i].z - z;
            if (dx*dx + dz*dz <= MERGE_R * MERGE_R) {
                uint32_t n0 = objs[i].n, n1 = n_obs, nt = n0 + n1;
                objs[i].x = (objs[i].x * n0 + x * n1) / nt;
                objs[i].y = (objs[i].y * n0 + y * n1) / nt;
                objs[i].z = (objs[i].z * n0 + z * n1) / nt;
                objs[i].h = (objs[i].h * n0 + h * n1) / nt;
                objs[i].n = (uint16_t)(nt > 65535 ? 65535 : nt);
                return;
            }
        }
        if (count < MAX_OBJS) objs[count++] = { cls, x, y, z, h,
                                                (uint16_t)n_obs };
    }

    // ---- persistence: "<room>.objs" ------------------------------------
    bool saveBeside(const char* mesh_path) {
        char p[208]; _path(mesh_path, p, sizeof(p));
        File f = SD_MMC.open(p, FILE_WRITE);
        if (!f) return false;
        f.write((const uint8_t*)"ROB1", 4);
        uint16_t n = (uint16_t)count;
        f.write((const uint8_t*)&n, 2);
        for (int i = 0; i < count; ++i)
            f.write((const uint8_t*)&objs[i], sizeof(RoomObj));
        f.close();
        return true;
    }
    bool loadBeside(const char* mesh_path) {
        clear();
        char p[208]; _path(mesh_path, p, sizeof(p));
        File f = SD_MMC.open(p, FILE_READ);
        if (!f) return false;
        char magic[4];
        bool ok = f.read((uint8_t*)magic, 4) == 4 && memcmp(magic, "ROB1", 4) == 0;
        if (ok) {
            uint16_t n = 0; f.read((uint8_t*)&n, 2);
            if (n > MAX_OBJS) n = MAX_OBJS;
            for (uint16_t i = 0; i < n; ++i)
                if (f.read((uint8_t*)&objs[i], sizeof(RoomObj)) == sizeof(RoomObj))
                    count = i + 1;
        }
        f.close();
        return ok && count > 0;
    }

    // ---- registration ---------------------------------------------------
    struct Transform { float cosr, sinr, tx, tz; int matches; };

    static constexpr float MATCH_R    = 0.8f;
    static const     int   MIN_MATCHES = 2;

    // Find the yaw+translation mapping `nw` (new scan objects, count nn) onto
    // this database. Returns identity with matches=0 when unregistrable.
    Transform registerScan(const RoomObj* nw, int nn) const {
        Transform best = { 1.0f, 0.0f, 0.0f, 0.0f, 0 };
        float bestErr = 1e30f;
        for (int deg = 0; deg < 360; deg += 10) {
            float cr = cosf(deg * (float)M_PI / 180.0f);
            float sr = sinf(deg * (float)M_PI / 180.0f);
            for (int i = 0; i < count; ++i) {
                for (int j = 0; j < nn; ++j) {
                    if (objs[i].cls != nw[j].cls) continue;
                    // hypothesis: new j maps onto db i under this rotation
                    float rx =  cr * nw[j].x + sr * nw[j].z;
                    float rz = -sr * nw[j].x + cr * nw[j].z;
                    float tx = objs[i].x - rx, tz = objs[i].z - rz;
                    int m = 0; float err = 0;
                    for (int k = 0; k < nn; ++k) {
                        float kx =  cr * nw[k].x + sr * nw[k].z + tx;
                        float kz = -sr * nw[k].x + cr * nw[k].z + tz;
                        float bd = 1e30f;
                        for (int q = 0; q < count; ++q) {
                            if (objs[q].cls != nw[k].cls) continue;
                            float dx = objs[q].x - kx, dz = objs[q].z - kz;
                            float d2 = dx*dx + dz*dz;
                            if (d2 < bd) bd = d2;
                        }
                        if (bd <= MATCH_R * MATCH_R) { ++m; err += bd; }
                    }
                    if (m > best.matches ||
                        (m == best.matches && m > 0 && err < bestErr)) {
                        best = { cr, sr, tx, tz, m };
                        bestErr = err;
                    }
                }
            }
        }
        if (best.matches < MIN_MATCHES)
            return { 1.0f, 0.0f, 0.0f, 0.0f, 0 };     // identity fallback
        return best;
    }

    static void apply(const Transform& t, float& x, float& z) {
        float rx =  t.cosr * x + t.sinr * z;
        float rz = -t.sinr * x + t.cosr * z;
        x = rx + t.tx; z = rz + t.tz;
    }

private:
    static void _path(const char* mesh_path, char* out, unsigned n) {
        strncpy(out, mesh_path, n - 1); out[n - 1] = '\0';
        char* dot = strrchr(out, '.');
        if (dot && (unsigned)(dot - out) + 6 < n) strcpy(dot, ".objs");
    }
};
