// room_path.h - Walked-path polyline sidecar: "<room>.path", magic PTH1.
//
// World metres stored as int16 centimetres. The path is floor-truth evidence
// for the visibility carve (you WALKED it, so it is open floor — the
// strongest floor evidence we have) and must survive reboots so OBJ-menu
// rebuilds don't shrink walk-scanned rooms.

#pragma once
#include <stdint.h>
#include <string.h>
#include <SD_MMC.h>

class RoomPath {
public:
    static const int MAX_PTS = 256;
    int16_t px_cm[MAX_PTS], pz_cm[MAX_PTS];
    int     count = 0;

    void clear() { count = 0; }

    void add(float x_m, float z_m) {
        if (count >= MAX_PTS) return;
        // skip micro-moves: record only >=20cm from the previous point
        if (count > 0) {
            float dx = x_m * 100.0f - px_cm[count-1];
            float dz = z_m * 100.0f - pz_cm[count-1];
            if (dx*dx + dz*dz < 400.0f) return;
        }
        px_cm[count] = (int16_t)(x_m * 100.0f);
        pz_cm[count] = (int16_t)(z_m * 100.0f);
        ++count;
    }

    float x(int i) const { return px_cm[i] * 0.01f; }
    float z(int i) const { return pz_cm[i] * 0.01f; }

    bool saveBeside(const char* mesh_path) {
        char p[208]; _path(mesh_path, p, sizeof(p));
        File f = SD_MMC.open(p, FILE_WRITE);
        if (!f) return false;
        f.write((const uint8_t*)"PTH1", 4);
        uint16_t n = (uint16_t)count;
        f.write((const uint8_t*)&n, 2);
        for (int i = 0; i < count; ++i) {
            f.write((const uint8_t*)&px_cm[i], 2);
            f.write((const uint8_t*)&pz_cm[i], 2);
        }
        f.close();
        return true;
    }

    bool loadBeside(const char* mesh_path) {
        clear();
        char p[208]; _path(mesh_path, p, sizeof(p));
        File f = SD_MMC.open(p, FILE_READ);
        if (!f) return false;
        char m[4];
        bool ok = f.read((uint8_t*)m, 4) == 4 && memcmp(m, "PTH1", 4) == 0;
        if (ok) {
            uint16_t n = 0; f.read((uint8_t*)&n, 2);
            if (n > MAX_PTS) n = MAX_PTS;
            for (uint16_t i = 0; i < n; ++i) {
                if (f.read((uint8_t*)&px_cm[i], 2) != 2) break;
                if (f.read((uint8_t*)&pz_cm[i], 2) != 2) break;
                count = i + 1;
            }
        }
        f.close();
        return ok && count > 0;
    }

    static void removeBeside(const char* mesh_path) {
        char p[208]; _path(mesh_path, p, sizeof(p));
        if (SD_MMC.exists(p)) SD_MMC.remove(p);   // quiet: no vfs error spam
    }

private:
    static void _path(const char* mesh_path, char* out, unsigned n) {
        strncpy(out, mesh_path, n - 1); out[n - 1] = '\0';
        char* dot = strrchr(out, '.');
        if (dot && (unsigned)(dot - out) + 6 < n) strcpy(dot, ".path");
    }
};
