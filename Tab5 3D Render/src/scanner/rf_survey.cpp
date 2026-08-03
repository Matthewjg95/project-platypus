// rf_survey.cpp
#include "rf_survey.h"
#include <SD_MMC.h>
#include <string.h>
#include <math.h>

// On-disk (little-endian): "RFS3" | ssid[33] | bssid[6] | chan u8 | count u16
//                          | count * Sample{ i16 x, i16 z, i8 rssi, i8 min,
//                                            i8 max, u8 antenna, i16 heading }
// RFS2 is the same minus the trailing heading (8-byte records); it still
// loads, with heading = -1 (unknown).
static const char* MAGIC     = "RFS3";
static const char* MAGIC_OLD = "RFS2";
static const int   REC_V2    = 8;

void RfSurvey::clear() {
    _count = 0;
    ssid[0] = '\0';
    memset(bssid, 0, sizeof(bssid));
    channel = 0;
    memset(_grid, 127, sizeof(_grid));
}

bool RfSurvey::add(int16_t x, int16_t z, int8_t rssi, int8_t mn, int8_t mx,
                   uint8_t antenna, int16_t heading) {
    if (_count >= MAX_SAMPLES) return false;
    _samples[_count++] = { x, z, rssi, mn, mx, antenna, heading };
    return true;
}

void RfSurvey::rfPath(const char* mesh_path, char* out, unsigned n) {
    strncpy(out, mesh_path, n - 1); out[n - 1] = '\0';
    char* dot = strrchr(out, '.');
    if (dot && (unsigned)(dot - out) + 4 < n) strcpy(dot, ".rf");
}

bool RfSurvey::saveBeside(const char* mesh_path) {
    char p[208]; rfPath(mesh_path, p, sizeof(p));
    File f = SD_MMC.open(p, FILE_WRITE);
    if (!f) return false;
    f.write((const uint8_t*)MAGIC, 4);
    f.write((const uint8_t*)ssid, 33);
    f.write(bssid, 6);
    f.write(&channel, 1);
    uint16_t n = (uint16_t)_count;
    f.write((const uint8_t*)&n, 2);
    for (int i = 0; i < _count; ++i)
        f.write((const uint8_t*)&_samples[i], sizeof(Sample));
    f.close();
    return true;
}

bool RfSurvey::loadBeside(const char* mesh_path) {
    clear();
    char p[208]; rfPath(mesh_path, p, sizeof(p));
    File f = SD_MMC.open(p, FILE_READ);
    if (!f) return false;
    char magic[4];
    bool v3 = false, ok = f.read((uint8_t*)magic, 4) == 4;
    if (ok) {
        v3 = memcmp(magic, MAGIC, 4) == 0;
        ok = v3 || memcmp(magic, MAGIC_OLD, 4) == 0;
    }
    if (ok) {
        f.read((uint8_t*)ssid, 33); ssid[32] = '\0';
        f.read(bssid, 6);
        f.read(&channel, 1);
        uint16_t n = 0;
        f.read((uint8_t*)&n, 2);
        if (n > MAX_SAMPLES) n = MAX_SAMPLES;
        for (uint16_t i = 0; i < n; ++i) {
            if (v3) {
                if (f.read((uint8_t*)&_samples[i], sizeof(Sample)) != (int)sizeof(Sample))
                    break;
            } else {                              // legacy: no heading field
                if (f.read((uint8_t*)&_samples[i], REC_V2) != REC_V2) break;
                _samples[i].heading = -1;
            }
            _count = i + 1;
        }
    }
    f.close();
    return ok && _count > 0;
}

void RfSurvey::computeGrid(int16_t xmin, int16_t xmax, int16_t zmin, int16_t zmax,
                           uint8_t antenna, float cutoff_frac) {
    memset(_grid, 127, sizeof(_grid));
    float w = (float)(xmax - xmin), h = (float)(zmax - zmin);
    if (w <= 0 || h <= 0) return;
    float diag = sqrtf(w * w + h * h);
    float cutoff2 = (cutoff_frac * diag) * (cutoff_frac * diag);

    for (int row = 0; row < GRID_H; ++row) {
        for (int col = 0; col < GRID_W; ++col) {
            float cx = xmin + (col + 0.5f) * w / GRID_W;
            float cz = zmin + (row + 0.5f) * h / GRID_H;
            float wsum = 0, vsum = 0, best2 = 1e30f;
            for (int i = 0; i < _count; ++i) {
                if (_samples[i].antenna != antenna) continue;   // layer filter
                float dx = cx - _samples[i].x, dz = cz - _samples[i].z;
                float d2 = dx * dx + dz * dz;
                if (d2 < best2) best2 = d2;
                float wgt = 1.0f / (d2 + 1.0f);
                wsum += wgt; vsum += wgt * _samples[i].rssi;
            }
            if (best2 <= cutoff2 && wsum > 0)
                _grid[row * GRID_W + col] = (int8_t)lrintf(vsum / wsum);
        }
    }
}

uint16_t RfSurvey::colorFor(int8_t dbm) {
    if (dbm == 127) return 0x2104;      // no data: dark grey
    if (dbm >= -55) return 0x07E0;      // green
    if (dbm >= -70) return 0xFFE0;      // yellow
    if (dbm >= -80) return 0xFC00;      // orange
    return 0xF800;                      // red
}
