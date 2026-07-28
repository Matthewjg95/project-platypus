// rf_survey.h - WiFi RSSI survey data for the heatmap-on-scanned-room feature.
//
// Holds tap-positioned RSSI samples for one room, persists them in a sidecar
// file next to the room's .mesh, and computes an inverse-distance-weighted
// (IDW) heat grid over the floor for rendering.
//
// COORDINATES: samples are stored in the room mesh's quantised int16 model
// space (same convention as the .lbl label sidecar) so the survey needs zero
// knowledge of the original scan's world frame. IDW distances therefore work
// in mesh units; the "no data" cutoff is a fraction of the room diagonal
// rather than metres. (Deviation from docs/HEATMAP_SPEC.md section 2, which
// drafted metres — mesh space turned out simpler and self-consistent.)
//
// No display code here — the scanner applet owns all drawing.

#pragma once
#include <stdint.h>

class RfSurvey {
public:
    static const int MAX_SAMPLES = 128;
    static const int GRID_W = 32;
    static const int GRID_H = 24;

    struct Sample {
        int16_t x, z;          // mesh model space (int16), floor plane
        int8_t  rssi;          // averaged dBm
        int8_t  rssi_min, rssi_max;
        uint8_t antenna;       // 0=INT 1=EXT (reserved; INT-only for now)
    };

    // ---- data -----------------------------------------------------------
    void clear();
    bool add(int16_t x, int16_t z, int8_t rssi, int8_t mn, int8_t mx,
             uint8_t antenna = 0);
    int            count() const { return _count; }
    const Sample&  sample(int i) const { return _samples[i]; }

    // Target AP identity (shown in the UI, matched during sampling)
    char    ssid[33] = {0};
    uint8_t bssid[6] = {0};
    uint8_t channel  = 0;

    // ---- persistence: "<room>.rf" beside "<room>.mesh" ------------------
    bool saveBeside(const char* mesh_path);
    bool loadBeside(const char* mesh_path);      // false if absent/invalid

    // Samples recorded for a given antenna (0=INT, 1=EXT).
    int countFor(uint8_t antenna) const {
        int n = 0;
        for (int i = 0; i < _count; ++i) if (_samples[i].antenna == antenna) ++n;
        return n;
    }

    // ---- heat grid ------------------------------------------------------
    // Recompute the IDW grid over the given XZ bounds (mesh space) using ONLY
    // the given antenna's samples — each antenna is its own heat layer. Cells
    // farther than `cutoff_frac` * bounds-diagonal from every such sample are
    // marked "no data" (value 127).
    void computeGrid(int16_t xmin, int16_t xmax, int16_t zmin, int16_t zmax,
                     uint8_t antenna = 0, float cutoff_frac = 0.20f);
    // Grid cell dBm value, or 127 = no data. col in [0,GRID_W), row in [0,GRID_H).
    int8_t gridValue(int col, int row) const { return _grid[row * GRID_W + col]; }

    // dBm -> RGB565 heat colour (green/yellow/orange/red); no-data -> dark grey.
    static uint16_t colorFor(int8_t dbm);

private:
    Sample  _samples[MAX_SAMPLES];
    int     _count = 0;
    int8_t  _grid[GRID_W * GRID_H];

    static void rfPath(const char* mesh_path, char* out, unsigned n);
};
