# RF Survey — WiFi Heatmap on a Self-Scanned Room

> **STATUS (2026-07-21): MVP IMPLEMENTED & FLASHED, awaiting hardware test.**
> `src/scanner/rf_survey.h/.cpp` + SURVEY state in `scanner_applet.h`
> ([RF] button in the 3D view). Implementation deviations from this spec:
> samples are stored in the mesh's **int16 model space** (like the .lbl
> sidecar), not metres — the IDW "no data" cutoff is 20% of the room diagonal;
> file magic is **RFS2** with the mesh-space layout; AP selection is
> **auto-strongest** (picker UI deferred); INT/EXT dual-antenna layers
> deferred (antenna field reserved in the format). Untested on hardware:
> WiFi bring-up inside the scanner applet, SD write during survey, and
> C6-link sharing if the antenna applet ran first in the same boot.

The Project Platypus fusion feature and contest centerpiece: scan a room into a
3D mesh (Room Scan), then walk the room sampling WiFi RSSI and paint the signal
onto the mesh you just made. Ekahau needs an imported floor plan; Platypus
draws its own.

**MVP = manual sampling** (user taps their position). SfM pose-tracked
auto-positioning is the roadmap version, not the contest version.

---

## 1. User flow (the contest demo path)

```
BROWSE room list ──open──> VIEW (3D mesh)
                              │  new [SURVEY] button
                              v
                           SURVEY
   ┌───────────────────────────────────────────────┐
   │  left 2/3: TOP-DOWN floor plan of the mesh    │
   │    (room footprint + object markers + past    │
   │     samples as colored discs)                 │
   │  right 1/3: live RSSI readout (big number),   │
   │    AP name, antenna INT/EXT toggle,           │
   │    [SAMPLE HERE] button                       │
   └───────────────────────────────────────────────┘
1. First entry: pick target AP (reuse antenna app's scan-and-pick list).
2. Walk somewhere. Tap your location on the floor plan -> crosshair.
3. Press SAMPLE -> 3 quick single-channel scans, averaged -> disc appears,
   colored by dBm. Auto-saves.
4. 8-12 samples in, toggle [HEAT] -> IDW-interpolated floor heatmap renders
   (2D top-down first; 3D floor painting once renderer supports face colors).
5. Optional wow: switch INT->EXT antenna, resample same spots -> two layers,
   compare internal vs MMCX antenna coverage on the same map.
```

Back button: SURVEY -> VIEW -> BROWSE (existing pattern, reset touch state).

## 2. Data model

```cpp
// One measurement at a user-declared position (room/mesh coordinates, metres,
// same frame as the .mesh: floor plane XZ, origin = scan origin).
struct RfSample {
    float   x, z;          // position on floor plane
    int8_t  rssi;          // averaged dBm of the sample burst
    int8_t  rssi_min, rssi_max;
    uint8_t antenna;       // 0=INT 1=EXT
    uint8_t flags;         // reserved
    uint32_t t_ms;         // millis() at capture (session-relative)
};

struct RfSurvey {
    char     ssid[33];
    uint8_t  bssid[6];
    uint8_t  channel;
    uint16_t count;
    RfSample samples[RF_MAX_SAMPLES];   // 256 is plenty; 3.5 KB
};
```

### On-disk: `<room>.rf` next to `<room>.mesh`
`/meshscan/buildings/<b>/<room>.rf`, little-endian:

```
magic  "RFS1"            4 bytes
ssid   char[33]          NUL-padded
bssid  u8[6]
chan   u8
count  u16
samples: count * 16 bytes  (f32 x, f32 z, i8 rssi, i8 min, i8 max,
                            u8 antenna, u8 flags, u8 pad, u16 pad)
```

Load on entering SURVEY if present; append + rewrite on each sample (file is
tiny, no chunking needed). Delete room -> delete .rf too (project_manager).

## 3. WiFi sampling (scanner side)

Reuse the antenna app's proven recipe, minimally:

```cpp
WiFi.setPins(12, 13, 11, 10, 9, 8, 15);   // Tab5 C6 - once, before mode()
WiFi.mode(WIFI_STA);
// AP pick: full scanNetworks() once, user taps AP (strongest preselected).
// Per sample: 3x single-channel scan (WiFi.scanNetworks(false,false,false,
// SCAN_DWELL, channel)), match BSSID, average RSSI. ~300-500 ms total.
```

- Sampling happens ONLY in SURVEY state (no background scanning; UART/scan
  pipeline untouched).
- INT/EXT toggle drives the RF switch exactly as the antenna applet does
  (`M5.In_I2C` expander write, 1 s settle before sampling).
- Coexistence note: WiFi (C6 SDIO) + SD + UART camera feed have all run
  together already; survey does not use the camera, so no new contention.

## 4. Rendering

### 4a. Top-down floor plan (MVP, pure 2D — ships first)
- Orthographic projection of mesh bounds onto XZ: `px = (x-xmin)/(xmax-xmin)*W`.
- Draw: room rectangle, object markers (from scan's acc list / .mesh objects)
  as small labeled squares, samples as filled discs (r=8) in the dBm color.
- Crosshair at pending tap position.

### 4b. Color scale (shared)
```
>= -55 dBm  green   0x07E0      -70..-80  orange  0xFC00
 -55..-70   yellow  0xFFE0      < -80     red     0xF800
no data     dark grey grid
```

### 4c. Heatmap interpolation (MVP)
Inverse-distance weighting over a coarse floor grid:
- Grid: 32 x 24 cells over mesh XZ bounds (fits any room aspect).
- `value(cell) = sum(w_i * rssi_i) / sum(w_i)`, `w_i = 1/(d_i^2 + eps)`;
  cells farther than ~2.5 m from every sample render as "no data".
- 32*24*~10 samples = trivial compute; recompute on each new sample.
- 2D view: fillRect per cell at 60% opacity feel (blend by drawing the grid
  first, discs/objects on top).

### 4d. 3D floor painting (stretch, after 2D works)
One contained renderer extension unlocks this AND Phase-2 colored geometry:

```cpp
struct Mesh {
    ...
    uint16_t* face_colors = nullptr;   // optional; RGB565 per face
};
// renderer.h: color = mesh.face_colors ? tint(face_colors[i], shade)
//                                      : _shade_color(rn);
```

Then SURVEY builds a floor-grid sub-mesh (2 triangles per cell, y=0.01) with
`face_colors` from the IDW grid and appends it to the room mesh for the 3D
view. Billboard-free, no new draw path.

## 5. Code layout

```
src/scanner/rf_survey.h/.cpp   RfSurvey store, .rf IO, IDW grid, color scale
src/scanner/scanner_applet.h   + SURVEY state: touch handling, 2D draw,
                               sample flow, AP picker (modeled on antenna's)
src/renderer.h                 + optional face_colors (stretch item 4d)
```

Keep rf_survey.* free of display calls (testable), scanner_applet owns UI —
same split as the rest of the scanner.

## 6. Effort plan

| Step | Scope | Est |
|------|-------|-----|
| 1 | rf_survey store + .rf IO + color scale | 0.5 d |
| 2 | SURVEY state: top-down view, tap, crosshair | 1 d |
| 3 | WiFi sample burst + AP picker | 0.5 d |
| 4 | IDW grid + 2D heat overlay | 0.5 d |
| 5 | INT/EXT dual-layer surveys | 0.5 d |
| 6 | (stretch) face_colors renderer + 3D floor paint | 1 d |

MVP (1-5) ≈ 3 days of focused work; contest-demoable after step 4.

## 7. Roadmap (post-contest)
- SfM pose-tracked sampling: Phase-2 pipeline already estimates camera pose;
  tag RSSI to live pose while walking -> dense heatmap, no tapping.
- Multi-AP surveys (store N surveys per room, switch layers).
- Channel-congestion overlay (antenna app's channel scanner data on the map).
- Export: .rf -> CSV/PNG via a desktop tool in meshscan/tools/.
```
