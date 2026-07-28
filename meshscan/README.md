> **ARCHIVED / reference.** The active M5Tab firmware now lives in the
> `Tab5 3D Render` project as a `ScannerApplet` (`src/scanner/`), integrated with
> the real ESP32-P4 / M5GFX renderer and using **its** native `.mesh` format.
> This `meshscan/` tree is kept for internal use and Phase-2 reference: the
> standalone format spec (`shared/mesh_format.h`), the `mesh_to_obj`/`kmodel_info`
> tools, and the original ESP32-S3/LVGL scaffold. The portable logic (CV pipeline,
> UART protocol, depth/room-fit, Unit V firmware) was copied into the Tab5 project.
> The Unit V firmware in `unitv/` is the live copy (the camera device is unchanged).

# MeshScan

Room-scanning system: an **M5Stack Unit V** (Kendryte K210) AI camera streams
JPEG frames + YOLO object detections over UART to an **M5Tab** (ESP32-S3), which
fits a room box, places detected objects by monocular depth, builds a `.mesh`,
and renders it in 3D. Scans are organised into buildings → rooms on SD.

```
Unit V (K210, MaixPy)            M5Tab (ESP32-S3, PlatformIO/Arduino)
  OV2640 ─ KPU YOLOv2              UART ─ frames+detections
     │                               │
     └── FRAME + DETECT packets ─────┴─► room fit → depth → mesh → SD → renderer
```

## Layout

| Path | What |
|------|------|
| `shared/mesh_format.h` | The `.mesh` binary format + in-memory render structs. **Source of truth.** |
| `unitv/` | MaixPy firmware (camera + YOLO + UART stream). |
| `m5tab/` | PlatformIO project (receive → fit → mesh → render → UI). |
| `m5tab/lib/MeshRenderer/` | Where your existing 3D renderer plugs in (stub provided). |
| `tools/` | Desktop helpers: `mesh_to_obj.py`, `kmodel_info.py`. |

## Build & flash (VS Code)

**M5Tab** — open `m5tab/` with the PlatformIO extension, then *Upload*.
Adjust pins/board in `platformio.ini` for your exact unit, and replace the LVGL
display/touch scaffold in `src/main.cpp` (marked `TODO`) with your panel driver
(M5GFX/LovyanGFX). Drop your renderer into `lib/MeshRenderer/` and remove the
stub (see that folder's README).

**Unit V** — copy `unitv/*.py` to the device with the MaixPy IDE or `ampy`, and
put your COCO-80 `.kmodel` at the path in `unitv/config.py`. `main.py` autoruns.

## Wire protocol (Unit V → M5Tab, little-endian)

```
FRAME   0xF1 | payload_len:u16 | jpeg... | crc8
DETECT  0xD1 | frame_id:u16 | object_count:u8 |
              [class_id:u8 conf:u8 x:u16 y:u16 w:u16 h:u16]*count | crc8
```
CRC-8 (poly 0x07, init 0x00) covers everything after the magic byte up to the
CRC. DETECT always follows its paired FRAME. Implemented in
`unitv/uart_protocol.py` and `m5tab/src/uart_receiver.cpp` (validated to agree).

## Phase 1 (implemented, end-to-end)

1. Unit V streams 320×240 JPEG + YOLO detections.
2. M5Tab receives/decodes, shows live preview with overlays.
3. *Scan* collects ~20 frames over a ~20 s 360° sweep.
4. `room_fitter` runs Sobel→Hough per frame; fits a rectangular box (footprint
   from object extents, confidence from Hough orientation clustering).
5. `depth_estimator` turns known object heights into distances + world XZ.
6. `mesh_builder` places the box room + object markers.
7. `mesh_writer` saves `/meshscan/buildings/<name>/<room>.mesh` in 4 KB chunks.
8. `mesh_renderer` loads it (float→int16, per-face normals) into your renderer.
9. `project_manager` + UI: buildings/rooms browse, scan, open, delete.

> Phase-1 room geometry is a deliberate heuristic — a single monocular pass
> can't recover true scale, so object distances anchor the box.

## Phase 2 (implemented — monocular SfM)

Real structure-from-motion in `m5tab/src/cv/`, plus an incremental writer:

| Module | Role |
|--------|------|
| `cv/orb_features` | FAST-9 corners + intensity-centroid orientation + rotated BRIEF-256 |
| `cv/feature_matcher` | Hamming brute-force + Lowe ratio + mutual cross-check |
| `cv/two_view_geometry` | RANSAC homography (degeneracy gate) + normalised 8-point essential + pose recovery |
| `cv/triangulator` | DLT triangulation |
| `cv/point_cloud` | PSRAM world-frame point accumulator |
| `cv/surface_recon` | greedy kNN triangulation → faces (streamed out) |
| `cv/slam_pipeline` | orchestrates per-frame: detect → match → pose → triangulate → accumulate |
| `mesh_stream_writer` | incremental `.mesh` write with count backpatching (verts/faces never fully in RAM) |
| `cv/linalg` | Jacobi eigensolver + 3×3 SVD (no Eigen) |

The geometry core is validated against a synthetic scene (rotation/translation
error 0.0°, epipolar residual ~1e-16) and the writer layout round-trips through
`tools/mesh_to_obj.py`.

**Enable it:** flip the runtime flag via `app_set_scan_phase2(true)` (wire a UI
toggle — TODO), or build with `-DMESHSCAN_PHASE2_DEFAULT`. Default is the robust
Phase-1 box fit.

> ⚠️ **Capture motion matters.** Triangulation needs a translation *baseline*. A
> camera rotating in place has none, so the essential matrix degenerates and
> depth is meaningless — the pipeline detects this (`planarity` near 1) and skips
> such pairs. For real geometry, **translate** the camera between frames (strafe
> along walls, or orbit around objects), don't just spin on the spot.

### Phase 2 — remaining TODOs

- Scale consistency: per-pair scale is fixed at 1 (drifts). Anchor with
  `SlamPipeline::setScale()` from a known object distance; full fix needs bundle
  adjustment + loop closure.
- Spatial grid for kNN in `surface_recon` (currently O(n²)).
- Sample real per-point colour from the source frame.
- Oriented normals / ball-pivoting / hole filling for a watertight surface.

## Tools

```bash
python tools/mesh_to_obj.py room.mesh room.obj   # -> Blender-ready .obj
python tools/kmodel_info.py model.kmodel         # inspect a K210 model
```
