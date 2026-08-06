# Code Tour — a guided reading order

The codebase is written to be read: headers open with a purpose block
explaining *why* the module exists, and hard-won lessons are documented at
the line where they were learned (search the source for `HISTORY`,
`LIFECYCLE RULE`, `hard-learned`, `field report` — the comments carry the
war stories). This tour orders the reading so each file makes sense by the
time you reach it.

One honest caveat up front: most modules are small and single-purpose, but
`scanner_applet.h` is the orchestrator and it is *large* (~2,500 lines). It
is organized by scan-screen state; the section map below is your index into
it.

## The 30-second architecture

```
 Unit V camera (K210, MaixPy)                Tab5 (ESP32-P4, Arduino)
 ───────────────────────────                 ─────────────────────────────
 sensor → KPU YOLOv2 → JPEG      UART        UartReceiver → FrameBuffer
        (every Nth frame)      460800 baud            │
 FRAME / DETECT packets, CRC-8  ────────►    ScannerApplet (state machine)
                                             │  DepthEstimator   (range)
                                             │  RoomObjDB        (persist+register)
                                             │  PoseTracker      (walk pose)
                                             │  room_carve       (floor evidence)
                                             │  ScanGeometry     (triangles)
                                             │  ScanMeshWriter   (quantise+save)
                                             │  RfSurvey         (heatmap layers)
                                             ▼
                                    M5View shell ← Viewer / Antenna applets
```

## Reading order

### 1. The platform (20 minutes)

| File | What you learn |
|---|---|
| `src/applet.h` | The whole applet contract: enter/update/render/back/pause/resume. Everything else plugs into this. |
| `src/shell.h` | Tile launcher, settings panel, touch routing. Why open/close is instant (animation trails were a bug class). |
| `src/main.cpp` | Boot order, the home-screen-only OTA lifecycle (begin-once rule), watchdog, serial screenshot protocol. |

### 2. The renderer (30 minutes)

| File | What you learn |
|---|---|
| `src/mesh.h` | int16 quantised vertices + faces; the format everything downstream targets. |
| `src/mesh_loader.h` | SD → PSRAM streaming load. |
| `src/renderer.h` | Transform → backface cull → flat shade → painter's sort → rasterise. `docs/RENDERER_INTERNALS.md` goes deeper. |
| `src/viewer_applet.h` | The renderer wrapped as an applet; trackball math in `vm3.h`. |

### 3. The camera link (20 minutes)

| File | What you learn |
|---|---|
| `meshscan/unitv/config.py` | Every camera knob: baud, model path, anchors, threshold, cadence. |
| `meshscan/unitv/main.py` + `uart_protocol.py` | Capture loop and the FRAME/DETECT framing (CRC-8). |
| `src/scanner/uart_receiver.*` | The Tab5 end: resync state machine, CRC validation, the 32KB-before-begin RX buffer lesson. |
| `src/scanner/frame_buffer.*` | Slots pairing a JPEG with its detections by frame id. |

### 4. The scanning brain (1 hour, the good part)

Read the small pure-logic modules first — none of them knows M5 exists:

| File | What you learn |
|---|---|
| `src/scanner/depth_estimator.*` | Monocular ranging from known object sizes + the box sanity gates (min size, aspect, edge-clip, range ceiling). |
| `src/scanner/room_objdb.h` | The per-room object database; observation merging; **landmark registration** (yaw × same-class-pair search) that lets rescans align themselves. |
| `src/scanner/pose_tracker.h` | Step detection → dead-reckoned walk pose. Feed it accel magnitude + heading; nothing else. |
| `src/scanner/room_path.h` | The walked path sidecar (floor-truth evidence). |
| `src/scanner/room_carve.h` | The visibility carve: origin + sight-lines + paths stamped into an occupancy grid. Why rooms come out L-shaped. |
| `src/scanner/scan_geometry.*` | Occupancy grid → greedy-meshed floor + markers + origin arrow. |
| `src/scanner/scan_mesh_writer.*` | World metres → int16 quantisation (the Y-mirror and winding notes matter). |
| `src/scanner/rf_survey.*` | Survey samples with **bearing** + capture quantisation; IDW heat grid; why RFS4 remaps on rebuilds. |

Then the orchestrator, with this index:

**`src/scanner/scanner_applet.h` section map** (in file order):
- BROWSE — room list, scroll, long-press delete, the Scan/Live/Map buttons
- SCAN — sweep dial, stillness-locked bias calibration, IMU yaw
  integration, walk mode (pose tick + RF sampling), `_sample()` where
  detections become observations
- `_write_phase1` / `_rebuild_from_db` — where a scan becomes a mesh:
  registration → merge → carve → geometry → quantise → sidecars
- VIEW — orbit render, label billboards, OBJ menu (per-room object
  toggles), SCAN+/WALK+/RF buttons
- BUILDING MAP — footprint thumbnails, drag-to-arrange, persistence
- RF SURVEY — AP picker, sampling, heat layers, DEAD/BEST callouts,
  bearing readout
- LIVE — detection viewfinder

### 5. The antenna suite (optional, self-contained)

`src/antenna/antenna_applet.cpp` is a deliberately self-contained fullscreen
applet (scope/channel/polar/walk screens) — it predates the shell and shows
what "absorb an existing sketch as an applet" looks like. `src/rf_switch.h`
is the I2C expander driver with the register-0x01 warning.

### 6. Phase-2 scaffolding

`src/scanner/cv/` — a portable SfM pipeline (ORB → matching → essential
matrix/RANSAC → triangulation → surface) that compiles and runs but isn't
wired into the product yet. It exists so evidence-based walls have a
landing site; treat it as reference, not as load-bearing.

## Conventions worth knowing

- **Portable vs device code**: anything under `scanner/` that doesn't
  include M5Unified is hardware-agnostic by design — the swap-the-hardware
  principle is enforced at include level.
- **Sidecar files are versioned**: `.objs` (ROB2), `.path` (PTH1), `.rf`
  (RFS4), labels (LBL1) — every loader accepts its older versions.
- **Comments explain *why*, at the site of the lesson** — the UART buffer,
  the expander reset bit, the OTA begin-once rule, the bias stillness lock
  are all documented where they bit us.
