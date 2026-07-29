# Project Platypus 🦫

Turning the **M5Stack Tab5** into an amalgamation of interests — a multi-app
touch platform that scans rooms in 3D with an AI camera, maps WiFi signal onto
the rooms it scanned, renders meshes, and tests antennas. An exercise in
pushing the Tab5 (ESP32-P4) and friends to their limits.

```
┌─────────────────────────────  M5View shell  ─────────────────────────────┐
│  3D Viewer          Room Scan                    Antenna                 │
│  .mesh renderer     UnitV camera → YOLO →        2.4GHz RSSI scope,      │
│  (orbit/zoom)       labeled 3D room meshes,      channel scan, polar     │
│                     RF heatmap surveys           plot, INT/EXT switch    │
└──────────────────────────────────────────────────────────────────────────┘
        Tab5: ESP32-P4 @360MHz, 1280x720 touch, BMI270 IMU, ESP32-C6 WiFi
        UnitV: Kendryte K210 (KPU NPU), OV7740, MaixPy — tethered via Grove UART
```

## What it does

- **Room Scan** — hold the tablet, sweep 360° (the BMI270 gyro measures the
  sweep; scan auto-completes at a full turn). The tethered UnitV camera streams
  JPEG + YOLOv2 detections (VOC-20, on-KPU) over 460800-baud UART; the Tab5
  estimates object positions from known heights + measured bearing and builds a
  labeled 3D mesh (floor plate + object markers + name billboards).
  **Additive scanning:** rescanning a room registers the new sweep against the
  room's persistent object database (yaw+translation solved from the objects
  themselves) and merges — rooms improve with every walk.
- **RF Survey** — open a scanned room, tap `RF`: walk the room, tap your
  position on the auto-generated floor plan, sample the target AP's RSSI.
  An IDW heat grid paints over the plan; **internal and external (MMCX)
  antennas record separate flippable heatmap layers** via the Tab5's RF
  switch. Surveys persist per room. (Ekahau needs an imported floor plan;
  Platypus draws its own.)
- **Live View** — realtime detection viewfinder: camera feed with
  class-colored boxes and name+confidence tags.
- **3D Viewer** — the original mesh renderer (int16 vertices, precomputed
  face normals, bucket-sorted painter's algorithm) for `.mesh` files from SD;
  `tools/stl_to_mesh.py` converts STLs.
- **Antenna** — 2.4GHz test suite (RSSI scope, channel scanner, polar plot,
  walk test, multipath, A/B dwell) using the Tab5's internal/external antenna
  switch.

## Repo layout

```
Tab5 3D Render/          the Tab5 firmware (PlatformIO, Arduino, ESP32-P4)
  src/                   shell + applets (viewer, scanner/, antenna/)
  src/scanner/cv/        portable SfM pipeline for Phase 2 (ORB, RANSAC,
                         essential matrix, triangulation, surface recon)
  docs/                  engineering docs: heatmap spec, detection tuning,
                         debug postmortems
  tools/                 stl_to_mesh.py, tab5_screenshot.py
meshscan/
  unitv/                 UnitV (K210/MaixPy) camera firmware — deploy as a
                         single bundled boot.py
  tools/                 unitv_bundle.py, unitv_upload.py, unitv_fix_sd_shadow.py
  (rest)                 archived first-generation reference implementation
M5Tab_AntennaTest.../    original standalone antenna sketch (pre-applet)
```

## Building & flashing

**Tab5** (VS Code + PlatformIO):
```bash
cd "Tab5 3D Render"
pio run -e tab5 -t upload                 # over USB (COM port)
pio run -e tab5-ota -t upload             # over WiFi once OTA firmware is on
```
- Create `Tab5 3D Render/include/wifi_creds.h` (gitignored) with
  `OTA_WIFI_SSID / OTA_WIFI_PASS / OTA_HOSTNAME` before building.
- OTA works while the device sits on the M5View home screen (green
  `OTA ready <ip>` chip). 60s task watchdog + serial heartbeat are built in.

**UnitV camera** (MaixPy v0.5.0 on the K210):
```bash
cd meshscan/tools
python unitv_bundle.py                    # bundles modules into one boot.py
python unitv_upload.py ../unitv/build/boot.py /flash/boot.py COM10
```
- The bundle exec-loads every module from RAM at boot (this MaixPy build's
  boot-time filesystem imports are unreliable) and makes SD-card shadowing
  impossible.
- Detection model: `meshscan/unitv/model/20class.kmodel` (tiny-YOLOv2 VOC-20,
  from kendryte-standalone-demo) goes on the camera SD as `/sd/20class.kmodel`.

**Debug niceties**
- `python "Tab5 3D Render/tools/tab5_screenshot.py" COM6 out.bmp` — send `SS`
  over USB, receive a pixel-perfect 1280x720 framebuffer capture.
- Serial heartbeat `[hb] up/home/ota/heap` every 5s pinpoints any lockup.

## Hard-won hardware notes (the short list)

- **Tab5 WiFi**: the P4 has no radio — WiFi rides an ESP32-C6 over hosted
  SDIO. Arduino defaults target the P4 EV-board; the Tab5 needs
  `WiFi.setPins(12, 13, 11, 10, 9, 8, 15)` *before* any WiFi call.
- **ArduinoOTA/mDNS must init at most once per boot** on hosted WiFi —
  an end()/begin() cycle hard-froze the loop task.
- **UART RX buffer**: `setRxBufferSize()` is a no-op *after* `begin()`.
  Real-room JPEGs (~14KB) overflowed the default buffer → CRC storm. 32KB,
  set before begin.
- **PI4IOE5V6408 expander (RF switch, 0x43)**: register 0x01 is *Chip
  ID/Control with bit0 = software reset* — writing it blanks the display
  (other rails share the chip). Output register is 0x05; Hi-Z (0x07) must be
  cleared to drive a pin.
- **K210/MaixPy**: `/sd` precedes `/flash` in sys.path (stale SD files shadow
  firmware); boot-time file opens fail intermittently on some files; ampy
  crawls at ~1KB/s and relative paths land on `/sd` (cwd) — hence the bundler
  + custom uploader.

## Status & roadmap

Working end-to-end today: scan → detect → labeled mesh → per-antenna WiFi
heatmap, plus OTA, screenshots, watchdog. Roadmap: detection upgrade (modern
kmodel V4 / custom classes — doors!), Phase-2 SfM geometry (evidence-based
walls), SfM-pose-tracked dense heatmaps, and the M5Stack Global Innovation
Contest 2026 entry.

---
*Built collaboratively with Claude (Anthropic) driving firmware, debugging,
and tooling over serial — including taking its own screenshots of the device.*
