# Project Platypus 🦫

**A self-mapping RF instrument.** An M5Stack **Tab5** with a tethered AI
camera scans a room into a floor plan — then measures the WiFi and paints the
signal onto the map it just drew, one heat layer per antenna, including a
custom patch antenna PCB. No imported floor plans anywhere in the loop: the
instrument draws its own.

Built for the **M5Stack Global Innovation Contest 2026**, as an exercise in
pushing the Tab5 (ESP32-P4) and friends to their limits.

```
┌─────────────────────────────  M5View shell  ─────────────────────────────┐
│  3D Viewer          Room Scan                    Antenna                 │
│  .mesh renderer     UnitV camera → YOLO →        2.4GHz RSSI scope,      │
│  (orbit/zoom)       carved floor plans,          channel scan, polar     │
│                     WALK+ scans, building map,   plot, INT/EXT switch    │
│                     RF heatmap surveys                                   │
└──────────────────────────────────────────────────────────────────────────┘
        Tab5: ESP32-P4 @360MHz, 1280x720 touch, BMI270 IMU, ESP32-C6 WiFi
        UnitV: Kendryte K210 (KPU NPU), OV7740, MaixPy — tethered via Grove UART
```

## What it does

- **Sweep scan** — stand in a corner, sweep 360°. The BMI270 gyro measures
  the turn (stillness-locked bias calibration, gravity-projected so grip
  doesn't matter); a **sweep dial** fills as you rotate, dropping a colored
  pip at the bearing of every object the camera confirms, and the scan
  finishes itself at a measured full turn. The tethered UnitV streams JPEG +
  YOLOv2 detections over 460800-baud UART; objects are ranged monocularly
  from known real-world sizes (width preferred for wide furniture), passed
  through box sanity gates, and placed by the **median** of their sightings.
- **Visibility-carved floors** — no wall sensor, so the floor is built from
  evidence: where you stood, everything you saw (each sight-line proves open
  floor between you and the object), and everywhere you walked. Rooms come
  out L-shaped when they are L-shaped — not bounding boxes.
- **WALK+** — extend a scanned room on foot. IMU step detection dead-reckons
  your path (drift absorbed by registering against the room's own objects as
  landmarks); the walked path becomes floor-truth for the carve, and —
- **Walk-fused RF survey** — while you walk, the target AP is sampled at
  your pose *and bearing*, so **one walk produces the room and its heatmap**.
  Manual tap-to-sample still works. Heat layers are per-antenna (internal vs
  the custom MMCX patch via the Tab5's RF switch), with a dBm legend and
  automatic DEAD / BEST spot callouts. Because every sample records its aim,
  the directional patch is measured honestly — and its strongest bearing
  draws an "AP this way?" ray. (Ekahau needs an imported floor plan;
  Platypus draws its own.)
- **Additive scanning** — rescanning registers new observations against the
  room's persistent object database (yaw + translation solved from the
  objects themselves) and merges. Rooms improve with every pass; a per-room
  **object menu** toggles anything the model found on/off, and the floor
  re-carves to match.
- **Building map** — every room's carved footprint on one top-down canvas;
  drag rooms to arrange your building, tap to open. Arrangements persist.
- **Live view** — realtime detection viewfinder with class-colored boxes.
- **3D Viewer** — the original mesh renderer (int16 vertices, precomputed
  normals, bucket-sorted painter's algorithm) for `.mesh` files from SD;
  `tools/stl_to_mesh.py` converts STLs.
- **Antenna** — 2.4GHz test suite (RSSI scope, channel scanner, polar plot,
  walk test, multipath, A/B dwell) driving the internal/external switch.

## Repo layout

```
Tab5 3D Render/          the Tab5 firmware (PlatformIO, Arduino, ESP32-P4)
  src/                   M5View shell + applets (viewer, scanner/, antenna/)
  src/scanner/cv/        portable SfM pipeline for Phase 2 (ORB, RANSAC,
                         essential matrix, triangulation, surface recon)
  docs/                  renderer internals, heatmap design, detection
                         tuning protocol, antenna/WiFi postmortem
  tools/                 stl_to_mesh, screenshots, OTA push, serial log
meshscan/
  unitv/                 UnitV (K210/MaixPy) camera firmware — deploys as a
                         single bundled boot.py
  tools/                 unitv_bundle, unitv_upload, mesh_to_obj, kmodel_info
  shared/                standalone .mesh format spec
docs/                    roadmap docs: ToF wall ranging, RF directionality,
                         custom detection model walkthrough
assets/                  build photos + device screenshots
```

## Building & flashing

**Tab5** (PlatformIO):
```bash
cd "Tab5 3D Render"
pio run -e tab5 -t upload                 # over USB
pwsh tools/ota_push.ps1                   # over WiFi (device on home screen)
```
- Create `Tab5 3D Render/include/wifi_creds.h` (gitignored) with
  `OTA_WIFI_SSID / OTA_WIFI_PASS / OTA_HOSTNAME` before building.
- OTA arms only on the M5View home screen (green `OTA ready` chip), so it
  never fights the applets that own the radio. 60s watchdog + serial
  heartbeat built in.

**UnitV camera** (MaixPy v0.5.0 on the K210):
```bash
cd meshscan/tools
python unitv_bundle.py                    # bundles modules into one boot.py
python unitv_upload.py ../unitv/build/boot.py /flash/boot.py COM10
```
- The bundle exec-loads every module from RAM at boot (this MaixPy build's
  boot-time filesystem imports are unreliable) and sidesteps SD shadowing.
- Detection model: tiny-YOLOv2 VOC-20 (`meshscan/unitv/model/20class.kmodel`,
  from kendryte-standalone-demo) goes on the camera SD as
  `/sd/20class.kmodel`.

**Debug niceties**
- `python "Tab5 3D Render/tools/tab5_screenshot.py" COM6 out.bmp` — send
  `SS` over USB, get a pixel-perfect 1280x720 framebuffer capture.
- `tools/serial_log.py` — timed telemetry capture (it diagnosed a camera
  "freeze" as a cable-strain CRC storm: frames stalled while CRC errors
  climbed, proving corrupt bytes on the wire, then flat-zero errors after).
- Serial heartbeat `[hb] up/home/ota/heap` every 5s pinpoints any lockup.

## Hard-won hardware notes (the short list)

- **Tab5 WiFi**: the P4 has no radio — WiFi rides an ESP32-C6 over hosted
  SDIO. Arduino defaults target the P4 EV-board; the Tab5 needs
  `WiFi.setPins(12, 13, 11, 10, 9, 8, 15)` *before* any WiFi call.
- **ArduinoOTA/mDNS must init at most once per boot** on hosted WiFi —
  an end()/begin() cycle hard-froze the loop task.
- **UART RX buffer**: `setRxBufferSize()` is a no-op *after* `begin()`.
  Real-room JPEGs (~8KB) overflowed the default buffer → CRC storm. 32KB,
  set before begin.
- **PI4IOE5V6408 expander (RF switch, 0x43)**: register 0x01 is *Chip
  ID/Control with bit0 = software reset* — writing it blanks the display
  (other rails share the chip). Output register is 0x05; Hi-Z (0x07) must be
  cleared to drive a pin.
- **K210/MaixPy**: `/sd` precedes `/flash` in sys.path (stale SD files shadow
  firmware); boot-time file opens fail intermittently on some files; ampy
  crawls at ~1KB/s — hence the bundler + custom uploader.
- **A directional antenna breaks position-only surveys**: patch RSSI is a
  function of position AND aim (10–15dB front-to-back), so every sample
  records its bearing — omni layers ignore it, the patch layer requires it.

## Roadmap

Measured wall polygons via a ToF ranger on the M5-Bus I2C (the sweep dial
becomes a live floor-plan radar — see `docs/TOF_RANGING.md`), a custom
detection model with door/window classes (`docs/CUSTOM_MODEL_WALKTHROUGH.md`),
Phase-2 SfM walls, and a printed enclosure with a swappable sensor pod.

---
*Built collaboratively with Claude (Anthropic) driving firmware, debugging,
and tooling over serial — including taking its own screenshots of the device.*
