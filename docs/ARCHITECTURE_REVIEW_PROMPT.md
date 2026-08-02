# External Architecture Review Prompt

Paste the block below into ChatGPT (or any reviewer) for an outside opinion on
Project Platypus's architecture. It is self-contained — no repo access needed.

---

You are reviewing the architecture of "Project Platypus," an embedded
room-mapping + RF-survey instrument built by a solo developer for the M5Stack
Global Innovation Contest. Review it as a senior embedded/robotics architect:
identify structural weaknesses, risky assumptions, and the highest-leverage
improvements. Be direct and specific; prioritize recommendations by
impact-per-effort. Do not restate the design back to me.

## Hardware
- M5Stack Tab5: ESP32-P4 (RISC-V, no radio), 1280x720 touch, BMI270 IMU,
  speaker, SD card. WiFi via onboard ESP32-C6 over SDIO ("hosted" WiFi).
  External antenna port with an I2C-expander-controlled RF switch; a custom
  antenna PCB mounts on the back.
- M5 Unit V (Kendryte K210) AI camera, tethered over Grove UART at 460800
  baud (~6-7 fps JPEG). Runs MaixPy v0.5.0 + tiny-YOLOv2 VOC-20 kmodel
  (real-world confidences only 16-27%). Sends JPEG frames + detections in a
  small framed binary protocol (CRC-8).
- Battery sled; future: VL53L1X ToF ranger on the internal I2C bus (exposed
  on the M5-Bus header), 3D-printed enclosure with a swappable sensor pod.

## Software structure (C++/Arduino via PlatformIO)
- Shell ("M5View"): tile launcher hosting applets behind a small interface
  (on_enter/update/render/back/pause/resume). Applets: 3D mesh Viewer, Room
  Scan, RF Survey (inside Room Scan), fullscreen Antenna test suite,
  ShadowScan (silhouette -> extruded STL).
- Room Scan pipeline (Phase 1, deliberately monocular-heuristic):
  1. User stands at a room corner (origin convention), sweeps 360 in place.
     Yaw = BMI270 gyro projected onto gravity, integrated; bias calibrated
     from a stillness-locked 900ms window; 2 deg/s deadband.
  2. Camera detections (every Nth frame) are ranged monocularly from known
     per-class real-world sizes (width preferred for wide furniture),
     placed at world XZ via yaw + pixel offset. Sanity gates: min box size,
     edge-clip fallback, aspect-ratio check, 8m ceiling.
  3. Observations accumulate per-object (same class within 0.6m merges);
     position = component-wise median of first 9 samples. An object needs
     2+ observations (4+ for "person") to earn a map marker.
  4. Rooms persist as: quantized int16 triangle mesh (custom .mesh format)
     + sidecars: .lbl (label anchors), .objs (object DB: class, world pos,
     observation count, visibility flag), .rf (RSSI samples per antenna).
  5. Additive rescans: new scan registers onto the room's object DB by
     brute-force yaw x same-class-pair translation search (landmarks =
     objects), then merges. Per-room object on/off menu rebuilds the mesh.
  6. Floor geometry: "visibility-carved" occupancy grid (0.25m cells) -
     stamped discs at origin + objects + sight-line corridors between them;
     meshed as greedy maximal rectangles (top surface + boundary skirts).
     Non-rectangular rooms emerge without wall sensing.
- RF Survey: taps on the room mesh record RSSI (per antenna: internal/
  external), IDW-interpolated 32x24 heatmap painted over the floor.
- Dev infra: WiFi OTA (home-screen-only, begin-once lifecycle), serial
  screenshot channel, 60s task watchdog, serial heartbeat.

## Known constraints and pain points
- K210 model is weak (VOC-20, 16-27% conf); person class overfires. Custom
  model via MaixHub is roadmapped.
- Rotation-only sweep = zero parallax, so no true depth; all ranging rides
  on class-size priors. ToF wall ranging is the planned fix.
- Camera link is one-way telemetry at 6-7fps; 921600 baud was unstable.
- Mesh format is int16-quantized with a per-mesh scale; labels/objects
  live in sidecars rather than the mesh container.
- Flash is at ~88% of a 1.25MB OTA slot (16MB chip; partition table
  upgrade pending).
- Everything is single-threaded on the Arduino loop task (16KB stack);
  UART drain competes with display pushes (RX overflow was an issue).
- Registration is O(360/10 x N^2) brute force; fine for N<=64 objects.

## Product goal
Contest demo (Aug 7): self-scanned floor plans with object markers + WiFi
heatmap layers painted on them ("WiFi weather map drawn by the device that
measured it"). Long-term: walk-scans (pose-tracked, non-360), multi-room
building assembly, ToF-measured wall polygons, custom detection model.

## Questions for you
1. Which architectural decisions will hurt most as the project scales to
   walk-scans and multi-room buildings, and what should change now vs later?
2. Is the object-landmark registration approach sound, or should we move to
   something else before adding more scan modes?
3. How would you restructure the storage model (.mesh + 3 sidecars) — is a
   single container format worth it, and what would you use on-device?
4. What's the highest-impact improvement to mapping quality that does NOT
   require new hardware?
5. Any embedded-systems red flags in the concurrency/dataflow story
   (single loop task, UART drain vs display, hosted WiFi)?
6. What would you cut or simplify — where is this over-engineered for what
   it needs to be?
