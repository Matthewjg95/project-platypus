# Build Guide — from parts to first scan

Everything to reproduce Project Platypus, in order. Nothing here assumes
prior M5Stack or PlatformIO experience; it does assume you can run commands
in a terminal.

**Difficulty:** Intermediate. There is no soldering and no fabrication —
assembly is cables and SD cards; the work is flashing two devices and one
config file. Budget an afternoon.

**The external antenna is optional.** Everything — scanning, surveys,
heatmaps — works with the Tab5's internal antenna alone. The external MMCX
port enables the *per-antenna comparison* layer (survey the same room on two
antennas and flip between heatmaps). Any 2.4GHz antenna with an MMCX pigtail
works there; ours is a self-designed patch PCB, but a $3 whip demonstrates
the feature identically.

---

## 0. Desktop prerequisites (before touching hardware)

Any OS works; commands below are shown for Windows PowerShell.

1. **Python 3.10+** — then:
   ```bash
   pip install pyserial pillow
   ```
   (`pyserial` drives flashing/telemetry tools, `pillow` the screenshot
   converter.)
2. **PlatformIO** — either the VS Code extension (easiest) or the CLI:
   ```bash
   pip install platformio
   ```
   The ESP32-P4 toolchain (pioarduino 54.03.21) downloads automatically on
   the first build. First build takes several minutes; later builds ~40s.
3. **USB serial drivers** — the Tab5 enumerates as a standard USB CDC device
   (no driver needed on Win10+/macOS/Linux). Note your COM port in Device
   Manager (ours shows as COM6 throughout the docs).
4. Clone the repo.

## 1. Hardware assembly (10 minutes)

1. **microSD into the Tab5** (FAT32, any size) — rooms, surveys, and
   settings live here.
2. **microSD into the Unit V** (FAT32, ≤32GB) — copy
   `meshscan/unitv/model/20class.kmodel` to the card root so it reads
   `/sd/20class.kmodel` on-device.
3. **Grove cable**: Unit V Grove port → the Tab5's Grove port (the HY2.0-4P
   next to the USB-C). This is the camera's data tether (UART, 460800).
4. Optional: **external antenna** onto the Tab5's MMCX RF connector.
5. Optional: battery / battery sled for untethered scanning.

Strain-relieve the Grove cable (a loop of tape works) — good practice on
any handheld rig with a moving tether.

## 2. Flash the Unit V camera (15 minutes)

The camera runs MaixPy **v0.5.0**. If your Unit V has different firmware,
install v0.5.0 first with kflash/kflash_gui (one-time; M5's docs cover it).

1. Plug the Unit V into USB (shows up as its own COM port — ours is COM10).
2. Build the single-file firmware bundle and upload it:
   ```bash
   cd meshscan/tools
   python unitv_bundle.py
   python unitv_upload.py ../unitv/build/boot.py /flash/boot.py COM10
   ```
3. Done. On boot the camera prints `[tele]`/`[det]` telemetry on its USB
   console (115200) while streaming to the Grove port — handy for sanity
   checks, not required for operation.

Why a bundle? This MaixPy build's boot-time filesystem imports are
unreliable, and `/sd` shadows `/flash` in its module path. One `boot.py`
executed from RAM sidesteps both classes of failure. Camera settings (baud,
model path, detection threshold, anchors) live in `meshscan/unitv/config.py`
— rebuild + re-upload after edits.

## 3. Configure + flash the Tab5 (20 minutes first time)

1. **Create your WiFi credentials header** (gitignored, never committed) at
   `Tab5 3D Render/include/wifi_creds.h`:
   ```cpp
   #pragma once
   #define OTA_WIFI_SSID  "your-network"
   #define OTA_WIFI_PASS  "your-password"
   #define OTA_HOSTNAME   "platypus-tab5"
   ```
   WiFi is used for OTA updates and the RF survey. (Without the file the
   build fails by design — there are no default credentials.)
2. **First flash is over USB.** Put the Tab5 in download mode — hold the
   power button until it enters the bootloader (the back-panel silkscreen
   documents the combos: HOLD = boot/download, double-press = on/off) —
   then:
   ```bash
   cd "Tab5 3D Render"
   pio run -e tab5 -t upload --upload-port COM6
   ```
3. **Known quirk:** after a USB flash the Tab5 often *stays* in download
   mode. Tap the power button once; you'll hear the boot beep and see the
   splash. (An `esptool --chip esp32p4 run` also works.)
4. **Every later flash can be over WiFi** — no cable, no download mode:
   ```bash
   pwsh tools/ota_push.ps1
   ```
   The device accepts OTA only while sitting on the home screen (green
   `OTA ready` chip, top right) — by design, so updates never fight the
   applets that use the radio.

## 4. First scan (5 minutes)

1. Power on → three tiles. Open **Room Scan**.
2. Tap **Scan new room**. Stand in a corner, face it, hold the tablet
   still — a tick sounds when the gyro is calibrated (~1s).
3. Sweep 360° at a comfortable pace, pointing the camera around the room.
   The ring fills as you turn; colored pips drop where objects are found.
   The scan finishes itself at a full turn.
4. The room appears as a 3D carved floor plan. Orbit with a finger,
   pinch to zoom. **OBJ** toggles detected objects; **SCAN+** / **WALK+**
   extend the room; **RF** starts a WiFi survey.
5. For the survey: pick your AP once (tap the AP row), then either walk the
   room during a **WALK+** (samples collect automatically at your position)
   or tap-your-position → **SAMPLE HERE** manually. The heatmap paints over
   the floor plan with a dBm legend and DEAD/BEST callouts; the **ANT**
   button flips between internal/external antenna layers.

## 5. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| "SD card not found" on boot | Tab5 SD not seated; reinsert, restart |
| Scan shows no camera preview | Grove cable seated? Camera powered (LED)? Its USB console should show `[tele]` lines |
| Preview freezes seconds at a time, then recovers | Known intermittent: the link drops into a CRC-error window and the receiver resyncs itself within a few seconds. Root cause under investigation; continuing the sweep rides through it |
| Detections but empty rooms | Sweep slower; ensure the room is well-lit (the K210 is light-hungry); check `[scan] uart ... crcErr` stays low on the serial monitor |
| OTA "no response" | Device must be ON the home screen; same WiFi as your PC |
| Tab5 dark after USB flash | It's parked in download mode — tap power once (§3.3) |
| Serial monitor gibberish | Tab5 CDC needs DTR asserted: the bundled tools set it; PuTTY/minicom must too |

## 6. Optional: debug + capture tooling

- Pixel-perfect device screenshots over USB:
  ```bash
  python "Tab5 3D Render/tools/tab5_screenshot.py" COM6 shot.bmp
  ```
- Timed telemetry capture (what diagnosed the cable fault):
  ```bash
  python "Tab5 3D Render/tools/serial_log.py" COM6 300 run.log
  ```
- 1Hz health lines on the serial monitor: `[scan] uart frames/detects/
  crcErr/resync` during scans, `[hb] up/home/ota/heap` heartbeat always.

## 7. Where to go next

- **`docs/CODE_TOUR.md`** — a guided reading order for the source.
- **`docs/`** roadmap specs — ToF wall ranging, custom detection model.
- Camera tuning: `meshscan/unitv/config.py` + `Tab5 3D Render/docs/DETECTION_TUNING.md`.
