# ToF Wall Ranging — Research & Integration Plan

> **STATUS (2026-08-01): post-contest revision.** ToF hardware + custom
> enclosure are roadmapped for after the Aug 7 contest deadline. Until then
> the camera-only path carries layout: visibility-carved floors (occupancy
> union of origin + objects + sight-lines) replaced the rectangular plate.

**Goal:** during the same 360° sweep-dial scan, sample a real distance at every
bearing → the scan produces the room's **actual measured wall outline** (a polar
polygon) instead of a bounding plate inferred from object extents. Objects from
the camera land inside it; the RF heatmap paints onto true room geometry.

This is the single biggest layout upgrade available: rotation-only sweeps give
the camera zero parallax (no depth from motion is possible), and the K210 only
emits 2D boxes — but a ToF ranger measures distance directly.

---

## Hardware findings (researched 2026-08-01)

### The bus problem is solved — no Grove conflict

The Unit V camera owns the Grove port's pins (G53/G54) as UART, and the U006
hub is a passive splitter — UART + I2C can never share those pins. But the
Tab5's **M5-Bus 30-pin header exposes the internal I2C bus directly**:

| M5-Bus pin | Signal |
|-----------|--------|
| SDA | **G31** |
| SCL | **G32** |
| 5V / 3V3 / GND | power |

G31/G32 is the same bus as the BMI270 IMU, RTC, touch, audio codec, and the
PI4IOE5V6408 expanders (0x43/0x44) — i.e. **`M5.In_I2C`, the exact API the
antenna applet already uses**. Proven code path, zero new drivers to bring up.

Address check — internal bus occupants: 0x10, 0x14/0x55, 0x32, 0x40, 0x41,
0x43, 0x44, 0x68. The ToF's **0x29 is free**. Traffic check: M5Unified
serializes In_I2C transactions; a 20–30 Hz ToF poll is light next to touch+IMU.

### Sensor choice

| Unit | Sensor | Range | Notes |
|------|--------|-------|-------|
| **Unit ToF4M (recommended)** | VL53L1X | 4 mm – 4 m | ~30–50 ms/reading, I2C 0x29, ~27° FoV (narrowable to ~15° via ROI) |
| Unit ToF | VL53L0X | up to 2 m | cheaper, but 2 m is too short for wall ranging across a room |
| Mini ToF 90° | VL53L0CXV0DH | 2 m | right-angle form factor — interesting for the enclosure, same 2 m limit |

**ToF4M** it is: 4 m covers typical rooms from the center; 0x29 on In_I2C.

### Wiring

Grove pigtail (Grove-to-Dupont cable, ~$2) from the ToF4M onto the M5-Bus
header: 5V, GND, SDA→G31, SCL→G32. Grove I2C logic is 3.3 V — matches the
internal bus. The enclosure routes/strains this cable.

### BOM

- M5Stack Unit ToF4M (VL53L1X) — ~$9
- Grove-to-Dupont conversion cable — ~$2
- (already owned) Unit V camera on the Grove port, unchanged

---

## Sampling math — it fits the sweep perfectly

A sweep takes ~20–60 s over 360°. At a conservative 20 Hz ToF poll:

- 20–60 s × 20 Hz = **400–1200 range samples per sweep**
- one reading every **0.3–0.9° of rotation** — massively oversampled for a
  wall outline binned at 5° (72 bearings)

Pipeline per sweep: tag each reading with the IMU yaw at capture → bin into 72
bearings → **median per bin** (same robustness philosophy as object ranging) →
polar polygon.

Refinements after the raw polygon works:
1. Split-and-merge line fitting → straight wall segments from the polygon
2. Optional Manhattan snap (90° corners) using the Hough confidence we
   already compute in RoomFitter
3. Corner extraction → the "face a corner" origin convention becomes a
   *measured* anchor, which also gives rescan registration a hard landmark

The 27° sensor cone slightly rounds corners (reads the nearest surface in the
cone); VL53L1X ROI configuration can narrow it to ~15° if that matters.

### Portability note (standing principle)

The wall-ranging pipeline consumes `(yaw_rad, distance_m)` pairs — nothing
else. Any ranger (I2C ToF, UART lidar module, ultrasonic) that produces those
pairs slots in behind the same interface. Keep the sensor driver ~40 lines and
isolated in `src/scanner/tof_ranger.h`.

---

## Enclosure (next phase)

Once camera + ToF + Tab5 travel together, a 3D-printed bracket becomes the
right move:

- **Rigid co-aim is the whole game:** ToF and camera optical axes parallel,
  both normal to the Tab5 back. A fixed yaw offset between them is fine —
  one constant in software — but it must not flex scan-to-scan.
- Back-shell clamshell around the Tab5 with a sensor pod top-center:
  Unit V + ToF4M side by side, Grove cable channel to the port, pigtail
  channel to the M5-Bus header (header access must stay open).
- Keep the power button, USB-C, speaker grille, and antenna connectors clear;
  leave the kickstand/hand-strap question for the design session.
- Stretch: the pod prints as a separate swappable module — future sensors
  (lidar, better camera) get a new pod, not a new shell. Matches the
  modularity principle.

---

## Integration order (when the ToF4M arrives)

1. `tof_ranger.h`: VL53L1X init + poll on `M5.In_I2C` (0x29), ~40 lines
2. Scan loop: sample (yaw, mm) at poll rate during SCAN; store 72-bin medians
3. `.wall` sidecar beside the mesh (bearing/distance table, "WAL1")
4. Mesh: replace the rectangular plate with the measured floor polygon +
   optional low wall ribs at the polygon edges
5. Sweep dial: draw the wall trace ON the ring as it's measured — the dial
   literally becomes a live floor-plan radar (contest demo gold)
6. RF survey: heatmap clipped/painted onto the measured polygon
