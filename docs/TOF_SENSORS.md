# Time-of-Flight Sensors — How They Actually Work

Research companion to `TOF_RANGING.md` (which covers our integration plan).
This is the physics and engineering background for the VL53L1X that's about
to join the instrument — what it measures, how, and where it lies.

---

## 1. The core idea, and why it's hard

Light travels ~30cm per nanosecond. Measure how long a photon takes to fly
to a wall and back, halve it, multiply by c: that's your distance. The
problem is the timescale — a millimetre of range resolution corresponds to
**~6.7 picoseconds** of round-trip time. No $5 chip times a single photon
that precisely. Every practical ToF sensor is a clever workaround for that
fact, and the workarounds split into two families.

## 2. Direct ToF (dToF) — what the VL53 family is

Direct ToF really does time photon flights — it just does it *statistically*.

1. A **VCSEL** (vertical-cavity surface-emitting laser, 940nm, Class 1
   eye-safe) fires thousands of sub-nanosecond pulses.
2. The receiver is a **SPAD array** (single-photon avalanche diodes) —
   pixels so sensitive that ONE returning photon triggers a detectable
   avalanche. The VL53L1X has a 16×16 SPAD array behind a lens.
3. Each detection is time-stamped by a TDC (time-to-digital converter) and
   dropped into a **histogram** of arrival times.
4. After thousands of pulses, the histogram shows a peak: ambient photons
   arrive uniformly at random (a flat floor), while photons that bounced
   off your wall pile up at one specific round-trip time. Find the peak,
   read the distance.

This histogram trick is why dToF has properties that matter for us:

- **Range accuracy is mostly independent of target brightness.** A dark
  wall returns fewer photons → shorter max range and a noisier peak, but
  the peak's *position* (= distance) doesn't shift. Intensity-based sensors
  can't say that.
- **Ambient light raises the histogram's noise floor** rather than biasing
  the answer — sunlight (which contains plenty of 940nm) shortens usable
  range but degrades gracefully.
- **Multiple returns are visible in principle** — glass edge + wall behind
  it produce two peaks. (The VL53L1X reports the strongest; fancier lidars
  report several.)

## 3. Indirect ToF (iToF) — the other family, for contrast

Instead of pulses, iToF emits **continuously modulated** light (tens of
MHz) and measures the **phase shift** between emitted and received
waveforms — phase maps to distance. It's how most depth *cameras* work
(phone depth sensors, Kinect-era devices): cheap per-pixel, whole images at
once. The catch: phase wraps around every c/(2·f_mod) metres, creating
distance ambiguity that needs multi-frequency tricks, and accuracy depends
on signal strength. For single-point ranging, dToF won this market — which
is why the part we're using is dToF.

## 4. The VL53L1X specifically

| Property | Value | What it means for Platypus |
|---|---|---|
| Principle | dToF, SPAD histogram | reflectance-tolerant wall ranges |
| Emitter | 940nm VCSEL, Class 1 | invisible, eye-safe, indoor-friendly |
| Max range | ~4m (long mode, low ambient) | covers a room from its centre |
| Min range | ~4cm | never an issue for walls |
| FoV | ~27° cone, lens-defined | see §5 — the big caveat |
| ROI | programmable 4×4…16×16 SPADs | narrows FoV to ~15°, steerable! |
| Timing budget | 20–1000ms per reading | precision/speed dial (§6) |
| Distance modes | short / medium / long | short = best ambient immunity, ~1.3m cap |
| Typical σ | mm-level near, cm-level far | good enough to beat our carve cells |
| Interface | I2C @ 0x29, interrupt pin | rides the Tab5's M5-Bus internal I2C |

The **ROI feature deserves attention**: you can tell the chip which SPAD
sub-array to use, which both narrows the cone *and steers it slightly*
(±~5°) without moving anything. A future trick: three interleaved ROIs
(left/centre/right) per bearing = a crude 3-zone scanning lidar from one
static part.

## 5. Failure modes — where ToF lies to you

Ranked by how much each will bite the wall-ranging use case:

1. **The cone problem (biggest).** 27° at 3m is a ~1.4m-wide footprint.
   The sensor reports (roughly) the nearest strong return in that cone —
   a chair 1m away wins over the wall 3m away. Mitigations: narrow ROI,
   mount high aiming slightly up (over furniture), and fuse with the
   camera's object map (we KNOW where the chair is — reject ranges that
   match known objects and keep the far return for walls).
2. **Specular surfaces.** A glossy wall, window, or mirror at a non-normal
   angle reflects the beam *away* — few photons return; you get max-range
   noise or a wrong bounce path (mirror shows the reflected room!). Matte
   drywall is ideal; glass walls will be our enemy. Detectable partially
   via signal-rate + range-status flags.
3. **Dark absorptive targets.** Black felt curtains at 4m may simply not
   return enough photons in long mode. Shows up as status "signal fail" —
   filterable.
4. **Ambient IR.** Direct sunlight through a window floods 940nm.
   Short-distance mode is the most immune; expect degraded range near
   bright windows, fine everywhere else indoors.
5. **Crosstalk / cover glass.** Photons bouncing off a window in front of
   the sensor create a false near peak. ST provides a crosstalk
   calibration; our enclosure should leave the sensor aperture OPEN or use
   the calibration if we glaze it.
6. **Wraparound.** Beyond-max-range targets can alias into short readings
   (status flag 7). Filter by status, always.

**The golden rule: never use a bare distance.** The chip reports range
status and signal/sigma estimates with every reading — a trustworthy
pipeline filters on status==0 and feeds sigma into the per-bin median we
already planned (TOF_RANGING.md).

## 6. The precision/speed dial

Timing budget = how long the chip accumulates histogram photons:

- 20ms budget → ~±5mm σ short range, weaker at distance, 50Hz possible
- 33–50ms → the sweet spot for a sweeping scan: 20–30Hz, cm-class at 3m
- 200ms+ → mm-class but far too slow for a rotating sweep

Our sweep math (TOF_RANGING.md): even 20Hz gives a reading every ~0.5° of
rotation — 72 five-degree wall bins get ~10 readings each for medians.
Plan: **medium distance mode, 33ms budget, ~25Hz**, ROI narrowed to
~15–20°, status-filtered, per-bin median. Revisit after bench tests.

## 7. Why ToF beats the alternatives for this job

| Tech | Why not |
|---|---|
| Ultrasonic (HC-SR04) | huge beam lobe (~30°+), slow (~50ms of sound flight), echo multipath in corners, cm accuracy at best |
| IR triangulation (Sharp GP2Y) | ~1.5m max, analog drift, beam still wide |
| Spinning lidar (RPLIDAR etc.) | actually ideal data — but $100+, large, a motor to power and mount; philosophically a different instrument |
| Stereo / structured light | serious compute + calibration; the P4 is busy |
| **dToF (VL53L1X)** | $9, 4m, mm–cm σ, I2C, no moving parts, and our sweep IS the scan motor — the user's rotation replaces the lidar spindle |

That last cell is the design insight: a spinning lidar is a ToF sensor
plus a motor. **Platypus already has the motor — it's the person.** The
sweep dial turns a $9 static ranger into a polar scanner.

## 8. What this means for the firmware (when the part arrives)

1. `tof_ranger.h` — init (distance mode, budget, ROI), poll, and a
   `Reading { mm, sigma_mm, status }` struct. Portable; nothing but I2C.
2. Scan loop: tag readings with IMU yaw; keep status==0; 72-bin medians.
3. **Fusion pass**: reject wall candidates that coincide with known camera
   objects (the cone problem, solved by the sensor we already have).
4. `.wall` sidecar + carve upgrade: measured polygon replaces/augments the
   visibility carve; sight-line evidence still fills where ToF was blocked.
5. Sweep dial: draw the wall trace live — the dial becomes a radar.

---
*Sources: ST VL53L1X datasheet & UM2356 API docs, ST FlightSense appnotes
(histogram dToF architecture), general SPAD/TDC literature. Bench-verify
timing-budget/σ numbers on our unit before trusting them in the pipeline.*
