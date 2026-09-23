# VL53L8CX Tab5/M024 Test and Evidence Plan

- **Owner:** Project Platypus
- **Status:** Planned; hardware ordered, no results recorded
- **Downstream consumer:** PlatypusOne
- **Promotion rule:** These tests may generate evidence for PlatypusOne, but
  they do not change its requirements or BOM. PlatypusOne must pass its own
  depth-sensing decision gate.

## Purpose

Determine whether the Pololu VL53L8CX carrier is electrically safe and useful
on the Tab5 rear M5-Bus through M024, and whether its 8×8 depth maps materially
improve Project Platypus room mapping and camera-context workflows.

The first proof is deliberately conservative: 3.3 V power, short wiring,
4×4 at 10 Hz, no cover window, and no optional interrupt or low-power pins.

## Hardware record

Complete this table before first power.

| Field | Recorded value |
|---|---|
| Tab5 hardware revision | |
| Tab5 firmware commit | |
| M024 revision/lot | |
| Sensor carrier | Pololu #3419, VL53L8CX |
| Carrier revision/markings | |
| 3.3 V regulator/test supply | |
| Wiring lengths | |
| Test equipment | |
| Test date and operator | |

Photograph both sides of the assemblies and the complete wiring before power.
Store the images with the run evidence.

## Stage 0 — disconnected inspection

- Map M024 pins 12, 17, 18, and the selected ground by continuity.
- Verify Pololu `SPI/I2C` is tied low.
- Verify `VIN` has no hard short to ground.
- Verify SDA and SCL are not shorted together, to ground, or to 5 V.
- Verify `AVDD` and `CORE/IOVDD` are unconnected.
- Record resistance/continuity results and photographs.

**Stop condition:** any ambiguous net, short, or connector-orientation doubt.

## Stage 1 — rail and idle-bus safety

With M024 installed but the sensor disconnected:

- measure pin 12 to ground;
- measure idle SDA and SCL;
- confirm neither communication line is pulled to 5 V;
- exercise display, touch, and IMU.

Then connect the powered-off sensor assembly and repeat rail measurements at
the sensor.

**Acceptance:** correct nominal rails, no heating, reset, touch failure, display
fault, or abnormal current. Record average and observed peak current rather
than relying only on vendor typical values.

## Stage 2 — conservative ranging

Start at 4×4 and 10 Hz.

- Confirm discovery at 7-bit address `0x29`.
- Run continuously for at least 30 minutes.
- Exercise touch and IMU throughout the run.
- Log frame count, valid zones, invalid/status codes, I²C errors, timeouts,
  resets, sensor temperature if available, and loop latency.
- Save the exact firmware commit and configuration beside the log.

**Acceptance:** no device reset or bus lockup; touch and IMU remain usable.
Any timeout or invalid zone remains in the evidence rather than being removed.

Only after this stage passes, attempt 8×8 and higher rates. Record the actual
sustainable rate; do not assume the requested rate was achieved.

## Stage 3 — depth quality matrix

Capture stationary runs for:

- matte light wall at approximately 0.5, 1, 2, 3, and 4 m;
- dark target;
- angled glossy target;
- glass or reflective failure case;
- mixed foreground/background scene;
- indoor daylight and a bright-window condition;
- repeated power cycles;
- cold start and warmed operation.

For each run preserve the complete 64-zone frames and status/confidence fields.
Report:

- valid-zone percentage;
- median distance by zone;
- temporal spread by zone;
- fixed-pattern differences across zones;
- maximum reliable range by target/lighting condition;
- repeatability across power cycles;
- observed failure signatures.

Do not convert repeatability into an accuracy claim. Reference distances must be
measured independently and the method recorded.

## Stage 4 — mechanical and camera relationship

- Rigidly fixture Unit V and the VL53L8CX.
- Record the physical offset and sensor-forward axes.
- Capture a repeatable target scene after at least five remove/reinstall or
  handling cycles.
- Quantify alignment change rather than describing it visually.
- Test whether the patch antenna can coexist without blocking either optical
  field or applying load to the sensor mount.

A changing camera/ToF relationship fails the mechanical concept even if both
sensors work independently.

## Stage 5 — application usefulness

Compare the existing camera/visibility-carved output against depth-assisted
output on the same scenes.

Evaluate separately:

1. wall, corner, opening, and doorway evidence;
2. foreground-object rejection;
3. camera-object placement context;
4. WALK+ registration;
5. RF-map clipping/context;
6. export usefulness.

Record both improvements and regressions. A feature is not promoted merely
because depth data can be displayed.

## Evidence package

Each run belongs under:

```text
docs/evidence/tof/<YYYY-MM-DD>-<short-run-name>/
  README.md
  configuration.json
  measurements.csv or frames.jsonl
  photos/
  derived/
```

The run README records purpose, procedure, expected result, actual result,
limitations, failures, and the exact commit that produced the data. Raw files
are immutable; regenerated plots and summaries go under `derived/`.

## Downstream decision packet

When testing is complete, publish one summary containing:

- which stages passed or failed;
- exact links to raw evidence;
- what the VL53L8CX proved for Project Platypus;
- what remains unknown for PlatypusOne;
- whether a VL53L9CX/UNO Q evaluation is justified;
- recommendation: reject, continue evaluating, or submit to PlatypusOne's ADR
  gate.

Until that summary exists, the only valid downstream status is **candidate**.
