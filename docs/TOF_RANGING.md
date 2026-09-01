# VL53L8CX Depth Ranging — Research & Integration Plan

> **STATUS (2026-09-01): selected post-contest architecture.**
> The previous VL53L1X single-range plan is superseded by a VL53L8CX-class
> 8x8 multizone ToF breakout mounted through an M5Stack Module Bus
> (**SKU M024**) on the Tab5 rear M5-Bus. Hardware remains unpurchased/unverified
> until the exact breakout, voltage requirements, and physical fit are closed.

## Decision

ProjectPlatypus will preserve the Unit V camera on the only Grove port and use
the rear M5-Bus for depth. The prototype path is:

```
Tab5 rear M5-Bus
        |
      M024
  power + I2C + optional control GPIO
        |
VL53L8CX breakout
        |
rigid camera/ToF sensor pod
```

The M024 is the wiring and mechanical prototype carrier. It is not a suitable
land pattern for soldering a bare VL53L8CX package directly. Use a regulated,
documented VL53L8CX breakout/module with accessible 2.54 mm pads, castellations,
or a small interposer. The exact breakout is a procurement decision that must
close before soldering.

This keeps the current Unit V UART link unchanged and upgrades the room scanner
from one range per heading to a 64-zone depth fan.

## Why this matters

The current room is visibility-carved from:

- the scan origin;
- detected object footprints;
- sight-line corridors to those objects;
- the WALK+ path.

That is useful evidence, but it does not directly observe walls. During a
360-degree sweep the VL53L8CX can measure many spatial rays at each orientation.
The mapper can mark space before each valid return as free and treat the return
as surface evidence. The resulting wall polygon and occupancy grid improve:

- wall, corner, doorway, and opening detection;
- camera-object distance and placement;
- WALK+ drift correction from structural landmarks;
- rescanning registration;
- RF heatmap clipping and wall-aware interpretation;
- room-geometry export for later CAD/Fusion workflows.

The 8x8 result is coarse depth evidence, not a precision CAD point cloud.

---

## Hardware path

### Grove port remains camera-only

The Unit V camera continues to own the Tab5 HY2.0-4P Grove port at G53/G54 as
UART. A passive Grove hub is not used to mix this UART with I2C.

### Rear M5-Bus and M024

M5Stack documents the Tab5 rear connector as supporting Module-series M5-Bus
expansion. The M024 provides a 54 x 54 mm, 2.54 mm-pitch, 200-hole prototyping
field connected to that bus.

Relevant Tab5 M5-Bus signals:

| Function | Tab5 signal | M5-Bus pin | Intended use |
|---|---:|---:|---|
| Ground | GND | 1, 3, or 5 | ToF ground |
| 3.3 V | 3V3 | 12 | Breakout power only if its input specification permits |
| Internal I2C data | G31 | 17 | ToF SDA |
| Internal I2C clock | G32 | 18 | ToF SCL |
| 5 V | switched EXT_5V_BUS | 28 | Use only if the selected breakout explicitly requires/accepts 5 V |
| Spare GPIO | selected after conflict audit | TBD | Optional interrupt or low-power/shutdown control |

The VL53L8CX default I2C address is 0x29, which does not collide with the
currently documented Tab5 internal-bus occupants. The address check must be
repeated against the actual firmware/hardware revision during bring-up.

### Do not assume the module voltage

The bare VL53L8CX is not a generic four-wire 5 V Grove sensor. Before assembly,
the selected breakout schematic must prove:

- allowed input-supply voltage;
- I/O voltage and any level shifting;
- onboard regulators and required decoupling;
- I2C pull-up values and pull-up rail;
- availability/default state of LPn, INT, and I2C/SPI selection pins;
- connector/pad pinout and orientation;
- optical cover/window guidance.

Prefer powering a compatible breakout from 3.3 V. Use the Tab5 switched 5 V
rail only when the breakout documentation explicitly supports it and firmware
enables `EXT5V_EN`.

### M024 prototype assembly

Minimum Rev-A build controls:

- socket/stack the M024 into the Tab5 rear connector with power off;
- use insulated solid wire or short secured jumpers on the M024;
- add local decoupling at the breakout per its manufacturer;
- provide strain relief so sensor-pod movement cannot work the solder joints;
- expose SDA, SCL, power, ground, and optional control test points;
- label pin 1, voltage, and sensor-forward direction;
- perform continuity and short checks before attaching the Tab5;
- first power from a current-limited source where practical;
- never hot-plug or rewire the M5-Bus.

A direct soldered prototype is acceptable after the breakout is verified.
The later clean solution is a small purpose-built M5-Bus depth/expansion PCB
derived from measured M024 results.

---

## Mechanical and antenna coexistence

### Rigid co-aim is mandatory

The ToF and Unit V optical relationship must not flex between scans. Record the
six-degree-of-freedom transform from ToF coordinates into the camera/IMU frame.
A known offset is acceptable; a changing offset is not.

The M024 may carry the breakout directly only if that orientation points the
sensor into the same scene as the camera. Otherwise it should carry the
electrical interface and a short, strain-relieved connection to a rigid
camera/ToF pod.

The enclosure/pod must control:

- ToF aperture and field-of-view clearance;
- camera and ToF axis alignment;
- cover-window material, air gap, crosstalk, and calibration;
- illumination leakage into the ToF aperture;
- cable bend radius and connector access;
- repeatable attachment to the Tab5.

### Patch antenna decision

The external directional patch antenna is a defining RF-survey capability, so
removing it is allowed as a prototype fallback, not the default architecture.

Use this priority order:

1. **Coexist:** fit the M024/ToF and patch antenna together; relocate the
   cable-mounted patch outside the ToF field of view if needed.
2. **Mode-dependent attachment:** use a removable or repositionable patch so
   geometry scanning and directional RF surveying retain their best sensor
   arrangements.
3. **Temporary removal:** remove the patch for initial depth bring-up or room
   scanning only if it physically blocks the M024, sensor pod, or optical field.
   The Tab5 internal antenna remains the fallback RF path.

Before committing an enclosure, perform a physical interference study covering
the M024 envelope, header engagement, patch/cable bend radius, hand placement,
ToF field of view, Unit V view, and Tab5 buttons/USB/speaker access.

Removing the patch means the EXT heat layer and directional-bearing experiments
are unavailable for that configuration; the software and saved survey format
must continue to preserve INT/EXT separation.

---

## Shared I2C bus risk

G31/G32 also serve Tab5 touch, IMU, RTC, audio, current monitoring, and I/O
expanders through `M5.In_I2C`. A multizone frame is materially heavier than
the old VL53L1X scalar read, so the earlier assumption that 20-30 Hz is “light”
does not automatically carry forward.

Bring-up matrix:

| Mode | Initial target | Measure |
|---|---:|---|
| 4x4 depth | 10 Hz | frame latency, valid zones, IMU/touch responsiveness |
| 8x8 depth | 10 Hz | bus utilization, missed IMU samples, UI latency |
| 8x8 depth | 15 Hz | only after 10 Hz passes |
| Multi-target output | lowest useful rate | memory and transfer cost versus mapping value |

Requirements:

- serialize access through the established internal-I2C path;
- keep acquisition asynchronous from rendering and file writes;
- timestamp the completed depth frame and the associated IMU pose;
- count timeouts, invalid frames, IMU gaps, and UI stalls;
- retain a reduced-rate/4x4 fallback;
- do not starve the IMU whose orientation makes the depth data useful.

The rear M5-Bus also exposes SPI-capable pins, but switching the VL53L8CX to SPI
is a later option requiring an exact breakout pinout, conflict audit, and driver
proof. I2C is the first implementation.

---

## Data contract

The old portable interface of `(yaw_rad, distance_m)` discards most of the
selected sensor's value. Preserve a complete sensor-neutral frame:

```cpp
struct DepthTarget {
    uint16_t distance_mm;
    uint16_t signal_rate;
    uint16_t sigma_mm;
    uint16_t ambient_rate;
    uint8_t  status;
};

struct DepthFrame {
    uint32_t timestamp_ms;
    SensorPose sensor_pose;
    uint8_t rows;
    uint8_t cols;
    uint8_t targets_per_zone;
    DepthTarget zones[64]; // extend storage if multi-target mode earns its cost
};
```

The implementation may use smaller/native types, but must preserve:

- timestamp;
- sensor pose and calibration identity;
- zone location/FOV;
- distance;
- validity/status;
- uncertainty or sigma;
- signal and ambient-light quality where exposed;
- target index when multi-target mode is enabled.

A compatibility adapter may collapse selected zones into scalar
`RangeSample { yaw, distance }` records for the first wall-polygon experiment.

---

## Mapping pipeline

For every valid zone:

1. associate the frame with interpolated IMU yaw/pitch/roll;
2. apply the calibrated ToF-to-device transform;
3. generate the zone ray from the sensor field-of-view model;
4. transform it into room coordinates;
5. mark cells before the return as observed free space;
6. mark the return neighborhood as surface evidence;
7. reject or down-weight poor status, high sigma, weak signal, or excessive
   ambient-light observations;
8. accumulate repeat observations before fitting walls.

Derived products:

- **2D wall polygon:** use the most useful middle rows, median/filter spatially,
  then fit straight segments and corners;
- **occupancy/free-space grid:** preserve all useful zones;
- **door/opening candidates:** sustained gaps/far returns between stable wall
  bands, requiring multiple viewpoints before confirmation;
- **vertical context:** upper/lower rows help distinguish wall, floor, ceiling,
  and foreground clutter when pitch calibration is adequate.

Do not assume that multi-target ranging sees through opaque furniture. It may
separate returns within a zone under supported conditions, but background-wall
recovery remains evidence-dependent.

---

## Camera and WALK+ fusion

For each Unit V detection, project its bounding-box center and coverage into the
ToF grid:

- use valid ToF range as the preferred object-depth measurement;
- retain the current known-size monocular estimate as fallback and cross-check;
- reject or flag large disagreement;
- distinguish foreground object depth from background-wall depth where the
  multizone/multi-target evidence supports it;
- store the ranging method and confidence with the object landmark.

For WALK+, compare observed wall/corner depth signatures against the accumulated
map. Use those matches to constrain drift; do not present this as full visual
SLAM until its error is measured.

---

## RF survey integration

The existing RF sample already records position, RSSI, antenna selection, and
heading. Do not attach every raw 64-zone frame to every RF record. Associate RF
samples with derived spatial context or a depth/map revision:

- nearest-wall distance and bearing;
- free-space/obstruction state in the antenna aim direction;
- room/opening identifier;
- walls intersecting a candidate AP-bearing ray;
- geometry/depth revision and confidence.

This supports wall-aware interpretation without breaking the existing RFS4
survey format prematurely. A future format revision must retain backward
loading and INT/EXT antenna separation.

---

## Staged implementation

### Stage 0 — bench and fit proof

- Obtain exact VL53L8CX breakout documentation.
- Dry-fit Tab5 + M024 + camera + patch antenna/cable.
- Decide direct M024 mount versus remote rigid sensor pod.
- Verify power, I2C pull-ups, address, and control pins.
- Capture current draw and thermal observations.
- Pass the I2C matrix above before modifying room geometry.

### Stage 1 — scalar compatibility

- Add an isolated VL53L8CX driver/adapter.
- Run 4x4 at 10 Hz initially.
- Collapse valid middle zones into 72 five-degree bearing bins.
- Save a versioned `.wall` sidecar and render a live polar trace.
- Compare repeated scans of one measured rectangular room.

### Stage 2 — full depth fan

- Preserve 8x8 frames and quality fields.
- Implement calibrated zone rays and occupancy/free-space accumulation.
- Fit wall segments/corners and replace visibility-carve boundaries where ToF
  evidence is valid.
- Retain visibility carve as fallback for unobserved/out-of-range areas.

### Stage 3 — camera and pose fusion

- Associate Unit V detections with zones.
- Compare ToF versus monocular distance errors.
- Add structural correction experiments to WALK+.
- Record failure cases: glass, dark/absorptive targets, reflective surfaces,
  oblique walls, strong ambient light, and ranges beyond 4 m.

### Stage 4 — RF context and export

- Clip heatmaps to the measured room polygon.
- Add wall-aware RF context and AP-bearing intersection experiments.
- Export simplified room geometry and evidence metadata for desktop/Fusion
  workflows without claiming CAD-grade accuracy.

### Stage 5 — purpose-built board

Promote the M024 prototype into a custom M5-Bus board only after the driver,
bus rate, mechanical pose, antenna coexistence, and power arrangement pass.
Carry forward test points, strain relief, labels, and configuration options.

---

## Prototype BOM

| Item | Qty | Status |
|---|---:|---|
| M5Stack Tab5 | 1 | owned |
| Unit V K210 camera + Grove cable | 1 | owned/current |
| M5Stack Module Bus M024 | 1 | selected prototype carrier |
| VL53L8CX regulated breakout/module | 1 | exact model TBD; schematic required |
| Insulated hookup wire / headers | as needed | select after breakout |
| Local decoupling and optional pull-up/configuration parts | TBD | derive from breakout schematic |
| Rigid camera/ToF bracket or sensor pod | 1 | design after dry fit |
| External patch antenna + cable | 1 | retain when mechanically compatible |

## Acceptance criteria

The architecture advances beyond M024 prototype when:

- 100 consecutive frames acquire without bus lockup;
- touch and IMU remain responsive at the selected scan rate;
- repeat scans of a known room produce bounded wall/corner error;
- the camera/ToF transform remains repeatable after handling;
- the aperture/window does not create unacceptable crosstalk;
- RF INT operation is unaffected and the EXT patch configuration is either
  retained or its mode-dependent mounting is documented;
- every saved depth-derived result records calibration, quality, and method;
- failure cases and fallback behavior are visible rather than silently accepted.

## Principal risks

- shared-I2C bandwidth or driver integration on ESP32-P4;
- wrong breakout voltage/pull-up assumptions;
- rear-module, hand, camera, and patch-antenna interference;
- flexible mounting corrupting camera/ToF calibration;
- 4 m limit in large rooms or from corner-origin scans;
- optical-window crosstalk and blocked field of view;
- low-resolution depth being overrepresented as CAD-quality geometry;
- loss of RF differentiation if the external patch is permanently removed.

## Sources

- [M5Stack Tab5 documentation](https://docs.m5stack.com/en/core/Tab5)
- [M5Stack Module Bus M024 documentation](https://docs.m5stack.com/en/module/bus)
- [ST VL53L8CX product page](https://www.st.com/en/imaging-and-photonics-solutions/vl53l8cx.html)
- [ST VL53L8CX datasheet](https://www.st.com/content/st_com/en/technical-documents/DS14161.html)
- [ST VL53L8CX user manual UM3109](https://www.st.com/content/st_com/en/technical-documents/UM3109.html)
