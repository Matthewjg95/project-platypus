# VL53L8CX Depth Ranging — Research & Integration Plan

> **STATUS (2026-09-01): selected post-contest architecture.**
> The previous VL53L1X single-range plan is superseded by a VL53L8CX-class
> 8x8 multizone ToF breakout mounted through an M5Stack Module Bus
> (**SKU M024**) on the Tab5 rear M5-Bus. **Pololu item #3419** is the selected
> purchasable breakout; purchase, Tab5 3.3 V rail capacity, bus behavior, and
> physical fit remain to be verified before permanent assembly.

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
land pattern for soldering a bare VL53L8CX package directly. The selected
interface board is the [Pololu VL53L8CX carrier, item #3419](https://www.pololu.com/product/3419):
13 x 23 mm, populated with the sensor, dual regulators, level shifting,
2.54 mm connections, and M2 mounting holes. Use it as a replaceable, rigidly
mounted module rather than committing a development-board footprint to the
future custom carrier.

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
| 5 V | switched EXT_5V_BUS | 28 | **Do not connect to Pololu #3419 in this I2C configuration** |
| Spare GPIO | selected after conflict audit | TBD | Optional interrupt or low-power/shutdown control |

Connector pin numbers are authoritative; apparent left/right position may mirror
between the Tab5 rear receptacle and the mating M024. Do not wire by counting
holes from a photograph. Verify the actual M024 nets with continuity mode before
the Tab5 is powered.

Relevant official connector rows:

| Official left pin | Signal | Official right pin | Signal |
|---:|---|---:|---|
| 1 | GND | 2 | G16 |
| 3 | GND | 4 | G17 |
| 5 | GND | 6 | Reset |
| 11 | G5 / SCK | **12** | **3V3** |
| **17** | **G31 / internal SDA** | **18** | **G32 / internal SCL** |
| 25 | **HVIN — prohibited** | 26 | G51 |
| 27 | **HVIN — prohibited** | 28 | **5V — prohibited for this hookup** |
| 29 | **HVIN — prohibited** | 30 | **BAT — prohibited** |

The VL53L8CX default I2C address is 0x29, which does not collide with the
currently documented Tab5 internal-bus occupants. The address check must be
repeated against the actual firmware/hardware revision during bring-up.

### Selected Pololu #3419 wiring

The Pololu carrier accepts 3.2–5.5 V, but its level shifter pulls the host-side
communication lines to the same voltage as `VIN`. ProjectPlatypus must therefore
power `VIN` from the Tab5 **3.3 V** M5-Bus rail. Connecting Pololu `VIN` to
M5-Bus pin 28 / 5 V risks applying 5 V to the Tab5's 3.3 V internal I2C bus.

| Pololu #3419 pin | M024 / Tab5 destination | Rev-A state | Reason |
|---|---|---|---|
| `VIN` | **M5-Bus pin 12 — 3V3** | Connect | Power and 3.3 V host-side logic reference |
| `GND` | **M5-Bus pin 1, 3, or 5 — GND** | Connect | Common return |
| `SDA/MOSI` | **M5-Bus pin 17 — G31 / internal SDA** | Connect | Shared internal I2C data |
| `SCL/MCLK` | **M5-Bus pin 18 — G32 / internal SCL** | Connect | Shared internal I2C clock |
| `SPI/I2C` | **GND** | Connect permanently | Low selects I2C; default/high selects SPI |
| `LP` | No connection initially | Leave open | Carrier pulls it high; retain for later address/reset control |
| `INT` | No connection initially | Leave open | Optional data-ready interrupt |
| `CS` | No connection | Leave open | SPI-only |
| `MISO` | No connection | Leave open | SPI-only |
| `SYNC` | No connection | Leave open | Optional external acquisition trigger |
| `AVDD` | **Do not connect** | Regulator output | Not a power input |
| `CORE/IOVDD` | **Do not connect** | Regulator output | Not a power input |

The Pololu board typically draws about 100 mA while ranging and can peak near
150 mA. M5Stack exposes 3V3 on M5-Bus pin 12 but does not publish an external
current allowance on the Tab5 product page. Confirm that allowance with M5Stack
or validate the rail before permanent integration. If the rail cannot support
the sensor, redesign power using the switched 5 V rail plus a dedicated 3.3 V
regulator; do not move Pololu `VIN` directly to 5 V while it shares Tab5 I2C.

The carrier's level shifter is sensitive to external loading. Keep SDA/SCL
wiring ideally below 8 cm, do not add pull-ups by default, begin at 400 kHz or
lower, and test the existing touch and IMU throughout bring-up.

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

A socketed/headered or short-wire prototype is acceptable after the breakout is
verified. Use the Pololu M2 holes to carry mechanical load; solder joints must
not locate the optical axis. The later clean solution is a small purpose-built
M5-Bus depth/expansion PCB derived from measured M024 results.

### No-fry assembly and first-power sequence

1. **Remove every power source.** Shut down, disconnect USB, remove/disconnect
   the battery, and wait at least five seconds. Never insert or rewire M5-Bus live.
2. **Map M024 with no sensor attached.** Use continuity mode to identify M5-Bus
   pins 12, 17, 18, and one of 1/3/5 on the actual module. Label the destination
   pads; do not rely on a photo or connector-view orientation.
3. **Verify M024 alone.** With only M024 inserted, power Tab5 and measure at the
   spacious verified pads: pin 12 to GND approximately 3.3 V; SDA and SCL should
   not show 5 V. Power down and remove all power again.
4. **Inspect the disconnected sensor assembly.** Verify `SPI/I2C` continuity
   to GND; no hard short from `VIN` to GND; SDA and SCL are not shorted to each
   other, GND, or 5 V; `AVDD` and `CORE/IOVDD` remain unconnected.
5. **Continuity-check end to end.** Confirm Pololu `VIN` reaches only M5 pin
   12, GND reaches only the chosen ground, SDA reaches only pin 17, and SCL
   reaches only pin 18.
6. **Insert only while off.** Secure the board and wires before power. Do not
   hold loose probes or clips near the live rear connector.
7. **First boot.** Check for heat, resets, display/touch trouble, or abnormal
   current. Power off immediately if any appears. Scan for address 0x29 before
   starting ranging.
8. **Functional safety check.** Exercise touch and IMU while acquiring 4x4 at
   10 Hz. Record timeouts and responsiveness before attempting 8x8 or a higher
   rate.

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

- Purchase Pololu VL53L8CX carrier #3419 and retain its schematic/product documentation.
- Confirm Tab5 M5-Bus 3.3 V external-current allowance or validate the rail.
- Dry-fit Tab5 + M024 + Pololu carrier + camera + patch antenna/cable.
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
| [Pololu VL53L8CX carrier #3419](https://www.pololu.com/product/3419) | 1 | **selected; purchase pending** |
| 2.54 mm headers/socket or short insulated hookup wire | as needed | keep SDA/SCL under 8 cm |
| M2 nylon fasteners/standoffs | as needed | carry mechanical load and preserve optical pose |
| Additional pull-ups | 0 initially | only add from measured bus evidence |
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
- Pololu level-shifter sensitivity on the already-populated internal I2C bus;
- unverified Tab5 3.3 V external-current allowance;
- accidental use of M5-Bus 5 V/HVIN/BAT or connector-view mirroring;
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
- [Pololu VL53L8CX carrier #3419 documentation](https://www.pololu.com/product/3419)
