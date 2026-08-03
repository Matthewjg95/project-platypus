# Hackster Writeup — DRAFT

Paste-ready for the Hackster editor. `[SLOT: ...]` marks where an asset or a
personal line goes. Keep the voice first-person — judges are reading YOU.

---

# Project Platypus: a tablet that maps your room, then maps your WiFi onto it

**Elevator (project summary field):**
An M5Stack Tab5 + Unit V AI camera that scans a room into a labeled 3D floor
plan — then walks the room measuring WiFi and paints the signal as a heatmap
onto the map it just drew. Custom antenna PCB included, and compared against
the internal antenna on the same floor plan.

**Cover image:** [SLOT: rig-in-hand photo or the A/B heatmap pair]

---

## The idea

WiFi heatmap tools assume you already have a floor plan. Room scanners assume
you care about the room. I wanted the instrument in the middle: one handheld
device that *draws its own map and then measures the invisible stuff on top
of it*. A WiFi weather map, drawn by the thing that measured the room.

[SLOT: one personal sentence — why this itch, why now]

## The hardware

[SLOT: assembly photo with callouts]

- **M5Stack Tab5** — ESP32-P4 (RISC-V, 360MHz), 1280×720 touch. The brains
  and the screen. Its ESP32-C6 co-processor does WiFi over SDIO.
- **M5 Unit V** — Kendryte K210 with a KPU neural accelerator, running
  tiny-YOLOv2 on-device. Tethered over a Grove UART at 460800 baud, streaming
  JPEG frames + detections (~6-7 fps).
- **Custom antenna PCB** ("Design A") — my own 2.4GHz antenna board, MMCX to
  the Tab5's external RF port, which has an I2C-expander-controlled switch
  between internal and external antennas. This is what makes the A/B heatmap
  comparison possible.
- Battery sled for untethered scanning.

No radio on the P4, no display on the K210, no depth sensor anywhere — every
capability lives on exactly one chip and they cooperate over two thin links.
That constraint shaped the whole design.

## How a scan works

[SLOT: sweep-dial mid-scan screenshot]

1. **Stand in a corner, face it, hold still.** The BMI270 gyro calibrates its
   bias from a stillness-locked window (any wobble restarts it — a tick says
   go). The corner becomes the room's origin.
2. **Sweep 360°.** Yaw is measured by projecting the gyro onto gravity, so it
   works no matter how you hold the tablet. A progress ring fills as you
   turn; every object the camera confirms drops a colored pip on the dial at
   the bearing it was found. The scan finishes itself at a measured full turn.
3. **Monocular ranging.** The K210 only gives 2D boxes, so distance comes
   from known real-world object sizes (a TV is ~0.55m wide; pixel width →
   range). Every estimate passes sanity gates (box size, aspect ratio,
   edge-clipping fallback, indoor range ceiling) and each object's position
   is the median of its sightings — one bad range can't drag a marker.
4. **The floor carves itself from evidence.** Everything the camera saw from
   the origin proves the sight-line between is open floor. Origin + objects +
   sight-line corridors stamp into an occupancy grid; the union becomes the
   floor. Rooms come out L-shaped when they ARE L-shaped — no wall sensor
   needed.

[SLOT: finished room screenshot — carved floor, labels, start arrow]

## Rooms that improve instead of resetting

Rescanning a room doesn't replace it. The room keeps a persistent object
database; a new scan registers onto it using the objects themselves as
landmarks (yaw × same-class-pair search), then merges. **WALK+** extends a
room on foot: IMU step detection dead-reckons your path, and everywhere you
walked becomes floor-truth for the carve — the room's landmarks absorb the
dead-reckoning drift.

A per-room object menu turns any detection on or off (the model's phantom
"person" problem, solved by policy: person needs higher confidence and more
sightings than a sofa does).

[SLOT: OBJ menu screenshot]

## The payoff: RF survey

[SLOT: the A/B heatmap pair, side by side — THE image of this project]

Open a scanned room, tap RF. Walk the room, tap your position on the floor
plan, sample. An inverse-distance-weighted heatmap builds over the floor,
with a dBm legend and automatic DEAD / BEST spot callouts.

Then flip the antenna switch and survey again: **the same room, two heat
layers — internal antenna vs my custom PCB — on a floor plan the device drew
itself.** [SLOT: one sentence on what the comparison actually showed]

## Building map

[SLOT: building map screenshot]

Every room's carved footprint on one top-down canvas. Drag rooms to arrange
your house; tap one to open it. When doorway detection lands (custom model,
roadmap), these hand-arranged offsets become the initial guess for automatic
assembly.

## Engineering war stories (the short list)

- **The camera's boot filesystem lies:** MaixPy v0.5 could open() files that
  import would not find — solved by bundling the entire camera app into one
  boot.py executed from RAM.
- **A 4KB mesh-writer buffer** on the 8KB Arduino loop stack caused
  intermittent crashes exactly at scan completion. Static buffers + a 16KB
  loop stack.
- **`setRxBufferSize` after `begin()` is a silent no-op** — one-frame scans
  until the 32KB RX buffer moved before UART begin.
- **Writing expander register 0x01 blanks the screen** — bit 0 is a software
  reset sharing the LCD rail. The antenna switch now uses the documented
  direction/output registers only.
- **ArduinoOTA must begin() exactly once per boot** on the hosted-WiFi P4 —
  an end()/begin() cycle wedges the loop task. OTA arms on the home screen
  only and merely pauses elsewhere.

## What's next

- **Custom detection model** (MaixHub): door/window/desk classes instead of
  VOC-20's cats and bottles — doors are 2.03m tall everywhere on Earth, the
  perfect ranging landmark, and doorways link rooms automatically.
- **ToF wall ranging:** a VL53L1X on the Tab5's M5-Bus I2C, sampling wall
  distance at every bearing during the same sweep — measured wall polygons,
  and the sweep dial becomes a live floor-plan radar.
- **3D-printed enclosure** with a swappable sensor pod.

## Try it

Code: [SLOT: repo URL — flip public at submission]
Everything is documented in-repo: wire protocol, mesh format, sidecar
formats, calibration history, and the debugging guides for every war story
above.

---

## BOM (Hackster "Things" section)

| Item | Qty | Note |
|---|---|---|
| M5Stack Tab5 | 1 | ESP32-P4 tablet |
| M5Stack Unit V | 1 | K210 AI camera |
| Grove cable | 1 | camera UART tether |
| Custom 2.4GHz antenna PCB | 1 | own design, "Design A" |
| MMCX pigtail | 1 | antenna → Tab5 RF port |
| Battery sled | 1 | untethered operation |
| microSD ×2 | 2 | Tab5 (rooms) + Unit V (model) |
