# Directional Antenna & the RF Survey

## The problem (why the patch felt underused)

The survey stored each sample as `(x, z, rssi)` — RSSI as a **scalar field
over position**. That model is correct for an omnidirectional antenna and
**wrong for a patch**.

A patch antenna has a main lobe broadside to the board, roughly 6–8 dBi of
gain, and 10–15 dB of front-to-back rejection. So its RSSI is a function of
**position AND aim**. Two samples taken from the same spot facing different
directions can differ by more than the entire green-to-red span of the
heatmap legend — and the old code would average them into meaningless mush
via IDW.

Put bluntly: the patch layer wasn't just under-utilised, it was measuring
something that isn't a well-defined quantity. A judge who knows RF would
notice. Dropping the antenna would have been the wrong fix; measuring it
correctly is the right one.

## What shipped

1. **Every sample records its bearing.** `Sample.heading` — degrees, `0` =
   the mesh `+Z` axis (the direction faced at scan origin), `-1` = unknown.
   File format `RFS2` → `RFS3`; old files still load with heading unknown.
2. **Bearing comes from the IMU**, using the same gravity-projected gyro and
   stillness-locked bias as the scanner. The survey asks you to *face the
   START arrow and hold still* on entry — that single act ties the survey's
   heading frame to the room's frame.
3. **Aim ticks on the plan.** Each sample dot draws a whisker showing which
   way the device faced.
4. **Honest UI copy.** With the patch selected: *"patch: AIM AT AP before
   sampling."* With the internal antenna: *"omni: aim does not matter."*
   The measurement procedure is now part of the instrument.
5. **AP direction estimate.** A directional antenna reads strongest when
   aimed at the source, so the best EXT sample's own bearing estimates where
   the AP is — drawn as a cyan ray labelled *"AP this way?"*. Labelled as an
   estimate on purpose: indoor multipath makes it approximate.

The demo line this buys: *"the internal antenna measures coverage; the patch
measures reach in a direction — and it can point back at the router."*

---

## Next: fuse RF sampling into WALK+ (post-contest)

**The idea (Matt's):** WALK+ already tracks pose and heading while you walk
the room. Sample RSSI during that same walk and one pass produces the room
map *and* the survey — no tapping, dense coverage, every sample
heading-tagged for free. The antenna suite's scope screen already proves the
sampling cadence works while moving.

**Why it's the right end state:** it removes the survey's most tedious step
(tap position → tap sample, repeated), it collects far more points than
anyone would tap by hand, and it makes the patch's directionality a natural
product of walking rather than a separate procedure.

### Feasibility notes (checked)

- `_rf_measure()` uses `WiFi.scanNetworks(..., dwell=120, channel)` ×2 —
  ~250–400 ms blocking. The antenna suite uses `dwell=60` (~100 ms), which
  is the cadence to copy.
- A 100 ms block is **under** the scanner's `dt > 0.25 s` IMU guard, so yaw
  integration survives. Sample every ~1.5 s → ~7% duty cycle, safe for both
  the IMU and the 32 KB camera UART buffer.
- WiFi must be brought up inside the scanner (`_wifi_up()`), and a target AP
  must already be chosen (the global saved AP). No AP → skip silently.

### The coordinate problem (must be solved first)

RF samples are stored in **quantised mesh space**, but during a walk the
final mesh doesn't exist yet — quantisation is computed at write time from
the finished geometry. Two consequences:

1. Walk-collected samples must be buffered in **world metres** and converted
   at `_write_phase1`, after `ScanMeshWriter::quantisation()` runs.
2. **Known issue, present today:** any rebuild that changes the geometry
   bounds (hiding an outlying object via the OBJ menu, or a WALK+/SCAN+ that
   grows the room) changes the quantisation — and existing `.rf` samples,
   stored in the *old* mesh space, silently shift relative to the new mesh.
   *Demo workaround: settle the object list before surveying a room.*

**The fix for both: store RF samples in world metres** (`RFS4`), converting
to mesh space only at draw time. That single change removes the shift bug
and unblocks walk fusion, since walk samples are natively in world metres.
Do this first, then the fusion is mostly plumbing.

### Sketch

```
_walk_rf_tick():                    # every ~1.5s during WALK+
    if !_walk or no target AP: return
    rssi = single-channel scan burst (dwell 60)
    buffer (pose.x, pose.z, heading, rssi, antenna)

_write_phase1():                    # after quantisation is known
    for each buffered sample: world -> mesh, _rf.add(...)
    _rf.saveBeside(path)
```

With RFS4 world-metre storage the conversion disappears entirely and the
buffer can write straight through.
