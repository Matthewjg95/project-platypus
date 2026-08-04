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

## Walk-fused survey — SHIPPED

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

### The coordinate problem (solved)

RF samples are stored in **quantised mesh space**, but during a walk the
final mesh doesn't exist yet — quantisation is computed at write time from
the finished geometry. And worse, a *pre-existing bug*: any rebuild that
changed the geometry bounds (hiding an object via the OBJ menu, a WALK+ that
grew the room) changed the quantisation, and existing samples — stored in
the *old* mesh space — silently slid relative to the new floor.

**Fix: `.rf` files now record the quantisation their samples were captured
under** (`qc[3]`, `qs`, format `RFS4`). On every rebuild the survey is
reloaded, `remap()`ed from the old quantisation to the new one, and saved.
The heatmap stays glued to the floor no matter how the room is edited. Files
without the field (`RFS2`/`RFS3`) load and adopt the current quantisation,
i.e. exactly today's behaviour, no worse.

That same machinery makes walk fusion trivial: walk samples buffer in world
metres and convert once, at the same moment.

### How it works now

- **Walk start:** if the room (or the saved global pick) has a target AP,
  WiFi comes up, the RF switch is set to the selected antenna, and the walk
  becomes a survey. No AP → skipped silently, walk behaves as before.
- **During the walk:** every 1.5 s, one single-channel scan at 60 ms dwell
  reads the target AP; the reading is buffered with the current pose and
  bearing. The mini-map drops a dot coloured by signal strength at each
  sample, so the heatmap visibly builds itself as you walk. The status line
  shows `Walk 12 st 8.4m | EXT RF:6 -58 dBm`.
- **At FINISH:** buffered samples convert world → mesh with the final
  quantisation and merge into the room's `.rf`, alongside anything sampled
  by hand.

### Demo consequence

The survey no longer requires tap-position-then-tap-sample. **One walk
produces the room and the heatmap together** — and because every sample
carries its bearing, the patch antenna's directionality is captured for
free. To survey with the patch: pick EXT in the RF screen once, then WALK+.

### Still worth doing later

- Choose the antenna directly from the walk screen (today it inherits the
  RF screen's selection — the HUD shows which, so it is unambiguous, just
  not adjustable in place).
- Multi-position bearing intersection for a true AP *fix* rather than a
  single-ray estimate.
