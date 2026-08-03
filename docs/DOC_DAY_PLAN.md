# Documentation Day — Aug 3 Plan

One rule drives the whole day: **capture is perishable, writing is not.**
Daylight, a working device, and your energy are the scarce inputs — spend
them on camera work. Words can happen at 10pm; good light cannot.

Second rule: **batch by mode.** Every switch between device-in-hand,
PC-tethered, and editing costs 15 minutes of momentum. The day is four
blocks, each in one mode.

---

## Block 1 — Validation + stills (morning, ~1.5h, device in hand)

Bright room, blinds open (the K210 lives on light).

1. One sweep of a familiar room → does the +15% sensitivity land right?
   - Good → firmware is FROZEN, say so out loud
   - Off → tell me ONE constant nudge, flash, retest once, then freeze anyway
2. Assembly photos: you have them — dump into `assets/photos/` now
   (phone → PC first thing, before they're stranded on the phone)
3. Missing stills to grab while the light is good:
   - Rig in hand at scanning posture (the "instrument" shot)
   - Close-up: antenna PCB with MMCX cable visible
   - Close-up: Unit V pod + Grove cable route
   - The Tab5 on the desk showing the home screen (hero product shot)

## Block 2 — Screen captures (late morning, ~1h, USB tethered)

Plug in; I drive `tab5_screenshot.py` while you drive the UI. Shot list —
one BMP each, I'll convert/crop:

- [ ] Boot splash (power-cycle, catch it in the 1.1s window — 2 tries)
- [ ] Home screen, OTA chip green
- [ ] Sweep dial mid-scan: arc ~40%, pips visible, preview live
- [ ] Finished room: carved floor + labels + start arrow (best-looking room)
- [ ] OBJ menu open (shows per-room curation)
- [ ] RF heatmap INT antenna: legend + DEAD/BEST visible
- [ ] RF heatmap EXT antenna: same room, same angle (the A/B pair!)
- [ ] Building map with 3+ rooms arranged
- [ ] Antenna suite: channel scan + polar plot
- [ ] Walk mini-map mid-walk (if walk demos well; skip if not)

**The A/B heatmap pair is the money asset. Do not leave this block
without it.**

## Block 3 — Video (afternoon, ~3h, filming mode)

Follow docs/DEMO_SCRIPT.md exactly. Order of operations:

1. Rehearse the full 2-minute flow twice, NO camera. Cut anything that
   misbehaved twice — cuts are free, retakes are not.
2. Film each script beat as its OWN clip (0:00 cold open, hardware pan,
   sweep hero shot, 3D result, RF walk, building map close). Short clips =
   editing is assembly, not surgery.
3. Sweep hero shot: film the SCREEN straight-on, tripod/propped phone.
   Two takes minimum.
4. B-roll, 10s each: hands rotating during sweep (wide), tapping RF
   samples, dragging rooms on the map.
5. Voiceover: record separately tonight over the cut — never live on set.

## Block 4 — Assembly + words (evening, PC only)

1. Rough-cut the video to ≤2:30. Good beats perfect; judges watch 30s.
2. Open docs/HACKSTER_WRITEUP.md (drafted — tonight's job is filling
   [SLOT]s with today's assets, not writing)
3. Create the Hackster project as DRAFT, paste sections, upload images
4. Flip project-platypus public ONLY at submission time (Aug 6/7)

---

## Asset hygiene (30 seconds that saves an hour)

```
assets/
  photos/      p01_rig_hand.jpg, p02_antenna_pcb.jpg ...
  screens/     s01_splash.png, s02_home.png ...
  video/       c01_coldopen.mp4, c02_hardware.mp4 ...
```
Number them in story order the moment they land. Hackster upload order = file
order = zero re-sorting.

## What I can do while you capture

- Convert/crop every screenshot as it lands
- Tighten writeup prose around the real assets
- Produce the wiring/architecture diagram for the writeup
- Final README pass on the contest repo (it becomes public-facing)

## Hard don'ts tomorrow

- No firmware changes after Block 1. None.
- No partition-table experiment. (Bricked device = no entry.)
- No new features, however small they look.
- Don't film in the evening — if Block 3 slips, it moves to Aug 4 morning.
