# Detection Quality — Test Protocol & Roadmap

Current state: kendryte tiny-YOLOv2 VOC-20 (2018) on the Unit V KPU, threshold
0.15, inference every 3rd frame. It works (person/tvmonitor/bottle confirmed
live) but confidences run low (16–27%) and placement is coarse. This doc is
the plan to make detection *good*, ordered by evidence-per-hour.

## The instrument we already have
The camera prints `[det] fid=N: class=NN%` for every detection and `[tele]`
lines with fps — over the USB-C console (COM10), while the Grove keeps
feeding the Tab5. Every test below is "point camera at X, capture 30–60 s of
console, count". Claude can run capture + scoring scripted; the human moves
the camera. Keep lighting constant within a test.

## Tier 1 — characterize what we have (one evening, no code changes)

**T1. Class × distance profile.** For each class we care about (person, chair,
tvmonitor, bottle, sofa if available): hold the camera on the object at ~1 m,
~2 m, ~3 m for 30 s each. Metrics per cell: detection rate (dets/inference)
and mean confidence. Output: a table that tells us (a) each class's usable
range, (b) whether 0.15 is the right threshold per class, (c) which classes
are hopeless on this model.

**T2. False-positive census.** 60 s pointed at object-free wall/floor, 60 s at
a cluttered but known scene. Count spurious dets by class. This sets the
threshold floor: raise until FP rate ~0, check what recall survives (T1 data).
(Early signal: bottles may be over-firing — the "27 objects" scan.)

**T3. Lighting sensitivity.** T1's 2 m case repeated: lights on / off /
backlit. If confidence collapses when backlit, scan UX guidance ("lights on,
avoid windows behind objects") is a free accuracy win for the demo.

**Analysis is scripted:** parse `[det]` lines → rate + confidence stats per
run. (Tool candidate: `meshscan/tools/det_bench.py`, ~an hour to write when
T1 starts.)

## Tier 2 — cheap wins on current hardware

- **Per-class thresholds** (camera-side, config table): VOC classes differ
  wildly in calibration; T1/T2 give the numbers. 30-minute change.
- **Tab5 accumulator tuning** with T1 stats: merge radius (now 0.6 m),
  MIN_OBSERVATIONS (now 2), and weighting confident detections higher when
  averaging positions.
- **Exposure tuning** if T3 shows sensitivity: lock AGC ceiling / brightness
  in `sensor` init.
- **NOT worth it:** chasing the input-geometry theory further (the model
  demonstrably runs at 320×240), or hand-editing anchors.

## Tier 3 — the real jump: replace the model
The 2018 kendryte model is the ceiling. We are **kmodel V4 capable** now
(MaixPy v0.5.0), which opens:

1. **MaixHub pre-trained V4 detector** (~an afternoon). Modern training =
   better calibration; expect usable confidences at 0.4+ instead of 0.2.
   Swap = download → SD → update anchors/classes in config.py → rebundle.
2. **Custom-trained model on MaixHub** (~a weekend, the contest-grade move).
   Train on exactly the indoor classes Room Scan needs — including ones VOC
   lacks: **door, window, desk, shelf** (door alone transforms room
   understanding). MaixHub's train service outputs kmodel + anchors directly.
   Needs a labeled dataset: MaixHub public sets + ~100–300 photos of real
   rooms (the Tab5 demo rooms themselves — free training data from scans).
3. (Fallback if MaixHub friction: aXeleRate local training → nncase convert.)

## Recommended roadmap (contest-aware)
1. **T1 + T2 next camera session** (~1 hr) → per-class thresholds + Tab5
   tuning from data, not vibes. Immediate demo improvement.
2. **MaixHub V4 model swap attempt** right after — biggest win per hour.
3. **Custom model** only if the V4 zoo model underwhelms AND schedule allows
   before Aug 7; otherwise post-contest.
4. Placement accuracy beyond detection (multi-bearing triangulation of the
   same object across the sweep — we have yaw per observation now) is a
   post-contest Phase-2-adjacent project.
