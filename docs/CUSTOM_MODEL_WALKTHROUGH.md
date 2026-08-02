# Custom Detection Model — Walkthrough (#1 roadmap item)

Replace the generic VOC-20 model (cats, bottles, 16-27% confidence) with a
room-mapping model trained on OUR classes. This is the single biggest
mapping-quality upgrade available with zero new hardware.

**Target classes (proposal, 10):** door, window, desk, monitor, chair, sofa,
bed, table, shelf, person. Doors and windows are the prize — they anchor
walls, doorway positions link rooms in the building map, and no amount of
tuning gets them out of VOC.

---

## Path A (recommended): MaixHub online training

MaixHub (maixhub.com, Sipeed's free training service) trains YOLO-family
detectors and exports **K210 kmodel V4** directly — the format our MaixPy
v0.5.0 firmware already runs. No local GPU, no nncase wrestling.

### 1. Collect the dataset — with the camera itself

The Unit V's own imaging quirks (lens, exposure, 320x240) should be IN the
training data. Two collection routes:

- **On-SD capture:** temporary MaixPy script that saves a JPEG to SD every
  2s while you slowly pan rooms. ~10 minutes per room gives hundreds of
  frames. (I can write `capture_dataset.py` for the Unit V when we start.)
- Phone photos work as filler but shift the domain; keep them a minority.

Guidelines:
- 150-300 images per class minimum, varied angles/lighting/distances
- Include NEGATIVES: frames with none of the classes (bare walls, floors)
- Capture at the camera's native aspect; MaixHub handles resize to 224x224

### 2. Label on MaixHub

Create a "Detection" project, upload, draw boxes in their web labeler (or
upload VOC/YOLO-format annotations if labeling locally in LabelImg).

### 3. Train + export

- Model type: YOLOv2 (K210-compatible backbone; MaixHub picks anchors)
- Export target: **K210 kmodel (V4)** — do NOT pick V5/K230 formats
- Download the bundle: `.kmodel` + generated anchors + class list

### 4. Deploy to the Unit V

1. Copy the kmodel to SD as `/sd/roomnet.kmodel` (the SD slot beats
   flashing it; our loader already prefers /sd)
2. Update `meshscan/unitv/config.py`:
   - `MODEL_PATH = "/sd/roomnet.kmodel"`
   - `ANCHORS = (...)` — the values MaixHub generated (5 pairs)
   - `CLASS_NAMES = (...)` — new class order, must match training exactly
   - `THRESHOLD` — start 0.30; custom models score far higher than the
     VOC-20's 16-27%, so the floor comes UP as quality goes up
3. Rebuild the bundled boot.py (`python meshscan/tools/unitv_bundle.py`)
   and push with `unitv_upload.py` (cable, dtr=False/rts=False)

### 5. Tab5-side changes (one file each)

- `object_labels.h/.cpp`: new class id -> name/colour table
- `depth_estimator.cpp`: new height/width tables — doors 2.03m tall(!),
  windows ~1.2m, desks 1.2-1.6m wide. Doors are a RANGING GIFT: nearly
  uniform height worldwide, tall (pixel-accurate), on every wall.
- `scanner_applet.h`: `_default_visible()` + `_acc_conf()`/`_min_obs()`
  per-class policies for the new ids
- Wire protocol, accumulation, carve, registration: **unchanged** — the
  architecture was built for this swap.

### 6. Validate (bench protocol)

Use the T1/T2 procedure in docs/DETECTION_TUNING.md: known objects at
measured distances, compare detected class + estimated range vs tape
measure. Tune per-class sizes from the errors.

---

## Path B: local training (fallback if MaixHub's free tier disappoints)

axelerate (Keras YOLOv2 for K210) or Sipeed's maix_train repo, then
`nncase 0.2.x` to compile to kmodel V4. Works but is a toolchain slog
(TF 1.x-era pins, nncase version sensitivity). Only if Path A dead-ends.

## Effort estimate

- Dataset capture+label: 2-4 hours (the slow part, mostly labeling)
- Training: ~1 hour on MaixHub's queue
- Deploy + Tab5 tables: ~1 hour
- Validation pass: 1 hour

Fits in a weekend; post-contest slot is fine, or pre-contest if a rainy
afternoon appears — the demo story gets much stronger with "door" and
"window" labels on the map.
