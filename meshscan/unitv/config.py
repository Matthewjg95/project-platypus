# config.py - MeshScan Unit V (K210 / MaixPy) configuration
#
# Keep this module import-light: it is loaded at boot before anything else.
# No numpy, no heavy stdlib. Plain constants only.

# ---- camera ---------------------------------------------------------------
# OV2640 on the Unit V. QVGA keeps JPEG small enough to stream at 115200 and
# matches what the depth estimator on the M5Tab assumes.
FRAME_WIDTH  = 320
FRAME_HEIGHT = 240

# Horizontal field of view of the OV2640 at this crop, in degrees. Used by the
# M5Tab depth/azimuth math, but kept here too so both ends share one number if
# you re-export. Measure yours and update — 60 deg is a reasonable default.
CAMERA_HFOV_DEG = 60.0

JPEG_QUALITY = 50        # 0..100; lower = smaller packets, blockier image.
                         # Real-room frames at 60 hit 13KB+, brushing the
                         # 460800-baud ceiling (~46KB/s); 50 buys fps headroom.

# Run KPU inference every Nth frame; intermediate frames re-send the last
# detections. Inference (~120ms) dominates the loop — every 3rd frame roughly
# triples stream fps while detection still updates ~1.7x/sec (scan samples 1/s).
DETECT_EVERY_N = 3

# ---- UART -----------------------------------------------------------------
# Start conservative. Bump to 1.5M or 3M once framing is proven stable.
UART_BAUD = 460800   # 4x of 115200. 921600 was too fast for the K210 UART. Tab5 must match.
UART_TX_PIN = 35         # Unit V grove pad -> M5Tab RX  (verify your wiring)
UART_RX_PIN = 34         # Unit V grove pad <- M5Tab TX

# ---- model ----------------------------------------------------------------
# Pascal VOC 20-class tiny-YOLOv2 (kmodel V3, works with this 2019 MaixPy).
# Put 20class.kmodel on the SD card root.
MODEL_PATH = "/sd/20class.kmodel"

# YOLOv2 detection layer parameters.
# DIAGNOSTIC: threshold dropped to 0.15 to establish whether the kendryte
# model emits ANYTHING under MaixPy (dets=0 mystery). If boxes appear at low
# confidence, raise back toward 0.4; if still zero, it's an input-geometry /
# model-compat issue and we switch to a MaixHub V4 model.
DETECT_THRESHOLD = 0.15  # min confidence to emit a detection
DETECT_NMS       = 0.3   # non-max-suppression IOU

# Anchors for the detection layer. These MUST match the anchors the .kmodel was
# trained with. These are the standard tiny-yolo-voc (20-class) anchors that the
# canonical 20class.kmodel uses. If your model differs, replace them.
ANCHORS = (1.08,  1.19,
           3.42,  4.41,
           6.63,  11.38,
           9.42,  5.11,
           16.62, 10.52)

# Pascal VOC 20-class names, index == class_id sent over UART. Order is the
# standard VOC order the 20class.kmodel was trained with — the M5Tab side
# (object_labels + depth_estimator) MUST use this same order.
CLASS_NAMES = (
    "aeroplane", "bicycle", "bird", "boat", "bottle",
    "bus", "car", "cat", "chair", "cow",
    "diningtable", "dog", "horse", "motorbike", "person",
    "pottedplant", "sheep", "sofa", "train", "tvmonitor",
)
