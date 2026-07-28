# detector.py - YOLOv2 object detector wrapper for the K210 KPU.
#
# Thin wrapper over MaixPy's `KPU` module. Loads a YOLOv2 .kmodel and runs the
# detection layer, returning plain tuples so the rest of the firmware stays
# decoupled from the KPU API.
#
# Degrades gracefully: if the model can't be loaded (e.g. no kmodel on SD yet),
# the detector stays "unavailable" and detect() returns [] — main.py keeps
# streaming camera frames so the Tab5 still gets a live feed. Add a kmodel and
# reboot to enable detections.
#
# Keep imports minimal — KPU only. No numpy.

import KPU as kpu
import config


class Detection:
    __slots__ = ("class_id", "confidence", "x", "y", "w", "h")

    def __init__(self, class_id, confidence, x, y, w, h):
        self.class_id = class_id
        self.confidence = confidence   # 0..100 (already scaled for UART)
        self.x = x                     # bbox top-left x, pixels
        self.y = y                     # bbox top-left y, pixels
        self.w = w                     # bbox width, pixels
        self.h = h                     # bbox height, pixels

    def as_tuple(self):
        return (self.class_id, self.confidence, self.x, self.y, self.w, self.h)


class Detector:
    """Owns the loaded kmodel and the YOLOv2 region layer."""

    def __init__(self,
                 model_path=config.MODEL_PATH,
                 anchors=config.ANCHORS,
                 threshold=config.DETECT_THRESHOLD,
                 nms=config.DETECT_NMS):
        self._task = None
        self.available = False
        try:
            self._task = kpu.load(model_path)
            ok = kpu.init_yolo2(self._task, threshold, nms, len(anchors) // 2, anchors)
            if not ok:
                raise ValueError("init_yolo2 failed")
            self.available = True
            print("[detector] model loaded: %s" % model_path)
        except Exception as e:
            # No model / bad model — keep running camera-only.
            self.available = False
            if self._task is not None:
                try: kpu.deinit(self._task)
                except Exception: pass
                self._task = None
            print("[detector] no model (%s): running camera-only, 0 detections" % e)

    def detect(self, img):
        """Run inference on a MaixPy image. Returns a list of Detection."""
        if not self.available:
            return []
        results = kpu.run_yolo2(self._task, img)
        out = []
        if results:
            for r in results:
                conf = int(r.value() * 100.0)
                if conf < 0:
                    conf = 0
                elif conf > 100:
                    conf = 100
                out.append(Detection(r.classid(), conf,
                                     r.x(), r.y(), r.w(), r.h()))
        return out

    def deinit(self):
        if self._task is not None:
            kpu.deinit(self._task)
            self._task = None
        self.available = False
