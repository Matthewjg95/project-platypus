# meshscan/ — UnitV camera firmware + desktop tools

The camera half of Project Platypus. The Tab5 firmware lives in
`Tab5 3D Render/`; this tree holds what runs on the **M5 Unit V (Kendryte
K210)** and the desktop helpers that deploy and inspect it.

| Path | What |
|------|------|
| `unitv/` | MaixPy firmware: OV7740 capture → KPU tiny-YOLOv2 → JPEG + detections streamed over Grove UART (460800) as framed, CRC-8-checked packets |
| `unitv/model/20class.kmodel` | tiny-YOLOv2 VOC-20 (kendryte-standalone-demo), lives on the camera SD as `/sd/20class.kmodel` |
| `tools/unitv_bundle.py` | bundles every module into ONE `boot.py` executed from RAM — this MaixPy build's boot-time filesystem imports are unreliable, and a single file also sidesteps `/sd` shadowing `/flash` |
| `tools/unitv_upload.py` | chunked base64 raw-REPL uploader (ampy crawls and mis-roots relative paths) |
| `tools/unitv_fix_sd_shadow.py` | removes stale SD copies that would shadow flash modules |
| `tools/mesh_to_obj.py`, `tools/kmodel_info.py` | inspect scan output / kmodel headers on the desktop |
| `shared/mesh_format.h` | standalone spec of the `.mesh` binary format |

## Deploy the camera

```bash
cd meshscan/tools
python unitv_bundle.py                                   # -> ../unitv/build/boot.py
python unitv_upload.py ../unitv/build/boot.py /flash/boot.py COM10
```

Wire protocol (camera → Tab5): `FRAME 0xF1 | len u16 | jpeg | crc8` and
`DETECT 0xD1 | frame_id u16 | count u8 | 10B/det | crc8` (CRC-8 poly 0x07).
Config (baud, model path, anchors, thresholds, detect-every-N) lives in
`unitv/config.py`; the camera prints `[det]`/`[tele]` telemetry on its USB
console while the Grove feeds the Tab5.
