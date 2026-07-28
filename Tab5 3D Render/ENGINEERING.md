# M5Stack Tab5 — Portable 3D Mesh Viewer
## Engineering Reference

---

## Project Structure

```
m5viewer/
├── platformio.ini          — Build system config
├── src/
│   ├── main.cpp            — App entry point & render loop
│   ├── mesh.h              — Data structures (Vertex, Face, Mesh, RenderState)
│   ├── mesh_loader.h       — SD card binary .mesh loader
│   ├── renderer.h          — Full 3D pipeline (transform → cull → project → rasterise)
│   └── input.h             — Touch drag input handler
└── tools/
    └── stl_to_mesh.py      — Offline STL → .mesh converter
```

---

## Architecture Overview

```
[SD Card .mesh file]
        │
        ▼
  mesh_loader.cpp          — Parse header, alloc PSRAM, stream vertex+face data
        │
        ▼
     Mesh (PSRAM)           — Vertex[N] int16×3  +  Face[M] uint16×3
        │
        ▼
   renderer.cpp
        ├─ Build 3×3 rotation matrix  (once per frame)
        ├─ Transform all N vertices   (once per frame, O(N))
        ├─ For each face M:
        │    ├─ Backface cull (2D cross product)
        │    ├─ Compute flat-shaded colour
        │    └─ fillTriangle()  via M5GFX
        │
        ▼
  M5GFX display (batched via startWrite/endWrite)
```

---

## Module Reference

### mesh.h

Core data types.  Header-only, no allocation.

| Type | Size | Description |
|------|------|-------------|
| `Vertex` | 6 bytes | int16 x,y,z — quantised world position |
| `Face` | 6 bytes | uint16 a,b,c — vertex indices |
| `Mesh` | 24 bytes | Pointers + counts; data in PSRAM |
| `RenderState` | 24 bytes | Euler angles, scale, camera Z, FOV |
| `ProjectedVertex` | 12 bytes | float sx,sy,z — screen space result |

### mesh_loader.h

`bool load_mesh(const char* path, Mesh& mesh)`

- Opens SD file, reads 8-byte header (vertex_count, face_count).
- Allocates vertex buffer and face buffer in PSRAM
  (`MALLOC_CAP_SPIRAM`).
- Streams vertex block then face block directly into PSRAM buffers.
- Returns `false` on any error; prints diagnostics to Serial.

**Memory usage:**
```
vertex_count × 6 bytes   (e.g. 10000 verts = 60 KB)
face_count   × 6 bytes   (e.g. 15000 faces = 90 KB)
ProjectedVertex scratch  = vertex_count × 12 bytes = 120 KB
Total for 10k/15k mesh   ≈ 270 KB PSRAM
```

### renderer.h

**`bool begin(M5GFX* display, uint32_t max_vertices)`**
Allocates projected-vertex scratch buffer in PSRAM.

**`void render_frame(const Mesh&, const RenderState&, uint16_t bg)`**

Pipeline steps:

1. **Rotation matrix** — Euler Ry·Rx·Rz, computed once.
2. **Vertex transform** — All N vertices: rotate + perspective project → screen coords.  O(N).
3. **Clear screen** — `fillScreen(bg)`.
4. **Face loop** — O(M):
   - Backface cull: 2D cross product of screen-space edges.
     `cross = (B-A) × (C-A)`.  Skip if ≥ 0 (back-facing in Y-down coords).
   - Flat shade: compute face normal from camera-space edge vectors, dot with fixed light.
   - `fillTriangle(ax,ay, bx,by, cx,cy, color)`.
5. **Batch flush** — `startWrite()` / `endWrite()` wraps steps 3–4.

**Performance scaling (estimated):**

| Triangles | Expected FPS (ESP32-P4) |
|-----------|------------------------|
| 5,000 | ~50 FPS |
| 10,000 | ~28 FPS |
| 15,000 | ~18 FPS |
| 20,000 | ~13 FPS |

Key bottleneck: `fillTriangle()` in M5GFX.  Each call involves SPI/parallel bus writes proportional to triangle screen area.

### input.h

`InputHandler::update()` → `InputDelta { dx, dy, active }`

- Polls `M5.Touch.getDetail()` each frame.
- Tracks previous touch position across frames.
- Applies 2-pixel deadzone to suppress jitter.
- Returns pixel deltas; caller multiplies by `DRAG_SENSITIVITY`
  (0.005 rad/px by default) to get rotation delta.

---

## .mesh File Format

```
Offset  Type      Description
------  --------  -----------
0       uint32    vertex_count
4       uint32    face_count
8       int16[3]  vertices[0]  — x, y, z
...     int16[3]  vertices[vertex_count-1]
8+6×vc  uint16[3] faces[0]    — a, b, c
...     uint16[3] faces[face_count-1]
```

All values little-endian.  No padding between records.

**File size formula:**
```
8  +  vertex_count × 6  +  face_count × 6   (bytes)
```

A 10k-vertex / 15k-face mesh = 8 + 60000 + 90000 = **150,008 bytes (~147 KB)**

---

## Python Converter — stl_to_mesh.py

### Installation

```bash
pip install numpy-stl        # required
pip install open3d           # optional: enables --decimate
```

### Usage

```bash
# Basic conversion
python stl_to_mesh.py part.stl part.mesh

# With in-script decimation (requires open3d)
python stl_to_mesh.py part.stl part.mesh --decimate 12000

# Inspect without writing
python stl_to_mesh.py part.stl part.mesh --info
```

### Pipeline steps

1. Load STL → merge duplicate vertices (numpy-stl).
2. Optionally decimate with open3d quadric simplification.
3. Centre bounding box at origin.
4. Scale longest axis to ±30000 int16 units.
5. Clip to int16 range, write header + vertex block + face block.

---

## Blender Workflow (Recommended)

For large or complex CAD parts, pre-process in Blender for best quality:

### Steps

1. **File → Import → STL** — import your CAD export.
2. Select the object.  In Properties → **Modifier** tab, add **Decimate**.
3. Set **Ratio** or switch to **Face Count** mode.
   - Target: 10,000–15,000 faces for 20–30 FPS.
4. Enable **Planar** mode and set **Angle Limit** to 5°–15° to preserve
   flat faces while decimating curved surfaces aggressively.
5. Click **Apply**.
6. **File → Export → STL (Binary)**.  ✓ Apply Modifiers.
7. Run `stl_to_mesh.py` on the exported file.

### Recommended Decimate settings

| Surface type | Ratio | Angle Limit |
|---|---|---|
| Mechanical (flat faces) | 0.1–0.3 | 5° |
| Organic / curved | 0.3–0.5 | 15° |
| Highly detailed | 0.05–0.1 | 3° |

---

## Performance Optimisation

### Implemented

| Technique | Where | Benefit |
|---|---|---|
| Transform once per frame | renderer.cpp | Eliminates N×M redundant transforms |
| int16 vertex storage | mesh.h | 2× smaller vertex data; better cache use |
| Backface culling | renderer.cpp | Skips ~50% of faces on convex meshes |
| PSRAM allocation | mesh_loader, renderer | Keeps internal SRAM free for stack |
| Display batching | renderer.cpp | Reduces SPI bus overhead |
| `-O2 -ffast-math` | platformio.ini | ~20–30% faster float math |

### Next optimisations (Phase 4)

1. **Z-sort faces** — Painter's algorithm; avoids intersecting
   triangles at low cost (~2 ms for 15k faces with radix sort).
2. **Tile-based rasteriser** — Split screen into 8×8 tiles; render
   only triangles overlapping each tile.  Reduces SPI transfer size.
3. **DMA-backed framebuffer** — Write pixel data off the CPU via DMA
   while the CPU processes the next batch of triangles.
4. **SIMD / fixed-point math** — ESP32-P4 supports vector
   instructions; replace float matrix multiply with fixed-point SIMD.
5. **Mesh bounding sphere cull** — Skip mesh entirely if off-screen.
6. **Face index sorting by depth** — Sort once on load; speeds up
   painter's algorithm to O(1) per frame for static camera changes.

---

## Debugging & Testing

### Serial diagnostics

Enable in `setup()`:
```cpp
Serial.begin(115200);
```

Renderer prints per-frame timing when HUD is active.
`load_mesh()` prints vertex/face counts and PSRAM usage on load.

### Frame time monitoring

The HUD overlay in `main.cpp` prints FPS and frame time every 500 ms.
Disable by commenting out the HUD block to remove `printf` overhead.

### Validating the .mesh file (Python)

```python
import struct, pathlib
data = pathlib.Path("part.mesh").read_bytes()
vc, fc = struct.unpack_from('<II', data, 0)
print(f"Vertices: {vc}  Faces: {fc}")
expected = 8 + vc*6 + fc*6
print(f"Expected size: {expected}  Actual: {len(data)}  {'OK' if len(data)==expected else 'MISMATCH'}")
```

### Common issues

| Symptom | Likely cause | Fix |
|---|---|---|
| Model invisible | Scale too small / cam_z too close | Increase `cam_z`; check `scale` |
| Inside-out normals | Winding order reversed in export | Flip normals in Blender or reverse face winding in converter |
| Corrupt triangles | Mesh not watertight after decimate | Use Blender "Fill Holes" before export |
| SD mount fails | CS pin wrong | Check `SD_CS_PIN` in platformio.ini |
| PSRAM alloc fails | Mesh too large | Decimate further; check `ESP.getFreePsram()` |
| Low FPS | Too many faces | Target ≤15k; enable `-O2 -ffast-math` |

---

## Development Phases

| Phase | Description | Status |
|---|---|---|
| 1 | Rotating hardcoded cube | ✅ Implemented |
| 2 | SD mesh loading | ✅ Implemented (toggle in main.cpp) |
| 3 | Touch rotation + zoom | ✅ Implemented |
| 4 | Z-sort, DMA, SIMD optimisation | 🔲 Planned |
| 5 | Lighting, Z-buffer, measurements | 🔲 Planned |
| 6 | Joystick navigation | 🔲 Designed (stub in input.h) |
| 7 | Vertex selection, annotations | 🔲 Future |

---

## Memory Budget (10k-vertex / 15k-face mesh)

| Region | Usage |
|---|---|
| Vertex PSRAM buffer | 60 KB |
| Face PSRAM buffer | 90 KB |
| ProjectedVertex scratch | 120 KB |
| M5GFX framebuffer (1280×720×2) | 1,843 KB |
| Firmware + stack + heap | ~512 KB |
| **Total PSRAM** | **~2.1 MB** |

ESP32-P4 has 8 MB PSRAM — comfortably within budget.
