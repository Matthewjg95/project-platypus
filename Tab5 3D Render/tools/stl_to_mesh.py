#!/usr/bin/env python3
"""
stl_to_mesh.py — Offline converter: STL → embedded .mesh format
================================================================

Converts a binary or ASCII STL file into the compact binary mesh
format consumed by the M5Stack Tab5 viewer.

Output format (little-endian):
  uint32  vertex_count
  uint32  face_count
  [Vertex × vertex_count]   each: int16 x, int16 y, int16 z
  [Face   × face_count]     each: uint16 a, uint16 b, uint16 c

Typical usage:
  pip install numpy-stl
  python stl_to_mesh.py gear.stl gear.mesh

With optional pre-decimation using open3d:
  pip install open3d
  python stl_to_mesh.py gear.stl gear.mesh --decimate 10000

Blender workflow (recommended for large CAD models):
  1. Import STL into Blender.
  2. Add Decimate modifier → set Face Count target (e.g. 15000).
  3. Apply modifier.
  4. File → Export → STL (binary).
  5. Run this script on the exported STL.
"""
import sys
import io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
import sys
import struct
import argparse
import pathlib
import numpy as np

try:
    from stl import mesh as stl_mesh_lib
    HAS_STL = True
except ImportError:
    HAS_STL = False

try:
    import open3d as o3d
    HAS_O3D = True
except ImportError:
    HAS_O3D = False


# --------------- Coordinate scaling -------------------------
# The viewer stores vertices as int16 (±32767).
# We map the longest axis of the bounding box to ±TARGET_RANGE.
TARGET_RANGE = 30000   # int16 units; leave ±2767 headroom


# --------------- STL loader ---------------------------------

def load_stl_numpy(path: str):
    """
    Returns (vertices Nx3 float32, faces Mx3 int32).
    Merges duplicate vertices so the face index references
    unique positions.
    """
    if not HAS_STL:
        sys.exit("numpy-stl not installed.  Run: pip install numpy-stl")

    stl = stl_mesh_lib.Mesh.from_file(path)

    # STL gives one set of 3 vertices per triangle (redundant).
    # Flatten to raw float array then find unique vertices.
    raw_verts = stl.vectors.reshape(-1, 3)   # (N*3, 3)

    # Round to 6 decimal places to merge near-duplicate verts
    raw_verts = np.round(raw_verts, decimals=6)

    # unique() returns unique rows + inverse mapping
    unique_verts, inverse = np.unique(raw_verts, axis=0,
                                       return_inverse=True)

    n_tris  = len(stl.vectors)
    faces   = inverse.reshape(n_tris, 3).astype(np.int32)

    print(f"  STL loaded:  {len(unique_verts)} unique verts  "
          f"{n_tris} faces")
    return unique_verts.astype(np.float32), faces


# --------------- Optional open3d decimation -----------------

def decimate_o3d(verts: np.ndarray, faces: np.ndarray,
                  target_faces: int):
    """Decimate using open3d's quadric simplification."""
    if not HAS_O3D:
        sys.exit("open3d not installed.  Run: pip install open3d")

    m = o3d.geometry.TriangleMesh()
    m.vertices  = o3d.utility.Vector3dVector(verts.astype(np.float64))
    m.triangles = o3d.utility.Vector3iVector(faces)
    m.remove_degenerate_triangles()
    m.remove_duplicated_vertices()

    m2 = m.simplify_quadric_decimation(target_number_of_triangles=target_faces)

    out_v = np.asarray(m2.vertices,  dtype=np.float32)
    out_f = np.asarray(m2.triangles, dtype=np.int32)
    print(f"  After decimate: {len(out_v)} verts  {len(out_f)} faces")
    return out_v, out_f


# --------------- Scale & quantise ---------------------------

def quantise_vertices(verts: np.ndarray) -> np.ndarray:
    """
    Normalise the mesh so its longest axis fits in ±TARGET_RANGE,
    then convert to int16.

    Returns int16 array of shape (N, 3).
    """
    # Centre at origin
    centre = (verts.min(axis=0) + verts.max(axis=0)) / 2.0
    verts  = verts - centre

    # Scale longest axis to TARGET_RANGE
    max_extent = np.abs(verts).max()
    if max_extent < 1e-9:
        max_extent = 1.0
    scale = TARGET_RANGE / max_extent
    verts = verts * scale

    # Clip to int16 range (should be safe but guard corners)
    verts = np.clip(verts, -32767, 32767)
    return verts.astype(np.int16)


# --------------- Validate -----------------------------------

def validate(verts: np.ndarray, faces: np.ndarray):
    errors = 0

    if len(verts) > 65535:
        print(f"  ERROR: {len(verts)} vertices exceeds uint16 index limit (65535).")
        print("         Decimate further with --decimate 60000 or less.")
        errors += 1

    if len(faces) > 131072:
        print(f"  WARNING: {len(faces)} faces is very large for embedded use.")
        print("           Target <20000 for 30 FPS on ESP32-P4.")

    bad_idx = faces[(faces < 0) | (faces >= len(verts))]
    if len(bad_idx):
        print(f"  ERROR: {len(bad_idx)} out-of-range face indices found.")
        errors += 1

    if errors:
        sys.exit(f"Validation failed with {errors} error(s).")

    # Print bbox of quantised verts for reference
    mn = verts.min(axis=0)
    mx = verts.max(axis=0)
    print(f"  Bounding box (int16 units):"
          f" X[{mn[0]}, {mx[0]}]"
          f" Y[{mn[1]}, {mx[1]}]"
          f" Z[{mn[2]}, {mx[2]}]")


# --------------- Write .mesh --------------------------------

def write_mesh(path: str, verts: np.ndarray, faces: np.ndarray):
    """
    Write compact binary .mesh file.

    Header: two uint32 (vertex_count, face_count)
    Body:   int16 triplets (vertices), then uint16 triplets (faces)
    """
    vc = len(verts)
    fc = len(faces)

    with open(path, 'wb') as f:
        # Header
        f.write(struct.pack('<II', vc, fc))

        # Vertices — int16 little-endian
        # Ensure correct dtype and C-contiguous layout
        v16 = np.ascontiguousarray(verts, dtype='<i2')
        f.write(v16.tobytes())

        # Faces — uint16 little-endian
        f16 = np.ascontiguousarray(faces, dtype='<u2')
        f.write(f16.tobytes())

    file_size = pathlib.Path(path).stat().st_size
    print(f"  Written: {path}")
    print(f"  File size: {file_size:,} bytes  "
          f"({file_size/1024:.1f} KB)")
    print(f"  Vertex buffer: {vc * 6:,} bytes  "
          f"Face buffer: {fc * 6:,} bytes")


# --------------- Main ---------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description='Convert STL → embedded .mesh format')
    parser.add_argument('input',  help='Input .stl file')
    parser.add_argument('output', help='Output .mesh file')
    parser.add_argument('--decimate', type=int, default=0,
                        metavar='N',
                        help='Target face count for open3d decimation '
                             '(requires open3d; skipped if 0)')
    parser.add_argument('--info', action='store_true',
                        help='Print info only, do not write output')
    args = parser.parse_args()

    print(f"\n=== STL to .mesh converter ===")
    print(f"  Input:  {args.input}")
    print(f"  Output: {args.output}")

    # 1. Load
    print("\n[1/4] Loading STL...")
    verts, faces = load_stl_numpy(args.input)

    # 2. Optional decimation
    if args.decimate > 0:
        print(f"\n[2/4] Decimating to ≤{args.decimate} faces...")
        verts, faces = decimate_o3d(verts, faces, args.decimate)
    else:
        print(f"\n[2/4] Skipping decimation (use --decimate N to reduce)")

    # 3. Scale & quantise
    print("\n[3/4] Quantising to int16...")
    verts_i16 = quantise_vertices(verts)

    # 4. Validate
    print("\n[4/4] Validating...")
    validate(verts_i16, faces)

    if args.info:
        print("\n--info flag set — skipping write.")
        return

    # 5. Write
    print("\n[5/5] Writing .mesh...")
    write_mesh(args.output, verts_i16, faces)

    print("\nDone.  Copy the .mesh file to /models/ on the SD card.\n")


if __name__ == '__main__':
    main()
