#!/usr/bin/env python3
"""mesh_to_obj.py - Convert a MeshScan .mesh binary into a Wavefront .obj.

Mirrors shared/mesh_format.h. Little-endian throughout.

Usage:
    python mesh_to_obj.py room.mesh [room.obj]

The .obj gets vertices, per-vertex normals (if present), and faces. Detected
objects are emitted as comments (# object <label> bbox ... color ...) since
.obj has no native concept for them.
"""
import struct
import sys


def _read(f, fmt):
    size = struct.calcsize(fmt)
    data = f.read(size)
    if len(data) != size:
        raise EOFError("unexpected end of file")
    return struct.unpack(fmt, data)


def load_mesh(path):
    with open(path, "rb") as f:
        magic = f.read(4)
        if magic != b"MESH":
            raise ValueError("not a .mesh file (bad magic %r)" % magic)
        version, flags, _pad = _read(f, "<BBH")
        room_id, timestamp = _read(f, "<II")
        bounds = _read(f, "<6f")

        (vcount,) = _read(f, "<I")
        verts = list(_read(f, "<%df" % (vcount * 3)))

        (fcount,) = _read(f, "<I")
        faces = list(_read(f, "<%dH" % (fcount * 3)))

        (ncount,) = _read(f, "<I")
        normals = list(_read(f, "<%db" % (ncount * 3))) if ncount else []

        (ocount,) = _read(f, "<B")
        objects = []
        for _ in range(ocount):
            (label_len,) = _read(f, "<B")
            label = f.read(label_len).decode("utf-8", "replace")
            bbox = _read(f, "<6f")
            color = _read(f, "<3B")
            objects.append((label, bbox, color))

        eof = f.read(4)
        if eof != b"MEND":
            sys.stderr.write("warning: missing MEND marker (got %r)\n" % eof)

    return {
        "version": version, "room_id": room_id, "timestamp": timestamp,
        "bounds": bounds, "vertices": verts, "faces": faces,
        "normals": normals, "objects": objects,
    }


def write_obj(mesh, path):
    v, fa, no = mesh["vertices"], mesh["faces"], mesh["normals"]
    with open(path, "w") as o:
        o.write("# MeshScan export  room_id=%d  ts=%d\n" % (mesh["room_id"], mesh["timestamp"]))
        o.write("# bounds %s\n" % (",".join("%.4f" % b for b in mesh["bounds"])))
        for lbl, bb, col in mesh["objects"]:
            o.write("# object %-16s bbox=%s rgb=%d,%d,%d\n"
                    % (lbl, ",".join("%.3f" % c for c in bb), col[0], col[1], col[2]))
        for i in range(0, len(v), 3):
            o.write("v %.6f %.6f %.6f\n" % (v[i], v[i+1], v[i+2]))
        have_n = bool(no)
        for i in range(0, len(no), 3):
            # stored as int8 [-127,127]; normalise back to unit-ish floats
            o.write("vn %.4f %.4f %.4f\n" % (no[i]/127.0, no[i+1]/127.0, no[i+2]/127.0))
        for i in range(0, len(fa), 3):
            a, b, c = fa[i] + 1, fa[i+1] + 1, fa[i+2] + 1   # .obj is 1-indexed
            if have_n:
                o.write("f %d//%d %d//%d %d//%d\n" % (a, a, b, b, c, c))
            else:
                o.write("f %d %d %d\n" % (a, b, c))


def main(argv):
    if len(argv) < 2:
        sys.stderr.write(__doc__)
        return 2
    src = argv[1]
    dst = argv[2] if len(argv) > 2 else src.rsplit(".", 1)[0] + ".obj"
    mesh = load_mesh(src)
    write_obj(mesh, dst)
    print("wrote %s  (%d verts, %d faces, %d objects)"
          % (dst, len(mesh["vertices"]) // 3, len(mesh["faces"]) // 3, len(mesh["objects"])))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
