/* ============================================================================
 * mesh_format.h  —  MeshScan ".mesh" binary format specification + in-memory
 *                   structures shared by the M5Tab firmware and desktop tools.
 *
 * This header is the single source of truth for the on-disk layout. The C++
 * firmware (mesh_writer / mesh_renderer) and the Python tools (mesh_to_obj.py)
 * must agree byte-for-byte with what is written here.
 *
 * Byte order: LITTLE-ENDIAN for all multi-byte integers and IEEE-754 floats.
 *             (Both the K210 and the ESP32-S3 are little-endian, so no swap.)
 *
 * ---------------------------------------------------------------------------
 * ON-DISK LAYOUT (sections appear in this exact order)
 * ---------------------------------------------------------------------------
 *
 *  HEADER
 *    char     magic[4]      "MESH"
 *    uint8    version       == MESH_VERSION
 *    uint8    flags         reserved (0)
 *    uint16   _pad          reserved (0), keeps following fields 4-aligned
 *    uint32   room_id
 *    uint32   timestamp     unix seconds (UTC)
 *    float32  bounds[6]     xmin,xmax, ymin,ymax, zmin,zmax  (model space, metres)
 *
 *  VERTICES
 *    uint32   vertex_count
 *    float32  xyz[vertex_count*3]    interleaved x,y,z   (metres, model space)
 *
 *  FACES
 *    uint32   face_count
 *    uint16   idx[face_count*3]      interleaved i0,i1,i2 (CCW winding, outward)
 *
 *  NORMALS  (per-vertex, optional — count may be 0)
 *    uint32   normal_count           == vertex_count, or 0 if absent
 *    int8     nxyz[normal_count*3]   each component normalised to [-127,127]
 *
 *  OBJECTS  (detected items rendered as labels / markers)
 *    uint8    object_count
 *    repeated object_count times:
 *      uint8   label_len
 *      char    label[label_len]      (not NUL-terminated on disk)
 *      float32 bbox3d[6]             xmin,xmax, ymin,ymax, zmin,zmax (metres)
 *      uint8   color_rgb[3]
 *
 *  EOF
 *    char     marker[4]      "MEND"
 *
 * ---------------------------------------------------------------------------
 * Versioning: bump MESH_VERSION on any layout change. Readers must reject a
 * file whose version they do not understand rather than mis-parse it.
 * ==========================================================================*/
#ifndef MESHSCAN_MESH_FORMAT_H
#define MESHSCAN_MESH_FORMAT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ----- on-disk constants ------------------------------------------------- */
#define MESH_MAGIC0 'M'
#define MESH_MAGIC1 'E'
#define MESH_MAGIC2 'S'
#define MESH_MAGIC3 'H'
#define MESH_VERSION 1u

#define MESH_EOF0 'M'
#define MESH_EOF1 'E'
#define MESH_EOF2 'N'
#define MESH_EOF3 'D'

#define MESH_MAX_OBJECTS 255
#define MESH_MAX_LABEL   31   /* max chars we keep in RAM (on disk len is uint8) */

/* ----- header as written to disk (read field-by-field, do NOT fread the
 *       whole struct: alignment/padding is compiler-defined). Provided for
 *       documentation and size reference only. -------------------------- */
typedef struct {
    char     magic[4];     /* "MESH" */
    uint8_t  version;      /* MESH_VERSION */
    uint8_t  flags;        /* reserved */
    uint16_t _pad;         /* reserved */
    uint32_t room_id;
    uint32_t timestamp;    /* unix seconds */
    float    bounds[6];    /* xmin,xmax,ymin,ymax,zmin,zmax */
} MeshHeader;              /* 40 bytes on disk: 16 fixed + 24 bounds */

/* ==========================================================================
 * IN-MEMORY render structures.
 *
 * IMPORTANT: these must match the existing renderer's expectations exactly.
 * Vertices are int16 (model floats are scaled into int16 range on load — see
 * mesh_renderer.cpp). Normals are PER-FACE Vec3f, precomputed at load time
 * (the on-disk per-vertex int8 normals are not used directly by the renderer).
 * ==========================================================================*/
typedef struct { float   x, y, z; } Vec3f;
typedef struct { int16_t x, y, z; } Vertex;   /* 6 bytes */
typedef struct { uint16_t a, b, c; } Face;    /* 6 bytes — indices into vertices */

typedef struct {
    uint32_t vertex_count;
    uint32_t face_count;
    Vertex*  vertices;        /* PSRAM: vertex_count * sizeof(Vertex)  */
    Face*    faces;           /* PSRAM: face_count   * sizeof(Face)    */
    Vec3f*   face_normals;    /* PSRAM: face_count   * sizeof(Vec3f)   */
} Mesh;

/* Detected object as kept in RAM (NUL-terminated label for convenience). */
typedef struct {
    char    label[MESH_MAX_LABEL + 1];
    float   bbox[6];          /* xmin,xmax,ymin,ymax,zmin,zmax, metres */
    uint8_t r, g, b;
} MeshObject;

/* Scale used to pack model-space metres into int16 vertex coordinates.
 * mesh_renderer centres the mesh on its bounds midpoint and multiplies by a
 * uniform factor so the largest extent maps to ~MESH_INT16_RANGE. */
#define MESH_INT16_RANGE 30000   /* leave headroom below INT16_MAX (32767) */

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* MESHSCAN_MESH_FORMAT_H */
