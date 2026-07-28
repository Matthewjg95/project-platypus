# MeshRenderer (existing renderer integration)

This folder is where your **existing 3D mesh renderer** plugs in.

## Contract

`MeshRenderer.h` declares the four entry points MeshScan calls:

```cpp
void renderer_load_mesh(const Mesh* mesh);
void renderer_set_camera(float azimuth, float elevation, float distance);
void renderer_draw_frame(uint16_t* framebuffer);
void renderer_add_billboard(const char* label, float x, float y, float z, uint32_t color);
```

`Mesh`, `Vertex` (int16), `Face` (uint16) and `Vec3f` come from
`shared/mesh_format.h`. `face_normals` is per-face and is precomputed by
`src/mesh_renderer.cpp` at load time.

## Integrating

1. Drop your renderer sources into this folder.
2. Define `MESHSCAN_REAL_RENDERER` in your renderer (e.g. a build flag) so
   `MeshRenderer_stub.cpp` compiles to nothing, then delete the stub.
3. If your function names/signatures differ, either add thin wrappers here or
   adjust `src/mesh_renderer.cpp`.

## Coordinate space to confirm

`mesh_renderer.cpp` centres the mesh on its bounds and scales metres into int16
vertex coords (`MESH_INT16_RANGE` in `shared/mesh_format.h`). Billboards are
passed in that **same scaled space**. If your renderer wants billboard
positions in raw metres, define `MESH_BILLBOARD_IN_METRES` in
`src/mesh_renderer.cpp`.
