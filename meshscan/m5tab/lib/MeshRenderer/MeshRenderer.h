// MeshRenderer.h - Interface to the EXISTING 3D mesh renderer.
//
// This declares the renderer entry points the MeshScan firmware calls. The real
// implementation is your existing renderer; drop it into this folder (replace
// MeshRenderer_stub.cpp) keeping these signatures, or adapt the signatures and
// update src/mesh_renderer.cpp to match.
//
// Coordinate space note (please confirm): mesh_renderer.cpp scales model-space
// metres into int16 vertex coordinates centred on the mesh bounds. It passes
// billboard positions in that SAME scaled space. If your renderer expects
// billboard coords in raw metres instead, set MESH_BILLBOARD_IN_METRES in
// src/mesh_renderer.cpp.

#pragma once
#include "../../../shared/mesh_format.h"

// Upload a mesh for rendering. The renderer may keep the pointer; mesh_renderer
// owns the backing PSRAM and frees it only when loading a replacement.
void renderer_load_mesh(const Mesh* mesh);

// Orbit camera: azimuth/elevation in radians, distance in render units.
void renderer_set_camera(float azimuth, float elevation, float distance);

// Render the current mesh + camera into a 16-bit (RGB565) framebuffer.
void renderer_draw_frame(uint16_t* framebuffer);

// Add a floating text label at a world position with an 0x00RRGGBB colour.
void renderer_add_billboard(const char* label, float x, float y, float z, uint32_t color);
