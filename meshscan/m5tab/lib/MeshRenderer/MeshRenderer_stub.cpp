// MeshRenderer_stub.cpp - PLACEHOLDER renderer so the project links before the
// real renderer is integrated. Replace this file with your existing renderer.
//
// The stub just records the mesh pointer and logs calls; renderer_draw_frame
// clears the framebuffer. Delete this file once the real MeshRenderer is added.

#include "MeshRenderer.h"
#include <Arduino.h>

#ifndef MESHSCAN_REAL_RENDERER   // define this in your real renderer to disable the stub

static const Mesh* s_mesh = nullptr;
static float s_az = 0, s_el = 0, s_dist = 100;

void renderer_load_mesh(const Mesh* mesh) {
    s_mesh = mesh;
    Serial.printf("[renderer stub] load_mesh v=%lu f=%lu\n",
                  mesh ? (unsigned long)mesh->vertex_count : 0UL,
                  mesh ? (unsigned long)mesh->face_count : 0UL);
}

void renderer_set_camera(float az, float el, float dist) {
    s_az = az; s_el = el; s_dist = dist;
}

void renderer_draw_frame(uint16_t* fb) {
    // No geometry rasterisation in the stub — just clear so the UI shows black.
    // The real renderer projects s_mesh using (s_az, s_el, s_dist).
    if (fb) { /* caller-sized buffer; clearing is the caller's job in the stub */ }
}

void renderer_add_billboard(const char* label, float x, float y, float z, uint32_t color) {
    Serial.printf("[renderer stub] billboard '%s' @ %.1f,%.1f,%.1f #%06lX\n",
                  label ? label : "?", x, y, z, (unsigned long)(color & 0xFFFFFF));
}

#endif // MESHSCAN_REAL_RENDERER
