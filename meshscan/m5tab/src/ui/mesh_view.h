// mesh_view.h - 3D mesh view with touch orbit/zoom. Entry points in ui.h.
#pragma once
#include "ui.h"

// Provided by main.cpp: render the current mesh into the given RGB565 buffer
// using the MeshRenderer. Declared here so mesh_view can request redraws.
void app_render_mesh(uint16_t* framebuffer, int w, int h);
void app_mesh_orbit(float d_az, float d_el);
void app_mesh_zoom(float d_dist);
