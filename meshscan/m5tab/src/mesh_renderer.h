// mesh_renderer.h - Bridge between the .mesh file format and the existing
// renderer (lib/MeshRenderer). Loads a .mesh from SD into the int16 render
// Mesh, precomputes per-face normals, registers object billboards, and drives
// the orbit camera from touch input.

#pragma once
#include <stdint.h>
#include "../../shared/mesh_format.h"

class MeshRenderer {
public:
    bool loadFromFile(const char* path);   // parse + upload to the renderer
    void unload();                         // free PSRAM, drop current mesh

    // Touch-driven orbit. Deltas are in radians / render units.
    void orbit(float d_azimuth, float d_elevation);
    void zoom(float d_distance);
    void setCamera(float azimuth, float elevation, float distance);

    // Render the current view into a 16-bit framebuffer.
    void draw(uint16_t* framebuffer);

    bool  hasMesh() const { return _mesh.vertices != nullptr; }
    const Mesh* mesh() const { return &_mesh; }

private:
    Mesh  _mesh{};                 // owns vertices/faces/face_normals in PSRAM
    float _az = 0.0f, _el = 0.4f, _dist = (float)MESH_INT16_RANGE * 1.6f;

    void computeFaceNormals();
};
