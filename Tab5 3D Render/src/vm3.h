#pragma once
#include <math.h>

// ============================================================
// vm3.h — 3x3 rotation matrix for touch "trackball" orbit.
//
// Shared by the 3D Viewer and the Room Scanner mesh view so both applets
// rotate identically (incremental screen-space rotation, no gimbal/Euler
// inversion). Convert to RenderState Euler angles for the renderer via the
// extraction in each applet's draw step:
//   rs.rot_x = asinf(-m[1][2]);
//   rs.rot_y = atan2f(m[0][2], m[2][2]);
//   rs.rot_z = atan2f(m[1][0], m[1][1]);
// ============================================================
struct VM3 {
    float m[3][3];
    static VM3 identity() {
        VM3 r;
        for (int i=0;i<3;i++) for (int j=0;j<3;j++) r.m[i][j]=(i==j)?1.f:0.f;
        return r;
    }
    static VM3 rot_y(float a) {
        VM3 r=identity();
        r.m[0][0]=cosf(a); r.m[0][2]=sinf(a);
        r.m[2][0]=-sinf(a); r.m[2][2]=cosf(a);
        return r;
    }
    static VM3 rot_x(float a) {
        VM3 r=identity();
        r.m[1][1]=cosf(a); r.m[1][2]=-sinf(a);
        r.m[2][1]=sinf(a); r.m[2][2]=cosf(a);
        return r;
    }
    VM3 operator*(const VM3& b) const {
        VM3 r;
        for (int i=0;i<3;i++) for (int j=0;j<3;j++) {
            r.m[i][j]=0;
            for (int k=0;k<3;k++) r.m[i][j]+=m[i][k]*b.m[k][j];
        }
        return r;
    }
    void transform(float ix, float iy, float iz,
                   float& ox, float& oy, float& oz) const {
        ox = m[0][0]*ix + m[0][1]*iy + m[0][2]*iz;
        oy = m[1][0]*ix + m[1][1]*iy + m[1][2]*iz;
        oz = m[2][0]*ix + m[2][1]*iy + m[2][2]*iz;
    }
};
