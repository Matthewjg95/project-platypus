// two_view_geometry.cpp
#include "two_view_geometry.h"
#include "linalg.h"
#include "triangulator.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

namespace cv {

// --- small deterministic RNG -------------------------------------------------
static uint32_t s_rng = 0x12345678u;
static inline uint32_t rng() { s_rng ^= s_rng<<13; s_rng ^= s_rng>>17; s_rng ^= s_rng<<5; return s_rng; }
static void pick_distinct(int k, int n, int* out) {
    for (int i = 0; i < k; ++i) {
        bool dup; int v;
        do { v = (int)(rng() % (uint32_t)n); dup = false;
             for (int j = 0; j < i; ++j) if (out[j] == v) { dup = true; break; }
        } while (dup);
        out[i] = v;
    }
}

// ===================== Homography (4-point DLT) =============================
static bool homography_from4(const float* p1, const float* p2, const int* idx, float H[9]) {
    float A[8*9];
    for (int i = 0; i < 4; ++i) {
        float x = p1[2*idx[i]], y = p1[2*idx[i]+1];
        float X = p2[2*idx[i]], Y = p2[2*idx[i]+1];
        float* r0 = &A[(2*i+0)*9];
        float* r1 = &A[(2*i+1)*9];
        r0[0]=-x; r0[1]=-y; r0[2]=-1; r0[3]=0; r0[4]=0; r0[5]=0; r0[6]=x*X; r0[7]=y*X; r0[8]=X;
        r1[0]=0; r1[1]=0; r1[2]=0; r1[3]=-x; r1[4]=-y; r1[5]=-1; r1[6]=x*Y; r1[7]=y*Y; r1[8]=Y;
    }
    return smallest_singular_vector(A, 8, 9, H);
}

static float homography_transfer_err2(const float H[9], float x, float y, float X, float Y) {
    float w = H[6]*x + H[7]*y + H[8];
    if (fabsf(w) < 1e-9f) return 1e30f;
    float px = (H[0]*x + H[1]*y + H[2]) / w;
    float py = (H[3]*x + H[4]*y + H[5]) / w;
    float dx = px - X, dy = py - Y;
    return dx*dx + dy*dy;
}

int ransac_homography(const float* p1, const float* p2, int n,
                      float Hbest[9], uint8_t* inlier, int iters, float thresh) {
    if (n < 4) return 0;
    float t2 = thresh*thresh;
    int best = 0; float Hb[9] = {1,0,0,0,1,0,0,0,1};
    for (int it = 0; it < iters; ++it) {
        int idx[4]; pick_distinct(4, n, idx);
        float H[9];
        if (!homography_from4(p1, p2, idx, H)) continue;
        int cnt = 0;
        for (int i = 0; i < n; ++i)
            if (homography_transfer_err2(H, p1[2*i],p1[2*i+1], p2[2*i],p2[2*i+1]) < t2) ++cnt;
        if (cnt > best) { best = cnt; memcpy(Hb, H, sizeof(Hb)); }
    }
    memcpy(Hbest, Hb, sizeof(Hb));
    if (inlier)
        for (int i = 0; i < n; ++i)
            inlier[i] = homography_transfer_err2(Hb, p1[2*i],p1[2*i+1], p2[2*i],p2[2*i+1]) < t2;
    return best;
}

// ===================== Essential (normalised 8-point) ======================
bool essential_8point(const float* xn1, const float* xn2, int n, float E[9]) {
    if (n < 8) return false;
    // Build n x 9 system. Heap for large n.
    float stackA[8*9];
    float* A = (n == 8) ? stackA : (float*)malloc(sizeof(float)*n*9);
    if (!A) return false;
    for (int i = 0; i < n; ++i) {
        float x1=xn1[2*i], y1=xn1[2*i+1], x2=xn2[2*i], y2=xn2[2*i+1];
        float* r = &A[i*9];
        r[0]=x2*x1; r[1]=x2*y1; r[2]=x2; r[3]=y2*x1; r[4]=y2*y1; r[5]=y2; r[6]=x1; r[7]=y1; r[8]=1;
    }
    float e[9]; bool ok = smallest_singular_vector(A, n, 9, e);
    if (A != stackA) free(A);
    if (!ok) return false;

    // enforce rank-2, equal singular values: E = U diag(1,1,0) V^T
    float U[9],S[3],V[9];
    if (!svd3x3(e, U, S, V)) return false;
    float D[9] = {1,0,0, 0,1,0, 0,0,0};
    float UD[9], Vt[9];
    mat3_mul(U, D, UD);
    transpose(V, 3, 3, Vt);
    mat3_mul(UD, Vt, E);
    return true;
}

static float sampson_err2(const float E[9], float x1,float y1,float x2,float y2) {
    float X1[3]={x1,y1,1}, X2[3]={x2,y2,1};
    float Ex1[3]; mat3_mulvec(E, X1, Ex1);
    float Et[9]; transpose(E,3,3,Et);
    float Etx2[3]; mat3_mulvec(Et, X2, Etx2);
    float x2Ex1 = X2[0]*Ex1[0]+X2[1]*Ex1[1]+X2[2]*Ex1[2];
    float denom = Ex1[0]*Ex1[0]+Ex1[1]*Ex1[1]+Etx2[0]*Etx2[0]+Etx2[1]*Etx2[1];
    if (denom < 1e-12f) return 1e30f;
    return (x2Ex1*x2Ex1)/denom;
}

int ransac_essential(const float* p1, const float* p2, int n,
                     const CameraIntrinsics& K, float Ebest[9], uint8_t* inlier,
                     int iters, float thresh_norm) {
    if (n < 8) return 0;
    // pre-normalise all points to camera coords
    float* xn1 = (float*)malloc(sizeof(float)*n*2);
    float* xn2 = (float*)malloc(sizeof(float)*n*2);
    if (!xn1 || !xn2) { free(xn1); free(xn2); return 0; }
    for (int i = 0; i < n; ++i) {
        K.normalize(p1[2*i], p1[2*i+1], xn1[2*i], xn1[2*i+1]);
        K.normalize(p2[2*i], p2[2*i+1], xn2[2*i], xn2[2*i+1]);
    }
    float t2 = thresh_norm*thresh_norm;
    int best = 0; float Eb[9] = {0};
    for (int it = 0; it < iters; ++it) {
        int idx[8]; pick_distinct(8, n, idx);
        float s1[16], s2[16];
        for (int i = 0; i < 8; ++i) {
            s1[2*i]=xn1[2*idx[i]]; s1[2*i+1]=xn1[2*idx[i]+1];
            s2[2*i]=xn2[2*idx[i]]; s2[2*i+1]=xn2[2*idx[i]+1];
        }
        float E[9];
        if (!essential_8point(s1, s2, 8, E)) continue;
        int cnt = 0;
        for (int i = 0; i < n; ++i)
            if (sampson_err2(E, xn1[2*i],xn1[2*i+1], xn2[2*i],xn2[2*i+1]) < t2) ++cnt;
        if (cnt > best) { best = cnt; memcpy(Eb, E, sizeof(Eb)); }
    }
    memcpy(Ebest, Eb, sizeof(Eb));
    if (inlier)
        for (int i = 0; i < n; ++i)
            inlier[i] = sampson_err2(Eb, xn1[2*i],xn1[2*i+1], xn2[2*i],xn2[2*i+1]) < t2;
    free(xn1); free(xn2);
    return best;
}

// ===================== Pose recovery (cheirality) ==========================
static void negate3(float m[9]) { for (int i=0;i<9;++i) m[i] = -m[i]; }

bool recover_pose(const float E[9], const float* p1, const float* p2,
                  const uint8_t* inlier, int n, const CameraIntrinsics& K,
                  float Rout[9], float tout[3]) {
    float U[9],S[3],V[9];
    if (!svd3x3(E, U, S, V)) return false;
    float W[9]  = {0,-1,0, 1,0,0, 0,0,1};
    float Wt[9] = {0,1,0, -1,0,0, 0,0,1};
    float Vt[9]; transpose(V,3,3,Vt);
    float UW[9], R1[9], UWt[9], R2[9];
    mat3_mul(U, W, UW);  mat3_mul(UW, Vt, R1);
    mat3_mul(U, Wt, UWt); mat3_mul(UWt, Vt, R2);
    if (mat3_det(R1) < 0) negate3(R1);
    if (mat3_det(R2) < 0) negate3(R2);
    float t[3] = { U[2], U[5], U[8] };   // third column of U

    const float* Rs[4] = { R1, R1, R2, R2 };
    float ts[4][3] = { { t[0],t[1],t[2]}, {-t[0],-t[1],-t[2]},
                       { t[0],t[1],t[2]}, {-t[0],-t[1],-t[2]} };

    float P1n[12] = {1,0,0,0, 0,1,0,0, 0,0,1,0};   // [I|0]
    int bestCount = -1, bestK = 0;
    for (int c = 0; c < 4; ++c) {
        const float* R = Rs[c]; const float* tc = ts[c];
        float P2n[12] = { R[0],R[1],R[2],tc[0], R[3],R[4],R[5],tc[1], R[6],R[7],R[8],tc[2] };
        int cnt = 0;
        for (int i = 0; i < n; ++i) {
            if (inlier && !inlier[i]) continue;
            float x1,y1,x2,y2;
            K.normalize(p1[2*i],p1[2*i+1], x1,y1);
            K.normalize(p2[2*i],p2[2*i+1], x2,y2);
            float X[3];
            if (!triangulate_dlt(P1n, P2n, x1,y1, x2,y2, X)) continue;
            if (X[2] <= 0) continue;                       // in front of cam1
            float Xc2[3]; mat3_mulvec(R, X, Xc2);
            Xc2[0]+=tc[0]; Xc2[1]+=tc[1]; Xc2[2]+=tc[2];
            if (Xc2[2] <= 0) continue;                     // in front of cam2
            ++cnt;
        }
        if (cnt > bestCount) { bestCount = cnt; bestK = c; }
    }
    memcpy(Rout, Rs[bestK], sizeof(float)*9);
    tout[0]=ts[bestK][0]; tout[1]=ts[bestK][1]; tout[2]=ts[bestK][2];
    return bestCount > 0;
}

// ===================== top-level ===========================================
PoseResult estimate_relative_pose(const float* p1, const float* p2, int n,
                                  const CameraIntrinsics& K,
                                  float pixel_thresh, int ransac_iters) {
    PoseResult out{}; out.valid = false;
    if (n < 8) return out;

    uint8_t* hin = (uint8_t*)malloc(n);
    uint8_t* ein = (uint8_t*)malloc(n);
    if (!hin || !ein) { free(hin); free(ein); return out; }

    float H[9];
    int hcount = ransac_homography(p1, p2, n, H, hin, ransac_iters, pixel_thresh);

    float E[9];
    float thresh_norm = pixel_thresh / K.fx;     // approx pixel->normalised
    int ecount = ransac_essential(p1, p2, n, K, E, ein, ransac_iters, thresh_norm);

    out.inliers = ecount;
    out.planarity = (ecount > 0) ? (float)hcount / (float)ecount : 1.0f;

    if (ecount >= 8 && recover_pose(E, p1, p2, ein, n, K, out.R, out.t))
        out.valid = true;

    free(hin); free(ein);
    return out;
}

} // namespace cv
