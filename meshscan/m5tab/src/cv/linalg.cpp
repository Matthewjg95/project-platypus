// linalg.cpp
#include "linalg.h"
#include <math.h>
#include <string.h>

namespace cv {

void matmul(const float* A, int ar, int ac, const float* B, int bc, float* C) {
    for (int i = 0; i < ar; ++i)
        for (int j = 0; j < bc; ++j) {
            float s = 0.0f;
            for (int k = 0; k < ac; ++k) s += A[i*ac + k] * B[k*bc + j];
            C[i*bc + j] = s;
        }
}

void transpose(const float* A, int ar, int ac, float* C) {
    for (int i = 0; i < ar; ++i)
        for (int j = 0; j < ac; ++j)
            C[j*ar + i] = A[i*ac + j];
}

void mat3_mul(const float A[9], const float B[9], float C[9]) {
    matmul(A, 3, 3, B, 3, C);
}
void mat3_mulvec(const float A[9], const float v[3], float o[3]) {
    o[0] = A[0]*v[0] + A[1]*v[1] + A[2]*v[2];
    o[1] = A[3]*v[0] + A[4]*v[1] + A[5]*v[2];
    o[2] = A[6]*v[0] + A[7]*v[1] + A[8]*v[2];
}
float mat3_det(const float A[9]) {
    return A[0]*(A[4]*A[8]-A[5]*A[7])
         - A[1]*(A[3]*A[8]-A[5]*A[6])
         + A[2]*(A[3]*A[7]-A[4]*A[6]);
}
bool mat3_inverse(const float A[9], float o[9]) {
    float d = mat3_det(A);
    if (fabsf(d) < 1e-12f) return false;
    float id = 1.0f / d;
    o[0] =  (A[4]*A[8]-A[5]*A[7])*id;
    o[1] = -(A[1]*A[8]-A[2]*A[7])*id;
    o[2] =  (A[1]*A[5]-A[2]*A[4])*id;
    o[3] = -(A[3]*A[8]-A[5]*A[6])*id;
    o[4] =  (A[0]*A[8]-A[2]*A[6])*id;
    o[5] = -(A[0]*A[5]-A[2]*A[3])*id;
    o[6] =  (A[3]*A[7]-A[4]*A[6])*id;
    o[7] = -(A[0]*A[7]-A[1]*A[6])*id;
    o[8] =  (A[0]*A[4]-A[1]*A[3])*id;
    return true;
}

bool jacobi_eigen_symmetric(const float* Ain, int n, float* eval, float* evec) {
    if (n > 9) return false;
    float a[81];
    memcpy(a, Ain, sizeof(float) * n * n);
    // V starts as identity
    for (int i = 0; i < n*n; ++i) evec[i] = 0.0f;
    for (int i = 0; i < n; ++i) evec[i*n + i] = 1.0f;

    for (int sweep = 0; sweep < 100; ++sweep) {
        // off-diagonal magnitude
        float off = 0.0f;
        for (int p = 0; p < n; ++p)
            for (int q = p+1; q < n; ++q) off += a[p*n+q]*a[p*n+q];
        if (off < 1e-20f) break;

        for (int p = 0; p < n; ++p) {
            for (int q = p+1; q < n; ++q) {
                float apq = a[p*n+q];
                if (fabsf(apq) < 1e-18f) continue;
                float app = a[p*n+p], aqq = a[q*n+q];
                float phi = 0.5f * atan2f(2.0f*apq, aqq - app);
                float c = cosf(phi), s = sinf(phi);
                // rotate rows/cols p,q
                for (int k = 0; k < n; ++k) {
                    float akp = a[k*n+p], akq = a[k*n+q];
                    a[k*n+p] = c*akp - s*akq;
                    a[k*n+q] = s*akp + c*akq;
                }
                for (int k = 0; k < n; ++k) {
                    float apk = a[p*n+k], aqk = a[q*n+k];
                    a[p*n+k] = c*apk - s*aqk;
                    a[q*n+k] = s*apk + c*aqk;
                }
                // accumulate rotation into eigenvectors
                for (int k = 0; k < n; ++k) {
                    float vkp = evec[k*n+p], vkq = evec[k*n+q];
                    evec[k*n+p] = c*vkp - s*vkq;
                    evec[k*n+q] = s*vkp + c*vkq;
                }
            }
        }
    }
    for (int i = 0; i < n; ++i) eval[i] = a[i*n+i];
    return true;
}

bool smallest_singular_vector(const float* A, int rows, int cols, float* x) {
    if (cols > 9) return false;
    // M = A^T A  (cols x cols, symmetric)
    float M[81];
    for (int i = 0; i < cols; ++i)
        for (int j = 0; j < cols; ++j) {
            float s = 0.0f;
            for (int r = 0; r < rows; ++r) s += A[r*cols + i] * A[r*cols + j];
            M[i*cols + j] = s;
        }
    float eval[9], evec[81];
    if (!jacobi_eigen_symmetric(M, cols, eval, evec)) return false;
    // smallest eigenvalue index
    int mi = 0;
    for (int i = 1; i < cols; ++i) if (eval[i] < eval[mi]) mi = i;
    for (int i = 0; i < cols; ++i) x[i] = evec[i*cols + mi];   // column mi
    return true;
}

bool svd3x3(const float A[9], float U[9], float S[3], float V[9]) {
    // V and S^2 from eigen of A^T A
    float AtA[9];
    {
        float At[9]; transpose(A, 3, 3, At);
        mat3_mul(At, A, AtA);
    }
    float eval[3], evec[9];
    if (!jacobi_eigen_symmetric(AtA, 3, eval, evec)) return false;

    // sort eigenpairs descending
    int idx[3] = {0,1,2};
    for (int i = 0; i < 3; ++i)
        for (int j = i+1; j < 3; ++j)
            if (eval[idx[j]] > eval[idx[i]]) { int t=idx[i]; idx[i]=idx[j]; idx[j]=t; }

    for (int c = 0; c < 3; ++c) {
        float w = eval[idx[c]];
        S[c] = w > 0.0f ? sqrtf(w) : 0.0f;
        V[0*3+c] = evec[0*3 + idx[c]];
        V[1*3+c] = evec[1*3 + idx[c]];
        V[2*3+c] = evec[2*3 + idx[c]];
    }

    // U columns = A * V_c / S_c
    for (int c = 0; c < 3; ++c) {
        float vc[3] = { V[0*3+c], V[1*3+c], V[2*3+c] };
        float u[3];  mat3_mulvec(A, vc, u);
        float s = S[c];
        if (s > 1e-8f) { u[0]/=s; u[1]/=s; u[2]/=s; }
        else { u[0]=u[1]=u[2]=0.0f; }
        U[0*3+c]=u[0]; U[1*3+c]=u[1]; U[2*3+c]=u[2];
    }
    // Repair any zero/degenerate U columns to keep U orthonormal (cross product).
    auto norm3 = [](float* v){ float n=sqrtf(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
                               if(n>1e-8f){v[0]/=n;v[1]/=n;v[2]/=n;} return n; };
    float u0[3]={U[0],U[3],U[6]}, u1[3]={U[1],U[4],U[7]}, u2[3]={U[2],U[5],U[8]};
    if (norm3(u2) < 1e-6f) {
        u2[0]=u0[1]*u1[2]-u0[2]*u1[1];
        u2[1]=u0[2]*u1[0]-u0[0]*u1[2];
        u2[2]=u0[0]*u1[1]-u0[1]*u1[0];
        norm3(u2);
    }
    U[0]=u0[0];U[3]=u0[1];U[6]=u0[2];
    U[1]=u1[0];U[4]=u1[1];U[7]=u1[2];
    U[2]=u2[0];U[5]=u2[1];U[8]=u2[2];
    return true;
}

} // namespace cv
