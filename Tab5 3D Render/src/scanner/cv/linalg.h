// linalg.h - Tiny dense linear algebra for the Phase-2 SfM pipeline.
//
// Portable C++ (no Arduino / ESP-IDF deps) so it can be unit-tested on the
// desktop. Everything is float, row-major, small fixed sizes. Not optimised for
// large matrices — sizes here are <= 9x9.

#pragma once
#include <stdint.h>

namespace cv {

// C[ar x bc] = A[ar x ac] * B[ac x bc]
void matmul(const float* A, int ar, int ac, const float* B, int bc, float* C);

// C[ac x ar] = A[ar x ac]^T
void transpose(const float* A, int ar, int ac, float* C);

// 3x3 helpers
void mat3_mul(const float A[9], const float B[9], float C[9]);
void mat3_mulvec(const float A[9], const float v[3], float out[3]);
float mat3_det(const float A[9]);
bool  mat3_inverse(const float A[9], float out[9]);

// Cyclic Jacobi eigen-decomposition of a SYMMETRIC n x n matrix (n <= 9).
//   Ain  : n*n symmetric input (only used as input; not modified)
//   eval : n eigenvalues (unsorted)
//   evec : n*n, columns are the corresponding eigenvectors (orthonormal)
// Returns false if it fails to converge.
bool jacobi_eigen_symmetric(const float* Ain, int n, float* eval, float* evec);

// Eigenvector of A^T A with the SMALLEST eigenvalue — i.e. the least-squares
// null space of A (the homogeneous solution to A x = 0). A is rows x cols,
// cols <= 9. Writes `cols` values into x.
bool smallest_singular_vector(const float* A, int rows, int cols, float* x);

// SVD of a 3x3 matrix: A = U * diag(S) * V^T, with S sorted descending and
// U, V orthonormal. Returns false on numerical failure.
bool svd3x3(const float A[9], float U[9], float S[3], float V[9]);

} // namespace cv
