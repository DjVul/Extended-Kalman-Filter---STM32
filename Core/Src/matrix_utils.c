#include "matrix_utils.h"
#include <string.h>
#include <math.h>

// C = A + B  (m x n)
void matrix_add(const float *A, const float *B, float *C, uint8_t m, uint8_t n)
{
    for (uint8_t i = 0; i < m; i++) {
        for (uint8_t j = 0; j < n; j++) {
            C[i * n + j] = A[i * n + j] + B[i * n + j];
        }
    }
}

// C = A - B  (m x n)
void matrix_sub(const float *A, const float *B, float *C, uint8_t m, uint8_t n)
{
    for (uint8_t i = 0; i < m; i++) {
        for (uint8_t j = 0; j < n; j++) {
            C[i * n + j] = A[i * n + j] - B[i * n + j];
        }
    }
}

// C = A * B  (m x n) * (n x p) = (m x p)
void matrix_mul(const float *A, const float *B, float *C, uint8_t m, uint8_t n, uint8_t p)
{
    for (uint8_t i = 0; i < m; i++) {
        for (uint8_t j = 0; j < p; j++) {
            C[i * p + j] = 0.0f;
            for (uint8_t k = 0; k < n; k++) {
                C[i * p + j] += A[i * n + k] * B[k * p + j];
            }
        }
    }
}

// C = A * B^T  (m x n) * (p x n)^T = (m x p)
void matrix_mul_transpose(const float *A, const float *B, float *C, uint8_t m, uint8_t n, uint8_t p)
{
    for (uint8_t i = 0; i < m; i++) {
        for (uint8_t j = 0; j < p; j++) {
            C[i * p + j] = 0.0f;
            for (uint8_t k = 0; k < n; k++) {
                C[i * p + j] += A[i * n + k] * B[j * n + k];  // B^T je B[j][k]
            }
        }
    }
}

// B = A^T  (m x n) -> (n x m)
void matrix_transpose(const float *A, float *B, uint8_t m, uint8_t n)
{
    for (uint8_t i = 0; i < m; i++) {
        for (uint8_t j = 0; j < n; j++) {
            B[j * m + i] = A[i * n + j];
        }
    }
}

// B = A * s  (m x n)
void matrix_scalar_mul(const float *A, float s, float *B, uint8_t m, uint8_t n)
{
    uint16_t total = (uint16_t)m * n;
    for (uint16_t i = 0; i < total; i++) {
        B[i] = A[i] * s;
    }
}

// Kopiranje matrice: B = A  (m x n)
void matrix_copy(const float *A, float *B, uint8_t m, uint8_t n)
{
    uint16_t total = (uint16_t)m * n;
    memcpy(B, A, total * sizeof(float));
}

// Inverzija 3x3: inv = A^-1, vraća 1 ako je uspešno, 0 ako je singularna
uint8_t matrix_inv_3x3(const float A[3][3], float inv[3][3])
{
    float det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
              - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
              + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);

    // Provera da li je matrica singularna
    if (fabsf(det) < 1e-10f) {
        return 0;
    }

    float inv_det = 1.0f / det;

    inv[0][0] = (A[1][1] * A[2][2] - A[1][2] * A[2][1]) * inv_det;
    inv[0][1] = (A[0][2] * A[2][1] - A[0][1] * A[2][2]) * inv_det;
    inv[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * inv_det;

    inv[1][0] = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) * inv_det;
    inv[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * inv_det;
    inv[1][2] = (A[0][2] * A[1][0] - A[0][0] * A[1][2]) * inv_det;

    inv[2][0] = (A[1][0] * A[2][1] - A[1][1] * A[2][0]) * inv_det;
    inv[2][1] = (A[0][1] * A[2][0] - A[0][0] * A[2][1]) * inv_det;
    inv[2][2] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) * inv_det;

    return 1;
}
