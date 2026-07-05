#ifndef INC_MATRIX_UTILS_H_
#define INC_MATRIX_UTILS_H_

#include <stdint.h>

/**
 * Matricno sabiranje
 *
 * C (m x n) = A (m x n) + B (m x n)
 *
 *  A   Pokazivac na prvu matricu (m x n)
 *  B   Pokazivac na drugu matricu (m x n)
 *  C   Pokazivac na matricu rezultata (m x n)
 *  m   Broj vrsta (rows)
 *  n   Broj kolona (columns)
 */
void matrix_add(const float *A, const float *B, float *C, uint8_t m, uint8_t n);

/**
 * Matricno oduzimanje
 *
 * C (m x n) = A (m x n) - B (m x n)
 *
 *  A   Pokazivac na prvu matricu (m x n)
 *  B   Pokazivac na drugu matricu (m x n)
 *  C   Pokazivac na matricu rezultata (m x n)
 *  m   Broj vrsta (rows)
 *  n   Broj kolona (columns)
 */
void matrix_sub(const float *A, const float *B, float *C, uint8_t m, uint8_t n);

/**
 * Matricno mnozenje
 *
 * C (m x p) = A (m x n) * B (n x p)
 *
 *  A   Pokazivac na prvu matricu (m x n)
 *  B   Pokazivac na drugu matricu (n x p)
 *  C   Pokazivac na matricu rezultata (m x p)
 *  m   Broj vrsta matrice A
 *  n   Broj kolona matrice A / broj vrsta matrice B
 *  p   Broj kolona matrice B
 */
void matrix_mul(const float *A, const float *B, float *C,
                uint8_t m, uint8_t n, uint8_t p);

/**
 * Mnozenje matrice sa transponovanom matricom
 *
 * C (m x p) = A (m x n) * B (p x n)^T
 *
 *  A   Pokazivac na prvu matricu (m x n)
 *  B   Pokazivac na drugu matricu (p x n) - bice transponovana
 *  C   Pokazivac na matricu rezultata (m x p)
 *  m   Broj vrsta matrice A
 *  n   Broj kolona matrice A i matrice B
 *  p   Broj vrsta matrice B (postaje broj kolona u rezultatu)
 */
void matrix_mul_transpose(const float *A, const float *B, float *C,
                          uint8_t m, uint8_t n, uint8_t p);

/**
 * Transponovanje matrice
 *
 * B (n x m) = A^T (m x n)^T
 *
 *  A   Pokazivac na ulaznu matricu (m x n)
 *  B   Pokazivac na izlaznu matricu (n x m)
 *  m   Broj vrsta ulazne matrice
 *  n   Broj kolona ulazne matrice
 */
void matrix_transpose(const float *A, float *B, uint8_t m, uint8_t n);

/**
 * Mnozenje matrice skalarom
 *
 * B (m x n) = A (m x n) * s
 *
 *  A   Pokazivac na ulaznu matricu (m x n)
 *  s   Skalarna vrednost (float)
 *  B   Pokazivac na izlaznu matricu (m x n)
 *  m   Broj vrsta
 *  n   Broj kolona
 */
void matrix_scalar_mul(const float *A, float s, float *B, uint8_t m, uint8_t n);

/**
 * Kopiranje matrice
 *
 * B (m x n) = A (m x n)
 *
 *  A   Pokazivac na izvornu matricu (m x n)
 *  B   Pokazivac na odredisnu matricu (m x n)
 *  m   Broj vrsta
 *  n   Broj kolona
 */
void matrix_copy(const float *A, float *B, uint8_t m, uint8_t n);

/**
 * Inverzija 3x3 matrice
 *
 * inv (3x3) = A^-1 (3x3)
 *
 *  A   Ulazna matrica (3x3)
 *  inv Izlazna matrica (3x3) - inverzna matrica
 * uint8_t 1 ako je inverzija uspesna, 0 ako je matrica singularna
 */
uint8_t matrix_inv_3x3(const float A[3][3], float inv[3][3]);


#endif /* INC_MATRIX_UTILS_H_ */
