#ifndef MATRIX_UTILS_H
#define MATRIX_UTILS_H

#include "globals.h" // Để có _float_t và EKF_M, EKF_N

// In ma trận (hữu ích cho debug)
void mat_print(const _float_t *A, int rows, int cols, const char *name);

// Nhân ma trận: C = A * B
// A: rowsA x colsA
// B: colsA x colsB (phải là colsA của A)
// C: rowsA x colsB
void mat_mul(const _float_t *A, const _float_t *B, _float_t *C, int rowsA, int colsA, int colsB);

// Nhân ma trận với vector cột: C_vec = A_mat * B_vec
// A_mat: rowsA x colsA
// B_vec: colsA x 1 (vector cột)
// C_vec: rowsA x 1 (vector cột)
void mat_mul_vec(const _float_t *A_mat, const _float_t *B_vec, _float_t *C_vec, int rowsA, int colsA);

// Cộng ma trận: C = A + B
void mat_add(const _float_t *A, const _float_t *B, _float_t *C, int rows, int cols);

// Trừ ma trận: C = A - B
void mat_sub(const _float_t *A, const _float_t *B, _float_t *C, int rows, int cols);

// Chuyển vị ma trận: C = A^T
void mat_transpose(const _float_t *A, _float_t *C, int rowsA, int colsA);

// Nghịch đảo ma trận vuông bằng phương pháp Gauss-Jordan
// Trả về true nếu thành công, false nếu ma trận không khả nghịch.
// A là ma trận đầu vào (sẽ bị thay đổi), A_inv là ma trận nghịch đảo đầu ra.
// n là kích thước của ma trận vuông (n x n).
bool mat_inv_gj(const _float_t *A, _float_t *A_inv, int n);

#endif // MATRIX_UTILS_H
