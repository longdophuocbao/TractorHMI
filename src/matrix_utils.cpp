#include "matrix_utils.h"
#include <cmath>     // Cho fabs
#include <cstring>   // Cho memcpy, memset
#include <Arduino.h> // Cho Serial.print (nếu dùng mat_print)

void mat_print(const _float_t *A, int rows, int cols, const char *name)
{
  Serial.printf("Matrix %s (%d x %d):\n", name, rows, cols);
  for (int i = 0; i < rows; ++i)
  {
    Serial.print("  [ ");
    for (int j = 0; j < cols; ++j)
    {
      Serial.printf("%8.4f ", A[i * cols + j]);
    }
    Serial.println("]");
  }
}

void mat_mul(const _float_t *A, const _float_t *B, _float_t *C, int rowsA, int colsA, int colsB)
{
  memset(C, 0, rowsA * colsB * sizeof(_float_t));
  for (int i = 0; i < rowsA; ++i)
  {
    for (int j = 0; j < colsB; ++j)
    {
      for (int k = 0; k < colsA; ++k)
      {
        C[i * colsB + j] += A[i * colsA + k] * B[k * colsB + j];
      }
    }
  }
}

void mat_mul_vec(const _float_t *A_mat, const _float_t *B_vec, _float_t *C_vec, int rowsA, int colsA)
{
  memset(C_vec, 0, rowsA * sizeof(_float_t));
  for (int i = 0; i < rowsA; ++i)
  {
    for (int k = 0; k < colsA; ++k)
    {
      C_vec[i] += A_mat[i * colsA + k] * B_vec[k];
    }
  }
}

void mat_add(const _float_t *A, const _float_t *B, _float_t *C, int rows, int cols)
{
  for (int i = 0; i < rows * cols; ++i)
  {
    C[i] = A[i] + B[i];
  }
}

void mat_sub(const _float_t *A, const _float_t *B, _float_t *C, int rows, int cols)
{
  for (int i = 0; i < rows * cols; ++i)
  {
    C[i] = A[i] - B[i];
  }
}

void mat_transpose(const _float_t *A, _float_t *C, int rowsA, int colsA)
{
  for (int i = 0; i < rowsA; ++i)
  {
    for (int j = 0; j < colsA; ++j)
    {
      C[j * rowsA + i] = A[i * colsA + j];
    }
  }
}

bool mat_inv_gj(const _float_t *A_in, _float_t *A_inv_out, int n)
{
  _float_t A[n * n];
  memcpy(A, A_in, n * n * sizeof(_float_t)); // Sao chép A_in để không làm thay đổi bản gốc

  // Khởi tạo A_inv_out là ma trận đơn vị
  memset(A_inv_out, 0, n * n * sizeof(_float_t));
  for (int i = 0; i < n; ++i)
  {
    A_inv_out[i * n + i] = 1.0;
  }

  for (int i = 0; i < n; ++i)
  {
    // Tìm pivot
    _float_t pivot = A[i * n + i];
    int pivot_row = i;
    for (int k = i + 1; k < n; ++k)
    {
      if (fabs(A[k * n + i]) > fabs(pivot))
      {
        pivot = A[k * n + i];
        pivot_row = k;
      }
    }

    if (fabs(pivot) < 1e-12)
    { // Ma trận suy biến, không thể nghịch đảo
      return false;
    }

    // Hoán vị hàng (nếu cần)
    if (pivot_row != i)
    {
      for (int k = 0; k < n; ++k)
      {
        std::swap(A[i * n + k], A[pivot_row * n + k]);
        std::swap(A_inv_out[i * n + k], A_inv_out[pivot_row * n + k]);
      }
    }

    // Chuẩn hóa hàng pivot (A[i][i] = 1)
    _float_t div = A[i * n + i];
    for (int k = 0; k < n; ++k)
    {
      A[i * n + k] /= div;
      A_inv_out[i * n + k] /= div;
    }

    // Khử các phần tử khác trong cột i
    for (int k = 0; k < n; ++k)
    {
      if (k != i)
      {
        _float_t factor = A[k * n + i];
        for (int j = 0; j < n; ++j)
        {
          A[k * n + j] -= factor * A[i * n + j];
          A_inv_out[k * n + j] -= factor * A_inv_out[i * n + j];
        }
      }
    }
  }
  return true;
}
