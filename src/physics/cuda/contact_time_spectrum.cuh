#pragma once

#include <cfloat>
#include <cmath>
#include <cstddef>

#include "physics/contact/contact_time_search.hpp"

namespace sire::physics::contact::cuda::time_spectrum {

__device__ inline auto index(int stride, int row, int col) -> std::size_t {
  return static_cast<std::size_t>(row) * stride + col;
}

// Real Householder reflection. Normalize by the maximum entry before the
// norm so a subnormal or large column does not change the selected reflector.
__device__ inline auto makeReflector(double* vector, int count) -> double {
  double scale = 0.0;
  for (int i = 0; i < count; ++i) scale = ::fmax(scale, ::fabs(vector[i]));
  if (scale == 0.0) return 0.0;
  double norm_squared = 0.0;
  for (int i = 0; i < count; ++i) {
    vector[i] /= scale;
    norm_squared += vector[i] * vector[i];
  }
  const double norm = ::sqrt(norm_squared);
  vector[0] += ::copysign(norm, vector[0]);
  return 1.0 / (norm * ::fabs(vector[0]));
}

__device__ inline auto reflectRows(double* matrix, int stride, int row,
                                   int first_column, int last_column,
                                   const double* vector, int count, double beta)
    -> void {
  for (int col = first_column; col <= last_column; ++col) {
    double projection = 0.0;
    for (int i = 0; i < count; ++i) projection += vector[i] * matrix[index(stride, row + i, col)];
    projection *= beta;
    for (int i = 0; i < count; ++i) matrix[index(stride, row + i, col)] -= vector[i] * projection;
  }
}

__device__ inline auto reflectColumns(double* matrix, int stride, int column,
                                      int first_row, int last_row,
                                      const double* vector, int count, double beta)
    -> void {
  for (int row = first_row; row <= last_row; ++row) {
    double* values = matrix + index(stride, row, column);
    double projection = 0.0;
    for (int i = 0; i < count; ++i) projection += vector[i] * values[i];
    projection *= beta;
    for (int i = 0; i < count; ++i) values[i] -= vector[i] * projection;
  }
}

__device__ inline auto hessenberg(double* matrix, int n, double* vector) -> void {
  for (int col = 0; col < n - 2; ++col) {
    const int first = col + 1;
    const int count = n - first;
    for (int i = 0; i < count; ++i) vector[i] = matrix[index(n, first + i, col)];
    const double beta = makeReflector(vector, count);
    if (beta == 0.0) continue;
    reflectRows(matrix, n, first, col, n - 1, vector, count, beta);
    reflectColumns(matrix, n, first, 0, n - 1, vector, count, beta);
    for (int row = first + 1; row < n; ++row) matrix[index(n, row, col)] = 0.0;
  }
}

// Implicit real Francis QR step with the trailing two eigenvalues as shifts.
// Their sum and product are real even for a conjugate pair. Chase the bulge
// using small reflectors and preserve the upper Hessenberg structure.
__device__ inline auto francisStep(double* matrix, int n, int low, int high,
                                   double shift_sum, double shift_product) -> void {
  const double top = matrix[index(n, low, low)];
  const double lower = matrix[index(n, low + 1, low)];
  double vector[3]{(top - shift_sum) * top + shift_product + matrix[index(n, low, low + 1)] * lower,
                   lower * (top + matrix[index(n, low + 1, low + 1)] - shift_sum),
                   lower * matrix[index(n, low + 2, low + 1)]};
  for (int k = low; k < high; ++k) {
    const int count = high - k >= 2 ? 3 : 2;
    if (k != low) {
      for (int j = 0; j < count; ++j) vector[j] = matrix[index(n, k + j, k - 1)];
    }
    const double beta = makeReflector(vector, count);
    if (beta == 0.0) continue;
    reflectRows(matrix, n, k, k == low ? k : k - 1, high, vector, count, beta);
    const int last_row = k + count < high ? k + count : high;
    reflectColumns(matrix, n, k, low, last_row, vector, count, beta);
    if (k != low) {
      for (int j = 1; j < count; ++j) matrix[index(n, k + j, k - 1)] = 0.0;
    }
  }
}

using sire::physics::contact::detail::ContactTimeSpectrum;

__device__ inline auto tinySubdiagonal(const double* matrix, int n, int row)
    -> bool {
  const int col = row - 1;
  const double below = ::fabs(matrix[index(n, row, col)]);
  if (below <= DBL_MIN) return true;
  const double a = matrix[index(n, col, col)], d = matrix[index(n, row, row)];
  if (below > DBL_EPSILON * (::fabs(a) + ::fabs(d))) return false;

  // A tiny subdiagonal alone is insufficient when its paired superdiagonal
  // is large. This product check retains that coupling in the active block.
  const double above = ::fabs(matrix[index(n, col, row)]);
  const double ab = ::fmax(below, above), ba = ::fmin(below, above);
  const double aa = ::fmax(::fabs(d), ::fabs(a - d)), bb = ::fmin(::fabs(d), ::fabs(a - d));
  const double scale = aa + ab;
  return ba * (ab / scale) <= ::fmax(DBL_MIN, DBL_EPSILON * bb * (aa / scale));
}

__device__ inline auto normalSpectrum(const double* input, int leading, int n,
                                       double* workspace, ContactTimeSpectrum& result) -> bool {
  result = {0.0, 0.0};
  double scale = 0.0;
  for (int row = 0; row < n; ++row) {
    for (int col = 0; col < n; ++col) scale = ::fmax(scale, ::fabs(input[index(leading, row, col)]));
  }
  if (scale == 0.0) return true;
  for (int row = 0; row < n; ++row) {
    for (int col = 0; col < n; ++col) workspace[index(n, row, col)] = input[index(leading, row, col)] / scale;
  }
  hessenberg(workspace, n, workspace + static_cast<std::size_t>(n) * n);

  int high = n - 1;
  int iterations = 0;
  while (high >= 0) {
    int low = high;
    while (low > 0 && !tinySubdiagonal(workspace, n, low)) --low;
    if (low > 0) workspace[index(n, low, low - 1)] = 0.0;
    const double d = workspace[index(n, high, high)];
    if (low == high) {
      result.rate = ::fmax(result.rate, ::fabs(d));
      --high;
      iterations = 0;
      continue;
    }
    const double a = workspace[index(n, high - 1, high - 1)];
    const double b = workspace[index(n, high - 1, high)];
    const double c = workspace[index(n, high, high - 1)];
    if (low == high - 1) {
      const double half_difference = (a - d) / 2.0;
      const double discriminant = half_difference * half_difference + b * c;
      const double center = (a + d) / 2.0;
      if (discriminant < 0.0) {
        const double imaginary = ::sqrt(-discriminant);
        result.frequency = ::fmax(result.frequency, imaginary);
        result.rate = ::fmax(result.rate, ::hypot(center, imaginary));
      } else {
        result.rate = ::fmax(result.rate, ::fabs(center) + ::sqrt(discriminant));
      }
      high -= 2;
      iterations = 0;
      continue;
    }
    if (++iterations > 120) return false;
    double shift_sum = a + d;
    double shift_product = a * d - b * c;
    if (iterations % 16 == 0) {
      const double displacement = ::fabs(c) + ::fabs(workspace[index(n, high - 1, high - 2)]);
      const double shift = d + ::copysign(0.75 * displacement, d);
      shift_sum = 2.0 * shift;
      shift_product = shift * shift;
    }
    francisStep(workspace, n, low, high, shift_sum, shift_product);
  }
  result.rate *= scale;
  result.frequency *= scale;
  return ::isfinite(result.rate) && ::isfinite(result.frequency);
}

}  // namespace sire::physics::contact::cuda::time_spectrum
