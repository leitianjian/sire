#pragma once

#include <cmath>
#include <cstddef>

#ifdef __CUDACC__
#define SIRE_PADE_HOST_DEVICE __host__ __device__
#else
#define SIRE_PADE_HOST_DEVICE
#endif

namespace sire::core::screw::detail {

enum class PadeStatus { Success, NonFiniteInput, NormOverflow, Singular, NonFiniteResult };

struct PadeBuffers {
  double* scaled;
  double* power;
  double* numerator;
  double* denominator;
  double* scratch;
};

SIRE_PADE_HOST_DEVICE inline auto finite(double value) -> bool {
#ifdef __CUDA_ARCH__
  return ::isfinite(value);
#else
  return std::isfinite(value);
#endif
}

SIRE_PADE_HOST_DEVICE inline auto swap(double& first, double& second) -> void {
  const double temporary = first;
  first = second;
  second = temporary;
}

SIRE_PADE_HOST_DEVICE inline auto swap(double*& first, double*& second) -> void {
  double* temporary = first;
  first = second;
  second = temporary;
}

template <class Multiply>
SIRE_PADE_HOST_DEVICE auto preparePade(std::size_t n, const double* matrix,
                                      double time, PadeBuffers& buffers,
                                      int& squarings, Multiply multiply)
    -> PadeStatus {
  double norm = 0.0;
  for (std::size_t row = 0; row < n; ++row) {
    double row_norm = 0.0;
    for (std::size_t col = 0; col < n; ++col) {
      const auto index = row * n + col;
      const double value = time * matrix[index];
      if (!finite(value)) return PadeStatus::NonFiniteInput;
      buffers.scaled[index] = value;
      row_norm += ::fabs(value);
    }
    if (row_norm > norm) norm = row_norm;
  }
  if (!finite(norm)) return PadeStatus::NormOverflow;

  // Preserve the conservative extra factor of two in Sire's scaling rule.
  squarings = norm > 0.0 ? 1 + static_cast<int>(::ceil(::log2(norm))) : 0;
  if (squarings < 0) squarings = 0;
  const auto size = n * n;
  for (std::size_t i = 0; i < size; ++i) {
    const double value = ::ldexp(buffers.scaled[i], -squarings);
    buffers.scaled[i] = value;
    buffers.power[i] = value;
    const double identity = i / n == i % n ? 1.0 : 0.0;
    buffers.numerator[i] = identity + 0.5 * value;
    buffers.denominator[i] = identity - 0.5 * value;
  }
  double coefficient = 0.5;
  for (int degree = 2; degree <= 6; ++degree) {
    coefficient *= static_cast<double>(7 - degree) / (degree * (13 - degree));
    multiply(n, buffers.scaled, buffers.power, buffers.scratch);
    swap(buffers.power, buffers.scratch);
    for (std::size_t i = 0; i < size; ++i) {
      const double term = coefficient * buffers.power[i];
      buffers.numerator[i] += term;
      buffers.denominator[i] += degree % 2 ? -term : term;
    }
  }
  return PadeStatus::Success;
}

// Gaussian elimination with partial pivoting and n simultaneous RHS.
SIRE_PADE_HOST_DEVICE inline auto solvePade(std::size_t n, PadeBuffers buffers)
    -> PadeStatus {
  auto* denominator = buffers.denominator;
  auto* numerator = buffers.numerator;
  for (std::size_t col = 0; col < n; ++col) {
    std::size_t pivot = col;
    for (std::size_t row = col + 1; row < n; ++row) {
      if (::fabs(denominator[row * n + col]) > ::fabs(denominator[pivot * n + col])) {
        pivot = row;
      }
    }
    if (denominator[pivot * n + col] == 0.0) return PadeStatus::Singular;
    if (pivot != col) {
      for (std::size_t j = 0; j < n; ++j) {
        swap(denominator[col * n + j], denominator[pivot * n + j]);
        swap(numerator[col * n + j], numerator[pivot * n + j]);
      }
    }
    for (std::size_t row = col + 1; row < n; ++row) {
      const double factor = denominator[row * n + col] / denominator[col * n + col];
      denominator[row * n + col] = 0.0;
      for (std::size_t j = col + 1; j < n; ++j) {
        denominator[row * n + j] -= factor * denominator[col * n + j];
      }
      for (std::size_t j = 0; j < n; ++j) {
        numerator[row * n + j] -= factor * numerator[col * n + j];
      }
    }
  }
  for (std::size_t row = n; row-- > 0;) {
    for (std::size_t k = row + 1; k < n; ++k) {
      for (std::size_t col = 0; col < n; ++col) {
        numerator[row * n + col] -= denominator[row * n + k] * numerator[k * n + col];
      }
    }
    for (std::size_t col = 0; col < n; ++col) {
      numerator[row * n + col] /= denominator[row * n + row];
    }
  }
  return PadeStatus::Success;
}

template <class Multiply>
SIRE_PADE_HOST_DEVICE auto squarePade(std::size_t n, PadeBuffers& buffers,
                                     int squarings, Multiply multiply)
    -> PadeStatus {
  for (int step = 0; step < squarings; ++step) {
    multiply(n, buffers.numerator, buffers.numerator, buffers.scratch);
    swap(buffers.numerator, buffers.scratch);
  }
  for (std::size_t i = 0; i < n * n; ++i) {
    if (!finite(buffers.numerator[i])) return PadeStatus::NonFiniteResult;
  }
  return PadeStatus::Success;
}

}  // namespace sire::core::screw::detail

#undef SIRE_PADE_HOST_DEVICE
