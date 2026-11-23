#pragma once

#include <cfloat>
#include <cmath>
#include <cstddef>

#include "physics/contact/contact_time_search.hpp"

namespace sire::physics::contact::cuda::uniform_normal {

constexpr std::size_t capacity = 8;

struct Transition {
  double velocity_integral;
  double acceleration_integral;
};

// For x'' + damping*x' + stiffness*x = f, the two integrals of the
// homogeneous velocity response determine the entire affine transition.
__device__ inline auto transition(double stiffness, double damping, double time) -> Transition {
  const double frequency = ::sqrt(stiffness);
  const double scaled_frequency = frequency * time;
  const double scaled_damping = damping * time;
  if (::fmax(scaled_frequency, scaled_damping) < 0.125) {
    // At short times, evaluate the two integrals directly. Subtracting the
    // position transition from one loses the small forced displacement.
    const double scaled_stiffness = scaled_frequency * scaled_frequency;
    double previous = 0.0, current = 1.0, scale = 1.0;
    double velocity_integral = 1.0, acceleration_integral = 0.5;
    for (int order = 1; order <= 11; ++order) {
      const double next = -scaled_damping * current - scaled_stiffness * previous;
      scale /= order + 1;
      const double term = next * scale;
      velocity_integral += term;
      acceleration_integral += term / (order + 2);
      previous = current;
      current = next;
    }
    return {time * velocity_integral, time * time * acceleration_integral};
  }
  if (stiffness == 0.0) return {time, 0.5 * time * time};

  const double alpha = 0.5 * damping;
  const double discriminant = (alpha - frequency) * (alpha + frequency);
  double velocity_integral, displacement_complement;
  if (discriminant > 0.0) {
    const double root = ::sqrt(discriminant);
    const double slower = -stiffness / (alpha + root);
    const double width = 2.0 * root;
    velocity_integral = ::exp(slower * time) * (-::expm1(-width * time)) / width;
    displacement_complement = -::expm1(slower * time) + slower * velocity_integral;
  } else {
    const double root = ::sqrt(-discriminant);
    const double angle = root * time;
    const double half_sine = ::sin(0.5 * angle);
    const double sine_integral = root > 0.0 ? ::sin(angle) / root : time;
    velocity_integral = ::exp(-alpha * time) * sine_integral;
    displacement_complement = -::expm1(-alpha * time) * ::cos(angle) +
                              2.0 * half_sine * half_sine - alpha * velocity_integral;
  }
  return {velocity_integral, displacement_complement / stiffness};
}

// A symmetric normal stiffness matrix is diagonalized by orthogonal Jacobi
// rotations. This is used only when the damping matrix has the same modes.
__device__ inline auto diagonalize(std::size_t count, double* matrix, double* basis,
                                   double tolerance) -> bool {
  for (int sweep = 0; sweep < 24; ++sweep) {
    double largest_offdiag = 0.0;
    for (std::size_t i = 0; i < count; ++i) {
      for (std::size_t j = i + 1; j < count; ++j) {
        const double offdiag = matrix[i * count + j];
        largest_offdiag = ::fmax(largest_offdiag, ::fabs(offdiag));
        if (::fabs(offdiag) <= tolerance) continue;
        const double diagonal_i = matrix[i * count + i];
        const double diagonal_j = matrix[j * count + j];
        const double tau = (diagonal_j - diagonal_i) / (2.0 * offdiag);
        const double tangent = ::copysign(1.0, tau) / (::fabs(tau) + ::hypot(1.0, tau));
        const double cosine = 1.0 / ::sqrt(1.0 + tangent * tangent);
        const double sine = tangent * cosine;
        matrix[i * count + i] = diagonal_i - tangent * offdiag;
        matrix[j * count + j] = diagonal_j + tangent * offdiag;
        matrix[i * count + j] = matrix[j * count + i] = 0.0;
        for (std::size_t k = 0; k < count; ++k) {
          const double bi = basis[k * count + i], bj = basis[k * count + j];
          basis[k * count + i] = cosine * bi - sine * bj;
          basis[k * count + j] = sine * bi + cosine * bj;
          if (k == i || k == j) continue;
          const double mi = matrix[k * count + i], mj = matrix[k * count + j];
          matrix[k * count + i] = matrix[i * count + k] = cosine * mi - sine * mj;
          matrix[k * count + j] = matrix[j * count + k] = sine * mi + cosine * mj;
        }
      }
    }
    if (largest_offdiag <= tolerance) return true;
  }
  return false;
}

struct Flow {
  std::size_t n;
  double eigenvectors[capacity * capacity];
  double stiffness[capacity], damping[capacity], bias[capacity];
  Transition cached[capacity];
  bool has_cache{false};

  __device__ auto advance(double time, const double* from, double* to, bool fixed) -> bool {
    for (std::size_t i = 0; i < n; ++i) {
      const auto response = fixed && has_cache ? cached[i] : transition(stiffness[i], damping[i], time);
      if (fixed && !has_cache) cached[i] = response;
      const double restoring = bias[i] - stiffness[i] * from[i];
      const double damping_step = 1.0 - stiffness[i] * response.acceleration_integral -
                                  damping[i] * response.velocity_integral;
      to[i] = from[i] + response.velocity_integral * from[n + i] +
              response.acceleration_integral * restoring;
      to[n + i] = response.velocity_integral * restoring + damping_step * from[n + i];
      if (!::isfinite(to[i]) || !::isfinite(to[n + i])) return false;
    }
    has_cache = has_cache || fixed;
    return true;
  }

  __device__ auto depth(const double* state) const -> double {
    double minimum = DBL_MAX;
    for (std::size_t i = 0; i < n; ++i) {
      double value = 0.0, magnitude = 0.0;
      for (std::size_t j = 0; j < n; ++j) {
        const double term = eigenvectors[i * n + j] * state[j];
        value += term;
        magnitude += ::fabs(term);
      }
      if (!::isfinite(value)) return ::nan("");
      // When cancellation obscures the sign, let the original affine
      // trajectory decide whether a separation occurred.
      if (magnitude > 0.0 && ::fabs(value) <= 128 * DBL_EPSILON * magnitude) return ::nan("");
      minimum = ::fmin(minimum, value);
    }
    return minimum;
  }
};

__device__ inline auto initialize(
    Flow& flow, std::size_t stride, const double* matrix, const double* bias,
    const double* initial, double velocity_scale, double damping_ratio,
    double* transformed, detail::ContactTimeSpectrum& spectrum) -> bool {
  const auto n = flow.n;
  double stiffness[capacity * capacity];
  double magnitude = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    if (bias[i] != 0.0 || !::isfinite(initial[i]) || !::isfinite(initial[n + i])) return false;
    for (std::size_t j = 0; j < 2 * n; ++j) {
      if (matrix[i * stride + j] != (j == n + i ? velocity_scale : 0.0)) return false;
    }
    for (std::size_t j = 0; j < n; ++j) {
      const double entry = -velocity_scale * matrix[(n + i) * stride + j];
      if (!::isfinite(entry)) return false;
      stiffness[i * n + j] = entry;
      magnitude = ::fmax(magnitude, ::fabs(entry));
      flow.eigenvectors[i * n + j] = i == j ? 1.0 : 0.0;
    }
  }
  const double comparison_tolerance = 128 * DBL_EPSILON * magnitude;
  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t j = 0; j < n; ++j) {
      const double entry = stiffness[i * n + j];
      const double damping = -matrix[(n + i) * stride + n + j];
      const double expected = damping_ratio * entry;
      const double tolerance = 128 * DBL_EPSILON * ::fmax(::fabs(expected), damping_ratio * magnitude);
      if (!::isfinite(damping) || !::isfinite(expected) || ::fabs(damping - expected) > tolerance ||
          ::fabs(entry - stiffness[j * n + i]) > comparison_tolerance) return false;
    }
  }
  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t j = i + 1; j < n; ++j) {
      const double value = 0.5 * stiffness[i * n + j] + 0.5 * stiffness[j * n + i];
      stiffness[i * n + j] = stiffness[j * n + i] = value;
    }
  }
  if (!diagonalize(n, stiffness, flow.eigenvectors, 8 * DBL_EPSILON * magnitude)) return false;
  for (std::size_t i = 0; i < n; ++i) {
    const double diagonal = stiffness[i * n + i];
    if (diagonal < -comparison_tolerance) return false;
    flow.stiffness[i] = ::fmax(0.0, diagonal);
    flow.damping[i] = damping_ratio * flow.stiffness[i];
    const double alpha = 0.5 * flow.damping[i];
    const double omega = ::sqrt(flow.stiffness[i]);
    const double discriminant = (alpha - omega) * (alpha + omega);
    if (!::isfinite(discriminant)) return false;
    spectrum.rate = ::fmax(spectrum.rate, alpha + ::sqrt(::fmax(0.0, discriminant)));
    spectrum.frequency = ::fmax(spectrum.frequency, ::sqrt(::fmax(0.0, -discriminant)));
    double depth = 0.0, derivative = 0.0, forcing = 0.0;
    for (std::size_t j = 0; j < n; ++j) {
      const double projection = flow.eigenvectors[j * n + i];
      depth += projection * initial[j];
      derivative += projection * initial[n + j];
      forcing += projection * bias[n + j];
    }
    transformed[i] = depth;
    transformed[n + i] = velocity_scale * derivative;
    flow.bias[i] = velocity_scale * forcing;
  }
  return true;
}

}  // namespace sire::physics::contact::cuda::uniform_normal
