#pragma once

#include <cfloat>
#include <cmath>

#ifdef __CUDACC__
#define SIRE_ADMM_HD __host__ __device__
#else
#define SIRE_ADMM_HD
#endif

namespace sire::physics::contact::detail {

struct ContactVector {
  double x, y, normal;
};

SIRE_ADMM_HD inline auto tangentNorm(ContactVector value) -> double {
  return ::sqrt(value.x * value.x + value.y * value.y);
}

SIRE_ADMM_HD inline auto projectCoulombCone(ContactVector value, double mu)
    -> ContactVector {
  const double radius = tangentNorm(value);
  if (mu <= DBL_EPSILON) return {0, 0, ::fmax(0.0, value.normal)};
  if (value.normal >= 0 && radius <= mu * value.normal) return value;
  if (radius <= -value.normal / mu) return {0, 0, 0};
  const double normal = (mu * radius + value.normal) / (1 + mu * mu);
  if (normal <= 0) return {0, 0, 0};
  const double tangent = mu * normal / radius;
  return {tangent * value.x, tangent * value.y, normal};
}

SIRE_ADMM_HD inline auto projectDualCoulombCone(ContactVector value, double mu)
    -> ContactVector {
  if (mu <= DBL_EPSILON) return {value.x, value.y, ::fmax(0.0, value.normal)};
  return projectCoulombCone(value, 1 / mu);
}

constexpr double spectral_admm_eta = 1e-6;
constexpr double spectral_admm_tau = 0.5;
constexpr double spectral_admm_fallback = 1e-10;

struct SpectralPenalty {
  double rho;
  double multiplier;
};

SIRE_ADMM_HD inline auto initialSpectralPenalty(double estimate) -> SpectralPenalty {
  const double eta = spectral_admm_eta;
  const double largest = ::fmax(estimate, eta);
  const double condition = ::fmax(1.0, largest / eta);
  return {::fmax(::sqrt(largest * eta) * ::pow(condition, 0.2), eta),
          ::pow(condition, 0.05)};
}

SIRE_ADMM_HD inline auto adaptSpectralPenalty(SpectralPenalty& penalty,
                                              double primal, double dual) -> bool {
  constexpr double residual_ratio = 50;
  if (primal > residual_ratio * dual) penalty.rho *= penalty.multiplier;
  else if (dual > residual_ratio * primal) penalty.rho /= penalty.multiplier;
  else return false;
  penalty.rho = ::fmax(1e-12, ::fmin(1e12, penalty.rho));
  return true;
}

}  // namespace sire::physics::contact::detail

#undef SIRE_ADMM_HD
