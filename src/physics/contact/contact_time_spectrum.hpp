#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <Eigen/Eigenvalues>

#include "contact_time_search.hpp"

namespace sire::physics::contact::detail {

inline auto contactTimeSpectrum(std::size_t n, const double* matrix)
    -> ContactTimeSpectrum {
  using RowMajorMatrix = Eigen::Matrix<double, -1, -1, Eigen::RowMajor>;
  const Eigen::MatrixXd dense = Eigen::Map<const RowMajorMatrix>(matrix, n, n);
  const Eigen::EigenSolver<Eigen::MatrixXd> spectrum(dense, false);
  if (spectrum.info() != Eigen::Success) {
    throw std::runtime_error("Cannot determine contact time sampling scale");
  }
  ContactTimeSpectrum limits{0, 0};
  for (Eigen::Index i = 0; i < spectrum.eigenvalues().size(); ++i) {
    const auto eigenvalue = spectrum.eigenvalues()[i];
    limits.rate = std::max(limits.rate, std::abs(eigenvalue));
    limits.frequency = std::max(limits.frequency, std::abs(eigenvalue.imag()));
  }
  return limits;
}

}  // namespace sire::physics::contact::detail
