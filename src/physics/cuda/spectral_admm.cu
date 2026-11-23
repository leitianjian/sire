#include "spectral_admm.hpp"

#include <limits>

#include "physics/contact/spectral_admm_math.hpp"

namespace sire::physics::contact::cuda {
namespace {

using detail::ContactVector;
using detail::spectral_admm_eta;

constexpr std::size_t workspace_vectors = 10;
constexpr unsigned warp_width = 32;
constexpr unsigned warp_mask = 0xffffffffu;
constexpr int warp_contacts = 8;
static_assert(sizeof(std::size_t) <= sizeof(double) && alignof(std::size_t) <= alignof(double));

template <class T>
__device__ auto swapValues(T& first, T& second) -> void {
  const T temporary = first;
  first = second;
  second = temporary;
}

__device__ auto dot(std::size_t n, const double* lhs, const double* rhs) -> double {
  double sum = 0;
  for (std::size_t i = 0; i < n; ++i) sum += lhs[i] * rhs[i];
  return sum;
}

__device__ auto largestTridiagonalEigenvalue(int n, const double* diagonal,
                                            const double* off_diagonal) -> double {
  if (n == 1) return diagonal[0];
  if (n == 2) {
    return (diagonal[0] + diagonal[1]) / 2 +
           ::hypot((diagonal[0] - diagonal[1]) / 2, off_diagonal[0]);
  }
  // Closed form for a symmetric 3x3 matrix, after centering and scaling.
  const double mean = diagonal[0] / 3 + diagonal[1] / 3 + diagonal[2] / 3;
  const double centered[]{diagonal[0] - mean, diagonal[1] - mean, diagonal[2] - mean};
  const double spread = ::sqrt((dot(3, centered, centered) +
                                2 * dot(2, off_diagonal, off_diagonal)) / 6);
  if (spread == 0) return mean;
  const double a = centered[0] / spread, b = centered[1] / spread;
  const double c = centered[2] / spread;
  const double ab = off_diagonal[0] / spread, bc = off_diagonal[1] / spread;
  const double determinant = a * (b * c - bc * bc) - ab * ab * c;
  const double cosine = ::fmax(-1.0, ::fmin(1.0, determinant / 2));
  return mean + 2 * spread * ::cos(::acos(cosine) / 3);
}

__device__ auto estimateLargestEigenvalue(std::size_t n, const double* matrix,
                                           double* previous, double* current,
                                           double* work) -> double {
  for (std::size_t i = 0; i < n; ++i) {
    previous[i] = 0;
    current[i] = 1 + 0.01 * (i + 1);
  }
  const double initial_norm = ::sqrt(dot(n, current, current));
  for (std::size_t i = 0; i < n; ++i) current[i] /= initial_norm;
  double diagonal[3]{}, off_diagonal[2]{};
  double previous_beta = 0;
  for (int step = 0; step < 3; ++step) {
    for (std::size_t row = 0; row < n; ++row) {
      work[row] = dot(n, matrix + row * n, current) - previous_beta * previous[row];
    }
    const double alpha = dot(n, current, work);
    diagonal[step] = alpha;
    for (std::size_t i = 0; i < n; ++i) work[i] -= alpha * current[i];
    const double beta = ::sqrt(dot(n, work, work));
    if (step == 2 || !(beta > DBL_EPSILON) || !::isfinite(beta)) {
      return largestTridiagonalEigenvalue(step + 1, diagonal, off_diagonal);
    }
    off_diagonal[step] = beta;
    previous_beta = beta;
    for (std::size_t i = 0; i < n; ++i) {
      previous[i] = current[i];
      current[i] = work[i] / beta;
    }
  }
  return largestTridiagonalEigenvalue(3, diagonal, off_diagonal);
}

// Symmetric diagonal pivoting. Trailing entries remain in their original
// scale until their column is factored, as in the CPU LDLT implementation.
__device__ auto updatedDiagonal(std::size_t n, const double* factor, std::size_t col) -> double {
  double correction = 0.0;
  for (std::size_t k = 0; k < col; ++k) {
    const double term = factor[col * n + k];
    correction += term * (factor[k * n + k] * term);
  }
  return factor[col * n + col] - correction;
}

__device__ auto updatedLower(std::size_t n, const double* factor, std::size_t row, std::size_t col,
                              double inverse_diagonal) -> double {
  double product = 0.0;
  for (std::size_t k = 0; k < col; ++k) {
    product += factor[row * n + k] * (factor[k * n + k] * factor[col * n + k]);
  }
  return (factor[row * n + col] - product) * inverse_diagonal;
}

__device__ auto factorLDLT(std::size_t n, const double* matrix, double shift,
                           double* factor, double* inverse_diagonal,
                           std::size_t* permutation) -> bool {
  for (std::size_t i = 0; i < n * n; ++i) factor[i] = matrix[i];
  for (std::size_t i = 0; i < n; ++i) {
    factor[i * n + i] += shift;
    permutation[i] = i;
  }
  for (std::size_t col = 0; col < n; ++col) {
    std::size_t pivot = col;
    for (std::size_t row = col + 1; row < n; ++row) {
      if (::fabs(factor[row * n + row]) > ::fabs(factor[pivot * n + pivot])) pivot = row;
    }
    if (pivot != col) {
      for (std::size_t i = 0; i < n; ++i) {
        swapValues(factor[col * n + i], factor[pivot * n + i]);
      }
      for (std::size_t i = col; i < n; ++i) {
        swapValues(factor[i * n + col], factor[i * n + pivot]);
      }
      swapValues(permutation[col], permutation[pivot]);
    }
    const double diagonal = updatedDiagonal(n, factor, col);
    if (!::isfinite(diagonal) || diagonal == 0) return false;
    factor[col * n + col] = diagonal;
    inverse_diagonal[col] = 1.0 / diagonal;
    if (!::isfinite(inverse_diagonal[col])) return false;
    for (std::size_t row = col + 1; row < n; ++row) {
      factor[row * n + col] = updatedLower(n, factor, row, col, inverse_diagonal[col]);
    }
  }
  // Keep both triangular traversals contiguous during the repeated solves.
  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t j = 0; j < i; ++j) factor[j * n + i] = factor[i * n + j];
  }
  return true;
}

__device__ auto solveLDLT(std::size_t n, const double* factor,
                           const double* inverse_diagonal, const std::size_t* permutation, const double* rhs,
                           double* work, double* result) -> bool {
  for (std::size_t row = 0; row < n; ++row) {
    work[row] = rhs[permutation[row]] - dot(row, factor + row * n, work);
  }
  for (std::size_t row = 0; row < n; ++row) work[row] *= inverse_diagonal[row];
  for (std::size_t row = n; row-- > 0;) {
    work[row] -= dot(n - row - 1, factor + row * n + row + 1, work + row + 1);
    if (!::isfinite(work[row])) return false;
  }
  for (std::size_t row = 0; row < n; ++row) result[permutation[row]] = work[row];
  return true;
}

__device__ auto vectorAt(const double* vector) -> ContactVector {
  return {vector[0], vector[1], vector[2]};
}

__device__ auto storeVector(double* target, ContactVector value) -> void {
  target[0] = value.x;
  target[1] = value.y;
  target[2] = value.normal;
}

__device__ auto projectCachedCone(ContactVector value, const double* cone) -> ContactVector {
  const double mu = cone[0];
  if (mu <= DBL_EPSILON) return {0, 0, ::fmax(0.0, value.normal)};
  const double radius = detail::tangentNorm(value);
  if (value.normal >= 0 && radius <= mu * value.normal) return value;
  if (radius <= -value.normal * cone[1]) return {0, 0, 0};
  const double normal = (mu * radius + value.normal) * cone[2];
  if (normal <= 0) return {0, 0, 0};
  const double tangent = mu * normal / radius;
  return {tangent * value.x, tangent * value.y, normal};
}

__device__ auto solveProblem(const AdmmProblem& problem, std::size_t capacity,
                             AdmmInputs input, AdmmOutputs output,
                             double* workspace) -> AdmmResult {
  constexpr AdmmResult invalid{AdmmStatus::InvalidInput, 0, 0, 0, 0};
  constexpr AdmmResult failed{AdmmStatus::NumericalFailure, 0, 0, 0, 0};
  if (problem.contacts > capacity || !::isfinite(problem.step) || problem.step <= 0 ||
      !::isfinite(problem.tolerance) || problem.tolerance < 0 || problem.max_iterations == 0 ||
      (problem.has_force_guess && !input.force_guess) ||
      (problem.has_dual_guess && !input.dual_guess)) return invalid;
  if (problem.contacts == 0) return {AdmmStatus::Converged, 0, 0, 0, 0};

  const auto stride = 3 * capacity;
  const auto n = 3 * problem.contacts;
  auto* matrix = workspace;
  auto* factor = matrix + n * n;
  auto* g = factor + n * n;
  auto* force = g + n;
  auto* primal = force + n;
  auto* dual = primal + n;
  auto* previous_force = dual + n;
  auto* previous_primal = previous_force + n;
  auto* rhs = previous_primal + n;
  auto* solve_work = rhs + n;
  auto* inverse_diagonal = solve_work + n;
  auto* cones = inverse_diagonal + n;
  auto* permutation = reinterpret_cast<std::size_t*>(cones + n);

  for (std::size_t i = 0; i < n; ++i) {
    const double velocity = input.velocity[i], acceleration = input.acceleration[i];
    if (!::isfinite(velocity) || !::isfinite(acceleration)) return invalid;
    g[i] = velocity - problem.step * acceleration;
    for (std::size_t j = 0; j < n; ++j) {
      const double value = -0.5 * (input.inverse_mass[i * stride + j] +
                                   input.inverse_mass[j * stride + i]);
      if (!::isfinite(value)) return invalid;
      matrix[i * n + j] = value;
    }
    const double guess = problem.has_force_guess ? input.force_guess[i] : 0;
    if (!::isfinite(guess)) return invalid;
    force[i] = problem.step * guess;
    if (!::isfinite(force[i])) return failed;
  }
  for (std::size_t contact = 0; contact < problem.contacts; ++contact) {
    const auto offset = 3 * contact;
    const double mu = input.friction[contact], target = input.normal_target[contact];
    if (!::isfinite(mu) || mu < 0 || !::isfinite(target)) return invalid;
    cones[offset] = mu;
    cones[offset + 1] = mu > DBL_EPSILON ? 1.0 / mu : 0.0;
    cones[offset + 2] = 1.0 / (1.0 + mu * mu);
    g[offset + 2] += target;
    storeVector(primal + offset, projectCachedCone(vectorAt(force + offset), cones + offset));
  }
  for (std::size_t i = 0; i < n; ++i) {
    dual[i] = problem.has_dual_guess ? input.dual_guess[i] : dot(n, matrix + i * n, primal) + g[i];
    if (!::isfinite(dual[i])) return problem.has_dual_guess ? invalid : failed;
  }
  if (!problem.has_dual_guess) {
    for (std::size_t contact = 0; contact < problem.contacts; ++contact) {
      const auto offset = 3 * contact;
      const double mu = input.friction[contact];
      auto value = vectorAt(dual + offset);
      value.normal += mu * detail::tangentNorm(value);
      storeVector(dual + offset, detail::projectDualCoulombCone(value, mu));
    }
  }

  const double estimate = estimateLargestEigenvalue(n, matrix, previous_force,
                                                     previous_primal, rhs);
  if (!::isfinite(estimate)) return failed;
  auto spectral = detail::initialSpectralPenalty(estimate);
  double penalty = detail::spectral_admm_tau * spectral.rho;
  double inverse_penalty = 1.0 / penalty;
  bool factored = factorLDLT(n, matrix, spectral_admm_eta + penalty, factor, inverse_diagonal, permutation);
  AdmmResult result{AdmmStatus::IterationLimit, 0, 0, 0, 0};
  for (std::size_t iteration = 0; iteration < problem.max_iterations; ++iteration) {
    for (std::size_t i = 0; i < n; ++i) {
      previous_force[i] = force[i];
      previous_primal[i] = primal[i];
      rhs[i] = penalty * primal[i] + dual[i] - g[i] + spectral_admm_eta * force[i];
    }
    for (std::size_t contact = 0; contact < problem.contacts; ++contact) {
      const auto offset = 3 * contact;
      const double shift = input.friction[contact] * detail::tangentNorm(vectorAt(dual + offset));
      rhs[offset + 2] = penalty * primal[offset + 2] + dual[offset + 2] -
                        (g[offset + 2] + shift) + spectral_admm_eta * force[offset + 2];
    }
    if (!factored || !solveLDLT(n, factor, inverse_diagonal, permutation, rhs, solve_work, force)) {
      if (!factorLDLT(n, matrix, spectral_admm_eta + penalty + detail::spectral_admm_fallback,
                      factor, inverse_diagonal, permutation) ||
          !solveLDLT(n, factor, inverse_diagonal, permutation, rhs, solve_work, force)) return failed;
      factored = true;
    }
    result = {AdmmStatus::IterationLimit, iteration + 1, 0, 0, 0};
    for (std::size_t contact = 0; contact < problem.contacts; ++contact) {
      const auto offset = 3 * contact;
      const ContactVector projected = projectCachedCone(
          {force[offset] - dual[offset] * inverse_penalty,
           force[offset + 1] - dual[offset + 1] * inverse_penalty,
           force[offset + 2] - dual[offset + 2] * inverse_penalty}, cones + offset);
      storeVector(primal + offset, projected);
      for (std::size_t i = offset; i < offset + 3; ++i) {
        const double difference = force[i] - primal[i];
        const double dual_residual = spectral_admm_eta * (force[i] - previous_force[i]) +
                                     penalty * (primal[i] - previous_primal[i]);
        dual[i] -= penalty * difference;
        if (!::isfinite(difference) || !::isfinite(dual_residual) || !::isfinite(dual[i])) return failed;
        result.primal_residual = ::fmax(result.primal_residual, ::fabs(difference));
        result.dual_residual = ::fmax(result.dual_residual, ::fabs(dual_residual));
      }
      const double product = dot(3, primal + offset, dual + offset);
      if (!::isfinite(product)) return failed;
      result.complementarity_residual = ::fmax(result.complementarity_residual, ::fabs(product));
    }
    if (result.primal_residual <= problem.tolerance && result.dual_residual <= problem.tolerance &&
        result.complementarity_residual <= problem.tolerance) {
      result.status = AdmmStatus::Converged;
      break;
    }
    if (detail::adaptSpectralPenalty(spectral, result.primal_residual, result.dual_residual)) {
      penalty = detail::spectral_admm_tau * spectral.rho;
      inverse_penalty = 1.0 / penalty;
      factored = factorLDLT(n, matrix, spectral_admm_eta + penalty, factor, inverse_diagonal, permutation);
    }
  }
  // Require all forces to be finite before publishing any output.
  for (std::size_t i = 0; i < n; ++i) {
    force[i] = primal[i] / problem.step;
    if (!::isfinite(force[i])) return failed;
  }
  for (std::size_t i = 0; i < n; ++i) output.forces[i] = force[i];
  if (output.dual) {
    for (std::size_t i = 0; i < n; ++i) output.dual[i] = dual[i];
  }
  return result;
}

struct WarpWorkspace {
  static constexpr int dimension = 3 * warp_contacts;
  double matrix[dimension * dimension], factor[dimension * dimension];
  double inverse_diagonal[dimension], lanczos_current[dimension], lanczos_work[dimension];
  std::size_t permutation[dimension];
};

__device__ auto warpMaximum(double value) -> double {
#if __CUDA_ARCH__ >= 800
  // The bit order matches the numeric order for the nonnegative residuals.
  const auto bits = static_cast<unsigned long long>(__double_as_longlong(value));
  const auto local_high = static_cast<unsigned>(bits >> 32);
  const auto high = __reduce_max_sync(warp_mask, local_high);
  const auto low = __reduce_max_sync(warp_mask, local_high == high ? static_cast<unsigned>(bits) : 0u);
  return __longlong_as_double((static_cast<unsigned long long>(high) << 32) | low);
#else
  for (unsigned delta = warp_width / 2; delta; delta /= 2) {
    value = ::fmax(value, __shfl_down_sync(warp_mask, value, delta));
  }
  return __shfl_sync(warp_mask, value, 0);
#endif
}

__device__ auto warpContactVector(double value, int contact_start) -> ContactVector {
  return {__shfl_sync(warp_mask, value, contact_start),
          __shfl_sync(warp_mask, value, contact_start + 1),
          __shfl_sync(warp_mask, value, contact_start + 2)};
}

__device__ auto component(ContactVector value, int axis) -> double {
  return axis == 0 ? value.x : axis == 1 ? value.y : value.normal;
}

__device__ auto warpFactor(int n, int lane, double shift, WarpWorkspace& scratch,
                           int& reverse) -> bool {
  auto* factor = scratch.factor;
  for (int i = lane; i < n * n; i += warp_width) factor[i] = scratch.matrix[i];
  __syncwarp();
  if (lane < n) {
    factor[lane * n + lane] += shift;
    scratch.permutation[lane] = lane;
  }
  __syncwarp();

  for (int col = 0; col < n; ++col) {
    int pivot = col;
    if (lane == 0) {
      for (int row = col + 1; row < n; ++row) {
        if (::fabs(factor[row * n + row]) > ::fabs(factor[pivot * n + pivot])) pivot = row;
      }
    }
    pivot = __shfl_sync(warp_mask, pivot, 0);
    if (pivot != col) {
      if (lane < n) swapValues(factor[col * n + lane], factor[pivot * n + lane]);
      __syncwarp();
      if (lane >= col && lane < n) swapValues(factor[lane * n + col], factor[lane * n + pivot]);
      if (lane == 0) swapValues(scratch.permutation[col], scratch.permutation[pivot]);
      __syncwarp();
    }
    const double diagonal = __shfl_sync(warp_mask, lane == col ? updatedDiagonal(n, factor, col) : 0.0, col);
    if (!::isfinite(diagonal) || diagonal == 0.0) return false;
    const double inverse = __shfl_sync(warp_mask, lane == col ? 1.0 / diagonal : 0.0, col);
    if (!::isfinite(inverse)) return false;
    if (lane == col) {
      factor[col * n + col] = diagonal;
      scratch.inverse_diagonal[col] = inverse;
    }
    if (lane > col && lane < n) {
      factor[lane * n + col] = updatedLower(n, factor, lane, col, inverse);
    }
    __syncwarp();
  }
  if (lane < n) {
    for (int j = 0; j < lane; ++j) factor[j * n + lane] = factor[lane * n + j];
    for (int i = 0; i < n; ++i) {
      if (scratch.permutation[i] == static_cast<std::size_t>(lane)) reverse = i;
    }
  }
  __syncwarp();
  return true;
}

__device__ auto warpSolve(int n, int lane, int reverse, const WarpWorkspace& scratch,
                          double rhs, double& force) -> bool {
  const bool active = lane < n;
  const auto* row = scratch.factor + (active ? lane : 0) * n;
  const int source = active ? static_cast<int>(scratch.permutation[lane]) : 0;
  const double permuted = __shfl_sync(warp_mask, rhs, source);
  double sum = 0.0;
  for (int j = 0; j < n; ++j) {
    const double pivot = __shfl_sync(warp_mask, permuted - sum, j);
    if (active && lane > j) sum += row[j] * pivot;
  }
  const double scaled = active ? (permuted - sum) * scratch.inverse_diagonal[lane] : 0.0;
  sum = 0.0;
  for (int j = n; j-- > 0;) {
    const double pivot = __shfl_sync(warp_mask, scaled - sum, j);
    if (lane < j) sum += row[j] * pivot;
  }
  force = __shfl_sync(warp_mask, scaled - sum, reverse);
  return !__any_sync(warp_mask, active && !::isfinite(force));
}

__device__ auto solveWarpProblem(const AdmmProblem& problem, std::size_t capacity,
                                 AdmmInputs input, AdmmOutputs output, int lane,
                                 WarpWorkspace& scratch) -> AdmmResult {
  constexpr AdmmResult invalid{AdmmStatus::InvalidInput, 0, 0, 0, 0};
  constexpr AdmmResult failed{AdmmStatus::NumericalFailure, 0, 0, 0, 0};
  if (problem.contacts > capacity || !::isfinite(problem.step) || problem.step <= 0 ||
      !::isfinite(problem.tolerance) || problem.tolerance < 0 || problem.max_iterations == 0 ||
      (problem.has_force_guess && !input.force_guess) ||
      (problem.has_dual_guess && !input.dual_guess)) return invalid;
  if (problem.contacts == 0) return {AdmmStatus::Converged, 0, 0, 0, 0};

  const int n = static_cast<int>(3 * problem.contacts);
  const int stride = static_cast<int>(3 * capacity);
  const bool active = lane < n;
  const int contact = active ? lane / 3 : 0;
  const int axis = lane % 3;
  const int start = 3 * contact;
  double g = 0.0, force = 0.0, penalty = 0.0;
  const double mu = input.friction[contact];
  const double cone[3]{mu, mu > DBL_EPSILON ? 1.0 / mu : 0.0, 1.0 / (1.0 + mu * mu)};
  bool valid = true;
  if (active) {
    const double velocity = input.velocity[lane], acceleration = input.acceleration[lane];
    const double guess = problem.has_force_guess ? input.force_guess[lane] : 0.0;
    const double target = input.normal_target[contact];
    valid = ::isfinite(velocity) && ::isfinite(acceleration) && ::isfinite(guess) &&
            ::isfinite(target) && ::isfinite(mu) && mu >= 0.0;
    g = velocity - problem.step * acceleration;
    if (axis == 2) g += target;
    force = problem.step * guess;
    for (int col = 0; col < n; ++col) {
      const double entry = -0.5 * (input.inverse_mass[lane * stride + col] +
                                   input.inverse_mass[col * stride + lane]);
      valid = valid && ::isfinite(entry);
      scratch.matrix[lane * n + col] = entry;
    }
  }
  if (__any_sync(warp_mask, !valid)) return invalid;
  if (__any_sync(warp_mask, active && !::isfinite(force))) return failed;
  const auto force_contact = warpContactVector(force, start);
  double primal = component(projectCachedCone(force_contact, cone), axis);
  if (active) scratch.inverse_diagonal[lane] = primal;
  __syncwarp();
  double dual = 0.0;
  if (active) {
    dual = problem.has_dual_guess ? input.dual_guess[lane]
                                  : dot(n, scratch.matrix + lane * n, scratch.inverse_diagonal) + g;
  }
  if (__any_sync(warp_mask, active && !::isfinite(dual))) {
    return problem.has_dual_guess ? invalid : failed;
  }
  if (!problem.has_dual_guess) {
    auto contact_dual = warpContactVector(dual, start);
    contact_dual.normal += mu * detail::tangentNorm(contact_dual);
    dual = component(detail::projectDualCoulombCone(contact_dual, mu), axis);
  }

  double largest = 0.0;
  if (lane == 0) {
    largest = estimateLargestEigenvalue(n, scratch.matrix, scratch.inverse_diagonal,
                                       scratch.lanczos_current, scratch.lanczos_work);
  }
  largest = __shfl_sync(warp_mask, largest, 0);
  if (!::isfinite(largest)) return failed;
  auto spectral = detail::initialSpectralPenalty(largest);
  penalty = detail::spectral_admm_tau * spectral.rho;
  double inverse_penalty = 1.0 / penalty;
  int reverse = 0;
  bool factored = warpFactor(n, lane, spectral_admm_eta + penalty, scratch, reverse);
  AdmmResult result{AdmmStatus::IterationLimit, 0, 0, 0, 0};
  for (std::size_t iteration = 0; iteration < problem.max_iterations; ++iteration) {
    const double previous_force = force, previous_primal = primal;
    const auto dual_contact = warpContactVector(dual, start);
    const double shift = axis == 2 ? mu * detail::tangentNorm(dual_contact) : 0.0;
    const double rhs = penalty * primal + dual - (g + shift) + spectral_admm_eta * force;
    if (!factored || !warpSolve(n, lane, reverse, scratch, rhs, force)) {
      if (!warpFactor(n, lane, spectral_admm_eta + penalty + detail::spectral_admm_fallback, scratch, reverse) ||
          !warpSolve(n, lane, reverse, scratch, rhs, force)) return failed;
      factored = true;
    }

    const auto projection_input = warpContactVector(force - dual * inverse_penalty, start);
    primal = component(projectCachedCone(projection_input, cone), axis);
    double difference = 0.0, dual_residual = 0.0;
    if (active) {
      difference = force - primal;
      dual_residual = spectral_admm_eta * (force - previous_force) +
                      penalty * (primal - previous_primal);
      dual -= penalty * difference;
    }
    const auto primal_contact = warpContactVector(primal, start);
    const auto new_dual_contact = warpContactVector(dual, start);
    double complementarity = 0.0;
    complementarity += primal_contact.x * new_dual_contact.x;
    complementarity += primal_contact.y * new_dual_contact.y;
    complementarity += primal_contact.normal * new_dual_contact.normal;
    if (__any_sync(warp_mask, active && (!::isfinite(difference) || !::isfinite(dual_residual) ||
                                       !::isfinite(dual) || !::isfinite(complementarity)))) return failed;
    result = {AdmmStatus::IterationLimit, iteration + 1,
              warpMaximum(::fabs(difference)), warpMaximum(::fabs(dual_residual)),
              warpMaximum(active ? ::fabs(complementarity) : 0.0)};
    if (result.primal_residual <= problem.tolerance && result.dual_residual <= problem.tolerance &&
        result.complementarity_residual <= problem.tolerance) {
      result.status = AdmmStatus::Converged;
      break;
    }
    if (detail::adaptSpectralPenalty(spectral, result.primal_residual, result.dual_residual)) {
      penalty = detail::spectral_admm_tau * spectral.rho;
      inverse_penalty = 1.0 / penalty;
      factored = warpFactor(n, lane, spectral_admm_eta + penalty, scratch, reverse);
    }
  }
  force = primal / problem.step;
  if (__any_sync(warp_mask, active && !::isfinite(force))) return failed;
  if (active) {
    output.forces[lane] = force;
    if (output.dual) output.dual[lane] = dual;
  }
  return result;
}

__global__ void solveBatch(std::size_t batch_size, std::size_t capacity,
                           AdmmInputs inputs, AdmmOutputs outputs,
                           std::size_t workspace_stride, double* workspace) {
  const unsigned group = threadIdx.x / warp_width;
  const int lane = static_cast<int>(threadIdx.x % warp_width);
  const auto index = static_cast<std::size_t>(blockIdx.x) * spectral_admm_problems_per_block + group;
  if (index >= batch_size) return;
  const auto contact_offset = capacity * index;
  const auto offset = 3 * contact_offset;
  const AdmmInputs local{inputs.problems + index, inputs.friction + contact_offset,
                        inputs.inverse_mass + offset * (3 * capacity), inputs.velocity + offset,
                        inputs.normal_target + contact_offset, inputs.acceleration + offset,
                        inputs.force_guess ? inputs.force_guess + offset : nullptr,
                        inputs.dual_guess ? inputs.dual_guess + offset : nullptr};
  const AdmmOutputs output{outputs.forces + offset, outputs.dual ? outputs.dual + offset : nullptr,
                           outputs.results + index};
  const auto& problem = *local.problems;
  __shared__ WarpWorkspace scratch[spectral_admm_problems_per_block];
  if (problem.contacts <= warp_contacts) {
    const auto result = solveWarpProblem(problem, capacity, local, output, lane, scratch[group]);
    if (lane == 0) *output.results = result;
  } else if (lane == 0) {
    *output.results = solveProblem(problem, capacity, local, output, workspace + workspace_stride * index);
  }
}

}  // namespace

auto spectralAdmmWorkspaceSize(std::size_t batch_size, std::size_t capacity)
    -> std::size_t {
  constexpr auto max_elements = std::numeric_limits<std::size_t>::max() / sizeof(double);
  if (batch_size == 0 || capacity == 0 || capacity > max_elements / 3) return 0;
  const auto dimension = 3 * capacity;
  const auto per_problem = max_elements / batch_size;
  if (dimension > per_problem / 2 / dimension) return 0;
  const auto matrices = 2 * dimension * dimension;
  constexpr auto vector_count = workspace_vectors + 1;  // include the permutation
  if (dimension > (per_problem - matrices) / vector_count) return 0;
  return batch_size * (matrices + vector_count * dimension) * sizeof(double);
}

auto launchShiftedSpectralAdmm(std::size_t batch_size, std::size_t capacity,
                               AdmmInputs inputs, AdmmOutputs outputs,
                               void* workspace, std::size_t workspace_size,
                               cudaStream_t stream) -> cudaError_t {
  if (batch_size == 0) return cudaSuccess;
  const auto required = spectralAdmmWorkspaceSize(batch_size, capacity);
  const auto blocks = (batch_size - 1) / spectral_admm_problems_per_block + 1;
  if (required == 0 || workspace_size < required || !workspace || !inputs.problems ||
      !inputs.friction || !inputs.inverse_mass || !inputs.velocity || !inputs.normal_target ||
      !inputs.acceleration || !outputs.forces || !outputs.results ||
      blocks > std::numeric_limits<int>::max()) return cudaErrorInvalidValue;
  solveBatch<<<static_cast<unsigned>(blocks), spectral_admm_problems_per_block * warp_width, 0, stream>>>(
      batch_size, capacity, inputs, outputs, required / batch_size / sizeof(double),
      static_cast<double*>(workspace));
  return cudaGetLastError();
}

}  // namespace sire::physics::contact::cuda
