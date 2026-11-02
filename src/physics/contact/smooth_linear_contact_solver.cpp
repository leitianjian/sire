#include "sire/physics/contact/smooth_linear_contact_solver.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <set>
#include <stdexcept>

#include <eigen3/Eigen/Dense>
#include <aris/core/reflection.hpp>

#include "sire/core/profiler.hpp"
#include "sire/physics/physics_engine.hpp"

namespace sire::physics::contact::smooth_linear {
namespace {
using MatrixRM = Eigen::Matrix<double, -1, -1, Eigen::RowMajor>;

auto solveNormalLcp(const Eigen::MatrixXd& K, const Eigen::VectorXd& g,
                    sire::Size max_iterations, double tolerance,
                    Eigen::VectorXd& impulse) -> SmoothLinearSolveInfo {
  const int n = static_cast<int>(g.size());
  std::vector<bool> active(n, false);
  for (int i = 0; i < n; ++i) active[i] = g[i] < -tolerance;
  Eigen::VectorXd scaling(n);
  for (int i = 0; i < n; ++i)
    scaling[i] = std::max(1e-12, K.row(i).cwiseAbs().maxCoeff());
  auto residual = [&](const Eigen::VectorXd& candidate) {
    const Eigen::VectorXd w = K * candidate + g;
    double r = 0;
    for (int i = 0; i < n; ++i) {
      const double scaled_impulse = candidate[i] * scaling[i];
      r = std::max({r, -w[i], -scaled_impulse,
                    std::abs(std::min(scaled_impulse, w[i]))});
    }
    return r;
  };
  auto solveActive = [&]() {
    std::vector<int> indices;
    for (int i = 0; i < n; ++i) if (active[i]) indices.push_back(i);
    Eigen::VectorXd candidate = Eigen::VectorXd::Zero(n);
    if (indices.empty()) return candidate;
    const int size = static_cast<int>(indices.size());
    Eigen::MatrixXd matrix(size, size);
    Eigen::VectorXd rhs(size);
    for (int i = 0; i < size; ++i) {
      rhs[i] = -g[indices[i]] / scaling[indices[i]];
      for (int j = 0; j < size; ++j)
        matrix(i, j) = K(indices[i], indices[j]) / scaling[indices[i]];
    }
    const Eigen::VectorXd solution = matrix.completeOrthogonalDecomposition().solve(rhs);
    for (int i = 0; i < size; ++i) candidate[indices[i]] = solution[i];
    return candidate;
  };
  SmoothLinearSolveInfo info;
  std::set<std::vector<bool>> visited;
  for (sire::Size iteration = 0; iteration < max_iterations; ++iteration) {
    if (!visited.insert(active).second) break;
    ++info.iterations;
    impulse = solveActive();
    if (!impulse.allFinite()) break;
    const Eigen::VectorXd w = K * impulse + g;
    info.residual = residual(impulse);
    if (info.residual <= tolerance) {
      impulse = impulse.cwiseMax(0.0);
      info.residual = residual(impulse);
      if (info.residual <= tolerance) return info;
    }
    int remove = -1, add = -1;
    double most_negative_force = -tolerance, most_negative_gap = -tolerance;
    for (int i = 0; i < n; ++i) {
      if (active[i] && impulse[i] * scaling[i] < most_negative_force) {
        most_negative_force = impulse[i] * scaling[i];
        remove = i;
      }
      if (!active[i] && w[i] < most_negative_gap) {
        most_negative_gap = w[i];
        add = i;
      }
    }
    if (remove >= 0) active[remove] = false;
    else if (add >= 0) active[add] = true;
    else break;
  }
  // Exhaustive active-set verification handles small degenerate contact sets
  // without replacing the smooth law by a Coulomb/ADMM fallback.
  if (n <= 12) {
    info.enumerated = true;
    const std::uint64_t limit = std::uint64_t{1} << n;
    for (std::uint64_t mask = 0; mask < limit; ++mask) {
      for (int i = 0; i < n; ++i) active[i] = (mask >> i) & 1;
      ++info.iterations;
      impulse = solveActive();
      if (!impulse.allFinite()) continue;
      info.residual = residual(impulse);
      if (info.residual <= tolerance) {
        impulse = impulse.cwiseMax(0.0);
        info.residual = residual(impulse);
        if (info.residual <= tolerance) return info;
      }
    }
  }
  throw std::runtime_error("Smooth linear normal complementarity solve failed");
}
}  // namespace

auto cptContactForceSmoothLinear(
    sire::Size n, const std::vector<double>& friction,
    const std::vector<double>& inverse_mass, const std::vector<double>& velocity,
    const std::vector<double>& normal_target,
    const std::vector<double>& external_acceleration, double h,
    const std::vector<double>& threshold_velocity,
    std::vector<double>& contact_force, sire::Size max_iterations,
    double tolerance) -> SmoothLinearSolveInfo {
  SIRE_PROFILE_FUNCTION();
  if (!std::isfinite(h) || h <= 0 || !std::isfinite(tolerance) || tolerance <= 0)
    throw std::invalid_argument("Smooth solve requires positive finite h/tolerance");
  const int count = static_cast<int>(n), dimension = 3 * count;
  if (friction.size() != n || threshold_velocity.size() != n ||
      inverse_mass.size() != static_cast<size_t>(dimension * dimension) ||
      velocity.size() != static_cast<size_t>(dimension) ||
      normal_target.size() != n || external_acceleration.size() != static_cast<size_t>(dimension))
    throw std::invalid_argument("Smooth contact input dimensions differ");
  contact_force.assign(dimension, 0.0);
  if (n == 0) return {};
  const Eigen::Map<const MatrixRM> inverse(inverse_mass.data(), dimension, dimension);
  Eigen::MatrixXd G = -inverse;
  G = (0.5 * (G + G.transpose())).eval();
  Eigen::MatrixXd direction = Eigen::MatrixXd::Zero(dimension, count);
  for (int i = 0; i < count; ++i) {
    if (!std::isfinite(friction[i]) || friction[i] < 0 ||
        !std::isfinite(threshold_velocity[i]) || threshold_velocity[i] <= 0)
      throw std::invalid_argument("Smooth friction/threshold is invalid");
    const double speed = std::hypot(velocity[3 * i], velocity[3 * i + 1]);
    // Exactly linear below v_s, Coulomb magnitude above v_s; no cubic ramp.
    const double denominator = std::max(speed, threshold_velocity[i]);
    direction(3 * i, i) = -friction[i] * velocity[3 * i] / denominator;
    direction(3 * i + 1, i) = -friction[i] * velocity[3 * i + 1] / denominator;
    direction(3 * i + 2, i) = 1.0;
  }
  Eigen::MatrixXd normal_response(count, count);
  Eigen::VectorXd g(count);
  for (int i = 0; i < count; ++i) {
    normal_response.row(i) = G.row(3 * i + 2) * direction;
    g[i] = velocity[3 * i + 2] - h * external_acceleration[3 * i + 2] + normal_target[i];
  }
  if (!normal_response.allFinite() || !g.allFinite())
    throw std::invalid_argument("Smooth contact inputs contain NaN/Inf");
  Eigen::VectorXd normal_impulse(count);
  // Impulse coordinates keep the linear system independent of tiny h.
  auto info = solveNormalLcp(normal_response, g, max_iterations, tolerance, normal_impulse);
  const Eigen::VectorXd force = direction * normal_impulse / h;
  if (!force.allFinite()) throw std::runtime_error("Smooth contact force is non-finite");
  std::copy_n(force.data(), dimension, contact_force.data());
  return info;
}

SmoothLinearContactSolver::SmoothLinearContactSolver() {
  setContactModelMode("single_point");
}
auto SmoothLinearContactSolver::doInit(physics::PhysicsEngine*) -> void {
  clearContactSolverState();
}
auto SmoothLinearContactSolver::prepareContactForceInitialGuess(
    const std::vector<common::PenetrationAsPointPair>& pairs,
    const std::vector<std::array<double, 16>>&,
    const std::vector<sire::Size>& preserved, double,
    std::vector<double>& force) -> void {
  threshold_velocity_.clear();
  threshold_velocity_.reserve(preserved.size());
  auto& engine = *physicsEnginePtr();
  for (const auto index : preserved) {
    const auto& pair = pairs[index];
    const auto& properties = materialManager().getPropMapOrDefault({
        engine.queryGeometryPoolById(pair.id_A)->material(),
        engine.queryGeometryPoolById(pair.id_B)->material()});
    threshold_velocity_.push_back(properties.getPropValueOrDefault(
        "threshold_velocity", defaultVelocityThreshold()));
  }
  std::fill(force.begin(), force.end(), 0.0);
}
auto SmoothLinearContactSolver::solveContactForceQP(
    sire::Size n, std::vector<double>& friction,
    std::vector<double>& inverse_mass, std::vector<double>& velocity,
    std::vector<double>& normal_target, std::vector<double>& acceleration,
    double h, std::vector<double>& force, sire::Size max_iterations,
    double tolerance) -> double {
  last_info_ = cptContactForceSmoothLinear(n, friction, inverse_mass, velocity,
      normal_target, acceleration, h, threshold_velocity_, force, max_iterations, tolerance);
  return last_info_.residual;
}
auto SmoothLinearContactSolver::clearContactSolverState() -> void {
  threshold_velocity_.clear();
  last_info_ = {};
}

ARIS_REGISTRATION {
  aris::core::class_<SmoothLinearContactSolver>("SmoothLinearContactSolver")
      .inherit<ps_vs_solver_v5::PsVsSolverV5>();
}
}  // namespace sire::physics::contact::smooth_linear
