#include "sire/physics/contact/simple_admm_contact_solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>

#include <aris/core/reflection.hpp>
#include <aris/dynamic/pose.hpp>

#include "sire/core/profiler.hpp"

namespace sire::physics::contact::simple_admm {

namespace {

using MatrixXdRM = Eigen::Matrix<double, -1, -1, Eigen::RowMajor>;

auto projectCoulombCone(const Eigen::Vector3d& value, double mu)
    -> Eigen::Vector3d {
  const double normal = value.z();
  const double tangent_norm = value.head<2>().norm();

  if (mu <= std::numeric_limits<double>::epsilon()) {
    return Eigen::Vector3d(0.0, 0.0, std::max(0.0, normal));
  }
  if (normal >= 0.0 && tangent_norm <= mu * normal) return value;
  if (tangent_norm <= -normal / mu) return Eigen::Vector3d::Zero();

  const double projected_normal =
      (mu * tangent_norm + normal) / (1.0 + mu * mu);
  if (projected_normal <= 0.0) return Eigen::Vector3d::Zero();

  Eigen::Vector3d result;
  result.head<2>() = (mu * projected_normal / tangent_norm) * value.head<2>();
  result.z() = projected_normal;
  return result;
}

auto projectCoulombCones(const Eigen::VectorXd& value, int contact_count,
                         const std::vector<double>& friction)
    -> Eigen::VectorXd {
  Eigen::VectorXd result(value.size());
  for (int contact = 0; contact < contact_count; ++contact) {
    result.segment<3>(3 * contact) =
        projectCoulombCone(value.segment<3>(3 * contact), friction[contact]);
  }
  return result;
}

auto projectDualCoulombCone(const Eigen::Vector3d& value, double mu)
    -> Eigen::Vector3d {
  if (mu <= std::numeric_limits<double>::epsilon()) {
    return Eigen::Vector3d(value.x(), value.y(), std::max(0.0, value.z()));
  }
  return projectCoulombCone(value, 1.0 / mu);
}

auto projectDualCoulombCones(const Eigen::VectorXd& value, int contact_count,
                             const std::vector<double>& friction)
    -> Eigen::VectorXd {
  Eigen::VectorXd result(value.size());
  for (int contact = 0; contact < contact_count; ++contact) {
    result.segment<3>(3 * contact) = projectDualCoulombCone(
        value.segment<3>(3 * contact), friction[contact]);
  }
  return result;
}

/// Paper definition: Gamma(sigma)_i = (0, 0, mu_i ||sigma_T,i||).
auto computeDeSaxceShift(const Eigen::VectorXd& sigma, int contact_count,
                         const std::vector<double>& friction)
    -> Eigen::VectorXd {
  Eigen::VectorXd shift = Eigen::VectorXd::Zero(sigma.size());
  for (int contact = 0; contact < contact_count; ++contact) {
    shift(3 * contact + 2) =
        friction[contact] * sigma.segment<2>(3 * contact).norm();
  }
  return shift;
}

/// Estimate lambda_max(H) using Simple's configured three-vector Lanczos
/// decomposition. The nonuniform deterministic start keeps the estimate
/// reproducible and avoids an accidental orthogonality with common modes.
auto estimateLargestEigenvalue(const Eigen::MatrixXd& H) -> double {
  constexpr int lanczos_size = 3;
  const int steps = std::min(lanczos_size, static_cast<int>(H.rows()));
  Eigen::VectorXd q(H.rows());
  for (Eigen::Index i = 0; i < q.size(); ++i) {
    q(i) = 1.0 + 0.01 * static_cast<double>(i + 1);
  }
  q.normalize();

  Eigen::VectorXd q_previous = Eigen::VectorXd::Zero(H.rows());
  Eigen::MatrixXd tridiagonal = Eigen::MatrixXd::Zero(steps, steps);
  double beta_previous = 0.0;
  int completed_steps = 0;
  for (int step = 0; step < steps; ++step) {
    Eigen::VectorXd w = H * q - beta_previous * q_previous;
    const double alpha = q.dot(w);
    w -= alpha * q;
    tridiagonal(step, step) = alpha;
    completed_steps = step + 1;

    const double beta = w.norm();
    if (step + 1 == steps || !(beta > std::numeric_limits<double>::epsilon()) ||
        !std::isfinite(beta)) {
      break;
    }
    tridiagonal(step, step + 1) = beta;
    tridiagonal(step + 1, step) = beta;
    q_previous = q;
    q = w / beta;
    beta_previous = beta;
  }

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(
      tridiagonal.topLeftCorner(completed_steps, completed_steps),
      Eigen::EigenvaluesOnly);
  if (eigensolver.info() != Eigen::Success ||
      eigensolver.eigenvalues().size() == 0) {
    return 0.0;
  }
  return std::max(0.0, eigensolver.eigenvalues().maxCoeff());
}

auto complementarityResidual(const Eigen::VectorXd& force,
                             const Eigen::VectorXd& dual, int contact_count)
    -> double {
  double residual = 0.0;
  for (int contact = 0; contact < contact_count; ++contact) {
    residual = std::max(
        residual,
        std::abs(
            force.segment<3>(3 * contact).dot(dual.segment<3>(3 * contact))));
  }
  return residual;
}

// One iteration kernel for both variants. A null target preserves the paper
// baseline; a non-null target shifts only the NCP's free normal velocity.
auto solveSpectralAdmm(sire::Size n, std::vector<double>& fri_coef,
                       std::vector<double>& invM_3n, std::vector<double>& v0,
                       const std::vector<double>* normal_target,
                       std::vector<double>& b, double h,
                       const std::vector<double>* initial_force,
                       const std::vector<double>* initial_dual,
                       std::vector<double>& contactFce,
                       std::vector<double>* solved_dual, sire::Size max_iters,
                       double max_err) -> double {
  SIRE_PROFILE_FUNCTION();

  const int contact_count = static_cast<int>(n);
  const int dimension = 3 * contact_count;
  if (contact_count == 0) {
    std::fill(contactFce.begin(), contactFce.end(), 0.0);
    return 0.0;
  }
  if (fri_coef.size() < n ||
      invM_3n.size() < static_cast<size_t>(dimension * dimension) ||
      v0.size() < static_cast<size_t>(dimension) ||
      b.size() < static_cast<size_t>(dimension) ||
      (normal_target && normal_target->size() < n) ||
      contactFce.size() < static_cast<size_t>(dimension) || h <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }

  Eigen::Map<MatrixXdRM> inverse_mass(invM_3n.data(), dimension, dimension);
  Eigen::Map<Eigen::VectorXd> initial_velocity(v0.data(), dimension);
  Eigen::Map<Eigen::VectorXd> external_acceleration(b.data(), dimension);

  // Algorithm 1 is written in contact impulses lambda. Sire's invM_3n maps
  // forces to accelerations with the opposite sign, so the impulse-space
  // Delassus operator is -invM_3n (without h). Solving in impulse coordinates
  // preserves the paper's eta, rho, and absolute stopping-residual units.
  Eigen::MatrixXd G = -inverse_mass;
  G = 0.5 * (G + G.transpose());
  Eigen::VectorXd g = initial_velocity - h * external_acceleration;
  if (normal_target) {
    for (int contact = 0; contact < contact_count; ++contact) {
      if (!std::isfinite((*normal_target)[contact])) {
        return std::numeric_limits<double>::infinity();
      }
      // c = G*lambda + v0 - h*b is the physical post-step velocity.
      // Desired c_N = -v_target, so w_N = c_N + v_target. This constant
      // velocity shift enters g once, with no h scaling or extra constraint.
      g(3 * contact + 2) += (*normal_target)[contact];
    }
  }

  // Simple's default spectral-ADMM parameters. Pinocchio applies tau to the
  // augmented-Lagrangian penalty while rho itself follows the spectral rule.
  constexpr double eta = 1e-6;
  constexpr double tau = 0.5;
  constexpr double initial_rho_power = 0.2;
  constexpr double rho_power_increment = 0.05;
  constexpr double residual_ratio = 50.0;

  // In the rigid case R=0, the reference implementation uses m=eta and
  // L=lambda_max(G), then rho=sqrt(mL)*(L/m)^p.
  const double largest_eigenvalue = std::max(estimateLargestEigenvalue(G), eta);
  const double smallest_regularized_eigenvalue = eta;
  const double condition_number =
      std::max(1.0, largest_eigenvalue / smallest_regularized_eigenvalue);
  const double rho_multiplier = std::pow(condition_number, rho_power_increment);
  double rho = std::sqrt(largest_eigenvalue * smallest_regularized_eigenvalue) *
               std::pow(condition_number, initial_rho_power);
  rho = std::max(rho, eta);

  Eigen::VectorXd f = Eigen::VectorXd::Zero(dimension);
  if (initial_force &&
      initial_force->size() >= static_cast<size_t>(dimension)) {
    // Public Sire values are forces; Algorithm 1 iterates on impulses.
    f = h * Eigen::Map<const Eigen::VectorXd>(initial_force->data(), dimension);
  }
  Eigen::VectorXd y = projectCoulombCones(f, contact_count, fri_coef);
  Eigen::VectorXd z = Eigen::VectorXd::Zero(dimension);

  double effective_penalty = tau * rho;
  if (initial_dual && initial_dual->size() >= static_cast<size_t>(dimension)) {
    z = Eigen::Map<const Eigen::VectorXd>(initial_dual->data(), dimension);
  } else {
    // Match Pinocchio's first-solve initialization: construct a dual-cone
    // feasible value from the projected primal guess and the current problem.
    // Pinocchio applies the damped Delassus operator and then removes its
    // proximal diagonal here. With R=0 this reduces exactly to G*y + g.
    z = G * y + g;
    z += computeDeSaxceShift(z, contact_count, fri_coef);
    z = projectDualCoulombCones(z, contact_count, fri_coef);
  }

  Eigen::MatrixXd augmented_matrix;
  Eigen::LDLT<Eigen::MatrixXd> factorization;
  auto refactor = [&]() {
    augmented_matrix = G + (eta + effective_penalty) *
                               Eigen::MatrixXd::Identity(dimension, dimension);
    factorization.compute(augmented_matrix);
    return factorization.info() == Eigen::Success;
  };
  bool factorization_ok = refactor();

  double primal_residual = std::numeric_limits<double>::infinity();
  double dual_residual = std::numeric_limits<double>::infinity();
  double complementarity_residual = std::numeric_limits<double>::infinity();
  sire::Size iterations = 0;
  sire::Size factorization_updates = 1;

  for (sire::Size iteration = 0; iteration < max_iters; ++iteration) {
    ++iterations;
    const Eigen::VectorXd f_previous = f;
    const Eigen::VectorXd y_previous = y;

    // Algorithm 1, line 2: refresh the nonlinear De Saxce term every sweep.
    const Eigen::VectorXd shift =
        computeDeSaxceShift(z, contact_count, fri_coef);

    // Algorithm 1, lines 3-4 / Eq. (37).
    const Eigen::VectorXd rhs =
        effective_penalty * y + z - (g + shift) + eta * f_previous;
    if (factorization_ok) {
      f = factorization.solve(rhs);
      factorization_ok = factorization.info() == Eigen::Success;
    }
    if (!factorization_ok || !f.allFinite()) {
      // Numerical fallback only; it does not change the NCP being solved.
      const Eigen::MatrixXd regularized =
          augmented_matrix +
          1e-10 * Eigen::MatrixXd::Identity(dimension, dimension);
      f = regularized.ldlt().solve(rhs);
    }

    // Algorithm 1, lines 5-6 / Eqs. (39) and (36c).
    y = projectCoulombCones(f - z / effective_penalty, contact_count, fri_coef);
    z -= effective_penalty * (f - y);

    // Eqs. (40)-(43): all three absolute residuals are required.
    primal_residual = (f - y).lpNorm<Eigen::Infinity>();
    dual_residual =
        (eta * (f - f_previous) + effective_penalty * (y - y_previous))
            .lpNorm<Eigen::Infinity>();
    complementarity_residual = complementarityResidual(y, z, contact_count);

    if (primal_residual <= max_err && dual_residual <= max_err &&
        complementarity_residual <= max_err) {
      break;
    }

    // Sec. III-F spectral update: p <- p +/- 0.05 is equivalent to
    // rho <- rho * kappa^(+/- 0.05) because m and L remain fixed here.
    bool update_rho = false;
    if (primal_residual > residual_ratio * dual_residual) {
      rho *= rho_multiplier;
      update_rho = true;
    } else if (dual_residual > residual_ratio * primal_residual) {
      rho /= rho_multiplier;
      update_rho = true;
    }
    if (update_rho) {
      rho = std::clamp(rho, 1e-12, 1e12);
      effective_penalty = tau * rho;
      factorization_ok = refactor();
      ++factorization_updates;
    }
  }

  // Sire exposes contact forces, whereas Algorithm 1 returns impulses:
  // lambda = h * force.
  Eigen::Map<Eigen::VectorXd> output_force(contactFce.data(), dimension);
  output_force = y / h;
  if (solved_dual) {
    solved_dual->assign(z.data(), z.data() + z.size());
  }

  const double error = std::max(
      primal_residual, std::max(dual_residual, complementarity_residual));
  SIRE_PROFILE_PLOT("spectral_admm.primal_residual", primal_residual);
  SIRE_PROFILE_PLOT("spectral_admm.dual_residual", dual_residual);
  SIRE_PROFILE_PLOT("spectral_admm.complementarity_residual",
                    complementarity_residual);
  SIRE_PROFILE_PLOT("spectral_admm.rho", rho);
  SIRE_PROFILE_PLOT("spectral_admm.iterations",
                    static_cast<double>(iterations));
  SIRE_PROFILE_PLOT("spectral_admm.factorization_updates",
                    static_cast<double>(factorization_updates));
  return error;
}

}  // namespace

auto cptContactForceSpectralAdmm(
    sire::Size n, std::vector<double>& fri_coef, std::vector<double>& invM_3n,
    std::vector<double>& v0, std::vector<double>& v_target,
    std::vector<double>& b, double h, std::vector<double>& contactFce,
    sire::Size max_iters, double max_err) -> double {
  (void)v_target;
  return solveSpectralAdmm(n, fri_coef, invM_3n, v0, nullptr, b, h, nullptr,
                           nullptr, contactFce, nullptr, max_iters, max_err);
}

auto cptContactForceShiftedSpectralAdmm(
    sire::Size n, std::vector<double>& fri_coef, std::vector<double>& invM_3n,
    std::vector<double>& v0, std::vector<double>& v_target,
    std::vector<double>& b, double h, std::vector<double>& contactFce,
    sire::Size max_iters, double max_err) -> double {
  return solveSpectralAdmm(n, fri_coef, invM_3n, v0, &v_target, b, h, nullptr,
                           nullptr, contactFce, nullptr, max_iters, max_err);
}

auto cptContactForceSimpleAdmm(sire::Size n, std::vector<double>& fri_coef,
                               std::vector<double>& invM_3n,
                               std::vector<double>& v0,
                               std::vector<double>& v_target,
                               std::vector<double>& b, double h,
                               std::vector<double>& contactFce,
                               sire::Size max_iters, double max_err) -> double {
  return cptContactForceSpectralAdmm(n, fri_coef, invM_3n, v0, v_target, b, h,
                                     contactFce, max_iters, max_err);
}

auto SimpleAdmmContactSolver::solveContactForceQP(
    sire::Size n, std::vector<double>& fri_coef, std::vector<double>& invM_3n,
    std::vector<double>& v0, std::vector<double>& v_target,
    std::vector<double>& b, double h, std::vector<double>& contactFce,
    sire::Size max_iters, double max_err) -> double {
  return solveSpectralContactForce(false, n, fri_coef, invM_3n, v0, v_target, b,
                                   h, contactFce, max_iters, max_err);
}

auto SimpleAdmmContactSolver::setWarmStartEnabled(bool enabled) noexcept
    -> void {
  warm_start_enabled_ = enabled;
  if (!enabled) clearContactSolverState();
}

auto SimpleAdmmContactSolver::warmStartEnabled() const noexcept -> bool {
  return warm_start_enabled_;
}

auto SimpleAdmmContactSolver::solveSpectralContactForce(
    bool shifted, sire::Size n, std::vector<double>& fri_coef,
    std::vector<double>& invM_3n, std::vector<double>& v0,
    std::vector<double>& v_target, std::vector<double>& b, double h,
    std::vector<double>& contactFce, sire::Size max_iters, double max_err)
    -> double {
  const std::vector<double>* initial_force =
      has_primal_guess_ ? &contactFce : nullptr;
  const std::vector<double>* initial_dual =
      has_dual_guess_ ? &prepared_dual_guess_ : nullptr;
  const std::vector<double>* normal_target = shifted ? &v_target : nullptr;
  const double error = solveSpectralAdmm(
      n, fri_coef, invM_3n, v0, normal_target, b, h, initial_force,
      initial_dual, contactFce, &last_dual_solution_, max_iters, max_err);
  has_primal_guess_ = false;
  has_dual_guess_ = false;
  prepared_dual_guess_.clear();
  return error;
}

auto SimpleAdmmContactSolver::prepareContactForceInitialGuess(
    const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
    const std::vector<std::array<double, 16>>& contact_frames,
    const std::vector<sire::Size>& preserved_pair_indices, double,
    std::vector<double>& contact_force) -> void {
  has_primal_guess_ = false;
  has_dual_guess_ = false;
  prepared_dual_guess_.assign(contact_force.size(), 0.0);
  std::fill(contact_force.begin(), contact_force.end(), 0.0);
  if (!warm_start_enabled_ || warm_start_contacts_.empty()) return;

  // Simple does not supply an explicit dual guess. Pinocchio therefore keeps
  // the dual iterate stored in its persistent solver whenever the problem
  // dimension is unchanged. Reproduce that index-based behavior here.
  if (last_dual_solution_.size() == contact_force.size()) {
    prepared_dual_guess_ = last_dual_solution_;
    has_dual_guess_ = true;
  }

  sire::Size matched_contacts = 0;
  for (sire::Size contact = 0; contact < preserved_pair_indices.size();
       ++contact) {
    const sire::Size pair_index = preserved_pair_indices[contact];
    if (pair_index >= penetration_pairs.size() ||
        pair_index >= contact_frames.size()) {
      continue;
    }
    const auto& pair = penetration_pairs[pair_index];
    const WarmStartContact* best = nullptr;
    bool reversed = false;
    double best_distance = std::numeric_limits<double>::infinity();
    for (const auto& previous : warm_start_contacts_) {
      const bool same_order =
          previous.id_a == pair.id_A && previous.id_b == pair.id_B;
      const bool reverse_order =
          previous.id_a == pair.id_B && previous.id_b == pair.id_A;
      if (!same_order && !reverse_order) continue;
      double distance = 0.0;
      for (int axis = 0; axis < 3; ++axis) {
        const double delta = previous.point_W[axis] - pair.p_WC[axis];
        distance += delta * delta;
      }
      if (distance < best_distance) {
        best_distance = distance;
        best = &previous;
        reversed = reverse_order;
      }
    }
    if (!best) continue;

    double force_W[3];
    for (int axis = 0; axis < 3; ++axis) {
      const double sign = reversed ? -1.0 : 1.0;
      force_W[axis] = sign * best->force_W[axis];
    }
    double force_C[3];
    aris::dynamic::s_inv_pm_dot_v3(contact_frames[pair_index].data(), force_W,
                                   force_C);
    for (int axis = 0; axis < 3; ++axis) {
      contact_force[3 * contact + axis] = force_C[axis];
    }
    ++matched_contacts;
  }
  has_primal_guess_ = matched_contacts > 0;
}

auto SimpleAdmmContactSolver::commitContactForceSolution(
    const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
    const std::vector<std::array<double, 16>>& contact_frames,
    const std::vector<sire::Size>& preserved_pair_indices, double,
    const std::vector<double>& contact_force) -> void {
  if (!warm_start_enabled_) return;
  const sire::Size contact_count = preserved_pair_indices.size();
  if (contact_force.size() < 3 * contact_count ||
      last_dual_solution_.size() < 3 * contact_count) {
    clearContactSolverState();
    return;
  }

  std::vector<WarmStartContact> next;
  next.reserve(contact_count);
  for (sire::Size contact = 0; contact < contact_count; ++contact) {
    const sire::Size pair_index = preserved_pair_indices[contact];
    if (pair_index >= penetration_pairs.size() ||
        pair_index >= contact_frames.size()) {
      continue;
    }
    const auto& pair = penetration_pairs[pair_index];
    WarmStartContact record;
    record.id_a = pair.id_A;
    record.id_b = pair.id_B;
    for (int axis = 0; axis < 3; ++axis) {
      record.point_W[axis] = pair.p_WC[axis];
    }
    aris::dynamic::s_pm_dot_v3(contact_frames[pair_index].data(),
                               contact_force.data() + 3 * contact,
                               record.force_W.data());
    next.push_back(record);
  }
  warm_start_contacts_ = std::move(next);
}

auto SimpleAdmmContactSolver::clearContactSolverState() -> void {
  has_primal_guess_ = false;
  has_dual_guess_ = false;
  prepared_dual_guess_.clear();
  last_dual_solution_.clear();
  warm_start_contacts_.clear();
}

ShiftedSpectralAdmmContactSolver::ShiftedSpectralAdmmContactSolver() {
  setContactModelMode("single_point");
}

auto ShiftedSpectralAdmmContactSolver::solveContactForceQP(
    sire::Size n, std::vector<double>& fri_coef, std::vector<double>& invM_3n,
    std::vector<double>& v0, std::vector<double>& v_target,
    std::vector<double>& b, double h, std::vector<double>& contactFce,
    sire::Size max_iters, double max_err) -> double {
  return solveSpectralContactForce(true, n, fri_coef, invM_3n, v0, v_target, b,
                                   h, contactFce, max_iters, max_err);
}

ARIS_REGISTRATION {
  aris::core::class_<SimpleAdmmContactSolver>("SimpleAdmmContactSolver")
      .inherit<ps_vs_solver_v5::PsVsSolverV5>()
      .prop("warm_start_enabled", &SimpleAdmmContactSolver::setWarmStartEnabled,
            &SimpleAdmmContactSolver::warmStartEnabled);
  aris::core::class_<ShiftedSpectralAdmmContactSolver>(
      "ShiftedSpectralAdmmContactSolver")
      .inherit<SimpleAdmmContactSolver>();
}

}  // namespace sire::physics::contact::simple_admm
