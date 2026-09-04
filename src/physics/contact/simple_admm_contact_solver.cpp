#include "sire/physics/contact/simple_admm_contact_solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/Dense>

#include <aris/core/reflection.hpp>

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
  result.head<2>() =
      (mu * projected_normal / tangent_norm) * value.head<2>();
  result.z() = projected_normal;
  return result;
}

auto projectCoulombCones(const Eigen::VectorXd& value, int contact_count,
                         const std::vector<double>& friction)
    -> Eigen::VectorXd {
  Eigen::VectorXd result(value.size());
  for (int contact = 0; contact < contact_count; ++contact) {
    result.segment<3>(3 * contact) = projectCoulombCone(
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

/// Estimate lambda_max(H) as in the reference implementation's 20-step power
/// iteration.  The nonuniform deterministic start avoids an accidental
/// orthogonality between an all-ones vector and the dominant eigenspace.
auto estimateLargestEigenvalue(const Eigen::MatrixXd& H) -> double {
  Eigen::VectorXd direction(H.rows());
  for (Eigen::Index i = 0; i < direction.size(); ++i) {
    direction(i) = 1.0 + 0.01 * static_cast<double>(i + 1);
  }
  direction.normalize();

  double eigenvalue = 0.0;
  for (int iteration = 0; iteration < 20; ++iteration) {
    Eigen::VectorXd image = H * direction;
    const double image_norm = image.norm();
    if (!(image_norm > std::numeric_limits<double>::epsilon()) ||
        !std::isfinite(image_norm)) {
      return 0.0;
    }
    direction = image / image_norm;
    eigenvalue = direction.dot(H * direction);
  }
  return std::max(0.0, eigenvalue);
}

auto complementarityResidual(const Eigen::VectorXd& force,
                             const Eigen::VectorXd& dual,
                             int contact_count) -> double {
  double residual = 0.0;
  for (int contact = 0; contact < contact_count; ++contact) {
    residual = std::max(
        residual,
        std::abs(force.segment<3>(3 * contact)
                     .dot(dual.segment<3>(3 * contact))));
  }
  return residual;
}

// One iteration kernel for both variants. A null target preserves the paper
// baseline; a non-null target shifts only the NCP's free normal velocity.
auto solveSpectralAdmm(
    sire::Size n, std::vector<double>& fri_coef,
    std::vector<double>& invM_3n, std::vector<double>& v0,
    const std::vector<double>* normal_target, std::vector<double>& b, double h,
    std::vector<double>& contactFce, sire::Size max_iters,
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

  // Algorithm 1 parameters.  eta and the residual ratio come from the paper;
  // p0 and dp match the accompanying Pinocchio reference implementation.
  constexpr double eta = 1e-6;
  constexpr double initial_rho_power = 0.2;
  constexpr double rho_power_increment = 0.05;
  constexpr double residual_ratio = 10.0;

  // In the rigid case R=0, the reference implementation uses m=eta and
  // L=lambda_max(G), then rho=sqrt(mL)*(L/m)^p.
  const double largest_eigenvalue =
      std::max(estimateLargestEigenvalue(G), eta);
  const double smallest_regularized_eigenvalue = eta;
  const double condition_number =
      std::max(1.0, largest_eigenvalue / smallest_regularized_eigenvalue);
  const double rho_multiplier =
      std::pow(condition_number, rho_power_increment);
  double rho =
      std::sqrt(largest_eigenvalue * smallest_regularized_eigenvalue) *
      std::pow(condition_number, initial_rho_power);
  rho = std::max(rho, eta);

  Eigen::VectorXd f = Eigen::VectorXd::Zero(dimension);
  Eigen::VectorXd y = Eigen::VectorXd::Zero(dimension);
  Eigen::VectorXd z = Eigen::VectorXd::Zero(dimension);

  Eigen::MatrixXd augmented_matrix;
  Eigen::LDLT<Eigen::MatrixXd> factorization;
  auto refactor = [&]() {
    augmented_matrix =
        G + (eta + rho) * Eigen::MatrixXd::Identity(dimension, dimension);
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
        rho * y + z - (g + shift) + eta * f_previous;
    if (factorization_ok) {
      f = factorization.solve(rhs);
      factorization_ok = factorization.info() == Eigen::Success;
    }
    if (!factorization_ok || !f.allFinite()) {
      // Numerical fallback only; it does not change the NCP being solved.
      const Eigen::MatrixXd regularized =
          augmented_matrix + 1e-10 *
                                 Eigen::MatrixXd::Identity(dimension,
                                                           dimension);
      f = regularized.ldlt().solve(rhs);
    }

    // Algorithm 1, lines 5-6 / Eqs. (39) and (36c).
    y = projectCoulombCones(f - z / rho, contact_count, fri_coef);
    z -= rho * (f - y);

    // Eqs. (40)-(43): all three absolute residuals are required.
    primal_residual = (f - y).lpNorm<Eigen::Infinity>();
    dual_residual =
        (eta * (f - f_previous) + rho * (y - y_previous))
            .lpNorm<Eigen::Infinity>();
    complementarity_residual =
        complementarityResidual(f, z, contact_count);

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
      factorization_ok = refactor();
      ++factorization_updates;
    }
  }

  // Sire exposes contact forces, whereas Algorithm 1 returns impulses:
  // lambda = h * force.
  Eigen::Map<Eigen::VectorXd> output_force(contactFce.data(), dimension);
  output_force = y / h;

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
    sire::Size n, std::vector<double>& fri_coef,
    std::vector<double>& invM_3n, std::vector<double>& v0,
    std::vector<double>& v_target, std::vector<double>& b, double h,
    std::vector<double>& contactFce, sire::Size max_iters,
    double max_err) -> double {
  (void)v_target;
  return solveSpectralAdmm(n, fri_coef, invM_3n, v0, nullptr, b, h,
                           contactFce, max_iters, max_err);
}

auto cptContactForceShiftedSpectralAdmm(
    sire::Size n, std::vector<double>& fri_coef,
    std::vector<double>& invM_3n, std::vector<double>& v0,
    std::vector<double>& v_target, std::vector<double>& b, double h,
    std::vector<double>& contactFce, sire::Size max_iters,
    double max_err) -> double {
  return solveSpectralAdmm(n, fri_coef, invM_3n, v0, &v_target, b, h,
                           contactFce, max_iters, max_err);
}

auto cptContactForceSimpleAdmm(
    sire::Size n, std::vector<double>& fri_coef,
    std::vector<double>& invM_3n, std::vector<double>& v0,
    std::vector<double>& v_target, std::vector<double>& b, double h,
    std::vector<double>& contactFce, sire::Size max_iters,
    double max_err) -> double {
  return cptContactForceSpectralAdmm(n, fri_coef, invM_3n, v0, v_target, b, h,
                                     contactFce, max_iters, max_err);
}

auto SimpleAdmmContactSolver::solveContactForceQP(
    sire::Size n, std::vector<double>& fri_coef,
    std::vector<double>& invM_3n, std::vector<double>& v0,
    std::vector<double>& v_target, std::vector<double>& b, double h,
    std::vector<double>& contactFce, sire::Size max_iters,
    double max_err) -> double {
  return cptContactForceSpectralAdmm(n, fri_coef, invM_3n, v0, v_target, b, h,
                                     contactFce, max_iters, max_err);
}

ShiftedSpectralAdmmContactSolver::ShiftedSpectralAdmmContactSolver() {
  setContactModelMode("single_point");
}

auto ShiftedSpectralAdmmContactSolver::solveContactForceQP(
    sire::Size n, std::vector<double>& fri_coef,
    std::vector<double>& invM_3n, std::vector<double>& v0,
    std::vector<double>& v_target, std::vector<double>& b, double h,
    std::vector<double>& contactFce, sire::Size max_iters,
    double max_err) -> double {
  return cptContactForceShiftedSpectralAdmm(
      n, fri_coef, invM_3n, v0, v_target, b, h, contactFce, max_iters, max_err);
}

ARIS_REGISTRATION {
  aris::core::class_<SimpleAdmmContactSolver>("SimpleAdmmContactSolver")
      .inherit<ps_vs_solver_v5::PsVsSolverV5>();
  aris::core::class_<ShiftedSpectralAdmmContactSolver>(
      "ShiftedSpectralAdmmContactSolver")
      .inherit<SimpleAdmmContactSolver>();
}

}  // namespace sire::physics::contact::simple_admm
