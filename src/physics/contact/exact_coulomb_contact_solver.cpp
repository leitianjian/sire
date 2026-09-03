#include "sire/physics/contact/exact_coulomb_contact_solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <aris/core/reflection.hpp>
#include <eigen3/Eigen/Dense>

#include "sire/core/profiler.hpp"

namespace sire::physics::contact::exact_coulomb {
namespace {

using MatrixXdRM = Eigen::Matrix<double, -1, -1, Eigen::RowMajor>;

auto projectCone(const Eigen::Vector3d& x, double mu) -> Eigen::Vector3d {
  mu = std::max(0.0, mu);
  if (mu <= std::numeric_limits<double>::epsilon()) {
    return Eigen::Vector3d(0.0, 0.0, std::max(0.0, x(2)));
  }

  const Eigen::Vector2d tangent = x.head<2>();
  const double tangent_norm = tangent.norm();
  if (tangent_norm <= mu * x(2)) return x;
  if (x(2) <= -mu * tangent_norm) return Eigen::Vector3d::Zero();

  const double normal = (mu * tangent_norm + x(2)) / (1.0 + mu * mu);
  Eigen::Vector3d projected;
  if (tangent_norm > 0.0)
    projected.head<2>() = (mu * normal / tangent_norm) * tangent;
  else
    projected.head<2>().setZero();
  projected(2) = normal;
  return projected;
}

auto projectProductCone(const Eigen::VectorXd& x,
                        const std::vector<double>& mu) -> Eigen::VectorXd {
  Eigen::VectorXd projected(x.size());
  for (int i = 0; i < static_cast<int>(mu.size()); ++i)
    projected.segment<3>(3 * i) = projectCone(x.segment<3>(3 * i), mu[i]);
  return projected;
}

auto deSaxceCorrection(const Eigen::VectorXd& shifted_velocity,
                       const std::vector<double>& mu) -> Eigen::VectorXd {
  Eigen::VectorXd correction = Eigen::VectorXd::Zero(shifted_velocity.size());
  for (int i = 0; i < static_cast<int>(mu.size()); ++i) {
    correction(3 * i + 2) =
        std::max(0.0, mu[i]) * shifted_velocity.segment<2>(3 * i).norm();
  }
  return correction;
}

struct ResidualEvaluation {
  Eigen::VectorXd value;
  Eigen::MatrixXd jacobian;
  double normalized_norm{std::numeric_limits<double>::infinity()};
  double merit{std::numeric_limits<double>::infinity()};
};

auto evaluatePhysicalResidual(const Eigen::MatrixXd& H,
                              const Eigen::VectorXd& shifted_free_velocity,
                              const Eigen::VectorXd& f,
                              const std::vector<double>& mu,
                              const Eigen::VectorXd& rho_normal,
                              const Eigen::VectorXd& rho_tangent,
                              bool with_jacobian) -> ResidualEvaluation {
  const int dim = static_cast<int>(f.size());
  ResidualEvaluation result;
  result.value = Eigen::VectorXd::Zero(dim);
  if (with_jacobian) result.jacobian = Eigen::MatrixXd::Zero(dim, dim);

  const Eigen::VectorXd velocity = H * f + shifted_free_velocity;
  double maximum = 0.0;
  double merit = 0.0;

  for (int i = 0; i < static_cast<int>(mu.size()); ++i) {
    const int tangent_index = 3 * i;
    const int normal_index = tangent_index + 2;
    const double friction = std::max(0.0, mu[i]);
    const double force_normal = f(normal_index);
    const double velocity_normal = velocity(normal_index);
    const Eigen::Vector2d force_tangent = f.segment<2>(tangent_index);
    const Eigen::Vector2d velocity_tangent =
        velocity.segment<2>(tangent_index);

    // 0 <= f_N perpendicular to w_N >= 0, written as a projection equation.
    const double z_normal =
        force_normal - rho_normal(i) * velocity_normal;
    result.value(normal_index) = force_normal - std::max(0.0, z_normal);

    if (with_jacobian) {
      double projection_derivative = 0.5;
      if (z_normal > 0.0) projection_derivative = 1.0;
      if (z_normal < 0.0) projection_derivative = 0.0;
      result.jacobian.row(normal_index) =
          projection_derivative * rho_normal(i) * H.row(normal_index);
      result.jacobian(normal_index, normal_index) +=
          1.0 - projection_derivative;
    }

    // -w_T belongs to the normal cone of ||f_T|| <= mu f_N.
    const Eigen::Vector2d disk_argument =
        force_tangent - rho_tangent(i) * velocity_tangent;
    const double argument_norm = disk_argument.norm();
    const double radius = friction * std::max(0.0, force_normal);
    Eigen::Vector2d disk_projection;
    Eigen::Matrix2d projection_argument_jacobian = Eigen::Matrix2d::Zero();
    Eigen::Vector2d projection_radius_jacobian = Eigen::Vector2d::Zero();

    if (radius > 0.0 && argument_norm < radius) {
      disk_projection = disk_argument;
      projection_argument_jacobian.setIdentity();
    } else if (argument_norm > 0.0) {
      const Eigen::Vector2d direction = disk_argument / argument_norm;
      disk_projection = radius * direction;
      projection_argument_jacobian =
          (radius / argument_norm) *
          (Eigen::Matrix2d::Identity() - direction * direction.transpose());
      projection_radius_jacobian = direction;
    } else {
      disk_projection.setZero();
    }
    result.value.segment<2>(tangent_index) =
        force_tangent - disk_projection;

    if (with_jacobian) {
      Eigen::MatrixXd tangent_selector = Eigen::MatrixXd::Zero(2, dim);
      tangent_selector.block<2, 2>(0, tangent_index).setIdentity();
      const Eigen::MatrixXd disk_argument_jacobian =
          tangent_selector - rho_tangent(i) * H.middleRows(tangent_index, 2);
      Eigen::RowVectorXd radius_jacobian = Eigen::RowVectorXd::Zero(dim);
      const double positive_part_derivative =
          force_normal > 0.0 ? 1.0 : (force_normal < 0.0 ? 0.0 : 0.5);
      radius_jacobian(normal_index) =
          friction * positive_part_derivative;
      result.jacobian.middleRows(tangent_index, 2) =
          tangent_selector -
          projection_argument_jacobian * disk_argument_jacobian -
          projection_radius_jacobian * radius_jacobian;
    }

    const double scale =
        1.0 + std::max({std::abs(force_normal), force_tangent.norm(),
                        rho_normal(i) * std::abs(velocity_normal),
                        rho_tangent(i) * velocity_tangent.norm()});
    const double block_normalized =
        result.value.segment<3>(tangent_index).norm() / scale;
    maximum = std::max(maximum, block_normalized);
    merit += block_normalized * block_normalized;
  }

  result.normalized_norm = maximum;
  result.merit = 0.5 * merit;
  return result;
}

auto solveFrozenCorrectionQp(const Eigen::MatrixXd& H,
                             const Eigen::VectorXd& linear_term,
                             const std::vector<double>& mu,
                             const Eigen::VectorXd& initial,
                             double gamma, double tolerance)
    -> Eigen::VectorXd {
  const int dim = static_cast<int>(initial.size());
  const Eigen::MatrixXd regularized_hessian =
      H + (1.0 / gamma) * Eigen::MatrixXd::Identity(dim, dim);
  const double alpha = 0.95 / std::max(regularized_hessian.norm(), 1e-12);
  Eigen::VectorXd x = projectProductCone(initial, mu);

  // This is the set-only specialization of PIPG.  The proximal term makes the
  // frozen De Saxce subproblem strongly convex even for rank-deficient H.
  constexpr int kMaximumInnerIterations = 30;
  for (int iteration = 0; iteration < kMaximumInnerIterations; ++iteration) {
    const Eigen::VectorXd next = projectProductCone(
        x - alpha * (regularized_hessian * x + linear_term), mu);
    const double fixed_point_error =
        (next - x).lpNorm<Eigen::Infinity>() /
        std::max(1.0, x.lpNorm<Eigen::Infinity>());
    x = next;
    if (fixed_point_error <= tolerance) break;
  }
  return x;
}

auto forwardBackwardForwardStep(
    const Eigen::MatrixXd& H, const Eigen::VectorXd& shifted_free_velocity,
    const std::vector<double>& mu, const Eigen::VectorXd& f, double gamma,
    double inner_tolerance) -> Eigen::VectorXd {
  const Eigen::VectorXd velocity = H * f + shifted_free_velocity;
  const Eigen::VectorXd correction = deSaxceCorrection(velocity, mu);
  const Eigen::VectorXd linear_term =
      shifted_free_velocity + correction - f / gamma;
  const Eigen::VectorXd proximal_point = solveFrozenCorrectionQp(
      H, linear_term, mu, f, gamma, inner_tolerance);

  const Eigen::VectorXd proximal_velocity =
      H * proximal_point + shifted_free_velocity;
  const Eigen::VectorXd proximal_correction =
      deSaxceCorrection(proximal_velocity, mu);
  return proximal_point - gamma * (proximal_correction - correction);
}

auto tryMixedSemismoothNewton(
    const Eigen::MatrixXd& H, const Eigen::VectorXd& shifted_free_velocity,
    const std::vector<double>& mu, const Eigen::VectorXd& rho_normal,
    const Eigen::VectorXd& rho_tangent, const Eigen::VectorXd& f,
    Eigen::VectorXd& accepted) -> bool {
  const auto current = evaluatePhysicalResidual(
      H, shifted_free_velocity, f, mu, rho_normal, rho_tangent, true);
  if (!std::isfinite(current.merit) || current.merit == 0.0) return false;

  const Eigen::MatrixXd normal_matrix =
      current.jacobian.transpose() * current.jacobian;
  const Eigen::VectorXd right_hand_side =
      -current.jacobian.transpose() * current.value;
  const double matrix_scale =
      std::max(1.0, normal_matrix.diagonal().cwiseAbs().maxCoeff());
  double regularization = 1e-10 * matrix_scale;

  for (int regularization_attempt = 0; regularization_attempt < 5;
       ++regularization_attempt) {
    Eigen::MatrixXd system = normal_matrix;
    system.diagonal().array() += regularization;
    const Eigen::VectorXd step = system.ldlt().solve(right_hand_side);
    if (step.allFinite()) {
      for (double damping = 1.0; damping >= 1.0 / 128.0; damping *= 0.5) {
        const Eigen::VectorXd trial_f = f + damping * step;
        const auto trial = evaluatePhysicalResidual(
            H, shifted_free_velocity, trial_f, mu, rho_normal, rho_tangent,
            false);
        if (std::isfinite(trial.merit) &&
            trial.merit <= (1.0 - 1e-4 * damping) * current.merit) {
          accepted = trial_f;
          return true;
        }
      }
    }
    regularization *= 10.0;
  }
  return false;
}

enum class PredictedContactMode : std::uint8_t {
  kOpening = 0,
  kSticking = 1,
  kSliding = 2,
  kFrictionless = 3,
  kUncertain = 4,
};

auto predictorStepSizes(const Eigen::MatrixXd& H,
                        const std::vector<double>& mu, double omega)
    -> Eigen::VectorXd {
  const int contact_count = static_cast<int>(mu.size());
  const double global_scale = std::max(H.norm(), 1e-12);
  Eigen::VectorXd steps(contact_count);
  for (int i = 0; i < contact_count; ++i) {
    double block_row_bound = 0.0;
    for (int j = 0; j < contact_count; ++j)
      block_row_bound += H.block<3, 3>(3 * i, 3 * j).norm();
    const double local_bound =
        (1.0 + std::max(0.0, mu[i])) * block_row_bound;
    steps(i) = omega /
               std::max(local_bound, 1e-6 * global_scale);
  }
  return steps;
}

auto blockProjectedPredictorSweep(
    const Eigen::MatrixXd& H,
    const Eigen::VectorXd& shifted_free_velocity,
    const std::vector<double>& mu, const Eigen::VectorXd& step_sizes,
    double relaxation, bool reverse, Eigen::VectorXd& f) -> bool {
  const int contact_count = static_cast<int>(mu.size());
  Eigen::VectorXd velocity = H * f + shifted_free_velocity;
  for (int sweep = 0; sweep < contact_count; ++sweep) {
    const int i = reverse ? contact_count - 1 - sweep : sweep;
    const int index = 3 * i;
    Eigen::Vector3d contact_operator = velocity.segment<3>(index);
    contact_operator(2) +=
        std::max(0.0, mu[i]) * contact_operator.head<2>().norm();

    const Eigen::Vector3d old_force = f.segment<3>(index);
    const Eigen::Vector3d projected = projectCone(
        old_force - step_sizes(i) * contact_operator, mu[i]);
    const Eigen::Vector3d delta =
        relaxation * (projected - old_force);
    if (!delta.allFinite()) return false;

    f.segment<3>(index) += delta;
    velocity.noalias() += H.middleCols(index, 3) * delta;
  }
  return f.allFinite();
}

auto classifyContactModes(const Eigen::MatrixXd& H,
                          const Eigen::VectorXd& shifted_free_velocity,
                          const Eigen::VectorXd& f,
                          const std::vector<double>& mu)
    -> std::vector<PredictedContactMode> {
  constexpr double kForceTolerance = 1e-6;
  // This classifier gates basin entry rather than certifying convergence.
  // A deliberately broad velocity margin lets it recognize a stable rough
  // mode while the exact natural residual remains the final authority.
  constexpr double kVelocityTolerance = 5e-2;
  constexpr double kConeMargin = 1e-2;
  constexpr double kAlignmentTolerance = 1e-2;
  const Eigen::VectorXd velocity = H * f + shifted_free_velocity;
  std::vector<PredictedContactMode> modes(mu.size(),
                                          PredictedContactMode::kUncertain);

  for (int i = 0; i < static_cast<int>(mu.size()); ++i) {
    const int index = 3 * i;
    const Eigen::Vector2d tangent_force = f.segment<2>(index);
    const Eigen::Vector2d tangent_velocity = velocity.segment<2>(index);
    const double normal_force = f(index + 2);
    const double normal_velocity = velocity(index + 2);
    const double friction = std::max(0.0, mu[i]);
    const double tangent_force_norm = tangent_force.norm();
    const double tangent_velocity_norm = tangent_velocity.norm();
    const double force_scale =
        1.0 + std::max(std::abs(normal_force), tangent_force_norm);
    const double velocity_scale =
        1.0 + std::max(std::abs(normal_velocity), tangent_velocity_norm);
    const double force_tolerance = kForceTolerance * force_scale;
    const double velocity_tolerance = kVelocityTolerance * velocity_scale;

    if (normal_force <= force_tolerance &&
        normal_velocity >= -velocity_tolerance) {
      modes[i] = PredictedContactMode::kOpening;
      continue;
    }

    const bool active_normal = normal_force > force_tolerance &&
                               std::abs(normal_velocity) <= velocity_tolerance;
    const double cone_radius = friction * std::max(0.0, normal_force);
    const double cone_tolerance =
        force_tolerance + kConeMargin * cone_radius;
    if (active_normal && friction <= std::numeric_limits<double>::epsilon() &&
        tangent_force_norm <= force_tolerance) {
      modes[i] = PredictedContactMode::kFrictionless;
      continue;
    }
    if (active_normal && tangent_velocity_norm <= velocity_tolerance &&
        tangent_force_norm + cone_tolerance < cone_radius) {
      modes[i] = PredictedContactMode::kSticking;
      continue;
    }

    if (active_normal && tangent_velocity_norm > velocity_tolerance &&
        std::abs(tangent_force_norm - cone_radius) <= cone_tolerance &&
        tangent_force_norm > force_tolerance) {
      const double alignment =
          -tangent_force.dot(tangent_velocity) /
          std::max(tangent_force_norm * tangent_velocity_norm, 1e-30);
      if (alignment >= 1.0 - kAlignmentTolerance) {
        modes[i] = PredictedContactMode::kSliding;
        continue;
      }
    }
  }
  return modes;
}

auto containsUncertainMode(const std::vector<PredictedContactMode>& modes)
    -> bool {
  return std::find(modes.begin(), modes.end(),
                   PredictedContactMode::kUncertain) != modes.end();
}

}  // namespace

auto cptContactForceExactCoulombWithOptions(
    sire::Size n, std::vector<double>& fri_coef,
    std::vector<double>& invM_3n, std::vector<double>& v0,
    std::vector<double>& v_target, std::vector<double>& b, double h,
    std::vector<double>& contactFce, sire::Size max_iters,
    double max_err, const ExactCoulombOptions& options,
    ExactCoulombStatistics* statistics) -> double {
  SIRE_PROFILE_FUNCTION();
  ExactCoulombStatistics local_statistics;
  ExactCoulombStatistics& stats = statistics != nullptr
                                      ? *statistics
                                      : local_statistics;
  stats = ExactCoulombStatistics{};
  const int contact_count = static_cast<int>(n);
  const int dim = 3 * contact_count;
  if (contact_count == 0) {
    std::fill(contactFce.begin(), contactFce.end(), 0.0);
    stats.initial_residual = 0.0;
    stats.final_residual = 0.0;
    stats.converged = true;
    return 0.0;
  }
  if (fri_coef.size() < n ||
      invM_3n.size() < static_cast<size_t>(dim * dim) ||
      v0.size() < static_cast<size_t>(dim) ||
      v_target.size() < n || b.size() < static_cast<size_t>(dim) ||
      contactFce.size() < static_cast<size_t>(dim) || h <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }

  const Eigen::Map<const MatrixXdRM> W(invM_3n.data(), dim, dim);
  const Eigen::Map<const Eigen::VectorXd> v0_map(v0.data(), dim);
  const Eigen::Map<const Eigen::VectorXd> b_map(b.data(), dim);
  Eigen::MatrixXd H = -h * W;
  H = 0.5 * (H + H.transpose());

  // c = H f + g is the post-step contact velocity.  PsVsSolverV5 stores the
  // desired normal velocity as -v_target, hence w_N = c_N + v_target.
  Eigen::VectorXd shifted_free_velocity = v0_map - h * b_map;
  for (int i = 0; i < contact_count; ++i)
    shifted_free_velocity(3 * i + 2) += v_target[i];

  const double hessian_scale = std::max(H.norm(), 1e-12);
  const double base_rho = 1.0 / hessian_scale;
  Eigen::VectorXd rho_normal(contact_count);
  Eigen::VectorXd rho_tangent(contact_count);
  for (int i = 0; i < contact_count; ++i) {
    const int tangent_index = 3 * i;
    const int normal_index = tangent_index + 2;
    const double normal_curvature = std::max(
        std::abs(H(normal_index, normal_index)), 1e-6 * hessian_scale);
    const double tangent_curvature =
        std::max(0.5 * (std::abs(H(tangent_index, tangent_index)) +
                        std::abs(H(tangent_index + 1, tangent_index + 1))),
                 1e-6 * hessian_scale);
    rho_normal(i) = std::clamp(1.0 / normal_curvature,
                               1e-3 * base_rho, 1e3 * base_rho);
    rho_tangent(i) = std::clamp(1.0 / tangent_curvature,
                                1e-3 * base_rho, 1e3 * base_rho);
  }

  Eigen::VectorXd f = Eigen::Map<const Eigen::VectorXd>(contactFce.data(), dim);
  if (!f.allFinite()) f.setZero();
  f = projectProductCone(f, fri_coef);

  const double maximum_friction =
      std::max(1.0, *std::max_element(fri_coef.begin(),
                                     fri_coef.begin() + contact_count));
  const double maximum_gamma = 1.0 / (maximum_friction * hessian_scale);
  double gamma = 0.5 * maximum_gamma;
  const double inner_tolerance = std::max(1e-10, 0.1 * max_err);

  for (int iteration = 0; iteration < static_cast<int>(max_iters);
       ++iteration) {
    const auto current = evaluatePhysicalResidual(
        H, shifted_free_velocity, f, fri_coef, rho_normal, rho_tangent, false);
    if (iteration == 0) stats.initial_residual = current.normalized_norm;
    if (options.collect_history) {
      stats.residual_history.push_back(current.normalized_norm);
      stats.step_history.push_back(ExactCoulombStep::kInitial);
    }
    if (current.normalized_norm <= max_err) {
      stats.converged = true;
      break;
    }

    const bool newton_only =
        options.mode == ExactCoulombMode::kSemismoothNewtonOnly;
    const bool try_newton =
        newton_only ||
        (options.mode == ExactCoulombMode::kHybrid &&
         static_cast<sire::Size>(iteration) >=
             options.newton_start_iteration &&
         current.normalized_norm <= options.newton_switch_residual);
    if (try_newton) {
      ++stats.newton_attempts;
      Eigen::VectorXd newton_candidate;
      if (tryMixedSemismoothNewton(
              H, shifted_free_velocity, fri_coef, rho_normal, rho_tangent, f,
              newton_candidate)) {
        f = newton_candidate;
        ++stats.newton_accepted;
        ++stats.nonlinear_iterations;
        if (options.collect_history)
          stats.step_history.back() = ExactCoulombStep::kNewton;
        continue;
      }

      ++stats.newton_rejected;
      if (options.collect_history)
        stats.step_history.back() = ExactCoulombStep::kRejectedNewton;
      if (newton_only) {
        stats.terminated_on_newton_rejection = true;
        break;
      }
    }

    // FBF-only always takes this path.  Hybrid reaches it before entering the
    // local Newton basin and whenever the Newton merit line search rejects a
    // local model.  Newton-only never receives this fallback.
    Eigen::VectorXd best_candidate = f;
    double best_merit = std::numeric_limits<double>::infinity();
    double trial_gamma = gamma;
    for (int safeguard = 0; safeguard < 4; ++safeguard) {
      const Eigen::VectorXd candidate = forwardBackwardForwardStep(
          H, shifted_free_velocity, fri_coef, f, trial_gamma,
          inner_tolerance);
      const auto candidate_residual = evaluatePhysicalResidual(
          H, shifted_free_velocity, candidate, fri_coef, rho_normal,
          rho_tangent, false);
      if (candidate_residual.merit < best_merit) {
        best_merit = candidate_residual.merit;
        best_candidate = candidate;
        gamma = trial_gamma;
      }
      if (candidate_residual.merit <= 1.25 * current.merit) break;
      trial_gamma *= 0.5;
      ++stats.fbf_backtracks;
    }
    f = best_candidate;
    gamma = std::min(maximum_gamma, 1.05 * gamma);
    ++stats.fbf_steps;
    ++stats.nonlinear_iterations;
    if (options.collect_history)
      stats.step_history.back() = ExactCoulombStep::kFbf;
  }

  // The residual extension permits temporary infeasible Newton iterates.  The
  // public result and its reported error always correspond to a cone-feasible
  // force, including when the iteration budget is exhausted.
  f = projectProductCone(f, fri_coef);
  const auto final_residual = evaluatePhysicalResidual(
      H, shifted_free_velocity, f, fri_coef, rho_normal, rho_tangent, false);
  Eigen::Map<Eigen::VectorXd>(contactFce.data(), dim) = f;
  if (!std::isfinite(stats.initial_residual))
    stats.initial_residual = final_residual.normalized_norm;
  stats.final_residual = final_residual.normalized_norm;
  stats.converged = final_residual.normalized_norm <= max_err;
  if (options.collect_history) {
    stats.residual_history.push_back(final_residual.normalized_norm);
    stats.step_history.push_back(ExactCoulombStep::kInitial);
  }

  SIRE_PROFILE_PLOT("exact_coulomb.residual", final_residual.normalized_norm);
  SIRE_PROFILE_PLOT("exact_coulomb.newton_steps",
                    static_cast<double>(stats.newton_accepted));
  SIRE_PROFILE_PLOT("exact_coulomb.fallback_steps",
                    static_cast<double>(stats.fbf_steps));
  return final_residual.normalized_norm;
}

auto cptContactForceExactCoulomb(
    sire::Size n, std::vector<double>& fri_coef,
    std::vector<double>& invM_3n, std::vector<double>& v0,
    std::vector<double>& v_target, std::vector<double>& b, double h,
    std::vector<double>& contactFce, sire::Size max_iters,
    double max_err) -> double {
  ExactCoulombOptions options;
  options.mode = ExactCoulombMode::kHybrid;
  return cptContactForceExactCoulombWithOptions(
      n, fri_coef, invM_3n, v0, v_target, b, h, contactFce, max_iters,
      max_err, options);
}

auto cptContactForceExactCoulombFbf(
    sire::Size n, std::vector<double>& fri_coef,
    std::vector<double>& invM_3n, std::vector<double>& v0,
    std::vector<double>& v_target, std::vector<double>& b, double h,
    std::vector<double>& contactFce, sire::Size max_iters,
    double max_err) -> double {
  ExactCoulombOptions options;
  options.mode = ExactCoulombMode::kFbfOnly;
  return cptContactForceExactCoulombWithOptions(
      n, fri_coef, invM_3n, v0, v_target, b, h, contactFce, max_iters,
      max_err, options);
}

auto cptContactForceExactCoulombSemismoothNewton(
    sire::Size n, std::vector<double>& fri_coef,
    std::vector<double>& invM_3n, std::vector<double>& v0,
    std::vector<double>& v_target, std::vector<double>& b, double h,
    std::vector<double>& contactFce, sire::Size max_iters,
    double max_err) -> double {
  ExactCoulombOptions options;
  options.mode = ExactCoulombMode::kSemismoothNewtonOnly;
  options.newton_start_iteration = 0;
  return cptContactForceExactCoulombWithOptions(
      n, fri_coef, invM_3n, v0, v_target, b, h, contactFce, max_iters,
      max_err, options);
}

auto cptContactForceSingleLoopFbfSsn(
    sire::Size n, std::vector<double>& fri_coef,
    std::vector<double>& invM_3n, std::vector<double>& v0,
    std::vector<double>& v_target, std::vector<double>& b, double h,
    std::vector<double>& contactFce, sire::Size max_iters,
    double max_err) -> double {
  SIRE_PROFILE_FUNCTION();
  const int contact_count = static_cast<int>(n);
  const int dim = 3 * contact_count;
  if (contact_count == 0) {
    std::fill(contactFce.begin(), contactFce.end(), 0.0);
    return 0.0;
  }
  if (fri_coef.size() < n ||
      invM_3n.size() < static_cast<size_t>(dim * dim) ||
      v0.size() < static_cast<size_t>(dim) ||
      v_target.size() < n || b.size() < static_cast<size_t>(dim) ||
      contactFce.size() < static_cast<size_t>(dim) || h <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }

  const Eigen::Map<const MatrixXdRM> W(invM_3n.data(), dim, dim);
  const Eigen::Map<const Eigen::VectorXd> v0_map(v0.data(), dim);
  const Eigen::Map<const Eigen::VectorXd> b_map(b.data(), dim);
  Eigen::MatrixXd H = -h * W;
  H = 0.5 * (H + H.transpose());

  Eigen::VectorXd shifted_free_velocity = v0_map - h * b_map;
  for (int i = 0; i < contact_count; ++i)
    shifted_free_velocity(3 * i + 2) += v_target[i];

  const double hessian_scale = std::max(H.norm(), 1e-12);
  const double base_rho = 1.0 / hessian_scale;
  Eigen::VectorXd rho_normal(contact_count);
  Eigen::VectorXd rho_tangent(contact_count);
  for (int i = 0; i < contact_count; ++i) {
    const int tangent_index = 3 * i;
    const int normal_index = tangent_index + 2;
    const double normal_curvature = std::max(
        std::abs(H(normal_index, normal_index)), 1e-6 * hessian_scale);
    const double tangent_curvature =
        std::max(0.5 * (std::abs(H(tangent_index, tangent_index)) +
                        std::abs(H(tangent_index + 1, tangent_index + 1))),
                 1e-6 * hessian_scale);
    rho_normal(i) = std::clamp(1.0 / normal_curvature,
                               1e-3 * base_rho, 1e3 * base_rho);
    rho_tangent(i) = std::clamp(1.0 / tangent_curvature,
                                1e-3 * base_rho, 1e3 * base_rho);
  }

  Eigen::VectorXd initial =
      Eigen::Map<const Eigen::VectorXd>(contactFce.data(), dim);
  if (!initial.allFinite()) initial.setZero();
  initial = projectProductCone(initial, fri_coef);

  // u is Tseng's outer FBF state. p is the persistent approximation of
  // J_{gamma A}(u - gamma B(u)). Unlike the nested solver, p is not reset and
  // the frozen-correction QP is not solved to completion on every iteration.
  Eigen::VectorXd outer_state = initial;
  Eigen::VectorXd prox_state = initial;

  const double maximum_friction =
      std::max(1.0, *std::max_element(fri_coef.begin(),
                                     fri_coef.begin() + contact_count));
  const double maximum_gamma = 1.0 / (maximum_friction * hessian_scale);
  double gamma = 0.5 * maximum_gamma;
  sire::Size newton_attempts = 0;
  sire::Size newton_accepted = 0;
  sire::Size tracked_fbf_steps = 0;
  sire::Size backtracks = 0;

  for (sire::Size iteration = 0; iteration < max_iters; ++iteration) {
    const auto current = evaluatePhysicalResidual(
        H, shifted_free_velocity, prox_state, fri_coef, rho_normal,
        rho_tangent, false);
    if (current.normalized_norm <= max_err) break;

    // Use the same opportunistic mixed SSN accelerator as Exact-Coulomb.
    // Accepted Newton iterates restart both coupled single-loop states.
    if (iteration >= 2) {
      ++newton_attempts;
      Eigen::VectorXd newton_candidate;
      if (tryMixedSemismoothNewton(
              H, shifted_free_velocity, fri_coef, rho_normal, rho_tangent,
              prox_state, newton_candidate)) {
        prox_state = newton_candidate;
        outer_state = newton_candidate;
        ++newton_accepted;
        continue;
      }
    }

    const Eigen::VectorXd outer_velocity =
        H * outer_state + shifted_free_velocity;
    const Eigen::VectorXd outer_correction =
        deSaxceCorrection(outer_velocity, fri_coef);

    Eigen::VectorXd best_prox = prox_state;
    Eigen::VectorXd best_outer = outer_state;
    double best_merit = std::numeric_limits<double>::infinity();
    double best_gamma = gamma;
    double trial_gamma = gamma;

    for (int safeguard = 0; safeguard < 4; ++safeguard) {
      const Eigen::MatrixXd regularized_hessian =
          H + (1.0 / trial_gamma) *
                  Eigen::MatrixXd::Identity(dim, dim);
      const double alpha =
          0.95 / std::max(regularized_hessian.norm(), 1e-12);
      const Eigen::VectorXd linear_term =
          shifted_free_velocity + outer_correction -
          outer_state / trial_gamma;

      // Exactly one persistent projected-gradient update of the frozen-
      // correction resolvent. The next nonlinear iteration continues from
      // this prox state instead of restarting an inner loop.
      const Eigen::VectorXd prox_candidate = projectProductCone(
          prox_state -
              alpha * (regularized_hessian * prox_state + linear_term),
          fri_coef);
      const Eigen::VectorXd prox_velocity =
          H * prox_candidate + shifted_free_velocity;
      const Eigen::VectorXd prox_correction =
          deSaxceCorrection(prox_velocity, fri_coef);
      const Eigen::VectorXd outer_candidate =
          prox_candidate -
          trial_gamma * (prox_correction - outer_correction);

      const auto candidate_residual = evaluatePhysicalResidual(
          H, shifted_free_velocity, prox_candidate, fri_coef, rho_normal,
          rho_tangent, false);
      if (outer_candidate.allFinite() &&
          std::isfinite(candidate_residual.merit) &&
          candidate_residual.merit < best_merit) {
        best_merit = candidate_residual.merit;
        best_prox = prox_candidate;
        best_outer = outer_candidate;
        best_gamma = trial_gamma;
      }
      if (outer_candidate.allFinite() &&
          candidate_residual.merit <= 1.25 * current.merit) {
        break;
      }
      trial_gamma *= 0.5;
      ++backtracks;
    }

    prox_state = best_prox;
    outer_state = best_outer;
    gamma = std::min(maximum_gamma, 1.05 * best_gamma);
    ++tracked_fbf_steps;
  }

  // The prox state is the physical solution of the coupled fixed-point
  // system. The outer state is only Tseng's extrapolated tracking variable.
  Eigen::VectorXd output = projectProductCone(prox_state, fri_coef);
  const auto final_residual = evaluatePhysicalResidual(
      H, shifted_free_velocity, output, fri_coef, rho_normal, rho_tangent,
      false);
  Eigen::Map<Eigen::VectorXd>(contactFce.data(), dim) = output;

  SIRE_PROFILE_PLOT("single_loop_fbf_ssn.residual",
                    final_residual.normalized_norm);
  SIRE_PROFILE_PLOT("single_loop_fbf_ssn.newton_attempts",
                    static_cast<double>(newton_attempts));
  SIRE_PROFILE_PLOT("single_loop_fbf_ssn.newton_accepted",
                    static_cast<double>(newton_accepted));
  SIRE_PROFILE_PLOT("single_loop_fbf_ssn.fbf_steps",
                    static_cast<double>(tracked_fbf_steps));
  SIRE_PROFILE_PLOT("single_loop_fbf_ssn.backtracks",
                    static_cast<double>(backtracks));
  return final_residual.normalized_norm;
}

auto cptContactForceExplicitFbfGatedSsn(
    sire::Size n, std::vector<double>& fri_coef,
    std::vector<double>& invM_3n, std::vector<double>& v0,
    std::vector<double>& v_target, std::vector<double>& b, double h,
    std::vector<double>& contactFce, sire::Size max_iters,
    double max_err) -> double {
  SIRE_PROFILE_FUNCTION();
  const int contact_count = static_cast<int>(n);
  const int dim = 3 * contact_count;
  if (contact_count == 0) {
    std::fill(contactFce.begin(), contactFce.end(), 0.0);
    return 0.0;
  }
  if (fri_coef.size() < n ||
      invM_3n.size() < static_cast<size_t>(dim * dim) ||
      v0.size() < static_cast<size_t>(dim) ||
      v_target.size() < n || b.size() < static_cast<size_t>(dim) ||
      contactFce.size() < static_cast<size_t>(dim) || h <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }

  const Eigen::Map<const MatrixXdRM> W(invM_3n.data(), dim, dim);
  const Eigen::Map<const Eigen::VectorXd> v0_map(v0.data(), dim);
  const Eigen::Map<const Eigen::VectorXd> b_map(b.data(), dim);
  Eigen::MatrixXd H = -h * W;
  H = 0.5 * (H + H.transpose());

  Eigen::VectorXd shifted_free_velocity = v0_map - h * b_map;
  for (int i = 0; i < contact_count; ++i)
    shifted_free_velocity(3 * i + 2) += v_target[i];

  const double hessian_scale = std::max(H.norm(), 1e-12);
  const double base_rho = 1.0 / hessian_scale;
  Eigen::VectorXd rho_normal(contact_count);
  Eigen::VectorXd rho_tangent(contact_count);
  for (int i = 0; i < contact_count; ++i) {
    const int tangent_index = 3 * i;
    const int normal_index = tangent_index + 2;
    const double normal_curvature = std::max(
        std::abs(H(normal_index, normal_index)), 1e-6 * hessian_scale);
    const double tangent_curvature =
        std::max(0.5 * (std::abs(H(tangent_index, tangent_index)) +
                        std::abs(H(tangent_index + 1, tangent_index + 1))),
                 1e-6 * hessian_scale);
    rho_normal(i) = std::clamp(1.0 / normal_curvature,
                               1e-3 * base_rho, 1e3 * base_rho);
    rho_tangent(i) = std::clamp(1.0 / tangent_curvature,
                                1e-3 * base_rho, 1e3 * base_rho);
  }

  Eigen::VectorXd f = Eigen::Map<const Eigen::VectorXd>(contactFce.data(), dim);
  if (!f.allFinite()) f.setZero();
  f = projectProductCone(f, fri_coef);

  // With A = N_K, B contains both the affine contact velocity and de Saxce's
  // nonlinear correction. J_{gamma A} is the analytic product-cone
  // projection, so one FBF iteration has no nested frozen-QP solve.
  const auto evaluate_explicit_operator =
      [&](const Eigen::VectorXd& force) -> Eigen::VectorXd {
    const Eigen::VectorXd velocity = H * force + shifted_free_velocity;
    return velocity + deSaxceCorrection(velocity, fri_coef);
  };

  const double maximum_friction =
      std::max(0.0, *std::max_element(fri_coef.begin(),
                                     fri_coef.begin() + contact_count));
  // For symmetric H, the maximum absolute row sum bounds ||H||_2. The
  // de Saxce map has a conservative (1 + mu_max) composition bound.
  const double hessian_bound = std::max(
      H.cwiseAbs().rowwise().sum().maxCoeff(), 1e-12);
  const double operator_bound = (1.0 + maximum_friction) * hessian_bound;
  const double maximum_gamma = 0.95 / operator_bound;
  double gamma = 0.5 * maximum_gamma;

  constexpr double kNewtonSwitchResidual = 5e-2;
  constexpr sire::Size kNewtonStartIteration = 2;
  constexpr sire::Size kNewtonRejectionCooldown = 5;
  constexpr int kMaximumFbfBacktracks = 8;
  constexpr double kFbfLineSearchEta = 0.95;
  sire::Size newton_cooldown = 0;
  sire::Size newton_attempts = 0;
  sire::Size newton_accepted = 0;
  sire::Size newton_rejected = 0;
  sire::Size explicit_fbf_steps = 0;
  sire::Size fbf_backtracks = 0;

  for (sire::Size iteration = 0; iteration < max_iters; ++iteration) {
    const auto current = evaluatePhysicalResidual(
        H, shifted_free_velocity, f, fri_coef, rho_normal, rho_tangent, false);
    if (current.normalized_norm <= max_err) break;

    const bool newton_gate_open =
        iteration >= kNewtonStartIteration && newton_cooldown == 0 &&
        current.normalized_norm <= kNewtonSwitchResidual;
    if (newton_gate_open) {
      ++newton_attempts;
      Eigen::VectorXd newton_candidate;
      if (tryMixedSemismoothNewton(
              H, shifted_free_velocity, fri_coef, rho_normal, rho_tangent, f,
              newton_candidate)) {
        f = newton_candidate;
        ++newton_accepted;
        continue;
      }
      ++newton_rejected;
      newton_cooldown = kNewtonRejectionCooldown;
    }

    const Eigen::VectorXd operator_at_f = evaluate_explicit_operator(f);
    Eigen::VectorXd accepted_candidate = f;
    double accepted_gamma = gamma;
    bool accepted_fbf = false;
    double trial_gamma = gamma;
    for (int backtrack = 0; backtrack < kMaximumFbfBacktracks;
         ++backtrack) {
      const Eigen::VectorXd proximal_point =
          projectProductCone(f - trial_gamma * operator_at_f, fri_coef);
      const Eigen::VectorXd operator_at_proximal =
          evaluate_explicit_operator(proximal_point);

      // Tseng's local Lipschitz test. It directly safeguards the explicit B
      // evaluation and is independent of the physical-residual merit scale.
      const Eigen::VectorXd proximal_delta = proximal_point - f;
      const Eigen::VectorXd operator_delta =
          operator_at_proximal - operator_at_f;
      const bool line_search_ok =
          trial_gamma * operator_delta.norm() <=
          kFbfLineSearchEta * proximal_delta.norm() + 1e-14;
      if (line_search_ok) {
        const Eigen::VectorXd candidate =
            proximal_point - trial_gamma * operator_delta;
        if (candidate.allFinite()) {
          accepted_candidate = candidate;
          accepted_gamma = trial_gamma;
          accepted_fbf = true;
          break;
        }
      }
      trial_gamma *= 0.5;
      ++fbf_backtracks;
    }

    if (!accepted_fbf) {
      // The last-resort projected forward step remains finite and feasible;
      // normally the local Lipschitz backtracking accepts before this path.
      accepted_candidate = projectProductCone(
          f - trial_gamma * operator_at_f, fri_coef);
      accepted_gamma = trial_gamma;
    }
    f = accepted_candidate;
    gamma = std::min(maximum_gamma, 1.05 * accepted_gamma);
    if (newton_cooldown > 0) --newton_cooldown;
    ++explicit_fbf_steps;
  }

  f = projectProductCone(f, fri_coef);
  const auto final_residual = evaluatePhysicalResidual(
      H, shifted_free_velocity, f, fri_coef, rho_normal, rho_tangent, false);
  Eigen::Map<Eigen::VectorXd>(contactFce.data(), dim) = f;

  SIRE_PROFILE_PLOT("explicit_fbf_gated_ssn.residual",
                    final_residual.normalized_norm);
  SIRE_PROFILE_PLOT("explicit_fbf_gated_ssn.newton_attempts",
                    static_cast<double>(newton_attempts));
  SIRE_PROFILE_PLOT("explicit_fbf_gated_ssn.newton_accepted",
                    static_cast<double>(newton_accepted));
  SIRE_PROFILE_PLOT("explicit_fbf_gated_ssn.newton_rejected",
                    static_cast<double>(newton_rejected));
  SIRE_PROFILE_PLOT("explicit_fbf_gated_ssn.fbf_steps",
                    static_cast<double>(explicit_fbf_steps));
  SIRE_PROFILE_PLOT("explicit_fbf_gated_ssn.backtracks",
                    static_cast<double>(fbf_backtracks));
  return final_residual.normalized_norm;
}

auto cptContactForceModePredictorSsnWithOptions(
    sire::Size n, std::vector<double>& fri_coef,
    std::vector<double>& invM_3n, std::vector<double>& v0,
    std::vector<double>& v_target, std::vector<double>& b, double h,
    std::vector<double>& contactFce, sire::Size max_iters, double max_err,
    const ModePredictorSsnOptions& options,
    ModePredictorSsnStatistics* statistics) -> double {
  SIRE_PROFILE_FUNCTION();
  ModePredictorSsnStatistics local_statistics;
  ModePredictorSsnStatistics& stats =
      statistics != nullptr ? *statistics : local_statistics;
  stats = ModePredictorSsnStatistics{};

  const int contact_count = static_cast<int>(n);
  const int dim = 3 * contact_count;
  if (contact_count == 0) {
    std::fill(contactFce.begin(), contactFce.end(), 0.0);
    stats.initial_residual = 0.0;
    stats.final_residual = 0.0;
    stats.converged = true;
    return 0.0;
  }
  if (fri_coef.size() < n ||
      invM_3n.size() < static_cast<size_t>(dim * dim) ||
      v0.size() < static_cast<size_t>(dim) || v_target.size() < n ||
      b.size() < static_cast<size_t>(dim) ||
      contactFce.size() < static_cast<size_t>(dim) || h <= 0.0 ||
      max_iters == 0 || max_err <= 0.0 ||
      options.predictor_omega <= 0.0 ||
      options.predictor_relaxation <= 0.0 ||
      options.predictor_relaxation > 1.0 ||
      options.predictor_merit_growth < 1.0 ||
      options.newton_gate_residual <= 0.0 ||
      options.forced_newton_residual <= 0.0 ||
      !std::isfinite(options.predictor_omega) ||
      !std::isfinite(options.predictor_relaxation) ||
      !std::isfinite(options.predictor_merit_growth) ||
      !std::isfinite(options.newton_gate_residual) ||
      !std::isfinite(options.forced_newton_residual)) {
    return std::numeric_limits<double>::infinity();
  }

  const Eigen::Map<const MatrixXdRM> W(invM_3n.data(), dim, dim);
  const Eigen::Map<const Eigen::VectorXd> v0_map(v0.data(), dim);
  const Eigen::Map<const Eigen::VectorXd> b_map(b.data(), dim);
  Eigen::MatrixXd H = -h * W;
  H = 0.5 * (H + H.transpose());

  Eigen::VectorXd shifted_free_velocity = v0_map - h * b_map;
  for (int i = 0; i < contact_count; ++i)
    shifted_free_velocity(3 * i + 2) += v_target[i];

  const double hessian_scale = std::max(H.norm(), 1e-12);
  const double base_rho = 1.0 / hessian_scale;
  Eigen::VectorXd rho_normal(contact_count);
  Eigen::VectorXd rho_tangent(contact_count);
  for (int i = 0; i < contact_count; ++i) {
    const int tangent_index = 3 * i;
    const int normal_index = tangent_index + 2;
    const double normal_curvature = std::max(
        std::abs(H(normal_index, normal_index)), 1e-6 * hessian_scale);
    const double tangent_curvature =
        std::max(0.5 * (std::abs(H(tangent_index, tangent_index)) +
                        std::abs(H(tangent_index + 1, tangent_index + 1))),
                 1e-6 * hessian_scale);
    rho_normal(i) = std::clamp(1.0 / normal_curvature,
                               1e-3 * base_rho, 1e3 * base_rho);
    rho_tangent(i) = std::clamp(1.0 / tangent_curvature,
                                1e-3 * base_rho, 1e3 * base_rho);
  }

  Eigen::VectorXd f = Eigen::Map<const Eigen::VectorXd>(contactFce.data(), dim);
  if (!f.allFinite()) f.setZero();
  f = projectProductCone(f, fri_coef);

  const Eigen::VectorXd predictor_steps = predictorStepSizes(
      H, fri_coef, std::clamp(options.predictor_omega, 1e-3, 0.99));
  const double maximum_friction =
      std::max(1.0, *std::max_element(fri_coef.begin(),
                                     fri_coef.begin() + contact_count));
  const double maximum_gamma = 1.0 / (maximum_friction * hessian_scale);
  double fbf_gamma = 0.5 * maximum_gamma;
  const double inner_tolerance = std::max(1e-10, 0.1 * max_err);

  enum class Phase { kPredictor, kNewton, kRepair, kFbfRescue };
  const auto initial = evaluatePhysicalResidual(
      H, shifted_free_velocity, f, fri_coef, rho_normal, rho_tangent, false);
  stats.initial_residual = initial.normalized_norm;
  Phase phase = initial.normalized_norm <= options.forced_newton_residual
                    ? Phase::kNewton
                    : Phase::kPredictor;

  std::vector<PredictedContactMode> previous_modes;
  sire::Size stable_mode_count = 0;
  sire::Size predictor_sweeps_since_rescue = 0;
  sire::Size repair_sweeps_remaining = 0;
  sire::Size newton_rejection_cycles = 0;
  sire::Size stagnation_window_sweeps = 0;
  double stagnation_window_residual = initial.normalized_norm;
  bool first_newton_attempted = false;
  bool used_full_hybrid_fallback = false;

  for (sire::Size iteration = 0; iteration < max_iters; ++iteration) {
    const auto current = evaluatePhysicalResidual(
        H, shifted_free_velocity, f, fri_coef, rho_normal, rho_tangent, false);
    if (current.normalized_norm <= max_err) {
      stats.converged = true;
      break;
    }

    if (phase == Phase::kNewton) {
      if (!first_newton_attempted) {
        first_newton_attempted = true;
        stats.residual_before_first_newton = current.normalized_norm;
      }
      ++stats.newton_attempts;
      Eigen::VectorXd newton_candidate;
      if (tryMixedSemismoothNewton(
              H, shifted_free_velocity, fri_coef, rho_normal, rho_tangent, f,
              newton_candidate)) {
        if (stats.newton_attempts == 1) stats.first_newton_accepted = true;
        f = newton_candidate;
        ++stats.newton_accepted;
        ++stats.nonlinear_iterations;
        newton_rejection_cycles = 0;
        continue;
      }

      ++stats.newton_rejected;
      ++newton_rejection_cycles;
      // Predictor sweeps maintain cone feasibility.  Newton's residual
      // extension permits temporary infeasible iterates, so repair starts from
      // their cone projection.
      f = projectProductCone(f, fri_coef);
      if (newton_rejection_cycles >=
          std::max<sire::Size>(1,
                               options.maximum_newton_rejection_cycles)) {
        phase = Phase::kFbfRescue;
      } else {
        phase = Phase::kRepair;
        repair_sweeps_remaining =
            std::max<sire::Size>(1, options.repair_sweeps);
      }
      continue;
    }

    if (phase == Phase::kFbfRescue) {
      Eigen::VectorXd best_candidate = f;
      double best_merit = std::numeric_limits<double>::infinity();
      double trial_gamma = fbf_gamma;
      for (int safeguard = 0; safeguard < 4; ++safeguard) {
        const Eigen::VectorXd candidate = forwardBackwardForwardStep(
            H, shifted_free_velocity, fri_coef, f, trial_gamma,
            inner_tolerance);
        const auto candidate_residual = evaluatePhysicalResidual(
            H, shifted_free_velocity, candidate, fri_coef, rho_normal,
            rho_tangent, false);
        if (candidate.allFinite() &&
            candidate_residual.merit < best_merit) {
          best_candidate = candidate;
          best_merit = candidate_residual.merit;
          fbf_gamma = trial_gamma;
        }
        if (candidate.allFinite() &&
            candidate_residual.merit <= 1.25 * current.merit)
          break;
        trial_gamma *= 0.5;
        ++stats.fbf_backtracks;
      }
      if (best_merit <= 1.25 * current.merit)
        f = projectProductCone(best_candidate, fri_coef);
      fbf_gamma = std::min(maximum_gamma, 1.05 * fbf_gamma);
      ++stats.fbf_rescues;
      ++stats.nonlinear_iterations;

      previous_modes.clear();
      stable_mode_count = 0;
      predictor_sweeps_since_rescue = 0;
      newton_rejection_cycles = 0;
      stagnation_window_sweeps = 0;
      const auto after_rescue = evaluatePhysicalResidual(
          H, shifted_free_velocity, f, fri_coef, rho_normal, rho_tangent,
          false);
      stagnation_window_residual = after_rescue.normalized_norm;
      phase = Phase::kPredictor;

      if (stats.fbf_rescues >=
          std::max<sire::Size>(1, options.maximum_fbf_rescues)) {
        const sire::Size remaining = max_iters - iteration - 1;
        if (remaining > 0) {
          Eigen::Map<Eigen::VectorXd>(contactFce.data(), dim) = f;
          ExactCoulombOptions fallback_options;
          fallback_options.mode = ExactCoulombMode::kHybrid;
          ExactCoulombStatistics fallback_statistics;
          cptContactForceExactCoulombWithOptions(
              n, fri_coef, invM_3n, v0, v_target, b, h, contactFce, remaining,
              max_err, fallback_options, &fallback_statistics);
          f = Eigen::Map<const Eigen::VectorXd>(contactFce.data(), dim);
          ++stats.full_hybrid_fallbacks;
          stats.nonlinear_iterations +=
              fallback_statistics.nonlinear_iterations;
          stats.newton_attempts += fallback_statistics.newton_attempts;
          stats.newton_accepted += fallback_statistics.newton_accepted;
          stats.newton_rejected += fallback_statistics.newton_rejected;
          stats.fbf_backtracks += fallback_statistics.fbf_backtracks;
          used_full_hybrid_fallback = true;
        }
        break;
      }
      continue;
    }

    // Predictor and repair share the same cheap alternating block projection.
    Eigen::VectorXd best_candidate = f;
    double best_merit = current.merit;
    bool accepted_sweep = false;
    double trial_relaxation = options.predictor_relaxation;
    for (int safeguard = 0; safeguard < 3; ++safeguard) {
      Eigen::VectorXd candidate = f;
      const bool reverse = (stats.predictor_sweeps % 2) != 0;
      if (!blockProjectedPredictorSweep(
              H, shifted_free_velocity, fri_coef, predictor_steps,
              trial_relaxation, reverse, candidate)) {
        trial_relaxation *= 0.5;
        ++stats.predictor_backtracks;
        continue;
      }
      const auto candidate_residual = evaluatePhysicalResidual(
          H, shifted_free_velocity, candidate, fri_coef, rho_normal,
          rho_tangent, false);
      if (candidate_residual.merit < best_merit) {
        best_candidate = candidate;
        best_merit = candidate_residual.merit;
      }
      if (candidate_residual.merit <=
          options.predictor_merit_growth * current.merit) {
        best_candidate = candidate;
        accepted_sweep = true;
        break;
      }
      trial_relaxation *= 0.5;
      ++stats.predictor_backtracks;
    }
    // If the broad acceptance test rejects every trial, retaining the best
    // candidate (initialized to f) makes the predictor non-worsening.
    f = best_candidate;
    (void)accepted_sweep;
    ++stats.predictor_sweeps;
    ++stats.nonlinear_iterations;
    ++predictor_sweeps_since_rescue;

    const auto after_predictor = evaluatePhysicalResidual(
        H, shifted_free_velocity, f, fri_coef, rho_normal, rho_tangent, false);
    const auto modes =
        classifyContactModes(H, shifted_free_velocity, f, fri_coef);
    const bool uncertain = containsUncertainMode(modes);
    if (uncertain) ++stats.uncertain_mode_sweeps;
    if (!previous_modes.empty()) {
      sire::Size changes = 0;
      for (sire::Size i = 0; i < modes.size(); ++i)
        if (modes[i] != previous_modes[i]) ++changes;
      stats.mode_changes += changes;
      if (changes == 0 && !uncertain)
        ++stable_mode_count;
      else
        stable_mode_count = 0;
    } else {
      stable_mode_count = uncertain ? 0 : 1;
    }
    previous_modes = modes;

    const bool gate_open =
        after_predictor.normalized_norm <= options.forced_newton_residual ||
        (after_predictor.normalized_norm <= options.newton_gate_residual &&
         stable_mode_count >= options.stable_mode_sweeps && !uncertain);

    ++stagnation_window_sweeps;
    bool predictor_stagnated = false;
    if (stagnation_window_sweeps >= 4) {
      predictor_stagnated =
          after_predictor.normalized_norm >
          0.9 * stagnation_window_residual;
      stagnation_window_sweeps = 0;
      stagnation_window_residual = after_predictor.normalized_norm;
    }

    if (phase == Phase::kRepair) {
      if (repair_sweeps_remaining > 0) --repair_sweeps_remaining;
      if (repair_sweeps_remaining == 0) {
        phase = gate_open ? Phase::kNewton : Phase::kFbfRescue;
      }
    } else if (gate_open &&
               predictor_sweeps_since_rescue >=
                   options.minimum_predictor_sweeps) {
      phase = Phase::kNewton;
    } else if (predictor_stagnated ||
               predictor_sweeps_since_rescue >=
                   std::max<sire::Size>(1,
                                        options.maximum_predictor_sweeps)) {
      phase = Phase::kFbfRescue;
    }
  }

  if (!used_full_hybrid_fallback) {
    f = projectProductCone(f, fri_coef);
    Eigen::Map<Eigen::VectorXd>(contactFce.data(), dim) = f;
  }
  const auto final_residual = evaluatePhysicalResidual(
      H, shifted_free_velocity, f, fri_coef, rho_normal, rho_tangent, false);
  stats.final_residual = final_residual.normalized_norm;
  stats.converged = final_residual.normalized_norm <= max_err;

  SIRE_PROFILE_PLOT("mode_predictor_ssn.residual",
                    final_residual.normalized_norm);
  SIRE_PROFILE_PLOT("mode_predictor_ssn.predictor_sweeps",
                    static_cast<double>(stats.predictor_sweeps));
  SIRE_PROFILE_PLOT("mode_predictor_ssn.predictor_backtracks",
                    static_cast<double>(stats.predictor_backtracks));
  SIRE_PROFILE_PLOT("mode_predictor_ssn.newton_attempts",
                    static_cast<double>(stats.newton_attempts));
  SIRE_PROFILE_PLOT("mode_predictor_ssn.newton_accepted",
                    static_cast<double>(stats.newton_accepted));
  SIRE_PROFILE_PLOT("mode_predictor_ssn.newton_rejected",
                    static_cast<double>(stats.newton_rejected));
  SIRE_PROFILE_PLOT("mode_predictor_ssn.fbf_rescues",
                    static_cast<double>(stats.fbf_rescues));
  SIRE_PROFILE_PLOT("mode_predictor_ssn.full_fallbacks",
                    static_cast<double>(stats.full_hybrid_fallbacks));
  return final_residual.normalized_norm;
}

auto cptContactForceModePredictorSsn(
    sire::Size n, std::vector<double>& fri_coef,
    std::vector<double>& invM_3n, std::vector<double>& v0,
    std::vector<double>& v_target, std::vector<double>& b, double h,
    std::vector<double>& contactFce, sire::Size max_iters,
    double max_err) -> double {
  ModePredictorSsnOptions options;
  return cptContactForceModePredictorSsnWithOptions(
      n, fri_coef, invM_3n, v0, v_target, b, h, contactFce, max_iters,
      max_err, options, nullptr);
}

auto ExactCoulombContactSolver::solveContactForceQP(
    sire::Size n, std::vector<double>& fri_coef,
    std::vector<double>& invM_3n, std::vector<double>& v0,
    std::vector<double>& v_target, std::vector<double>& b, double h,
    std::vector<double>& contactFce, sire::Size max_iters,
    double max_err) -> double {
  return cptContactForceExactCoulomb(n, fri_coef, invM_3n, v0, v_target, b, h,
                                     contactFce, max_iters, max_err);
}

ARIS_REGISTRATION {
  aris::core::class_<ExactCoulombContactSolver>("ExactCoulombContactSolver")
      .inherit<ps_vs_solver_v5::PsVsSolverV5>();
}

}  // namespace sire::physics::contact::exact_coulomb
