#ifndef SIRE_SMOOTH_LINEAR_CONTACT_SOLVER_HPP_
#define SIRE_SMOOTH_LINEAR_CONTACT_SOLVER_HPP_

#include "sire/physics/contact/ps_vs_solver_v5.hpp"

namespace sire::physics::contact::smooth_linear {

struct SmoothLinearSolveInfo {
  sire::Size iterations{0};
  bool enumerated{false};
  double residual{0.0};
};

/// Linear velocity regularization from the previous smooth law, evaluated at
/// the current contact tangential velocity. Forces use the common V5 sign and
/// normal-target convention. No collision, mass assembly or integration here.
SIRE_API auto cptContactForceSmoothLinear(
    sire::Size n, const std::vector<double>& friction,
    const std::vector<double>& inverse_mass, const std::vector<double>& velocity,
    const std::vector<double>& normal_target,
    const std::vector<double>& external_acceleration, double h,
    const std::vector<double>& threshold_velocity,
    std::vector<double>& contact_force, sire::Size max_iterations = 200,
    double tolerance = 1e-8) -> SmoothLinearSolveInfo;

/// Only the contact-force kernel differs from the shifted ADMM baseline.
/// Collision, inverse contact inertia, depth baselines, separation-time search,
/// force recording, integration and native batching all remain in V5.
class SIRE_API SmoothLinearContactSolver : public ps_vs_solver_v5::PsVsSolverV5 {
 public:
  SmoothLinearContactSolver();
  auto doInit(physics::PhysicsEngine* engine_ptr) -> void override;
  auto lastIterations() const -> sire::Size { return last_info_.iterations; }
  auto lastResidual() const -> double { return last_info_.residual; }
  auto lastEnumerated() const -> bool { return last_info_.enumerated; }

 protected:
  auto prepareContactForceInitialGuess(
      const std::vector<common::PenetrationAsPointPair>& pairs,
      const std::vector<std::array<double, 16>>& frames,
      const std::vector<sire::Size>& preserved, double h,
      std::vector<double>& force) -> void override;
  auto solveContactForceQP(
      sire::Size n, std::vector<double>& friction,
      std::vector<double>& inverse_mass, std::vector<double>& velocity,
      std::vector<double>& normal_target, std::vector<double>& acceleration,
      double h, std::vector<double>& force, sire::Size max_iterations,
      double tolerance) -> double override;
  auto clearContactSolverState() -> void override;

 private:
  std::vector<double> threshold_velocity_;
  SmoothLinearSolveInfo last_info_;
};

}  // namespace sire::physics::contact::smooth_linear
#endif
