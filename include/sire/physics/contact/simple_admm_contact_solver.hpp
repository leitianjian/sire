#ifndef SIRE_SIMPLE_ADMM_CONTACT_SOLVER_HPP_
#define SIRE_SIMPLE_ADMM_CONTACT_SOLVER_HPP_

#include <array>
#include <vector>

#include "sire/physics/contact/ps_vs_solver_v5.hpp"

namespace sire::physics::contact::simple_admm {

/// Contact solver baseline following Algorithm 1 of Le Lidec et al., RSS 2024.
///
/// This class only replaces the assembled contact-force solve.  Collision
/// handling, contact-frame construction, force application, and time stepping
/// are inherited from PsVsSolverV5 so comparisons isolate the numerical
/// contact solver.
class SIRE_API SimpleAdmmContactSolver : public ps_vs_solver_v5::PsVsSolverV5 {
 public:
  SimpleAdmmContactSolver() = default;
  ~SimpleAdmmContactSolver() override = default;

  auto setWarmStartEnabled(bool enabled) noexcept -> void;
  auto warmStartEnabled() const noexcept -> bool;

 protected:
  auto solveContactForceQP(
      sire::Size n, std::vector<double>& fri_coef, std::vector<double>& invM_3n,
      std::vector<double>& v0, std::vector<double>& v_target,
      std::vector<double>& b, double h, std::vector<double>& contactFce,
      sire::Size max_iters, double max_err) -> double override;

  auto solveSpectralContactForce(
      bool shifted, sire::Size n, std::vector<double>& fri_coef,
      std::vector<double>& invM_3n, std::vector<double>& v0,
      std::vector<double>& v_target, std::vector<double>& b, double h,
      std::vector<double>& contactFce, sire::Size max_iters, double max_err)
      -> double;

  auto prepareContactForceInitialGuess(
      const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
      const std::vector<std::array<double, 16>>& contact_frames,
      const std::vector<sire::Size>& preserved_pair_indices, double h,
      std::vector<double>& contact_force) -> void override;
  auto commitContactForceSolution(
      const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
      const std::vector<std::array<double, 16>>& contact_frames,
      const std::vector<sire::Size>& preserved_pair_indices, double h,
      const std::vector<double>& contact_force) -> void override;
  auto clearContactSolverState() -> void override;
  auto contactSolverMaxIterations() const noexcept -> sire::Size override {
    return 1000;
  }

 private:
  struct WarmStartContact {
    sire::geometry::GeometryId id_a{};
    sire::geometry::GeometryId id_b{};
    std::array<double, 3> point_W{};
    std::array<double, 3> force_W{};
  };

  bool warm_start_enabled_{true};
  bool has_primal_guess_{false};
  bool has_dual_guess_{false};
  std::vector<double> prepared_dual_guess_;
  std::vector<double> last_dual_solution_;
  std::vector<WarmStartContact> warm_start_contacts_;
};

/// Spectral ADMM on the target-shifted NCP. The only numerical change from
/// SimpleAdmmContactSolver is g_N += v_target (desired velocity = -v_target).
/// Defaults to the single-point contact model, including depth baselines and
/// contact end-time computation. Reference-style cross-step contact-force
/// warm starting is enabled by default and can be disabled explicitly.
class SIRE_API ShiftedSpectralAdmmContactSolver
    : public SimpleAdmmContactSolver {
 public:
  ShiftedSpectralAdmmContactSolver();
  ~ShiftedSpectralAdmmContactSolver() override = default;

 protected:
  auto solveContactForceQP(
      sire::Size n, std::vector<double>& fri_coef, std::vector<double>& invM_3n,
      std::vector<double>& v0, std::vector<double>& v_target,
      std::vector<double>& b, double h, std::vector<double>& contactFce,
      sire::Size max_iters, double max_err) -> double override;
};

/// Standalone rigid-contact spectral ADMM entry point for Python comparisons.
///
/// The common Sire trace interface does not carry a contact-compliance vector,
/// so this baseline evaluates the published rigid case R = 0.  v_target is
/// intentionally ignored: modifying g with Sire's target velocity would create
/// an adapted solver rather than reproduce the method described in the paper.
/// Algorithm 1 is evaluated in impulse coordinates and converted back to force
/// at the public boundary, preserving the paper's parameter and residual units.
SIRE_API auto cptContactForceSpectralAdmm(
    sire::Size n, std::vector<double>& fri_coef, std::vector<double>& invM_3n,
    std::vector<double>& v0, std::vector<double>& v_target,
    std::vector<double>& b, double h, std::vector<double>& contactFce,
    sire::Size max_iters = 1000, double max_err = 1e-8) -> double;

/// Same impulse-space iteration as the baseline, with g = v0 - h*b + E_N*t.
/// Here t is v_target, so the shifted normal velocity is c_N + v_target.
/// The target is a velocity: do not scale it by h or add it to De Saxce Gamma.
SIRE_API auto cptContactForceShiftedSpectralAdmm(
    sire::Size n, std::vector<double>& fri_coef, std::vector<double>& invM_3n,
    std::vector<double>& v0, std::vector<double>& v_target,
    std::vector<double>& b, double h, std::vector<double>& contactFce,
    sire::Size max_iters = 1000, double max_err = 1e-8) -> double;

/// Backward-compatible alias. Prefer cptContactForceSpectralAdmm in new code.
auto cptContactForceSimpleAdmm(
    sire::Size n, std::vector<double>& fri_coef, std::vector<double>& invM_3n,
    std::vector<double>& v0, std::vector<double>& v_target,
    std::vector<double>& b, double h, std::vector<double>& contactFce,
    sire::Size max_iters = 1000, double max_err = 1e-8) -> double;

}  // namespace sire::physics::contact::simple_admm

#endif  // SIRE_SIMPLE_ADMM_CONTACT_SOLVER_HPP_
