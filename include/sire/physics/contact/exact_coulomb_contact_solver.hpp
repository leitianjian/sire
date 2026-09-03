#ifndef SIRE_EXACT_COULOMB_CONTACT_SOLVER_HPP_
#define SIRE_EXACT_COULOMB_CONTACT_SOLVER_HPP_

#include <cstdint>
#include <limits>
#include <vector>

#include "sire/physics/contact/ps_vs_solver_v5.hpp"

namespace sire::physics::contact::exact_coulomb {

/// Update policy used by the target-shifted exact Coulomb experiment.
///
/// All three policies share the same problem construction, physical residual,
/// stopping test, cone projection and initial force.  This makes the ablation
/// compare the nonlinear update itself rather than three subtly different
/// contact formulations.
enum class ExactCoulombMode {
  kFbfOnly,
  kSemismoothNewtonOnly,
  kHybrid,
};

enum class ExactCoulombStep : std::uint8_t {
  kInitial = 0,
  kFbf = 1,
  kNewton = 2,
  kRejectedNewton = 3,
};

struct ExactCoulombOptions {
  ExactCoulombMode mode{ExactCoulombMode::kHybrid};

  /// Hybrid tries Newton only after this many accepted nonlinear updates.
  /// Newton-only ignores this field and starts from iteration zero.
  sire::Size newton_start_iteration{2};

  /// Optional local-basin gate.  Infinity reproduces the opportunistic
  /// strategy used by the production solver: the merit line search alone
  /// decides whether a Newton model is trustworthy.
  double newton_switch_residual{std::numeric_limits<double>::infinity()};

  /// Residual histories are useful for convergence plots but deliberately
  /// disabled for timing runs.
  bool collect_history{false};
};

struct ExactCoulombStatistics {
  sire::Size nonlinear_iterations{0};
  sire::Size newton_attempts{0};
  sire::Size newton_accepted{0};
  sire::Size newton_rejected{0};
  sire::Size fbf_steps{0};
  sire::Size fbf_backtracks{0};
  double initial_residual{std::numeric_limits<double>::infinity()};
  double final_residual{std::numeric_limits<double>::infinity()};
  bool converged{false};
  bool terminated_on_newton_rejection{false};
  std::vector<double> residual_history;
  std::vector<ExactCoulombStep> step_history;
};

/// Tuning parameters for the contact-mode predictor + mixed SSN experiment.
/// The predictor is deliberately inexact: it only has to enter a stable
/// opening/sticking/sliding region.  ExactCoulomb's physical residual remains
/// the sole convergence test.
struct ModePredictorSsnOptions {
  double predictor_omega{0.8};
  double predictor_relaxation{0.9};
  sire::Size minimum_predictor_sweeps{2};
  sire::Size maximum_predictor_sweeps{8};
  sire::Size stable_mode_sweeps{2};
  double newton_gate_residual{5e-2};
  double forced_newton_residual{1e-3};
  sire::Size repair_sweeps{2};
  sire::Size maximum_newton_rejection_cycles{2};
  sire::Size maximum_fbf_rescues{3};
  double predictor_merit_growth{1.25};
};

struct ModePredictorSsnStatistics {
  sire::Size nonlinear_iterations{0};
  sire::Size predictor_sweeps{0};
  sire::Size predictor_backtracks{0};
  sire::Size mode_changes{0};
  sire::Size uncertain_mode_sweeps{0};
  sire::Size newton_attempts{0};
  sire::Size newton_accepted{0};
  sire::Size newton_rejected{0};
  sire::Size fbf_rescues{0};
  sire::Size fbf_backtracks{0};
  sire::Size full_hybrid_fallbacks{0};
  double initial_residual{std::numeric_limits<double>::infinity()};
  double residual_before_first_newton{
      std::numeric_limits<double>::infinity()};
  double final_residual{std::numeric_limits<double>::infinity()};
  bool first_newton_accepted{false};
  bool converged{false};
};

/// Target-shifted unilateral Coulomb solver.
///
/// The globalization phase uses a De Saxce forward-backward-forward iteration. Once
/// close enough, an opportunistic mixed semismooth Newton step is applied to
/// the exact normal-complementarity/tangential-maximum-dissipation residual.
/// A rejected Newton step falls back to the globally convergent phase.
class SIRE_API ExactCoulombContactSolver
    : public ps_vs_solver_v5::PsVsSolverV5 {
 public:
  ExactCoulombContactSolver() = default;
  ~ExactCoulombContactSolver() override = default;

 protected:
  auto solveContactForceQP(
      sire::Size n, std::vector<double>& fri_coef,
      std::vector<double>& invM_3n, std::vector<double>& v0,
      std::vector<double>& v_target, std::vector<double>& b, double h,
      std::vector<double>& contactFce, sire::Size max_iters,
      double max_err) -> double override;
};

/// Standalone entry point used by Python comparisons and regression tests.
auto cptContactForceExactCoulomb(
    sire::Size n, std::vector<double>& fri_coef,
    std::vector<double>& invM_3n, std::vector<double>& v0,
    std::vector<double>& v_target, std::vector<double>& b, double h,
    std::vector<double>& contactFce, sire::Size max_iters = 200,
    double max_err = 1e-8) -> double;

/// Common experimental entry point used for the FBF/Newton/hybrid ablation.
auto cptContactForceExactCoulombWithOptions(
    sire::Size n, std::vector<double>& fri_coef,
    std::vector<double>& invM_3n, std::vector<double>& v0,
    std::vector<double>& v_target, std::vector<double>& b, double h,
    std::vector<double>& contactFce, sire::Size max_iters, double max_err,
    const ExactCoulombOptions& options,
    ExactCoulombStatistics* statistics = nullptr) -> double;

/// Same mathematical problem as cptContactForceExactCoulomb, using only FBF.
auto cptContactForceExactCoulombFbf(
    sire::Size n, std::vector<double>& fri_coef,
    std::vector<double>& invM_3n, std::vector<double>& v0,
    std::vector<double>& v_target, std::vector<double>& b, double h,
    std::vector<double>& contactFce, sire::Size max_iters = 200,
    double max_err = 1e-8) -> double;

/// Same mathematical problem, using damped mixed semismooth Newton only.
/// A rejected line search terminates rather than silently taking an FBF step.
auto cptContactForceExactCoulombSemismoothNewton(
    sire::Size n, std::vector<double>& fri_coef,
    std::vector<double>& invM_3n, std::vector<double>& v0,
    std::vector<double>& v_target, std::vector<double>& b, double h,
    std::vector<double>& contactFce, sire::Size max_iters = 200,
    double max_err = 1e-8) -> double;

/// Single-loop tracked-prox FBF with the same A/B splitting as the nested
/// solver, accelerated by the same safeguarded mixed semismooth Newton step.
/// One projected-gradient resolvent update is retained across nonlinear
/// iterations instead of solving every frozen-correction QP to convergence.
auto cptContactForceSingleLoopFbfSsn(
    sire::Size n, std::vector<double>& fri_coef,
    std::vector<double>& invM_3n, std::vector<double>& v0,
    std::vector<double>& v_target, std::vector<double>& b, double h,
    std::vector<double>& contactFce, sire::Size max_iters = 200,
    double max_err = 1e-8) -> double;

/// Explicit Tseng FBF with A = N_K and the Delassus/de Saxce terms in B.
/// The resolvent is therefore one analytic product-cone projection per
/// iteration. Mixed semismooth Newton is attempted only inside a residual
/// gate, with a cooldown after rejection to avoid repeated failed factorizations.
auto cptContactForceExplicitFbfGatedSsn(
    sire::Size n, std::vector<double>& fri_coef,
    std::vector<double>& invM_3n, std::vector<double>& v0,
    std::vector<double>& v_target, std::vector<double>& b, double h,
    std::vector<double>& contactFce, sire::Size max_iters = 200,
    double max_err = 1e-8) -> double;

/// Contact-structure-aware basin-entry predictor followed by mixed SSN.
///
/// A few alternating block projected sweeps identify opening, sticking and
/// sliding modes.  A residual-and-mode-stability gate then enables consecutive
/// mixed SSN steps.  Rejected Newton models receive cheap predictor repair
/// sweeps; repeated rejection or predictor stagnation triggers one full FBF
/// rescue.  The existing robust hybrid is the final fallback.
auto cptContactForceModePredictorSsn(
    sire::Size n, std::vector<double>& fri_coef,
    std::vector<double>& invM_3n, std::vector<double>& v0,
    std::vector<double>& v_target, std::vector<double>& b, double h,
    std::vector<double>& contactFce, sire::Size max_iters = 200,
    double max_err = 1e-8) -> double;

auto cptContactForceModePredictorSsnWithOptions(
    sire::Size n, std::vector<double>& fri_coef,
    std::vector<double>& invM_3n, std::vector<double>& v0,
    std::vector<double>& v_target, std::vector<double>& b, double h,
    std::vector<double>& contactFce, sire::Size max_iters, double max_err,
    const ModePredictorSsnOptions& options,
    ModePredictorSsnStatistics* statistics = nullptr) -> double;

}  // namespace sire::physics::contact::exact_coulomb

#endif
