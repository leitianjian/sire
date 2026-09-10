#ifndef SIRE_PS_VS_SOLVER_V5_HPP_
#define SIRE_PS_VS_SOLVER_V5_HPP_

#include <array>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <coal/broadphase/broadphase_callbacks.h>
#include <coal/broadphase/broadphase_collision_manager.h>
#include <coal/broadphase/default_broadphase_callbacks.h>
#include <coal/collision.h>
#include <coal/collision_data.h>
#include <coal/collision_object.h>

#include <aris/core/expression_calculator.hpp>
#include <aris/core/object.hpp>

#include "sire/core/sire_decl_def_macro.hpp"
#include "sire/physics/collision/collided_objects_callback.hpp"
#include "sire/physics/collision/collision_filter.hpp"
#include "sire/physics/common/penetration_as_point_pair.hpp"
#include "sire/physics/common/point_pair_contact_info.hpp"
#include "sire/physics/contact/contact_solver.hpp"
#include "sire/physics/contact/contact_solver_result.hpp"

namespace sire::physics::contact::ps_vs_solver_v5 {

/// @brief ADMM-based DAE-NCP contact solver (v5)
///
/// Internally solves the contact problem:
///   min  ½ fᵀ·H·f + fᵀ·(g + s)
///    f
///   s.t.  H_N·f = a_pos            (DAE normal target)
///         ‖f_Tⁱⁱ‖ ≤ μ_i·f_Nⁱⁱ      (Coulomb cone)
///
/// using ADMM inner loop + outer fixed-point iteration for Γ self-consistency.
class SIRE_API PsVsSolverV5 : public ContactSolver {
 public:
  PsVsSolverV5();
  virtual ~PsVsSolverV5();
  SIRE_DECLARE_MOVE_CTOR(PsVsSolverV5);
  virtual auto doInit(physics::PhysicsEngine* engine_ptr) -> void override {};

  // Material manager
  auto resetMaterialManager(core::MaterialManager* manager) -> void;
  auto materialManager() -> core::MaterialManager&;

  auto setDefaultStiffness(double k) noexcept -> void;
  auto defaultStiffness() noexcept -> double;
  auto setDefaultCr(double cr) noexcept -> void;
  auto defaultCr() noexcept -> double;
  auto setDefaultVelocityThreshold(double tv) noexcept -> void;
  auto defaultVelocityThreshold() noexcept -> double;
  auto debugByRecords() -> nlohmann::json override;

  virtual auto cptContactSolverResult(
      const aris::dynamic::Model* current_state,
      std::vector<common::PenetrationAsPointPair>& penetration_pairs,
      std::vector<std::array<double, 16>>& T_C_vec, ContactSolverResult& result)
      -> void override;

  /// Compute contact forces only — no integration, no time update, no events.
  auto cptContactForces(
      aris::dynamic::Model& model, sire::physics::PhysicsEngine& engine,
      std::vector<common::PenetrationAsPointPair>& penetration_pairs,
      std::vector<std::array<double, 16>>& T_C_vec,
      std::vector<common::PointPairContactInfo>& contact_info,
      double suggest_dt) -> double;

 protected:
  auto supportsSinglePointContactMode() const -> bool override { return true; }

  /// Solve the assembled 3D contact-force QP.  Kept virtual so alternative
  /// numerical methods can reuse the (rather involved) v5 simulation/event
  /// pipeline without duplicating it.
  virtual auto solveContactForceQP(
      sire::Size n, std::vector<double>& fri_coef, std::vector<double>& invM_3n,
      std::vector<double>& v0, std::vector<double>& v_target,
      std::vector<double>& b, double h, std::vector<double>& contactFce,
      sire::Size max_iters, double max_err) -> double;

  /// Hooks for solvers that carry contact-space iterates across simulation
  /// steps.  The default implementation is deliberately stateless.
  virtual auto prepareContactForceInitialGuess(
      const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
      const std::vector<std::array<double, 16>>& contact_frames,
      const std::vector<sire::Size>& preserved_pair_indices, double h,
      std::vector<double>& contact_force) -> void;
  virtual auto commitContactForceSolution(
      const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
      const std::vector<std::array<double, 16>>& contact_frames,
      const std::vector<sire::Size>& preserved_pair_indices, double h,
      const std::vector<double>& contact_force) -> void;
  virtual auto clearContactSolverState() -> void;
  virtual auto contactSolverMaxIterations() const noexcept -> sire::Size {
    return 200;
  }

 private:
  struct Imp;
  std::unique_ptr<Imp> imp_;
};

/// @brief Standalone ADMM contact force computation (same interface as v4).
auto cptContactForceWithTargetState5(
    sire::Size n, std::vector<double>& fri_coef, std::vector<double>& invM_3n,
    std::vector<double>& v0, std::vector<double>& v_target,
    std::vector<double>& b, double h, std::vector<double>& contactFce,
    sire::Size max_iters = 30, double max_err = 1e-8) -> double;

/// @brief Nested ADMM solver (v6): frozen De Saxce inner solves, no DAE target.
auto cptContactForceWithTargetState6(
    sire::Size n, std::vector<double>& fri_coef, std::vector<double>& invM_3n,
    std::vector<double>& v0, std::vector<double>& v_target,
    std::vector<double>& b, double h, std::vector<double>& contactFce,
    sire::Size max_iters = 30, double max_err = 1e-8) -> double;

/// @brief Davis-Yin three-operator splitting (v7): no augmented Lagrangian.
auto cptContactForceWithTargetState7(
    sire::Size n, std::vector<double>& fri_coef, std::vector<double>& invM_3n,
    std::vector<double>& v0, std::vector<double>& v_target,
    std::vector<double>& b, double h, std::vector<double>& contactFce,
    sire::Size max_iters = 30, double max_err = 1e-8) -> double;

/// @brief PDDY — DYS with exact F prox, linear convergence.
auto cptContactForceWithTargetState8(
    sire::Size n, std::vector<double>& fri_coef, std::vector<double>& invM_3n,
    std::vector<double>& v0, std::vector<double>& v_target,
    std::vector<double>& b, double h, std::vector<double>& contactFce,
    sire::Size max_iters = 30, double max_err = 1e-8) -> double;

}  // namespace sire::physics::contact::ps_vs_solver_v5

#endif  // SIRE_PS_VS_SOLVER_V5_HPP_
