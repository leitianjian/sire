#ifndef SIRE_CONTINUOUS_FORCE_CONTACT_SOLVER_HPP_
#define SIRE_CONTINUOUS_FORCE_CONTACT_SOLVER_HPP_

#include <map>
#include <string>
#include <utility>

#include <hpp/fcl/broadphase/broadphase_callbacks.h>
#include <hpp/fcl/broadphase/broadphase_collision_manager.h>
#include <hpp/fcl/broadphase/default_broadphase_callbacks.h>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/collision_data.h>
#include <hpp/fcl/collision_object.h>

#include <aris/core/expression_calculator.hpp>
#include <aris/core/object.hpp>

#include "sire/core/sire_decl_def_macro.hpp"
#include "sire/physics/collision/collided_objects_callback.hpp"
#include "sire/physics/collision/collision_filter.hpp"
#include "sire/physics/common/penetration_as_point_pair.hpp"
#include "sire/physics/common/point_pair_contact_info.hpp"
#include "sire/physics/contact/contact_solver.hpp"
#include "sire/physics/contact/contact_solver_result.hpp"

namespace sire::physics {
namespace contact {
using namespace std;
using namespace hpp;
/* contact-based implementation
 */
class SIRE_API ContinuousForceContactSolver : public ContactSolver {
 public:
  ContinuousForceContactSolver();
  virtual ~ContinuousForceContactSolver();
  SIRE_DECLARE_MOVE_CTOR(ContinuousForceContactSolver);
  virtual auto doInit(physics::PhysicsEngine* engine_ptr) -> void override{};

  // Material manager
  auto resetMaterialManager(core::MaterialManager* manager) -> void;
  auto materialManager() -> core::MaterialManager&;

  auto setDefaultStiffness(double k) noexcept -> void;
  auto defaultStiffness() noexcept -> double;
  auto setDefaultCr(double d) noexcept -> void;
  auto defaultCr() noexcept -> double;

  virtual auto cptContactSolverResult(
      const aris::dynamic::Model* current_state,
      const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
      const std::vector<std::array<double, 16>>& T_C_vec,
      ContactSolverResult& result) -> void override;

 private:
  auto cptContactForce(double A, double B, double k, double D, double r,
                       double w, double t) -> double;
  auto cptPenaltyODE(double contact_time, double projected_start_diff_v,
                     double cr, double m, double k, double delta_t) -> double;
  struct Imp;
  std::unique_ptr<Imp> imp_;
};
}  // namespace contact
}  // namespace sire::physics
#endif