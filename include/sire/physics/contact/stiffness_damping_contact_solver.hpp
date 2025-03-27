#ifndef SIRE_STIFFNESS_DAMPING_CONTACT_SOLVER_HPP_
#define SIRE_STIFFNESS_DAMPING_CONTACT_SOLVER_HPP_

#include <map>
#include <string>
#include <utility>

#include <coal/broadphase/broadphase_callbacks.h>
#include <coal/broadphase/broadphase_collision_manager.h>
#include <coal/broadphase/default_broadphase_callbacks.h>
#include <coal/collision.h>
#include <coal/collision_data.h>
#include <coal/collision_object.h>

#include <aris/core/expression_calculator.hpp>
#include <aris/core/object.hpp>

#include "sire/physics/collision/collided_objects_callback.hpp"
#include "sire/physics/collision/collision_filter.hpp"
#include "sire/physics/common/penetration_as_point_pair.hpp"
#include "sire/physics/common/point_pair_contact_info.hpp"
#include "sire/physics/contact/contact_solver.hpp"
#include "sire/physics/contact/contact_solver_result.hpp"

namespace sire::physics {
namespace contact {
using namespace std;
using namespace coal;
/* contact-based implementation
 */
class SIRE_API StiffnessDampingContactSolver : public ContactSolver {
 public:
  StiffnessDampingContactSolver();
  virtual ~StiffnessDampingContactSolver();
  ARIS_DECLARE_BIG_FOUR(StiffnessDampingContactSolver);
  virtual auto doInit(physics::PhysicsEngine* engine_ptr) -> void override{};

  auto setDefaultStiffness(double k) noexcept -> void;
  auto defaultStiffness() noexcept -> double;

  auto setDefaultDamping(double d) noexcept -> void;
  auto defaultDamping() noexcept -> double;

  virtual auto cptContactSolverResult(
      const aris::dynamic::Model* current_state,
      const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
      const std::vector<std::array<double, 16>>& T_C_vec,
      ContactSolverResult& result) -> void override;

 private:
  struct Imp;
  aris::core::ImpPtr<Imp> imp_;
};
}  // namespace contact
}  // namespace sire::physics
#endif