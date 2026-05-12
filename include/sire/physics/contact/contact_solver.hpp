#ifndef SIRE_CONTACT_SOLVER_HPP_
#define SIRE_CONTACT_SOLVER_HPP_

#include <string>

#include <aris/core/expression_calculator.hpp>
#include <aris/core/object.hpp>
#include <aris/server/control_server.hpp>

#include "sire/core/material_manager.hpp"
#include "sire/physics/common/penetration_as_point_pair.hpp"
#include "sire/physics/contact/contact_solver_result.hpp"

namespace sire::physics {
class PhysicsEngine;
namespace contact {
/* contact-based implementation
 */
class SIRE_API ContactSolver {
 public:
  ContactSolver() = default;
  virtual ~ContactSolver() = default;
  ARIS_DECLARE_BIG_FOUR(ContactSolver);

  auto init(physics::PhysicsEngine* engine_ptr) -> void;
  virtual auto doInit(physics::PhysicsEngine* engine_ptr) -> void{};

  auto physicsEnginePtr() -> physics::PhysicsEngine* { return engine_ptr_; };

  virtual auto cptContactSolverResult(
      const aris::dynamic::Model* current_state,
      std::vector<common::PenetrationAsPointPair>& penetration_pairs,
      std::vector<std::array<double, 16>>& T_C_vec,
      ContactSolverResult& result) -> void = 0;
  virtual auto debugByRecords() -> nlohmann::json {
    return nlohmann::json();
    // Default implementation does nothing.
    // Derived classes can override this method to implement debugging behavior.
  }

 private:
  physics::PhysicsEngine* engine_ptr_{nullptr};
};
}  // namespace contact
}  // namespace sire::physics
#endif