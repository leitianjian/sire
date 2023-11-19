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
using namespace std;
using namespace hpp;
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
  auto partPoolPtr() -> aris::core::PointerArray<aris::dynamic::Part,
                                                 aris::dynamic::Element>* {
    return part_pool_ptr_;
  }
  auto partSize() -> sire::Size { return part_size_; }

  virtual auto cptContactSolverResult(
      const aris::dynamic::Model* current_state,
      const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
      const std::vector<std::array<double, 16>>& T_C_vec,
      ContactSolverResult& result) -> void = 0;

 private:
  physics::PhysicsEngine* engine_ptr_{nullptr};
  aris::core::PointerArray<aris::dynamic::Part, aris::dynamic::Element>*
      part_pool_ptr_{nullptr};
  sire::Size part_size_{0};
};
}  // namespace contact
}  // namespace sire::physics
#endif