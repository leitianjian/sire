#include "sire/physics/contact/contact_solver.hpp"

#include <aris/core/reflection.hpp>

#include "sire/physics/collision/collision_detection.hpp"
#include "sire/physics/collision/collision_exists_callback.hpp"
#include "sire/physics/physics_engine.hpp"

namespace sire::physics::contact {
ARIS_DEFINE_BIG_FOUR_CPP(ContactSolver);

auto ContactSolver::init(physics::PhysicsEngine* engine_ptr) -> void {
  engine_ptr_ = engine_ptr;
  doInit(engine_ptr);
}
ARIS_REGISTRATION {
  typedef sire::physics::collision::CollisionFilter& (
      ContactSolver::*CollisionFilterPoolFunc)();
  aris::core::class_<ContactSolver>("ContactSolver");
}
}  // namespace sire::physics::contact