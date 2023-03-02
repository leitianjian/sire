#include "sire/physics/physics_engine.hpp"

#include <aris/core/reflection.hpp>

#include "sire/physics/collision/collision_detection_engine.hpp"
#include "sire/physics/contact/contact_engine.hpp"

namespace sire::physics {
using namespace std;
struct PhysicsEngine::Imp {
  unique_ptr<collision::CollisionDetectionEngine> collision_detection_engine_;
  unique_ptr<contact::ContactEngine> contact_engine_;
};
auto PhysicsEngine::resetCollisionDetectionEngine(
    collision::CollisionDetectionEngine* collision_detection_engine_in)
    -> void {
  imp_->collision_detection_engine_.reset(collision_detection_engine_in);
}

auto PhysicsEngine::collisionDetectionEngine()
    -> collision::CollisionDetectionEngine& {
  return *imp_->collision_detection_engine_;
}

auto PhysicsEngine::resetContactEngine(
    contact::ContactEngine* contact_engine_in) -> void {
  imp_->contact_engine_.reset(contact_engine_in);
}

auto PhysicsEngine::contactEngine() -> contact::ContactEngine& {
  return *imp_->contact_engine_;
}

auto PhysicsEngine::instance() noexcept -> PhysicsEngine& {
  static PhysicsEngine instance;
  return instance;
}

auto PhysicsEngine::init() -> void {}

PhysicsEngine::PhysicsEngine() : imp_(new Imp) {}
PhysicsEngine::~PhysicsEngine() = default;
ARIS_REGISTRATION {
  typedef sire::physics::collision::CollisionDetectionEngine& (
      PhysicsEngine::*CollisionDetectionEngineFunc)();
  typedef sire::physics::contact::ContactEngine& (PhysicsEngine::*ContactEngineFunc)();
  aris::core::class_<PhysicsEngine>("PhysicsEngine")
      .prop("CollisionDetectionEngine",
            &PhysicsEngine::resetCollisionDetectionEngine,
            CollisionDetectionEngineFunc(
                &PhysicsEngine::collisionDetectionEngine))
      .prop("ContactEngine", &PhysicsEngine::resetContactEngine,
            ContactEngineFunc(&PhysicsEngine::contactEngine));
}
}  // namespace sire::physics