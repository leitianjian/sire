#ifndef SIRE_PHYSICS_ENGINE_HPP_
#define SIRE_PHYSICS_ENGINE_HPP_

#include <sire_lib_export.h>

#include "sire/core/module_base.hpp"
#include "sire/physics/collision/collision_detection_engine.hpp"
#include "sire/physics/contact/contact_engine.hpp"

namespace sire::physics {
class PhysicsEngine : public core::SireModuleBase {
 public:
  static auto instance() noexcept -> PhysicsEngine&;

  // collision_detection_engine //
  auto resetCollisionDetectionEngine(
      collision::CollisionDetectionEngine* collision_detection_engine_in)
      -> void;
  auto collisionDetectionEngine() -> collision::CollisionDetectionEngine&;
  auto collisionDetectionEngine() const
      -> const collision::CollisionDetectionEngine& {
    return const_cast<PhysicsEngine*>(this)->collisionDetectionEngine();
  }

  // contact_engine //
  auto resetContactEngine(contact::ContactEngine* contact_engine_in) -> void;
  auto contactEngine() -> contact::ContactEngine&;
  auto contactEngine() const -> const contact::ContactEngine& {
    return const_cast<PhysicsEngine*>(this)->contactEngine();
  }

  // calculate contact wrench of model
  auto calModelContactWrench() -> void;

  // engine state control
  auto init() -> void;

  virtual ~PhysicsEngine();
  PhysicsEngine(const PhysicsEngine&) = delete;
  PhysicsEngine& operator=(const PhysicsEngine&) = delete;

 private:
  PhysicsEngine();

 private:
  struct Imp;
  std::unique_ptr<Imp> imp_;
};
}  // namespace sire::physics
#endif