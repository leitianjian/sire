#include "sire/collision/collision_module.hpp"
#include <aris/core/reflection.hpp>

namespace sire::modules {
struct CollisionModule::Imp {
  unique_ptr<collision::CollisionEngine> collision_engine_;
};
auto CollisionModule::getCollisionEngine() -> collision::CollisionEngine& {
  return *imp_->collision_engine_;
}
auto CollisionModule::resetCollisionEngine(collision::CollisionEngine* engine)
    -> void {
  imp_->collision_engine_.reset(engine);
}
auto CollisionModule::init() -> void { imp_->collision_engine_->init(); }
CollisionModule::CollisionModule() : imp_(new Imp) {}
CollisionModule::~CollisionModule() = default;
ARIS_REGISTRATION {
  aris::core::class_<CollisionModule>("SireCollisionModule")
      .inherit<core::SireModuleBase>()
      .prop("collision_engine", &CollisionModule::resetCollisionEngine,
            &CollisionModule::getCollisionEngine);
}
}  // namespace sire::modules