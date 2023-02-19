#include "sire/physics/collision/collision_module.hpp"

#include <aris/core/reflection.hpp>

namespace sire::collision {
struct CollisionModule::Imp {
  unique_ptr<CollisionDetectionEngine> collision_detection_engine_;
};
auto CollisionModule::getCollisionDetectionEngine() -> CollisionDetectionEngine& {
  return *imp_->collision_detection_engine_;
}
auto CollisionModule::resetCollisionDetectionEngine(CollisionDetectionEngine* engine)
    -> void {
  imp_->collision_detection_engine_.reset(engine);
}
auto CollisionModule::init() -> void { imp_->collision_detection_engine_->init(); }
CollisionModule::CollisionModule() : imp_(new Imp) {}
CollisionModule::~CollisionModule() = default;
ARIS_REGISTRATION {
  aris::core::class_<CollisionModule>("SireCollisionModule")
      .inherit<core::SireModuleBase>()
      .prop("collision_detection_engine", &CollisionModule::resetCollisionDetectionEngine,
            &CollisionModule::getCollisionDetectionEngine);
}
}  // namespace sire::modules