#ifndef SIRE_COLLISION_MODULE_HPP_
#define SIRE_COLLISION_MODULE_HPP_

#include <sire_lib_export.h>

#include "sire/physics/collision/collision_detection_engine.hpp"
#include "sire/core/module_base.hpp"

namespace sire::collision {
using namespace std;
class SIRE_API CollisionModule : public core::SireModuleBase {
 public:
  auto getCollisionDetectionEngine() -> collision::CollisionDetectionEngine&;
  auto resetCollisionDetectionEngine(collision::CollisionDetectionEngine* engine) -> void;
  auto virtual init() -> void override;
  CollisionModule();
  virtual ~CollisionModule();

 private:
  struct Imp;
  unique_ptr<Imp> imp_;
};
}  // namespace sire::modules
#endif