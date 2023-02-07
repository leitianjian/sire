#ifndef COLLISION_MODULE_H_
#define COLLISION_MODULE_H_

#include "sire/collision/collision_engine.hpp"
#include "sire/core/module_base.hpp"
#include <sire_lib_export.h>

namespace sire::modules {
using namespace std;
class SIRE_API CollisionModule : public core::SireModuleBase {
 public:
  auto getCollisionEngine() -> collision::CollisionEngine&;
  auto resetCollisionEngine(collision::CollisionEngine* engine) -> void;
  auto virtual init() -> void override;
  CollisionModule();
  virtual ~CollisionModule();

 private:
  struct Imp;
  unique_ptr<Imp> imp_;
};
}  // namespace sire::modules
#endif