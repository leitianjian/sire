#ifndef COLLISION_EXISTS_CALLBACK_H
#define COLLISION_EXISTS_CALLBACK_H

#include <string>

#include <hpp/fcl/broadphase/broadphase_callbacks.h>
#include <hpp/fcl/broadphase/broadphase_collision_manager.h>
#include <hpp/fcl/broadphase/default_broadphase_callbacks.h>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/collision_data.h>
#include <hpp/fcl/collision_object.h>

#include "sire/collision/collision_filter.hpp"
#include "sire/collision/geometry/collision_geometry.hpp"

namespace sire::collision {
using namespace std;
using namespace hpp;
// drake-based implementation
class SIRE_API CollisionExistsCallback : public fcl::CollisionCallBackBase {
 public:
  fcl::CollisionData data;
  bool collide(fcl::CollisionObject* o1, fcl::CollisionObject* o2);
  CollisionExistsCallback(CollisionFilter* filter);
  virtual ~CollisionExistsCallback() = default;

 private:
  CollisionFilter* filter_;
};
}  // namespace sire::collision
#endif