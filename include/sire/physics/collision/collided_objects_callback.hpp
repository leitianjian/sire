#ifndef SIRE_COLLIDED_OBJECTS_CALLBACK_HPP_
#define SIRE_COLLIDED_OBJECTS_CALLBACK_HPP_

#include <string>

#include <hpp/fcl/broadphase/broadphase_callbacks.h>
#include <hpp/fcl/broadphase/broadphase_collision_manager.h>
#include <hpp/fcl/collision_data.h>
#include <hpp/fcl/collision_object.h>

#include "sire/physics/collision/collision_filter.hpp"
#include "sire/physics/collision/geometry/collision_geometry.hpp"

namespace sire::collision {
using namespace std;
using namespace hpp;
// Stored collided object when collision detected
class SIRE_API CollidedObjectsCallback : public fcl::CollisionCallBackBase {
 public:
  fcl::CollisionData data;
  bool collide(fcl::CollisionObject* o1, fcl::CollisionObject* o2) override;
  auto addCollidedObject(fcl::CollisionObject* o1, fcl::CollisionObject* o2)
      -> void;
  auto queryCollidedObject(fcl::CollisionObject* o1, fcl::CollisionObject* o2)
      -> bool;
  auto collidedObjectMap() -> set<CollisionObjectsPair>&;
  CollidedObjectsCallback(CollisionFilter* filter);
  virtual ~CollidedObjectsCallback() = default;

 private:
  CollisionFilter* filter_;
  set<CollisionObjectsPair> collidedObjectMap_;
};
}  // namespace sire::collision
#endif