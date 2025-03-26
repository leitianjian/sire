#ifndef SIRE_COLLIDED_OBJECTS_CALLBACK_HPP_
#define SIRE_COLLIDED_OBJECTS_CALLBACK_HPP_

#include <string>

#include <coal/broadphase/broadphase_callbacks.h>
#include <coal/broadphase/broadphase_collision_manager.h>
#include <coal/collision_data.h>
#include <coal/collision_object.h>

#include "sire/physics/collision/collision_filter.hpp"

namespace sire::physics::collision {
using namespace std;
using namespace coal;
// Stored collided object when collision detected
class SIRE_API CollidedObjectsCallback : public CollisionCallBackBase {
 public:
  CollisionData data;
  bool collide(CollisionObject* o1, CollisionObject* o2) override;
  auto addCollidedObject(CollisionObject* o1, CollisionObject* o2)
      -> void;
  auto queryCollidedObject(CollisionObject* o1, CollisionObject* o2)
      -> bool;
  auto collidedObjectMap() -> set<CollisionObjectsPair>&;
  CollidedObjectsCallback(CollisionFilter* filter);
  virtual ~CollidedObjectsCallback() = default;

 private:
  CollisionFilter* filter_;
  set<CollisionObjectsPair> collidedObjectMap_;
};
}  // namespace sire::physics::collision
#endif