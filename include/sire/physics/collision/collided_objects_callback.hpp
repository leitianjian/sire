#ifndef SIRE_COLLIDED_OBJECTS_CALLBACK_HPP_
#define SIRE_COLLIDED_OBJECTS_CALLBACK_HPP_

#include <string>
#include <set>

#include <coal/broadphase/broadphase_callbacks.h>
#include <coal/broadphase/broadphase_collision_manager.h>
#include <coal/broadphase/default_broadphase_callbacks.h>
#include <coal/collision_data.h>
#include <coal/collision_object.h>

#include "sire/physics/collision/collision_filter.hpp"

namespace sire::physics::collision {
using CollisionObjectsPair =
    std::pair<sire::geometry::GeometryId, sire::geometry::GeometryId>;
// Stored collided object when collision detected
class CollidedObjectsCallback : public coal::CollisionCallBackBase {
 public:
  coal::CollisionData data;
  bool collide(coal::CollisionObject* o1, coal::CollisionObject* o2) override;
  auto addCollidedObject(coal::CollisionObject* o1, coal::CollisionObject* o2) -> void;
  auto queryCollidedObject(coal::CollisionObject* o1, coal::CollisionObject* o2) -> bool;
  auto collidedObjectMap() -> std::set<CollisionObjectsPair>&;
  CollidedObjectsCallback(CollisionFilter* filter);
  virtual ~CollidedObjectsCallback() = default;

 private:
  CollisionFilter* filter_;
  std::set<CollisionObjectsPair> collidedObjectMap_;
};
}  // namespace sire::physics::collision
#endif