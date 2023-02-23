#include "sire/physics/collision/collided_objects_callback.hpp"

#include <hpp/fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <hpp/fcl/distance.h>
#include <hpp/fcl/math/transform.h>

#include <aris/core/reflection.hpp>
#include <aris/server/control_server.hpp>

#include "sire/physics/collision/geometry/collidable_geometry.hpp"

namespace sire::collision {
auto CollidedObjectsCallback::collide(fcl::CollisionObject* o1,
                                      fcl::CollisionObject* o2) -> bool {
  if (!filter_->canCollideWith(o1, o2)) {
    return false;
  }
  if (queryCollidedObject(o1, o2)) {
    return false;
  }
  auto* collision_data = static_cast<fcl::CollisionData*>(&data);
  const fcl::CollisionRequest& request = collision_data->request;
  fcl::CollisionResult& result = collision_data->result;

  fcl::collide(o1, o2, request, result);
  if (result.isCollision()) {
    addCollidedObject(o1, o2);
  }
  return false;
}
auto CollidedObjectsCallback::collidedObjectMap()
    -> set<std::pair<GeometryId, GeometryId>>& {
  return collidedObjectMap_;
}
auto CollidedObjectsCallback::addCollidedObject(fcl::CollisionObject* o1,
                                                fcl::CollisionObject* o2)
    -> void {
  if (o1 == o2) return;
  GeometryId id_1 =
      filter_->queryGeometryIdByPtr(o1->collisionGeometry().get());
  GeometryId id_2 =
      filter_->queryGeometryIdByPtr(o2->collisionGeometry().get());
  if (id_1 == id_2) return;
  if (id_1 < id_2) {
    collidedObjectMap_.insert(pair(id_1, id_2));
  } else {
    collidedObjectMap_.insert(pair(id_2, id_1));
  }
}
auto CollidedObjectsCallback::queryCollidedObject(fcl::CollisionObject* o1,
                                                  fcl::CollisionObject* o2)
    -> bool {
  if (o1 == o2) return true;
  GeometryId id_1 =
      filter_->queryGeometryIdByPtr(o1->collisionGeometry().get());
  GeometryId id_2 =
      filter_->queryGeometryIdByPtr(o2->collisionGeometry().get());
  if (id_1 == id_2) return true;
  if (id_1 < id_2) {
    return collidedObjectMap_.find(pair(id_1, id_2)) !=
           collidedObjectMap_.end();
  } else {
    return collidedObjectMap_.find(pair(id_1, id_2)) !=
           collidedObjectMap_.end();
  }
}
CollidedObjectsCallback::CollidedObjectsCallback(CollisionFilter* filter_in)
    : fcl::CollisionCallBackBase(), filter_(filter_in) {
  data.request.num_max_contacts = 10;
  data.request.enable_contact = false;
  data.request.gjk_tolerance = 2e-12;
};
}  // namespace sire::collision