#include "sire/collision/collision_exists_callback.hpp"
#include <aris/core/reflection.hpp>
#include <aris/server/control_server.hpp>
#include <hpp/fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <hpp/fcl/distance.h>
#include <hpp/fcl/math/transform.h>
#include <hpp/fcl/mesh_loader/assimp.h>
#include <hpp/fcl/mesh_loader/loader.h>
#include <hpp/fcl/shape/geometric_shapes.h>
#include <fstream>
#include <io.h>
#include <mutex>
#include <stdio.h>
#include <string>
#include <thread>
namespace sire::collision {
auto CollisionExistsCallback::collide(fcl::CollisionObject* o1,
                                      fcl::CollisionObject* o2) -> bool {
  if (!filter_->canCollideWith(o1, o2)) {
    return false;
  }
  auto* collision_data = static_cast<fcl::CollisionData*>(&data);
  const fcl::CollisionRequest& request = collision_data->request;
  fcl::CollisionResult& result = collision_data->result;

  if (collision_data->done) return true;

  fcl::collide(o1, o2, request, result);

  if (result.isCollision() &&
      result.numContacts() >= request.num_max_contacts) {
    collision_data->done = true;
  }
  return collision_data->done;
}
CollisionExistsCallback::CollisionExistsCallback(CollisionFilter* filter)
    : fcl::CollisionCallBackBase() {
  filter_ = filter;
};
}  // namespace sire::collision