#include "sire/physics/collision/collision_exists_callback.hpp"

#include <stdio.h>

#include <fstream>
#include <mutex>
#include <string>
#include <thread>

#include <hpp/fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <hpp/fcl/distance.h>
#include <hpp/fcl/math/transform.h>
#include <hpp/fcl/mesh_loader/assimp.h>
#include <hpp/fcl/mesh_loader/loader.h>
#include <hpp/fcl/shape/geometric_shapes.h>

#include <aris/core/reflection.hpp>
#include <aris/server/control_server.hpp>

#include "sire/core/sire_assert.hpp"

namespace sire::collision {
namespace has_collisions {
CallbackData::CallbackData(CollisionFilter* collision_filter_in)
    : collision_filter_(*collision_filter_in) {
  SIRE_DEMAND(collision_filter_in != nullptr);
  collision_data_.request.num_max_contacts = 1;
  collision_data_.request.num_max_contacts = 1;
  collision_data_.request.enable_contact = false;
  collision_data_.request.gjk_tolerance = 2e-12;
}
}  // namespace has_collisions
auto CollisionExistsCallback::collide(fcl::CollisionObject* o1,
                                      fcl::CollisionObject* o2) -> bool {
  if (!data.collision_filter_.canCollideWith(o1, o2)) return false;
  if (data.collision_data_.done) return data.collision_exist_;

  SIRE_ASSERT(data.collision_data_.request.num_max_contacts == 1);

  fcl::collide(o1, o2, data.collision_data_.request,
               data.collision_data_.result);

  data.collision_exist_ = data.collision_data_.result.isCollision();
  if (data.collision_exist_) {
    data.collision_data_.done = true;
  }
  return data.collision_exist_;
}
CollisionExistsCallback::CollisionExistsCallback(CollisionFilter* filter_in)
    : fcl::CollisionCallBackBase(), data(filter_in) {
  SIRE_DEMAND(filter_in != nullptr);
};
}  // namespace sire::collision