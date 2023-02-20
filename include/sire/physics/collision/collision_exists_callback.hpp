#ifndef SIRE_COLLISION_EXISTS_CALLBACK_HPP_
#define SIRE_COLLISION_EXISTS_CALLBACK_HPP_

#include <string>

#include <hpp/fcl/broadphase/broadphase_callbacks.h>
#include <hpp/fcl/broadphase/broadphase_collision_manager.h>
#include <hpp/fcl/broadphase/default_broadphase_callbacks.h>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/collision_data.h>
#include <hpp/fcl/collision_object.h>

#include "sire/physics/collision/collision_filter.hpp"

namespace sire::collision {
using namespace std;
using namespace hpp;
namespace has_collisions {
struct CallbackData {
  /* Constructs the fully-specified callback data. The values are as described
     in the class documentation. The parameters are all aliased in the data and
     must remain valid at least as long as the CallbackData instance.

     @param collision_filter_in     The collision filter system. Aliased.  */
  explicit CallbackData(CollisionFilter* collision_filter_in);

  /* The collision filter system.  */
  CollisionFilter& collision_filter_;

  /* The parameters for the fcl object-object collision function.  */
  fcl::CollisionData collision_data_;

  /* The result of the collisions exist query.  */
  bool collision_exist_{false};
};
}  // namespace has_collisions
// drake-based implementation
class SIRE_API CollisionExistsCallback : public fcl::CollisionCallBackBase {
 public:
  has_collisions::CallbackData data;
  auto collide(fcl::CollisionObject* o1, fcl::CollisionObject* o2) -> bool override;
  CollisionExistsCallback(CollisionFilter* filter_in);
  virtual ~CollisionExistsCallback() = default;
};
}  // namespace sire::collision
#endif