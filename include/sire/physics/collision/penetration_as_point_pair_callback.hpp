#ifndef SIRE_PENETRATION_AS_POINT_PAIR_CALLBACK_HPP_
#define SIRE_PENETRATION_AS_POINT_PAIR_CALLBACK_HPP_

#include <string>

#include <coal/broadphase/broadphase_callbacks.h>
#include <coal/broadphase/broadphase_collision_manager.h>
#include <coal/collision_data.h>
#include <coal/collision_object.h>

#include "sire/physics/collision/collision_filter.hpp"
#include "sire/physics/common/penetration_as_point_pair.hpp"

namespace sire::physics::collision {
using namespace std;
using namespace coal;
// Stored collided object when collision detected
class SIRE_API PenetrationAsPointPairCallback
    : public CollisionCallBackBase {
 public:
  CollisionRequest request;
  bool collide(CollisionObject* fcl_object_A_ptr,
               CollisionObject* fcl_object_B_ptr) override;
  auto calcDistance(const CollisionObject* a,
                    const CollisionObject* b,
                    const CollisionRequest& request,
                    common::PenetrationAsPointPair* pair_data) -> void;

  PenetrationAsPointPairCallback(
      CollisionFilter* filter_in,
      vector<common::PenetrationAsPointPair>* point_pairs_in);
  virtual ~PenetrationAsPointPairCallback() = default;

 private:
  CollisionFilter* filter_;
  vector<common::PenetrationAsPointPair>* point_pairs;
};
}  // namespace sire::physics::collision
#endif