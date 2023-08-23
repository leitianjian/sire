#ifndef SIRE_PENETRATION_AS_POINT_PAIR_CALLBACK_HPP_
#define SIRE_PENETRATION_AS_POINT_PAIR_CALLBACK_HPP_

#include <string>

#include <hpp/fcl/broadphase/broadphase_callbacks.h>
#include <hpp/fcl/broadphase/broadphase_collision_manager.h>
#include <hpp/fcl/collision_data.h>
#include <hpp/fcl/collision_object.h>

#include "sire/physics/collision/collision_filter.hpp"
#include "sire/physics/common/penetration_as_point_pair.hpp"

namespace sire::physics::collision {
using namespace std;
using namespace hpp;
// Stored collided object when collision detected
class SIRE_API PenetrationAsPointPairCallback
    : public fcl::CollisionCallBackBase {
 public:
  fcl::CollisionRequest request;
  bool collide(fcl::CollisionObject* fcl_object_A_ptr,
               fcl::CollisionObject* fcl_object_B_ptr) override;
  auto calcDistance(const fcl::CollisionObject* a,
                    const fcl::CollisionObject* b,
                    const fcl::CollisionRequest& request,
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