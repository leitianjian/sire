#ifndef SIRE_COLLISION_FILTER_HPP
#define SIRE_COLLISION_FILTER_HPP

#include <map>
#include <string>

#include <hpp/fcl/broadphase/broadphase_callbacks.h>
#include <hpp/fcl/broadphase/broadphase_collision_manager.h>
#include <hpp/fcl/broadphase/default_broadphase_callbacks.h>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/collision_data.h>
#include <hpp/fcl/collision_object.h>

#include <aris/core/expression_calculator.hpp>

#include "sire/physics/geometry/collidable_geometry.hpp"
#include "sire/physics/physics.hpp"

namespace sire::physics::collision {
using namespace std;
using namespace hpp;
// drake-based implementation
enum CollisionRelationship {
  kUnfiltered = 0,
  kFiltered = 1,
};
class SIRE_API CollisionFilter {
 public:
  using GeometryMap = map<physics::GeometryId, CollisionRelationship>;
  using FilterState = map<physics::GeometryId, GeometryMap>;
  auto addGeometry(geometry::CollidableGeometry& geo) -> bool;
  auto addGeometry(GeometryId id, fcl::CollisionObject* obj_ptr) -> bool;
  auto updateGeometry(GeometryId id, fcl::CollisionObject* obj_ptr) -> bool;
  auto removeGeometry(GeometryId id) -> bool;
  auto canCollideWith(GeometryId id_1, GeometryId id_2) -> bool;
  auto canCollideWith(const fcl::CollisionObject* o1,
                      const fcl::CollisionObject* o2) -> bool;
  inline auto queryGeometryIdByPtr(const fcl::CollisionGeometry* ptr)
      -> GeometryId;
  inline auto queryGeometryIdByPtr(fcl::CollisionGeometry* ptr) -> GeometryId;
  auto containsGeometry(GeometryId id) -> bool;
  auto setStateMat(aris::core::Matrix mat) -> void;
  auto stateMat() -> aris::core::Matrix;
  auto loadMatConfig() -> void;
  auto saveMatConfig() -> void;
  auto init() -> void;
  CollisionFilter();
  virtual ~CollisionFilter();
  ARIS_DECLARE_BIG_FOUR(CollisionFilter);

 private:
  struct Imp;
  aris::core::ImpPtr<Imp> imp_;
};
}  // namespace sire::physics::collision
#endif