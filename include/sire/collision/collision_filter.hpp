#ifndef COLLISION_FILTER_H
#define COLLISION_FILTER_H

#include "sire/geometry/geometry.hpp"
#include <aris/core/expression_calculator.hpp>
#include <hpp/fcl/broadphase/broadphase_callbacks.h>
#include <hpp/fcl/broadphase/broadphase_collision_manager.h>
#include <hpp/fcl/broadphase/default_broadphase_callbacks.h>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/collision_data.h>
#include <hpp/fcl/collision_object.h>
#include <map>
#include <string>

namespace sire::collision {
using namespace std;
using namespace hpp;
// drake-based implementation
enum CollisionRelationship {
  kUnfiltered = 0,
  kFiltered = 1,
};
class SIRE_API CollisionFilter {
 public:
  using GeometryMap = map<geometry::GeometryId, CollisionRelationship>;
  using FilterState = map<geometry::GeometryId, GeometryMap>;
  auto addGeometry(geometry::CollisionGeometry& geo) -> bool;
  auto addGeometry(geometry::GeometryId id, fcl::CollisionObject* obj_ptr)
      -> bool;
  auto updateGeometry(geometry::GeometryId id, fcl::CollisionObject* obj_ptr)
      -> bool;
  auto removeGeometry(geometry::GeometryId id) -> bool;
  auto canCollideWith(geometry::GeometryId id_1, geometry::GeometryId id_2)
      -> bool;
  auto canCollideWith(fcl::CollisionObject* o1, fcl::CollisionObject* o2)
      -> bool;
  inline auto queryGeometryIdByPtr(const fcl::CollisionGeometry* ptr)
      -> geometry::GeometryId;
  inline auto queryGeometryIdByPtr(fcl::CollisionGeometry* ptr)
      -> geometry::GeometryId;
  auto containsGeometry(geometry::GeometryId id) -> bool;
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
}  // namespace sire::collision
#endif