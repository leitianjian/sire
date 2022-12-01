#ifndef COLLISION_ENGINE_H
#define COLLISION_ENGINE_H

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

class SIRE_API FilteredCollisionCallback : public fcl::CollisionCallBackBase {
 public:
  fcl::CollisionData data;
  bool collide(fcl::CollisionObject* o1, fcl::CollisionObject* o2);
  FilteredCollisionCallback(CollisionFilter* filter);
  virtual ~FilteredCollisionCallback() = default;

 private:
  CollisionFilter* filter_;
};

class SIRE_API FilteredCollidedObjectsCallback
    : public fcl::CollisionCallBackBase {
 public:
  fcl::CollisionData data;
  bool collide(fcl::CollisionObject* o1, fcl::CollisionObject* o2);
  auto addCollidedObject(fcl::CollisionObject* o1, fcl::CollisionObject* o2)
      -> void;
  auto queryCollidedObject(fcl::CollisionObject* o1, fcl::CollisionObject* o2)
      -> bool;
  auto collidedObjectMap()
      -> set<std::pair<geometry::GeometryId, geometry::GeometryId>>&;
  FilteredCollidedObjectsCallback(CollisionFilter* filter);
  virtual ~FilteredCollidedObjectsCallback() = default;

 private:
  CollisionFilter* filter_;
  set<std::pair<geometry::GeometryId, geometry::GeometryId>> collidedObjectMap_;
};

// filter和geometry配置都先读进去，之后通过init进行碰撞管理器的初始化
class SIRE_API CollisionEngine {
 public:
  auto resetCollisionFilter(CollisionFilter* filter) -> void;
  auto collisionFilter() -> CollisionFilter&;
  auto resetDynamicGeometryPool(
      aris::core::PointerArray<geometry::CollisionGeometry,
                               aris::dynamic::Geometry>* pool) -> void;
  auto dynamicGeometryPool()
      -> aris::core::PointerArray<geometry::CollisionGeometry,
                                  aris::dynamic::Geometry>&;
  auto resetAnchoredGeometryPool(
      aris::core::PointerArray<geometry::CollisionGeometry,
                               aris::dynamic::Geometry>* pool) -> void;
  auto anchoredGeometryPool()
      -> aris::core::PointerArray<geometry::CollisionGeometry,
                                  aris::dynamic::Geometry>&;
  auto addDynamicGeometry(geometry::CollisionGeometry& dynamic_geometry)
      -> bool;
  auto addAnchoredGeometry(geometry::CollisionGeometry& anchored_geometry)
      -> bool;
  auto removeGeometry() -> bool;
  auto clearDynamicGeometry() -> bool;
  auto clearAnchoredGeometry() -> bool;
  auto updateLocation() -> bool;
  auto hasCollisions(FilteredCollidedObjectsCallback& callback) -> void;
  auto init() -> void;
  CollisionEngine();
  virtual ~CollisionEngine();
  CollisionEngine(const CollisionEngine& other) = delete;
  CollisionEngine(CollisionEngine&& other) = delete;
  CollisionEngine& operator=(const CollisionEngine& other) = delete;
  CollisionEngine& operator=(CollisionEngine&& other) = delete;

 private:
  struct Imp;
  unique_ptr<Imp> imp_;
};
}  // namespace sire::collision
#endif