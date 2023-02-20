#ifndef SIRE_COLLISION_DETECTION_ENGINE_HPP_
#define SIRE_COLLISION_DETECTION_ENGINE_HPP_

#include <map>
#include <string>

#include <hpp/fcl/broadphase/broadphase_callbacks.h>
#include <hpp/fcl/broadphase/broadphase_collision_manager.h>
#include <hpp/fcl/broadphase/default_broadphase_callbacks.h>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/collision_data.h>
#include <hpp/fcl/collision_object.h>

#include <aris/core/expression_calculator.hpp>

#include "sire/physics/collision/collided_objects_callback.hpp"
#include "sire/physics/collision/collision_filter.hpp"
#include "sire/physics/collision/geometry/collidable_geometry.hpp"

namespace sire::collision {
using namespace std;
using namespace hpp;
/* drake-based implementation
 * filter和geometry配置都先读进去，之后通过init进行碰撞管理器的初始化
 * 使用无锁数据同步从transfer中取数据
 */
class SIRE_API CollisionDetectionEngine {
 public:
  auto resetCollisionFilter(CollisionFilter* filter) -> void;
  auto collisionFilter() -> CollisionFilter&;
  auto resetDynamicGeometryPool(
      aris::core::PointerArray<geometry::CollidableGeometry,
                               aris::dynamic::Geometry>* pool) -> void;
  auto dynamicGeometryPool()
      -> aris::core::PointerArray<geometry::CollidableGeometry,
                                  aris::dynamic::Geometry>&;
  auto resetAnchoredGeometryPool(
      aris::core::PointerArray<geometry::CollidableGeometry,
                               aris::dynamic::Geometry>* pool) -> void;
  auto anchoredGeometryPool()
      -> aris::core::PointerArray<geometry::CollidableGeometry,
                                  aris::dynamic::Geometry>&;
  auto addDynamicGeometry(geometry::CollidableGeometry& dynamic_geometry)
      -> bool;
  auto addAnchoredGeometry(geometry::CollidableGeometry& anchored_geometry)
      -> bool;
  auto removeGeometry() -> bool;
  auto clearDynamicGeometry() -> bool;
  auto clearAnchoredGeometry() -> bool;
  auto updateLocation() -> bool;
  auto updateLocation(double* part_pq) -> bool;
  auto hasCollisions() -> bool;
  auto collidedObjects(CollidedObjectsCallback& callback) -> bool;
  auto init() -> void;
  CollisionDetectionEngine();
  virtual ~CollisionDetectionEngine();
  CollisionDetectionEngine(const CollisionDetectionEngine& other) = delete;
  CollisionDetectionEngine(CollisionDetectionEngine&& other) = delete;
  CollisionDetectionEngine& operator=(const CollisionDetectionEngine& other) =
      delete;
  CollisionDetectionEngine& operator=(CollisionDetectionEngine&& other) =
      delete;

 private:
  struct Imp;
  unique_ptr<Imp> imp_;
};
}  // namespace sire::collision
#endif