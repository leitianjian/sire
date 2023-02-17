#ifndef SIRE_COLLISION_ENGINE_HPP_
#define SIRE_COLLISION_ENGINE_HPP_

#include <map>
#include <string>

#include <hpp/fcl/broadphase/broadphase_callbacks.h>
#include <hpp/fcl/broadphase/broadphase_collision_manager.h>
#include <hpp/fcl/broadphase/default_broadphase_callbacks.h>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/collision_data.h>
#include <hpp/fcl/collision_object.h>

#include <aris/core/expression_calculator.hpp>

#include "sire/collision/collided_objects_callback.hpp"
#include "sire/collision/collision_filter.hpp"
#include "sire/collision/geometry/collision_geometry.hpp"

namespace sire::collision {
using namespace std;
using namespace hpp;
/* drake-based implementation
 * filter和geometry配置都先读进去，之后通过init进行碰撞管理器的初始化
 * 需要修改位姿更新方式，使用无锁数据同步从transfer中取数据
 */
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
  auto updateLocation(double* part_pq) -> bool;
  auto hasCollisions() -> bool;
  auto collidedObjects(CollidedObjectsCallback& callback) -> bool;
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