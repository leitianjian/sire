#ifndef SIRE_COLLISION_DETECTION_HPP_
#define SIRE_COLLISION_DETECTION_HPP_

#include <map>
#include <string>
#include <vector>

#include <hpp/fcl/broadphase/broadphase_callbacks.h>
#include <hpp/fcl/broadphase/broadphase_collision_manager.h>
#include <hpp/fcl/broadphase/default_broadphase_callbacks.h>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/collision_data.h>
#include <hpp/fcl/collision_object.h>

#include <aris/core/expression_calculator.hpp>
#include <aris/dynamic/model.hpp>

#include "sire/core/constants.hpp"
#include "sire/core/geometry/shape_base.hpp"
#include "sire/physics/collision/collided_objects_callback.hpp"
#include "sire/physics/collision/collision_filter.hpp"
#include "sire/physics/common/penetration_as_point_pair.hpp"
#include "sire/physics/geometry/collidable_geometry.hpp"

namespace sire::physics {
class PhysicsEngine;
namespace collision {
using namespace std;
using namespace hpp;

/* drake-based implementation
 * filter和geometry配置都先读进去，之后通过init进行碰撞管理器的初始化
 * 使用无锁数据同步从transfer中取数据
 */
class SIRE_API CollisionDetection {
 public:
  auto addDynamicGeometry2FCL(geometry::CollidableGeometry& dynamic_geometry)
      -> bool;
  auto addAnchoredGeometry2FCL(geometry::CollidableGeometry& anchored_geometry)
      -> bool;
  auto clearDynamicGeometries() -> bool;
  auto clearAnchoredGeometries() -> bool;
  // API droped
  auto updateLocation(const aris::dynamic::Model* model_ptr) -> bool;
  auto updateLocation(const double* part_pq) -> bool;
  auto updateLocation(const double* part_pq, const sire::Size part_id) -> bool;
  auto updateDynamicGeometriesMananger() -> void;
  auto updateAnchoredGeometriesMananger() -> void;
  auto numDynamicGeometries() -> sire::Size;
  auto hasCollisions() -> bool;
  auto collidedObjects(CollidedObjectsCallback& callback) -> bool;
  auto computePointPairPenetration(
      std::vector<common::PenetrationAsPointPair>& contacts) -> bool;
  auto init(physics::PhysicsEngine* engine_ptr) -> void;

  CollisionDetection();
  virtual ~CollisionDetection();
  ARIS_DECLARE_BIG_FOUR(CollisionDetection);

 private:
  struct Imp;
  aris::core::ImpPtr<Imp> imp_;
};
}  // namespace collision
}  // namespace sire::physics
#endif