#include "sire/physics/collision/collision_detection.hpp"

#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>

#include <coal/broadphase/broadphase_callbacks.h>
#include <coal/broadphase/broadphase_collision_manager.h>
#include <coal/broadphase/broadphase_dynamic_AABB_tree.h>
#include <coal/broadphase/default_broadphase_callbacks.h>
#include <coal/collision.h>
#include <coal/collision_data.h>
#include <coal/collision_object.h>
#include <coal/distance.h>
#include <coal/math/transform.h>
#include <coal/mesh_loader/assimp.h>
#include <coal/mesh_loader/loader.h>
#include <coal/shape/geometric_shapes.h>

#include <aris/core/reflection.hpp>
#include <aris/server/control_server.hpp>

#include "sire/core/constants.hpp"
#include "sire/core/sire_assert.hpp"
#include "sire/physics/collision/collision_exists_callback.hpp"
#include "sire/physics/collision/height_field_contact_callback.hpp"
#include "sire/physics/collision/penetration_as_point_pair_callback.hpp"
#include "sire/physics/geometry/sphere_collision_geometry.hpp"
#include "sire/physics/physics_engine.hpp"

namespace sire::physics::collision {
using namespace coal;

struct CollisionDetection::Imp {
  // Owned resources
  DynamicAABBTreeCollisionManager dynamic_tree_;
  DynamicAABBTreeCollisionManager anchored_tree_;

  // Physical Engine managed resources
  aris::core::PointerArray<geometry::CollidableGeometry,
                           aris::dynamic::Geometry>* geometry_pool_ptr_{
      nullptr};
  std::unordered_map<GeometryId, geometry::CollidableGeometry*>*
      dynamic_objects_map_ptr_{nullptr};
  std::unordered_map<GeometryId, geometry::CollidableGeometry*>*
      anchored_objects_map_ptr_{nullptr};
  CollisionFilter* collision_filter_ptr_{nullptr};

  Imp() : geometry_pool_ptr_(nullptr), collision_filter_ptr_(nullptr) {}
};
CollisionDetection::CollisionDetection() : imp_(new Imp) {}
CollisionDetection::~CollisionDetection() = default;
ARIS_DEFINE_BIG_FOUR_CPP(CollisionDetection);

// Update dynamic_tree_ according to Model geometry position
auto CollisionDetection::updateLocation(const aris::dynamic::Model* model_ptr)
    -> bool {
  SIRE_ASSERT(model_ptr != nullptr);
  for (auto& geometry : *imp_->geometry_pool_ptr_) {
    if (geometry.isDynamic()) {
      double temp_pm[16];
      model_ptr->partPool().at(geometry.partId()).getPm(temp_pm);
      // NaN guard: if part pose is invalid (e.g. after integration divergence),
      // skip this update so coal's BVH doesn't segfault on bad data.
      bool pose_valid = true;
      for (int k = 0; k < 16; ++k) {
        if (!std::isfinite(temp_pm[k])) {
          pose_valid = false;
          break;
        }
      }
      if (!pose_valid) {
        std::ostringstream oss;
        oss << "[Sire] CollisionDetection: NaN pose for geometry "
            << geometry.id() << " part " << geometry.partId();
        throw std::runtime_error(oss.str());
      }
      geometry.updateLocation(temp_pm);
    }
  }
  imp_->dynamic_tree_.update();
  return true;
}
auto CollisionDetection::updateLocation(const double* part_pq) -> bool {
  SIRE_DEMAND(part_pq != nullptr);
  for (auto& geometry : *imp_->geometry_pool_ptr_) {
    if (geometry.isDynamic()) {
      double temp_pm[16];
      aris::dynamic::s_pq2pm(part_pq + 7 * geometry.partId(), temp_pm);
      // NaN guard for part_pq input
      bool pose_valid = true;
      for (int k = 0; k < 16; ++k) {
        if (!std::isfinite(temp_pm[k])) {
          pose_valid = false;
          break;
        }
      }
      if (!pose_valid) {
        std::ostringstream oss;
        oss << "[Sire] CollisionDetection: NaN pose for geometry "
            << geometry.id() << " part " << geometry.partId()
            << " (from part_pq)";
        throw std::runtime_error(oss.str());
      }
      geometry.updateLocation(temp_pm);
    }
  }
  imp_->dynamic_tree_.update();
  return true;
}

auto CollisionDetection::updateLocation(const double* part_pq,
                                        const sire::Size part_id) -> bool {
  SIRE_DEMAND(part_pq != nullptr);
  // Should not update ground part_pq
  SIRE_DEMAND(part_id != 0);
  for (auto& geometry : *imp_->geometry_pool_ptr_) {
    if (geometry.isDynamic() && geometry.partId() == part_id) {
      double temp_pm[16];
      aris::dynamic::s_pq2pm(part_pq, temp_pm);
      geometry.updateLocation(temp_pm);
    }
  }
  imp_->dynamic_tree_.update();
  return true;
}
auto CollisionDetection::addDynamicGeometry2FCL(
    geometry::CollidableGeometry& dynamic_geometry) -> bool {
  dynamic_geometry.init();
  dynamic_geometry.updateLocation(nullptr);
  dynamic_geometry.getCollisionObject()->computeAABB();
  imp_->dynamic_tree_.registerObject(dynamic_geometry.getCollisionObject());
  imp_->dynamic_tree_.update();
  return true;
}
auto CollisionDetection::addAnchoredGeometry2FCL(
    geometry::CollidableGeometry& anchored_geometry) -> bool {
  anchored_geometry.init();
  anchored_geometry.updateLocation(nullptr);
  anchored_geometry.getCollisionObject()->computeAABB();
  imp_->anchored_tree_.registerObject(anchored_geometry.getCollisionObject());
  imp_->anchored_tree_.update();
  return true;
}
auto CollisionDetection::updateDynamicGeometriesMananger() -> void {
  imp_->dynamic_tree_.update();
}
auto CollisionDetection::updateAnchoredGeometriesMananger() -> void {
  imp_->anchored_tree_.update();
}
auto CollisionDetection::clearDynamicGeometries() -> bool {
  imp_->dynamic_tree_.clear();
  return true;
}
auto CollisionDetection::clearAnchoredGeometries() -> bool {
  imp_->anchored_tree_.clear();
  return true;
}
auto CollisionDetection::hasCollisions() -> bool {
  CollisionExistsCallback callback(imp_->collision_filter_ptr_);
  imp_->dynamic_tree_.collide(&callback);
  imp_->dynamic_tree_.collide(&imp_->anchored_tree_, &callback);
  return callback.data.collision_exist_;
}

auto CollisionDetection::computePointPairPenetration(
    std::vector<common::PenetrationAsPointPair>& contacts) -> bool {
  PenetrationAsPointPairCallback callback(imp_->collision_filter_ptr_,
                                          &contacts);
  imp_->dynamic_tree_.collide(&callback);
  imp_->dynamic_tree_.collide(&imp_->anchored_tree_, &callback);
  return true;
}
auto CollisionDetection::computeHeightFieldPenetration(
    std::vector<common::PenetrationAsPointPair>& contacts) -> bool {
  HeightFieldContactCallback callback(imp_->collision_filter_ptr_, &contacts);
  imp_->dynamic_tree_.collide(&callback);
  imp_->dynamic_tree_.collide(&imp_->anchored_tree_, &callback);
  return true;
}
auto CollisionDetection::numDynamicGeometries() -> sire::Size {
  return imp_->dynamic_tree_.size();
}
auto CollisionDetection::init(physics::PhysicsEngine* engine_ptr) -> void {
  imp_->dynamic_objects_map_ptr_ = &engine_ptr->dynamicObjectsMap();
  imp_->anchored_objects_map_ptr_ = &engine_ptr->anchoredObjectsMap();
  imp_->collision_filter_ptr_ = &engine_ptr->collisionFilter();
  imp_->geometry_pool_ptr_ = &engine_ptr->geometryPool();
}

ARIS_REGISTRATION {
  aris::core::class_<CollisionDetection>("CollisionDetection");
}
}  // namespace sire::physics::collision