#include "sire/collision/collision_calculator.hpp"

#include <stdio.h>

#include <fstream>
#include <mutex>
#include <string>
#include <thread>

#include <hpp/fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <hpp/fcl/distance.h>
#include <hpp/fcl/math/transform.h>
#include <hpp/fcl/mesh_loader/assimp.h>
#include <hpp/fcl/mesh_loader/loader.h>
#include <hpp/fcl/shape/geometric_shapes.h>

#include <aris/core/reflection.hpp>
#include <aris/server/control_server.hpp>
namespace sire::collision {
struct CollisionCalculator::Imp {
  unique_ptr<aris::core::PointerArray<geometry::CollisionGeometry,
                                      aris::dynamic::Geometry>>
      dynamic_geometry_pool_;
  unique_ptr<aris::core::PointerArray<geometry::CollisionGeometry,
                                      aris::dynamic::Geometry>>
      anchored_geometry_pool_;
  fcl::DynamicAABBTreeCollisionManager dynamic_tree_;
  unordered_map<geometry::GeometryId, geometry::CollisionGeometry*>
      dynamic_objects_map_;
  fcl::DynamicAABBTreeCollisionManager anchored_tree_;
  unordered_map<geometry::GeometryId, geometry::CollisionGeometry*>
      anchored_objects_map_;
  unique_ptr<CollisionFilter> collision_filter_;
  aris::server::ControlServer* server_;
  aris::core::PointerArray<aris::dynamic::Part, aris::dynamic::Element>*
      part_pool_ptr_;
  aris::Size part_size_;
};
auto CollisionCalculator::resetCollisionFilter(CollisionFilter* filter)
    -> void {
  imp_->collision_filter_.reset(filter);
}
auto CollisionCalculator::collisionFilter() -> CollisionFilter& {
  return *imp_->collision_filter_;
}
auto CollisionCalculator::resetDynamicGeometryPool(
    aris::core::PointerArray<geometry::CollisionGeometry,
                             aris::dynamic::Geometry>* pool) -> void {
  imp_->dynamic_geometry_pool_.reset(pool);
}
auto CollisionCalculator::dynamicGeometryPool()
    -> aris::core::PointerArray<geometry::CollisionGeometry,
                                aris::dynamic::Geometry>& {
  return *imp_->dynamic_geometry_pool_;
}
auto CollisionCalculator::resetAnchoredGeometryPool(
    aris::core::PointerArray<geometry::CollisionGeometry,
                             aris::dynamic::Geometry>* pool) -> void {
  imp_->anchored_geometry_pool_.reset(pool);
}
auto CollisionCalculator::anchoredGeometryPool()
    -> aris::core::PointerArray<geometry::CollisionGeometry,
                                aris::dynamic::Geometry>& {
  return *imp_->anchored_geometry_pool_;
}
auto CollisionCalculator::addDynamicGeometry(
    geometry::CollisionGeometry& dynamic_geometry) -> bool {
  dynamic_geometry.updateLocation(nullptr);
  dynamic_geometry.getCollisionObject()->computeAABB();
  imp_->dynamic_tree_.registerObject(dynamic_geometry.getCollisionObject());
  imp_->dynamic_tree_.update();
  imp_->dynamic_objects_map_[dynamic_geometry.geometryId()] = &dynamic_geometry;
  return true;
}
auto CollisionCalculator::addAnchoredGeometry(
    geometry::CollisionGeometry& anchored_geometry) -> bool {
  anchored_geometry.updateLocation(nullptr);
  anchored_geometry.getCollisionObject()->computeAABB();
  imp_->anchored_tree_.registerObject(anchored_geometry.getCollisionObject());
  imp_->anchored_tree_.update();
  imp_->anchored_objects_map_[anchored_geometry.geometryId()] =
      &anchored_geometry;
  return false;
}
auto CollisionCalculator::removeGeometry() -> bool { return false; }
auto CollisionCalculator::clearDynamicGeometry() -> bool {
  imp_->dynamic_tree_.clear();
  return true;
}
auto CollisionCalculator::clearAnchoredGeometry() -> bool {
  imp_->anchored_tree_.clear();
  return true;
}
auto CollisionCalculator::updateLocation(double* part_pq) -> bool {
  if (!part_pq) {
    return true;
  }
  for (auto& dynamic_geometry : *imp_->dynamic_geometry_pool_) {
    double temp_pm[16];
    aris::dynamic::s_pq2pm(part_pq + 7 * dynamic_geometry.partId(), temp_pm);
    dynamic_geometry.updateLocation(temp_pm);
  }
  imp_->dynamic_tree_.update();
  return true;
}
auto CollisionCalculator::hasCollisions(fcl::CollisionCallBackBase& callback)
    -> void {
  CollidedObjectsCallback& callback_casted =
      dynamic_cast<CollidedObjectsCallback&>(callback);
  callback_casted.data.request.num_max_contacts = 10;
  callback_casted.data.request.enable_contact = false;
  callback_casted.data.request.gjk_tolerance = 2e-12;
  imp_->dynamic_tree_.collide(&callback_casted);
  imp_->dynamic_tree_.collide(&imp_->anchored_tree_, &callback_casted);
}
auto CollisionCalculator::init() -> void {
  for (auto& anchored_geometry : *imp_->anchored_geometry_pool_) {
    anchored_geometry.init();
    addAnchoredGeometry(anchored_geometry);
    imp_->collision_filter_->addGeometry(anchored_geometry);
  }
  for (auto& dynamic_geometry : *imp_->dynamic_geometry_pool_) {
    dynamic_geometry.init();
    addDynamicGeometry(dynamic_geometry);
    imp_->collision_filter_->addGeometry(dynamic_geometry);
  }
  imp_->collision_filter_->loadMatConfig();
  imp_->server_ = &aris::server::ControlServer::instance();
  imp_->part_pool_ptr_ =
      &dynamic_cast<aris::dynamic::Model*>(&imp_->server_->model())->partPool();
  imp_->part_size_ = imp_->part_pool_ptr_->size();
}
CollisionCalculator::CollisionCalculator() : imp_(new Imp) {}
CollisionCalculator::~CollisionCalculator(){};

ARIS_REGISTRATION {
  // aris::core::class_<aris::core::PointerArray<geometry::CollisionGeometry,
  //                                             aris::dynamic::Geometry>>(
  //     "GeometryPoolObject")
  //     .asRefArray();

  typedef aris::core::PointerArray<geometry::CollisionGeometry,
                                   aris::dynamic::Geometry>& (
      CollisionCalculator::*GeometryPoolFunc)();
  typedef sire::collision::CollisionFilter& (
      CollisionCalculator::*CollisionFilterPoolFunc)();
  aris::core::class_<CollisionCalculator>("CollisionCalculator")
      .prop("dynamic_geometry_pool",
            &CollisionCalculator::resetDynamicGeometryPool,
            GeometryPoolFunc(&CollisionCalculator::dynamicGeometryPool))
      .prop("anchored_geometry_pool",
            &CollisionCalculator::resetAnchoredGeometryPool,
            GeometryPoolFunc(&CollisionCalculator::anchoredGeometryPool))
      .prop("collision_filter", &CollisionCalculator::resetCollisionFilter,
            CollisionFilterPoolFunc(&CollisionCalculator::collisionFilter));
}
}  // namespace sire::collision