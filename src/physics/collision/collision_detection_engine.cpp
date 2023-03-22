#include "sire/physics/collision/collision_detection_engine.hpp"

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

#include "sire/core/constants.hpp"
#include "sire/physics/collision/collision_exists_callback.hpp"
#include "sire/transfer/part_pq_transfer.hpp"
namespace sire::collision {
struct CollisionDetectionEngine::Imp {
  unique_ptr<aris::core::PointerArray<geometry::CollidableGeometry,
                                      aris::dynamic::Geometry>>
      dynamic_geometry_pool_;
  unique_ptr<aris::core::PointerArray<geometry::CollidableGeometry,
                                      aris::dynamic::Geometry>>
      anchored_geometry_pool_;
  fcl::DynamicAABBTreeCollisionManager dynamic_tree_;
  unordered_map<GeometryId, geometry::CollidableGeometry*> dynamic_objects_map_;
  fcl::DynamicAABBTreeCollisionManager anchored_tree_;
  unordered_map<GeometryId, geometry::CollidableGeometry*>
      anchored_objects_map_;
  unique_ptr<CollisionFilter> collision_filter_;

  aris::server::ControlServer* server_;
  aris::core::PointerArray<aris::dynamic::Part, aris::dynamic::Element>*
      part_pool_ptr_;
  sire::Size part_size_;
};
auto CollisionDetectionEngine::resetCollisionFilter(CollisionFilter* filter)
    -> void {
  imp_->collision_filter_.reset(filter);
}
auto CollisionDetectionEngine::collisionFilter() -> CollisionFilter& {
  return *imp_->collision_filter_;
}
auto CollisionDetectionEngine::resetDynamicGeometryPool(
    aris::core::PointerArray<geometry::CollidableGeometry,
                             aris::dynamic::Geometry>* pool) -> void {
  imp_->dynamic_geometry_pool_.reset(pool);
}
auto CollisionDetectionEngine::dynamicGeometryPool()
    -> aris::core::PointerArray<geometry::CollidableGeometry,
                                aris::dynamic::Geometry>& {
  return *imp_->dynamic_geometry_pool_;
}
auto CollisionDetectionEngine::resetAnchoredGeometryPool(
    aris::core::PointerArray<geometry::CollidableGeometry,
                             aris::dynamic::Geometry>* pool) -> void {
  imp_->anchored_geometry_pool_.reset(pool);
}
auto CollisionDetectionEngine::anchoredGeometryPool()
    -> aris::core::PointerArray<geometry::CollidableGeometry,
                                aris::dynamic::Geometry>& {
  return *imp_->anchored_geometry_pool_;
}
auto CollisionDetectionEngine::addDynamicGeometry(
    geometry::CollidableGeometry& dynamic_geometry) -> bool {
  dynamic_geometry.updateLocation(nullptr);
  dynamic_geometry.getCollisionObject()->computeAABB();
  imp_->dynamic_tree_.registerObject(dynamic_geometry.getCollisionObject());
  imp_->dynamic_tree_.update();
  imp_->dynamic_objects_map_[dynamic_geometry.geometryId()] = &dynamic_geometry;
  return true;
}
auto CollisionDetectionEngine::addAnchoredGeometry(
    geometry::CollidableGeometry& anchored_geometry) -> bool {
  anchored_geometry.updateLocation(nullptr);
  anchored_geometry.getCollisionObject()->computeAABB();
  imp_->anchored_tree_.registerObject(anchored_geometry.getCollisionObject());
  imp_->anchored_tree_.update();
  imp_->anchored_objects_map_[anchored_geometry.geometryId()] =
      &anchored_geometry;
  return false;
}
auto CollisionDetectionEngine::removeGeometry() -> bool { return false; }
auto CollisionDetectionEngine::clearDynamicGeometry() -> bool {
  imp_->dynamic_tree_.clear();
  return true;
}
auto CollisionDetectionEngine::clearAnchoredGeometry() -> bool {
  imp_->anchored_tree_.clear();
  return true;
}
auto CollisionDetectionEngine::updateLocation() -> bool {
  sire::transfer::PartPQTransfer& transfer =
      dynamic_cast<sire::transfer::PartPQTransfer&>(
          imp_->server_->transferModelController());
  std::atomic<double*>& parts_pq_ref = transfer.getPartsPq();
  double* ptr = parts_pq_ref.load();
  if (ptr) {
    std::vector<double> buffer_pq(ptr, ptr + imp_->part_size_ * 7);
    for (auto& dynamic_geometry : *imp_->dynamic_geometry_pool_) {
      double temp_pm[16];
      aris::dynamic::s_pq2pm(buffer_pq.data() + 7 * dynamic_geometry.partId(),
                             temp_pm);
      dynamic_geometry.updateLocation(temp_pm);
    }
    imp_->dynamic_tree_.update();
    parts_pq_ref.exchange(nullptr);
    return true;
  }
  return false;

  // aris::core::Matrix buffer_pq(imp_->part_size_, 7);
  // std::any data;
  // imp_->server_->getRtData(
  //     [this, &buffer_pq](aris::server::ControlServer& cs,
  //                        const aris::plan::Plan* p, std::any& data) -> void {
  //       for (int i = 0; i < imp_->part_size_; ++i) {
  //         auto& part = imp_->part_pool_ptr_->at(i);
  //         part.getPq(buffer_pq.data() + 7 * i);
  //       }
  //     },
  //     data);
  // for (auto& dynamic_geometry : *imp_->dynamic_geometry_pool_) {
  //   double temp_pm[16];
  //   aris::dynamic::s_pq2pm(buffer_pq.data() + 7 * dynamic_geometry.partId(),
  //                          temp_pm);
  //   dynamic_geometry.updateLocation(temp_pm);
  // }
  // imp_->dynamic_tree_.update();
  // return true;
}
auto CollisionDetectionEngine::updateLocation(double* part_pq) -> bool {
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

auto CollisionDetectionEngine::hasCollisions() -> bool {
  CollisionExistsCallback callback(imp_->collision_filter_.get());
  imp_->dynamic_tree_.collide(&callback);
  imp_->dynamic_tree_.collide(&imp_->anchored_tree_, &callback);
  return callback.data.collision_exist_;
}

auto CollisionDetectionEngine::collidedObjects(
    CollidedObjectsCallback& callback_out) -> bool {
  imp_->dynamic_tree_.collide(&callback_out);
  imp_->dynamic_tree_.collide(&imp_->anchored_tree_, &callback_out);
  return callback_out.data.result.isCollision();
}

auto CollisionDetectionEngine::init() -> void {
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
  //imp_->part_pool_ptr_ =
  //    &dynamic_cast<aris::dynamic::MultiModel*>(&imp_->server_->model())
  //         ->subModels()
  //         .at(0)
  //         ->partPool();
  imp_->part_size_ = imp_->part_pool_ptr_->size();
}
// callback print
// cout << "has collision number: " <<
// callback.data.result.numContacts() << endl;
// if (callback.collidedObjectMap().size() != 0) {
//   cout << callback.collidedObjectMap().size() << endl;
// }
// for (auto& obj_pair : callback.collidedObjectMap()) {
//   cout << "collided object of "
//        // << imp_->collision_filter_->queryGeometryIdByPtr(c.o1) << "
//        "
//        << obj_pair.first
//        << " "
//        //<< imp_->collision_filter_->queryGeometryIdByPtr(c.o2) <<
//        endl;
//        << obj_pair.second << endl;
// }
CollisionDetectionEngine::CollisionDetectionEngine() : imp_(new Imp) {}
CollisionDetectionEngine::~CollisionDetectionEngine(){};

ARIS_REGISTRATION {
  aris::core::class_<aris::core::PointerArray<geometry::CollidableGeometry,
                                              aris::dynamic::Geometry>>(
      "GeometryPoolObject")
      .asRefArray();

  typedef aris::core::PointerArray<geometry::CollidableGeometry,
                                   aris::dynamic::Geometry>& (
      CollisionDetectionEngine::*GeometryPoolFunc)();
  typedef sire::collision::CollisionFilter& (
      CollisionDetectionEngine::*CollisionFilterPoolFunc)();
  aris::core::class_<CollisionDetectionEngine>("CollisionDetectionEngine")
      .prop("dynamic_geometry_pool",
            &CollisionDetectionEngine::resetDynamicGeometryPool,
            GeometryPoolFunc(&CollisionDetectionEngine::dynamicGeometryPool))
      .prop("anchored_geometry_pool",
            &CollisionDetectionEngine::resetAnchoredGeometryPool,
            GeometryPoolFunc(&CollisionDetectionEngine::anchoredGeometryPool))
      .prop(
          "collision_filter", &CollisionDetectionEngine::resetCollisionFilter,
          CollisionFilterPoolFunc(&CollisionDetectionEngine::collisionFilter));
}
}  // namespace sire::collision