#include "sire/collision/collision_engine.hpp"

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
#include <io.h>

#include <aris/core/reflection.hpp>
#include <aris/server/control_server.hpp>

#include "sire/transfer/part_pq_transfer.hpp"
namespace sire::collision {
struct CollisionEngine::Imp {
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
  // aris::core::Matrix part_pq_buffer_;
  // thread retrieve_part_pq_;
  atomic_bool collision_detection_running_;
  thread collision_detection_;
  aris::server::ControlServer* server_;
  aris::core::PointerArray<aris::dynamic::Part, aris::dynamic::Element>*
      part_pool_ptr_;
  aris::Size part_size_;
};
auto CollisionEngine::resetCollisionFilter(CollisionFilter* filter) -> void {
  imp_->collision_filter_.reset(filter);
}
auto CollisionEngine::collisionFilter() -> CollisionFilter& {
  return *imp_->collision_filter_;
}
auto CollisionEngine::resetDynamicGeometryPool(
    aris::core::PointerArray<geometry::CollisionGeometry,
                             aris::dynamic::Geometry>* pool) -> void {
  imp_->dynamic_geometry_pool_.reset(pool);
}
auto CollisionEngine::dynamicGeometryPool()
    -> aris::core::PointerArray<geometry::CollisionGeometry,
                                aris::dynamic::Geometry>& {
  return *imp_->dynamic_geometry_pool_;
}
auto CollisionEngine::resetAnchoredGeometryPool(
    aris::core::PointerArray<geometry::CollisionGeometry,
                             aris::dynamic::Geometry>* pool) -> void {
  imp_->anchored_geometry_pool_.reset(pool);
}
auto CollisionEngine::anchoredGeometryPool()
    -> aris::core::PointerArray<geometry::CollisionGeometry,
                                aris::dynamic::Geometry>& {
  return *imp_->anchored_geometry_pool_;
}
auto CollisionEngine::addDynamicGeometry(
    geometry::CollisionGeometry& dynamic_geometry) -> bool {
  dynamic_geometry.updateLocation(nullptr);
  dynamic_geometry.getCollisionObject()->computeAABB();
  imp_->dynamic_tree_.registerObject(dynamic_geometry.getCollisionObject());
  imp_->dynamic_tree_.update();
  imp_->dynamic_objects_map_[dynamic_geometry.geometryId()] = &dynamic_geometry;
  return true;
}
auto CollisionEngine::addAnchoredGeometry(
    geometry::CollisionGeometry& anchored_geometry) -> bool {
  anchored_geometry.updateLocation(nullptr);
  anchored_geometry.getCollisionObject()->computeAABB();
  imp_->anchored_tree_.registerObject(anchored_geometry.getCollisionObject());
  imp_->anchored_tree_.update();
  imp_->anchored_objects_map_[anchored_geometry.geometryId()] =
      &anchored_geometry;
  return false;
}
auto CollisionEngine::removeGeometry() -> bool { return false; }
auto CollisionEngine::clearDynamicGeometry() -> bool {
  imp_->dynamic_tree_.clear();
  return true;
}
auto CollisionEngine::clearAnchoredGeometry() -> bool {
  imp_->anchored_tree_.clear();
  return true;
}
auto CollisionEngine::updateLocation() -> bool {
  sire::transfer::PartPQTransfer& transfer =
      dynamic_cast<sire::transfer::PartPQTransfer&>(
          imp_->server_->transferModelController());
  std::atomic<double*>& parts_pq_ref = transfer.getPartsPq();
  double* ptr = parts_pq_ref.load();
  if (ptr) {
    std::vector<double> buffer_pq(ptr, ptr + imp_->part_size_ * 7);
    parts_pq_ref.exchange(nullptr);
    for (auto& dynamic_geometry : *imp_->dynamic_geometry_pool_) {
      double temp_pm[16];
      aris::dynamic::s_pq2pm(buffer_pq.data() + 7 * dynamic_geometry.partId(),
                             temp_pm);
      dynamic_geometry.updateLocation(temp_pm);
    }
    imp_->dynamic_tree_.update();
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
auto CollisionEngine::hasCollisions(CollisionExistsCallback& callback) -> void {
  callback.data.request.num_max_contacts = 10;
  callback.data.request.enable_contact = false;
  callback.data.request.gjk_tolerance = 2e-12;
  imp_->dynamic_tree_.collide(&callback);
  imp_->dynamic_tree_.collide(&imp_->anchored_tree_, &callback);
}
auto CollisionEngine::init() -> void {
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
  imp_->collision_detection_running_.store(true);
  imp_->collision_detection_ = std::thread([this]() {
    int64_t count = 1;
    auto& time_start = chrono::system_clock::now();
    while (imp_->collision_detection_running_) {
      if (imp_->server_->running()) {
        if (count % 100 == 0) {
          auto& time_now = chrono::system_clock::now();
          auto& duration = chrono::duration_cast<chrono::milliseconds>(
              time_now - time_start);
          cout << "cost: " << duration.count() << "ms" << endl;
          time_start = chrono::system_clock::now();
          count = 1;
        }
        if (this->updateLocation()) {
          CollisionExistsCallback callback(imp_->collision_filter_.get());
          this->hasCollisions(callback);
          ++count;
        }
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
      }
    }
  });
}
CollisionEngine::CollisionEngine() : imp_(new Imp) {}
CollisionEngine::~CollisionEngine() {
  imp_->collision_detection_running_.store(false);
};

ARIS_REGISTRATION {
  aris::core::class_<aris::core::PointerArray<geometry::CollisionGeometry,
                                              aris::dynamic::Geometry>>(
      "GeometryPoolObject")
      .asRefArray();

  typedef aris::core::PointerArray<geometry::CollisionGeometry,
                                   aris::dynamic::Geometry>& (
      CollisionEngine::*GeometryPoolFunc)();
  typedef sire::collision::CollisionFilter& (
      CollisionEngine::*CollisionFilterPoolFunc)();
  aris::core::class_<CollisionEngine>("CollisionEngine")
      .prop("dynamic_geometry_pool", &CollisionEngine::resetDynamicGeometryPool,
            GeometryPoolFunc(&CollisionEngine::dynamicGeometryPool))
      .prop("anchored_geometry_pool",
            &CollisionEngine::resetAnchoredGeometryPool,
            GeometryPoolFunc(&CollisionEngine::anchoredGeometryPool))
      .prop("collision_filter", &CollisionEngine::resetCollisionFilter,
            CollisionFilterPoolFunc(&CollisionEngine::collisionFilter));
}
}  // namespace sire::collision