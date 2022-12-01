#include "sire/collision/collision_engine.hpp"
#include <aris/core/reflection.hpp>
#include <aris/server/control_server.hpp>
#include <hpp/fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <hpp/fcl/distance.h>
#include <hpp/fcl/math/transform.h>
#include <hpp/fcl/mesh_loader/assimp.h>
#include <hpp/fcl/mesh_loader/loader.h>
#include <hpp/fcl/shape/geometric_shapes.h>
#include <fstream>
#include <io.h>
#include <mutex>
#include <stdio.h>
#include <string>
#include <thread>
namespace sire::collision {
struct CollisionFilter::Imp {
  FilterState filter_state_;
  unordered_map<fcl::CollisionGeometry*, geometry::GeometryId> geometry_map_;
  aris::core::Matrix state_mat_;
  aris::Size geo_size_{0};
};
auto CollisionFilter::addGeometry(geometry::CollisionGeometry& geo) -> bool {
  addGeometry(geo.geometryId(), geo.getCollisionObject());
  return true;
}
auto CollisionFilter::addGeometry(geometry::GeometryId id,
                                  fcl::CollisionObject* obj_ptr) -> bool {
  if (!containsGeometry(id)) {
    GeometryMap map;
    for (auto& geoMap : imp_->filter_state_) {
      if (id < geoMap.first) {
        map[geoMap.first] = CollisionRelationship::kUnfiltered;
      }
      if (id > geoMap.first) {
        imp_->filter_state_[geoMap.first][id] =
            CollisionRelationship::kUnfiltered;
      }
    }
    imp_->filter_state_[id] = map;
    imp_->geometry_map_[obj_ptr->collisionGeometry().get()] = id;
    ++imp_->geo_size_;
    return true;
  } else {
    return true;
  }
}
auto CollisionFilter::updateGeometry(geometry::GeometryId id,
                                     fcl::CollisionObject* obj_ptr) -> bool {
  if (containsGeometry(id)) {
    imp_->geometry_map_[obj_ptr->collisionGeometry().get()] = id;
    return true;
  } else {
    return true;
  }
}
auto CollisionFilter::removeGeometry(geometry::GeometryId id) -> bool {
  if (containsGeometry(id)) {
    for (auto& geoMap : imp_->filter_state_[id]) {
      if (id > geoMap.first) {
        imp_->filter_state_[geoMap.first].erase(id);
      }
    }
    imp_->filter_state_.erase(id);

    auto it =
        std::find_if(imp_->geometry_map_.begin(), imp_->geometry_map_.end(),
                     [&id](const auto& p) { return p.second == id; });
    if (it != imp_->geometry_map_.end()) {
      imp_->geometry_map_.erase(it);
    }
    return true;
  } else {
    return true;
  }
}
auto CollisionFilter::canCollideWith(geometry::GeometryId id_1,
                                     geometry::GeometryId id_2) -> bool {
  if (id_1 == id_2) return false;
  return id_1 < id_2 ? imp_->filter_state_[id_1][id_2] ==
                           CollisionRelationship::kUnfiltered
                     : imp_->filter_state_[id_2][id_1] ==
                           CollisionRelationship::kUnfiltered;
}
auto CollisionFilter::canCollideWith(fcl::CollisionObject* o1,
                                     fcl::CollisionObject* o2) -> bool {
  if (o1 == o2) return false;
  try {
    geometry::GeometryId id_1 =
        imp_->geometry_map_.at(o1->collisionGeometry().get());
    geometry::GeometryId id_2 =
        imp_->geometry_map_.at(o2->collisionGeometry().get());

    return canCollideWith(id_1, id_2);
  } catch (std::out_of_range& err) {
    return false;
  }
}
auto CollisionFilter::queryGeometryIdByPtr(const fcl::CollisionGeometry* ptr)
    -> geometry::GeometryId {
  return queryGeometryIdByPtr(const_cast<fcl::CollisionGeometry*>(ptr));
}
auto CollisionFilter::queryGeometryIdByPtr(fcl::CollisionGeometry* ptr)
    -> geometry::GeometryId {
  return imp_->geometry_map_.at(ptr);
}
auto CollisionFilter::containsGeometry(geometry::GeometryId id) -> bool {
  return imp_->filter_state_.find(id) != imp_->filter_state_.end();
}
auto CollisionFilter::setStateMat(aris::core::Matrix mat) -> void {
  imp_->state_mat_.swap(mat);
}
auto CollisionFilter::stateMat() -> aris::core::Matrix {
  return imp_->state_mat_;
}
auto CollisionFilter::loadMatConfig() -> void {
  // if (imp_->state_mat_.m() != imp_->state_mat_.n()) return;
  int i = 0;
  for (auto& [id_1, geo_map] : imp_->filter_state_) {
    int j = i + 1;
    for (auto& [id_2, relation] : geo_map) {
      if (i * imp_->geo_size_ + j < imp_->state_mat_.size()) {
        relation = imp_->state_mat_(0, i * imp_->geo_size_ + j) == 0
                       ? CollisionRelationship::kUnfiltered
                       : CollisionRelationship::kFiltered;
      } else {
        relation = CollisionRelationship::kFiltered;
      }
      ++j;
    }
    ++i;
  }
}
auto CollisionFilter::saveMatConfig() -> void {
  int32_t size = imp_->filter_state_.size();
  int i = 0;
  for (const auto& [id_1, geo_map] : imp_->filter_state_) {
    int j = i + 1;
    for (const auto& [id_2, relation] : geo_map) {
      imp_->state_mat_(0, i * imp_->geo_size_ + j) = relation;
      ++j;
    }
    ++i;
  }
}
CollisionFilter::CollisionFilter() : imp_(new Imp) {}
CollisionFilter::~CollisionFilter() = default;
ARIS_DEFINE_BIG_FOUR_CPP(CollisionFilter);

auto FilteredCollisionCallback::collide(fcl::CollisionObject* o1,
                                        fcl::CollisionObject* o2) -> bool {
  if (!filter_->canCollideWith(o1, o2)) {
    return false;
  }
  auto* collision_data = static_cast<fcl::CollisionData*>(&data);
  const fcl::CollisionRequest& request = collision_data->request;
  fcl::CollisionResult& result = collision_data->result;

  if (collision_data->done) return true;

  fcl::collide(o1, o2, request, result);

  if (result.isCollision() &&
      result.numContacts() >= request.num_max_contacts) {
    collision_data->done = true;
  }
  return collision_data->done;
}
FilteredCollisionCallback::FilteredCollisionCallback(CollisionFilter* filter)
    : fcl::CollisionCallBackBase() {
  filter_ = filter;
};

auto FilteredCollidedObjectsCallback::collide(fcl::CollisionObject* o1,
                                              fcl::CollisionObject* o2)
    -> bool {
  if (!filter_->canCollideWith(o1, o2)) {
    return false;
  }
  if (queryCollidedObject(o1, o2)) {
    return false;
  }
  auto* collision_data = static_cast<fcl::CollisionData*>(&data);
  const fcl::CollisionRequest& request = collision_data->request;
  fcl::CollisionResult& result = collision_data->result;

  fcl::collide(o1, o2, request, result);
  if (result.isCollision()) {
    addCollidedObject(o1, o2);
  }
  return false;
}
auto FilteredCollidedObjectsCallback::collidedObjectMap()
    -> set<std::pair<geometry::GeometryId, geometry::GeometryId>>& {
  return collidedObjectMap_;
}
auto FilteredCollidedObjectsCallback::addCollidedObject(
    fcl::CollisionObject* o1, fcl::CollisionObject* o2) -> void {
  if (o1 == o2) return;
  geometry::GeometryId id_1 =
      filter_->queryGeometryIdByPtr(o1->collisionGeometry().get());
  geometry::GeometryId id_2 =
      filter_->queryGeometryIdByPtr(o2->collisionGeometry().get());
  if (id_1 == id_2) return;
  if (id_1 < id_2) {
    collidedObjectMap_.insert(pair(id_1, id_2));
  } else {
    collidedObjectMap_.insert(pair(id_2, id_1));
  }
}
auto FilteredCollidedObjectsCallback::queryCollidedObject(
    fcl::CollisionObject* o1, fcl::CollisionObject* o2) -> bool {
  if (o1 == o2) return true;
  geometry::GeometryId id_1 =
      filter_->queryGeometryIdByPtr(o1->collisionGeometry().get());
  geometry::GeometryId id_2 =
      filter_->queryGeometryIdByPtr(o2->collisionGeometry().get());
  if (id_1 == id_2) return true;
  if (id_1 < id_2) {
    return collidedObjectMap_.find(pair(id_1, id_2)) !=
           collidedObjectMap_.end();
  } else {
    return collidedObjectMap_.find(pair(id_1, id_2)) !=
           collidedObjectMap_.end();
  }
}
FilteredCollidedObjectsCallback::FilteredCollidedObjectsCallback(
    CollisionFilter* filter)
    : fcl::CollisionCallBackBase() {
  filter_ = filter;
};

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
  aris::core::Matrix buffer_pq(imp_->part_size_, 7);
  std::any data;
  imp_->server_->getRtData(
      [this, &buffer_pq](aris::server::ControlServer& cs,
                         const aris::plan::Plan* p, std::any& data) -> void {
        for (int i = 0; i < imp_->part_size_; ++i) {
          auto& part = imp_->part_pool_ptr_->at(i);
          part.getPq(buffer_pq.data() + 7 * i);
        }
      },
      data);
  for (auto& dynamic_geometry : *imp_->dynamic_geometry_pool_) {
    double temp_pm[16];
    aris::dynamic::s_pq2pm(buffer_pq.data() + 7 * dynamic_geometry.partId(),
                           temp_pm);
    dynamic_geometry.updateLocation(temp_pm);
  }
  imp_->dynamic_tree_.update();
  return true;
}
auto CollisionEngine::hasCollisions(FilteredCollidedObjectsCallback& callback)
    -> void {
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
    int64_t count = 0;
    auto& time_start = chrono::system_clock::now();
    while (imp_->collision_detection_running_) {
      if (imp_->server_->running()) {
        if (count % 100 == 0) {
          auto& time_now = chrono::system_clock::now();
          auto& duration = chrono::duration_cast<chrono::milliseconds>(
              time_now - time_start);
          cout << "cost: " << duration.count() << "ms" << endl;
          time_start = chrono::system_clock::now();
        }
        this->updateLocation();
        FilteredCollidedObjectsCallback callback(imp_->collision_filter_.get());
        this->hasCollisions(callback);
        // cout << "has collision number: " <<
        // callback.data.result.numContacts() << endl;
        // if (callback.collidedObjectMap().size() != 0) {
        //   cout << callback.collidedObjectMap().size() << endl; 
        // }
        // for (auto& obj_pair : callback.collidedObjectMap()) {
        //   cout << "collided object of "
        //        // << imp_->collision_filter_->queryGeometryIdByPtr(c.o1) << " "
        //        << obj_pair.first
        //        << " "
        //        //<< imp_->collision_filter_->queryGeometryIdByPtr(c.o2) << endl;
        //        << obj_pair.second << endl;
        // }
        ++count;
      }
    }
  });
}
CollisionEngine::CollisionEngine() : imp_(new Imp) {}
CollisionEngine::~CollisionEngine() {
  imp_->collision_detection_running_.store(false);
};

ARIS_REGISTRATION {
  auto setFilterState = [](CollisionFilter* filter, aris::core::Matrix state) {
    filter->setStateMat(state);
  };
  auto getFilterState = [](CollisionFilter* filter) {
    return filter->stateMat();
  };
  aris::core::class_<CollisionFilter>("CollisionFilter")
      .prop("filter_state", &setFilterState, &getFilterState);

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