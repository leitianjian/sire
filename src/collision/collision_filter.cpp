#include "sire/collision/collision_filter.hpp"
#include <aris/core/reflection.hpp>
#include <aris/server/control_server.hpp>
#include <hpp/fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <hpp/fcl/distance.h>
#include <hpp/fcl/math/transform.h>
#include <hpp/fcl/shape/geometric_shapes.h>
#include <fstream>
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

ARIS_REGISTRATION {
  auto setFilterState = [](CollisionFilter* filter, aris::core::Matrix state) {
    filter->setStateMat(state);
  };
  auto getFilterState = [](CollisionFilter* filter) {
    return filter->stateMat();
  };
  aris::core::class_<CollisionFilter>("CollisionFilter")
      .prop("filter_state", &setFilterState, &getFilterState);
}
}  // namespace sire::collision