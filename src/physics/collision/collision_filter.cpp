#include "sire/physics/collision/collision_filter.hpp"

#include <fstream>
#include <mutex>
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
#include <coal/shape/geometric_shapes.h>

#include <aris/core/reflection.hpp>
#include <aris/server/control_server.hpp>

#include "sire/core/constants.hpp"

namespace sire::physics::collision {
using namespace coal;
struct CollisionFilter::Imp {
  FilterState filter_state_;
  std::unordered_map<CollisionGeometry*, GeometryId> geometry_map_;
  aris::core::Matrix state_mat_;
  sire::Size geo_size_{0};
};
auto CollisionFilter::addGeometry(geometry::CollidableGeometry& geo) -> bool {
  addGeometry(geo.geometryId(), geo.getCollisionObject());
  return true;
}
auto CollisionFilter::addGeometry(GeometryId id, CollisionObject* obj_ptr)
    -> bool {
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
auto CollisionFilter::updateGeometry(GeometryId id, CollisionObject* obj_ptr)
    -> bool {
  if (containsGeometry(id)) {
    imp_->geometry_map_[obj_ptr->collisionGeometry().get()] = id;
    return true;
  } else {
    return true;
  }
}
auto CollisionFilter::removeGeometry(GeometryId id) -> bool {
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
auto CollisionFilter::enableCollisionPair(GeometryId id_1, GeometryId id_2) -> void {
  if (id_1 == id_2) return;
  if (id_1 < id_2) {
    imp_->filter_state_[id_1][id_2] = CollisionRelationship::kUnfiltered;
  } else {
    imp_->filter_state_[id_2][id_1] = CollisionRelationship::kUnfiltered;
  }
}
auto CollisionFilter::canCollideWith(GeometryId id_1, GeometryId id_2) -> bool {
  if (id_1 == id_2) return false;
  return id_1 < id_2 ? imp_->filter_state_[id_1][id_2] ==
                           CollisionRelationship::kUnfiltered
                     : imp_->filter_state_[id_2][id_1] ==
                           CollisionRelationship::kUnfiltered;
}
auto CollisionFilter::canCollideWith(const CollisionObject* o1,
                                     const CollisionObject* o2) -> bool {
  if (o1 == o2) return false;
  try {
    GeometryId id_1 = imp_->geometry_map_.at(
        const_cast<CollisionObject*>(o1)->collisionGeometry().get());
    GeometryId id_2 = imp_->geometry_map_.at(
        const_cast<CollisionObject*>(o2)->collisionGeometry().get());

    return canCollideWith(id_1, id_2);
  } catch (std::out_of_range& err) {
    std::cout << err.what() << std::endl;
    return false;
  }
}
auto CollisionFilter::queryGeometryIdByPtr(const CollisionGeometry* ptr)
    -> GeometryId {
  return queryGeometryIdByPtr(const_cast<CollisionGeometry*>(ptr));
}
auto CollisionFilter::queryGeometryIdByPtr(CollisionGeometry* ptr)
    -> GeometryId {
  return imp_->geometry_map_.at(ptr);
}
auto CollisionFilter::containsGeometry(GeometryId id) -> bool {
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
  sire::Size size = imp_->filter_state_.size();
  imp_->state_mat_ = aris::core::Matrix(1, size * size, 0.0);
  for (Size i = 0; i < size; ++i) {
    imp_->state_mat_(0, i * size + i) = 1;
  }
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
}  // namespace sire::physics::collision