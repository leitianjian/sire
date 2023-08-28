#include "sire/physics/geometry/collidable.hpp"

#include <array>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>

#include <hpp/fcl/bvh/BVH_model.h>
#include <hpp/fcl/mesh_loader/assimp.h>
#include <hpp/fcl/mesh_loader/loader.h>
#include <hpp/fcl/shape/geometric_shapes.h>

#include <aris/core/reflection.hpp>
#include <aris/dynamic/model.hpp>
#include <aris/server/control_server.hpp>

namespace sire::physics::geometry {
struct Collidable::Imp {
  unique_ptr<fcl::CollisionObject> fcl_object_ptr_{nullptr};

  // TODO(leitianjian) 需要完成unorder_map的序列化，方便填写属性与配置注入
  unordered_map<string, double> contact_properties_;
};
auto Collidable::getCollisionObject() -> fcl::CollisionObject* {
  return imp_->fcl_object_ptr_.get();
}
auto Collidable::resetCollisionObject(fcl::CollisionObject* object) -> void {
  imp_->fcl_object_ptr_.reset(object);
}
auto Collidable::addContactProp(const string& name, double value) -> void {
  imp_->contact_properties_[name] = value;
}
auto Collidable::rmContactProp(const string& name) -> bool {
  if (auto search = imp_->contact_properties_.find(name);
      search != imp_->contact_properties_.end()) {
    imp_->contact_properties_.erase(search);
    return true;
  } else {
    return false;
  }
}
auto Collidable::updContactProp(const string& name, double new_value) -> void {
  imp_->contact_properties_[name] = new_value;
}
auto Collidable::getContactProp(const string& name) -> double {
  return imp_->contact_properties_.at(name);
}
auto Collidable::getContactPropOrDefault(const string& name,
                                         double default_value) -> double {
  if (auto search = imp_->contact_properties_.find(name);
      search != imp_->contact_properties_.end()) {
    return search->second;
  } else {
    return default_value;
  }
}
Collidable::Collidable() : imp_(new Imp) {}
Collidable::~Collidable() = default;
SIRE_DEFINE_MOVE_CTOR_CPP(Collidable);
}  // namespace sire::physics::geometry