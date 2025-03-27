#include "sire/physics/geometry/collidable.hpp"

#include <array>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>

#include <coal/BVH/BVH_model.h>
#include <coal/mesh_loader/assimp.h>
#include <coal/mesh_loader/loader.h>
#include <coal/shape/geometric_shapes.h>

#include <aris/core/object.hpp>
#include <aris/core/reflection.hpp>
#include <aris/dynamic/model.hpp>
#include <aris/server/control_server.hpp>

namespace sire::physics::geometry {
struct Collidable::Imp {
  unique_ptr<CollisionObject> fcl_object_ptr_{nullptr};

  // TODO(leitianjian) ��Ҫ���unorder_map�����л���������д����������ע��
  core::PropMap contact_properties_;
  std::string material_;
};
auto Collidable::getCollisionObject() -> CollisionObject* {
  return imp_->fcl_object_ptr_.get();
}
auto Collidable::resetCollisionObject(CollisionObject* object) -> void {
  imp_->fcl_object_ptr_.reset(object);
}
auto Collidable::setContactProp(const core::PropMap& map) -> void {
  imp_->contact_properties_ = map;
}
auto Collidable::setContactProp(core::PropMap& map) -> void {
  using std::swap;
  swap(imp_->contact_properties_, map);
}
auto Collidable::contactProp() const -> const core::PropMap& {
  return imp_->contact_properties_;
}
auto Collidable::material() const -> std::string { return imp_->material_; }
auto Collidable::setMaterial(const std::string& material) -> void {
  imp_->material_ = material;
}
Collidable::Collidable() : imp_(new Imp) {}
Collidable::~Collidable() = default;
SIRE_DEFINE_MOVE_CTOR_CPP(Collidable);

ARIS_REGISTRATION {
  auto setContactParam = [](Collidable* c, core::PropMap map) -> void {
    c->setContactProp(map);
  };
  auto getContactParam = [](Collidable* c) -> core::PropMap {
    return c->contactProp();
  };
  aris::core::class_<Collidable>("Collidable")
      .prop("contact_prop", &setContactParam, &getContactParam)
      .prop("material", &Collidable::setMaterial, &Collidable::material);
  ;
}
}  // namespace sire::physics::geometry