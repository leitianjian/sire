#include "sire/physics/geometry/collidable.hpp"

#include <array>
#include <memory>
#include <string>
#include <string_view>

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
};
auto Collidable::getCollisionObject() -> fcl::CollisionObject* {
  return imp_->fcl_object_ptr_.get();
}
auto Collidable::resetCollisionObject(fcl::CollisionObject* object)
    -> void {
  imp_->fcl_object_ptr_.reset(object);
}
Collidable::Collidable()
    : imp_(new Imp) {
}
Collidable::~Collidable() = default;
}  // namespace sire::physics::geometry