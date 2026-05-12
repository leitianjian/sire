#include "sire/physics/geometry/sphere_collision_geometry.hpp"

#include <array>
#include <memory>
#include <string>
#include <string_view>

#include <coal/BVH/BVH_model.h>
#include <coal/shape/geometric_shapes.h>

#include <aris/core/reflection.hpp>

namespace sire::physics::geometry {

auto SphereCollisionGeometry::to_json(nlohmann::json& j) const -> void {
  CollisionAdapter::to_json(j);
  j["radius"] = typedShape.radius();
}

auto SphereCollisionGeometry::init() -> void {
  resetCollisionObject(new coal::CollisionObject(
      std::make_shared<coal::Sphere>(typedShape.radius()), getCoalTransform()));
}

SphereCollisionGeometry::SphereCollisionGeometry(double radius, int part_id,
                                                 bool is_dynamic,
                                                 const double* prt_pm,
                                                 const std::string& material,
                                                 const std::string& propStr)
    : CollisionAdapter(part_id, is_dynamic, prt_pm, material, propStr, radius) {
}

SphereCollisionGeometry::~SphereCollisionGeometry() = default;

SIRE_DEFINE_MOVE_CTOR_CPP(SphereCollisionGeometry)

ARIS_REGISTRATION {
  auto setRadius = [](SphereCollisionGeometry* geo, double radius) -> void {
    geo->typedShape.setRadius(radius);
  };
  auto getRadius = [](SphereCollisionGeometry* geo) -> double {
    return geo->typedShape.getRadius();
  };
  aris::core::class_<SphereCollisionGeometry>("SphereCollisionGeometry")
      .inherit<CollidableGeometry>()
      .prop("radius", &setRadius, &getRadius);
}
}  // namespace sire::physics::geometry
