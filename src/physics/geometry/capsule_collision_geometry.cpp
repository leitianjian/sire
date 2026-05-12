#include "sire/physics/geometry/capsule_collision_geometry.hpp"

#include <coal/shape/geometric_shapes.h>
#include <aris/core/reflection.hpp>

namespace sire::physics::geometry {

auto CapsuleCollisionGeometry::to_json(nlohmann::json& j) const -> void {
  CollisionAdapter::to_json(j);
  j["radius"] = typedShape.radius();
  j["length"] = typedShape.length();
}

auto CapsuleCollisionGeometry::init() -> void {
  resetCollisionObject(new coal::CollisionObject(
      std::make_shared<coal::Capsule>(typedShape.radius(), typedShape.length()),
      getCoalTransform()));
}

CapsuleCollisionGeometry::CapsuleCollisionGeometry(double radius, double length,
                                                   int part_id, bool is_dynamic,
                                                   const double* prt_pm,
                                                   const std::string& material,
                                                   const std::string& propStr)
    : CollisionAdapter(part_id, is_dynamic, prt_pm, material, propStr, radius, length) {}

CapsuleCollisionGeometry::~CapsuleCollisionGeometry() = default;

SIRE_DEFINE_MOVE_CTOR_CPP(CapsuleCollisionGeometry)

ARIS_REGISTRATION {
  auto setRadius = [](CapsuleCollisionGeometry* geo, double radius) -> void {
    geo->typedShape.setRadius(radius);
  };
  auto setLength = [](CapsuleCollisionGeometry* cyl, double length) -> void {
    cyl->typedShape.setLength(length);
  };
  auto getRadius = [](CapsuleCollisionGeometry* geo) -> double {
    return geo->typedShape.radius();
  };
  auto getLength = [](CapsuleCollisionGeometry* cyl) -> double {
    return cyl->typedShape.length();
  };
  aris::core::class_<CapsuleCollisionGeometry>("CapsuleCollisionGeometry")
      .inherit<CollidableGeometry>()
      .prop("radius", &setRadius, &getRadius)
      .prop("length", &setLength, &getLength);
}
}  // namespace sire::physics::geometry
