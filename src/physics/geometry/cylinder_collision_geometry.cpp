#include "sire/physics/geometry/cylinder_collision_geometry.hpp"

#include <coal/shape/geometric_shapes.h>
#include <aris/core/reflection.hpp>

namespace sire::physics::geometry {
auto CylinderCollisionGeometry::to_json(nlohmann::json& j) const -> void {
  CollisionAdapter::to_json(j);
  j["radius"] = typedShape.radius();
  j["length"] = typedShape.length();
}
auto CylinderCollisionGeometry::init() -> void {
  resetCollisionObject(new coal::CollisionObject(
      std::make_shared<coal::Cylinder>(typedShape.radius(), typedShape.length()),
      getCoalTransform()));
}
CylinderCollisionGeometry::CylinderCollisionGeometry(double radius, double length,
                                                     int part_id, bool is_dynamic,
                                                     const double* prt_pm,
                                                     bool visible,
                                                     const std::string& material,
                                                     const std::string& propStr)
    : CollisionAdapter(part_id, is_dynamic, prt_pm, visible, material, propStr, radius, length) {}

CylinderCollisionGeometry::~CylinderCollisionGeometry() = default;

SIRE_DEFINE_MOVE_CTOR_CPP(CylinderCollisionGeometry)

ARIS_REGISTRATION {
  auto setRadius = [](CylinderCollisionGeometry* geo, double radius) -> void {
    geo->typedShape.setRadius(radius);
  };
  auto setLength = [](CylinderCollisionGeometry* cyl, double length) -> void {
    cyl->typedShape.setLength(length);
  };
  auto getRadius = [](CylinderCollisionGeometry* geo) -> double {
    return geo->typedShape.radius();
  };
  auto getLength = [](CylinderCollisionGeometry* cyl) -> double {
    return cyl->typedShape.length();
  };
  aris::core::class_<CylinderCollisionGeometry>("CylinderCollisionGeometry")
      .inherit<CollidableGeometry>()
      .prop("radius", &setRadius, &getRadius)
      .prop("length", &setLength, &getLength);
}
}  // namespace sire::physics::geometry
