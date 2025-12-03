#include "sire/physics/geometry/cylinder_collision_geometry.hpp"

#include <array>
#include <memory>
#include <string>
#include <string_view>

#include <coal/BVH/BVH_model.h>
#include <coal/shape/geometric_shapes.h>

#include <aris/core/reflection.hpp>

#include "sire/core/geometry/shape_calculator.hpp"

namespace sire::physics::geometry {
SIRE_DEFINE_TO_JSON_HEAD(CylinderCollisionGeometry) {
  GeometryOnPart::to_json(j);
  sire::geometry::ShapeToName cal;
  cylinderShape.Reify(&cal);
  j["shape_type"] = cal.string();
  j["radius"] = cylinderShape.radius();
  j["length"] = cylinderShape.length();
}

auto CylinderCollisionGeometry::init() -> void {
  Transform3s trans(
      (Matrix3s() << partPm()[0][0], partPm()[0][1], partPm()[0][2],
       partPm()[1][0], partPm()[1][1], partPm()[1][2], partPm()[2][0],
       partPm()[2][1], partPm()[2][2])
          .finished(),
      (Vec3s() << partPm()[0][3], partPm()[1][3], partPm()[2][3]).finished());
  // std::array<double, 3> temp = side();
  resetCollisionObject(new CollisionObject(
      make_shared<coal::Cylinder>(cylinderShape.radius(), cylinderShape.length()),
      trans));
}
CylinderCollisionGeometry::CylinderCollisionGeometry(double radius, double length,
                                                   int part_id, bool is_dynamic,
                                                   const double* prt_pm,
                                                   const std::string& material,
                                                   const std::string& propStr)
    : CollidableGeometry(prt_pm, part_id, is_dynamic, material, propStr),
      cylinderShape(radius, length) {}
CylinderCollisionGeometry::~CylinderCollisionGeometry() = default;
SIRE_DEFINE_MOVE_CTOR_CPP(CylinderCollisionGeometry)

SIRE_DEFINE_JSON_OUTER_TWO(CylinderCollisionGeometry)

ARIS_REGISTRATION {
  auto setRadius = [](CylinderCollisionGeometry* geo, double radius) -> void {
    geo->cylinderShape.setRadius(radius);
  };
  auto getRadius = [](CylinderCollisionGeometry* geo) -> double {
    return geo->cylinderShape.radius();
  };
  auto setLength = [](CylinderCollisionGeometry* geo, double length) -> void {
    geo->cylinderShape.setLength(length);
  };
  auto getLength = [](CylinderCollisionGeometry* geo) -> double {
    return geo->cylinderShape.length();
  };
  aris::core::class_<CylinderCollisionGeometry>("CylinderCollisionGeometry")
      .inherit<CollidableGeometry>()
      .prop("radius", &setRadius, &getRadius)
      .prop("length", &setLength, &getLength);
}
}  // namespace sire::physics::geometry