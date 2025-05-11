#include "sire/physics/geometry/capsule_collision_geometry.hpp"

#include <array>
#include <memory>
#include <string>
#include <string_view>

#include <coal/BVH/BVH_model.h>
#include <coal/shape/geometric_shapes.h>

#include <aris/core/reflection.hpp>

#include "sire/core/geometry/shape_calculator.hpp"

namespace sire::physics::geometry {
SIRE_DEFINE_TO_JSON_HEAD(CapsuleCollisionGeometry) {
  GeometryOnPart::to_json(j);
  sire::geometry::ShapeToName cal;
  capsuleShape.Reify(&cal);
  j["shape_type"] = cal.string();
  j["radius"] = capsuleShape.radius();
  j["length"] = capsuleShape.length();
}

auto CapsuleCollisionGeometry::init() -> void {
  Transform3s trans(
      (Matrix3s() << partPm()[0][0], partPm()[0][1], partPm()[0][2],
       partPm()[1][0], partPm()[1][1], partPm()[1][2], partPm()[2][0],
       partPm()[2][1], partPm()[2][2])
          .finished(),
      (Vec3s() << partPm()[0][3], partPm()[1][3], partPm()[2][3]).finished());
  // std::array<double, 3> temp = side();
  resetCollisionObject(new CollisionObject(
      make_shared<coal::Capsule>(capsuleShape.radius(), capsuleShape.length()),
      trans));
}
CapsuleCollisionGeometry::CapsuleCollisionGeometry(double radius, double length,
                                                   const double* prt_pm)
    : CollidableGeometry(prt_pm), capsuleShape(radius, length) {}
CapsuleCollisionGeometry::~CapsuleCollisionGeometry() = default;
SIRE_DEFINE_MOVE_CTOR_CPP(CapsuleCollisionGeometry)

SIRE_DEFINE_JSON_OUTER_TWO(CapsuleCollisionGeometry)

ARIS_REGISTRATION {
  auto setRadius = [](CapsuleCollisionGeometry* geo, double radius) -> void {
    geo->capsuleShape.setRadius(radius);
  };
  auto getRadius = [](CapsuleCollisionGeometry* geo) -> double {
    return geo->capsuleShape.radius();
  };
  auto setLength = [](CapsuleCollisionGeometry* geo, double length) -> void {
    geo->capsuleShape.setLength(length);
  };
  auto getLength = [](CapsuleCollisionGeometry* geo) -> double {
    return geo->capsuleShape.length();
  };
  aris::core::class_<CapsuleCollisionGeometry>("CapsuleCollisionGeometry")
      .inherit<CollidableGeometry>()
      .prop("radius", &setRadius, &getRadius)
      .prop("length", &setLength, &getLength);
}
}  // namespace sire::physics::geometry