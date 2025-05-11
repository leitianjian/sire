#include "sire/core/geometry/capsule_geometry.hpp"

#include <array>
#include <memory>
#include <string>
#include <string_view>

#include <aris/core/expression_calculator.hpp>
#include <aris/core/reflection.hpp>
#include <aris/dynamic/model.hpp>
#include <aris/server/control_server.hpp>

#include "sire/core/geometry/shape_calculator.hpp"

namespace sire::geometry {
SIRE_DEFINE_TO_JSON_HEAD(CapsuleGeometry) {
  GeometryOnPart::to_json(j);
  ShapeToName cal;
  capsuleShape.Reify(&cal);
  j["shape_type"] = cal.string();
  j["radius"] = capsuleShape.radius();
  j["length"] = capsuleShape.length();
}

CapsuleGeometry::CapsuleGeometry(double radius, double length,
                                 const double* prt_pm)
    : GeometryOnPart(prt_pm), capsuleShape(radius, length) {}

CapsuleGeometry::~CapsuleGeometry() = default;

ARIS_DEFINE_BIG_FOUR_CPP(CapsuleGeometry)

SIRE_DEFINE_JSON_OUTER_TWO(CapsuleGeometry)

ARIS_REGISTRATION {
  auto setRadius = [](CapsuleGeometry* geo, double radius) -> void {
    geo->capsuleShape.setRadius(radius);
  };
  auto getRadius = [](CapsuleGeometry* geo) -> double {
    return geo->capsuleShape.radius();
  };
  auto setLength = [](CapsuleGeometry* geo, double length) -> void {
    geo->capsuleShape.setLength(length);
  };
  auto getLength = [](CapsuleGeometry* geo) -> double {
    return geo->capsuleShape.length();
  };
  aris::core::class_<CapsuleGeometry>("CapsuleGeometry")
      .inherit<GeometryOnPart>()
      .prop("radius", &setRadius, &getRadius)
      .prop("length", &setLength, &getLength);
}
}  // namespace sire::geometry