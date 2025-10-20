#include "sire/core/geometry/cylinder_geometry.hpp"

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
SIRE_DEFINE_TO_JSON_HEAD(CylinderGeometry) {
  GeometryOnPart::to_json(j);
  ShapeToName cal;
  cylinderShape.Reify(&cal);
  j["shape_type"] = cal.string();
  j["radius"] = cylinderShape.radius();
  j["length"] = cylinderShape.length();
}

CylinderGeometry::CylinderGeometry(double radius, double length, int part_id,
                                   bool is_dynamic, const double* prt_pm)
    : GeometryOnPart(prt_pm, part_id, is_dynamic), cylinderShape(radius, length) {}

CylinderGeometry::~CylinderGeometry() = default;

ARIS_DEFINE_BIG_FOUR_CPP(CylinderGeometry)

SIRE_DEFINE_JSON_OUTER_TWO(CylinderGeometry)

ARIS_REGISTRATION {
  auto setRadius = [](CylinderGeometry* geo, double radius) -> void {
    geo->cylinderShape.setRadius(radius);
  };
  auto getRadius = [](CylinderGeometry* geo) -> double {
    return geo->cylinderShape.radius();
  };
  auto setLength = [](CylinderGeometry* geo, double length) -> void {
    geo->cylinderShape.setLength(length);
  };
  auto getLength = [](CylinderGeometry* geo) -> double {
    return geo->cylinderShape.length();
  };
  aris::core::class_<CylinderGeometry>("CylinderGeometry")
      .inherit<GeometryOnPart>()
      .prop("radius", &setRadius, &getRadius)
      .prop("length", &setLength, &getLength);
}
}  // namespace sire::geometry