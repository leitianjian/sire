#include "sire/core/geometry/capsule_geometry.hpp"

#include <array>
#include <memory>
#include <string>
#include <string_view>

#include <aris/core/expression_calculator.hpp>
#include <aris/core/reflection.hpp>
#include <aris/dynamic/model.hpp>

namespace sire::geometry {
auto CapsuleGeometry::to_json(nlohmann::json& j) const -> void {
  GeometryAdapter::to_json(j);
  j["radius"] = typedShape.radius();
  j["length"] = typedShape.length();
}

CapsuleGeometry::CapsuleGeometry(double radius, double length, int part_id,
                                 bool is_dynamic, const double* prt_pm)
    : GeometryAdapter(part_id, is_dynamic, prt_pm, radius, length) {}

CapsuleGeometry::~CapsuleGeometry() = default;

ARIS_DEFINE_BIG_FOUR_CPP(CapsuleGeometry)

ARIS_REGISTRATION {
  auto setRadius = [](CapsuleGeometry* cyl, double radius) -> void {
    cyl->typedShape.setRadius(radius);
  };
  auto setLength = [](CapsuleGeometry* cyl, double length) -> void {
    cyl->typedShape.setLength(length);
  };
  auto getRadius = [](CapsuleGeometry* cyl) -> double {
    return cyl->typedShape.radius();
  };
  auto getLength = [](CapsuleGeometry* cyl) -> double {
    return cyl->typedShape.length();
  };
  aris::core::class_<CapsuleGeometry>("CapsuleGeometry")
      .inherit<GeometryOnPart>()
      .prop("radius", &setRadius, &getRadius)
      .prop("length", &setLength, &getLength);
}
}  // namespace sire::geometry
