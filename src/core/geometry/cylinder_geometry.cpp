#include "sire/core/geometry/cylinder_geometry.hpp"

#include <aris/core/reflection.hpp>

namespace sire::geometry {
auto CylinderGeometry::to_json(nlohmann::json& j) const -> void {
  GeometryAdapter::to_json(j);
  j["radius"] = typedShape.radius();
  j["length"] = typedShape.length();
}

CylinderGeometry::CylinderGeometry(double radius, double length, int part_id,
                                   bool is_dynamic, const double* prt_pm)
    : GeometryAdapter(part_id, is_dynamic, prt_pm, radius, length) {}

CylinderGeometry::~CylinderGeometry() = default;

ARIS_DEFINE_BIG_FOUR_CPP(CylinderGeometry)

ARIS_REGISTRATION {
  auto setRadius = [](CylinderGeometry* cyl, double radius) -> void {
    cyl->typedShape.setRadius(radius);
  };
  auto setLength = [](CylinderGeometry* cyl, double length) -> void {
    cyl->typedShape.setLength(length);
  };
  auto getRadius = [](CylinderGeometry* cyl) -> double {
    return cyl->typedShape.radius();
  };
  auto getLength = [](CylinderGeometry* cyl) -> double {
    return cyl->typedShape.length();
  };
  aris::core::class_<CylinderGeometry>("CylinderGeometry")
      .inherit<GeometryOnPart>()
      .prop("radius", &setRadius, &getRadius)
      .prop("length", &setLength, &getLength);
}
}  // namespace sire::geometry
