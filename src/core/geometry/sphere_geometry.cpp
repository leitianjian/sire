#include "sire/core/geometry/sphere_geometry.hpp"

#include <array>
#include <memory>
#include <string>
#include <string_view>

#include <aris/core/reflection.hpp>
#include <aris/dynamic/model.hpp>
#include <aris/server/control_server.hpp>

namespace sire::geometry {
auto SphereGeometry::to_json(nlohmann::json& j) const -> void {
  GeometryAdapter::to_json(j);
  j["radius"] = typedShape.radius();
}

SphereGeometry::SphereGeometry(double radius, int part_id, bool is_dynamic,
                               const double* prt_pm)
    : GeometryAdapter(part_id, is_dynamic, prt_pm, radius) {}

SphereGeometry::~SphereGeometry() = default;

ARIS_DEFINE_BIG_FOUR_CPP(SphereGeometry)

ARIS_REGISTRATION {
  auto setRadius = [](SphereGeometry* geo, double radius) -> void {
    geo->typedShape.setRadius(radius);
  };
  auto getRadius = [](SphereGeometry* geo) -> double {
    return geo->typedShape.getRadius();
  };
  aris::core::class_<SphereGeometry>("SphereGeometry")
      .inherit<GeometryOnPart>()
      .prop("radius", &setRadius, &getRadius);
}
}  // namespace sire::geometry
