#include "sire/core/geometry/sphere_geometry.hpp"

#include <array>
#include <memory>
#include <string>
#include <string_view>

#include <aris/core/reflection.hpp>
#include <aris/dynamic/model.hpp>
#include <aris/server/control_server.hpp>

namespace sire::geometry {
auto SphereGeometry::radius() -> double { return SphereShape::radius(); }
auto SphereGeometry::setRadius(double radius_in) -> void {
  SphereShape::setRadius(radius_in);
}
SphereGeometry::SphereGeometry(double radius, const double* prt_pm)
    : GeometryOnPart(), SphereShape(radius) {}
SphereGeometry::~SphereGeometry() = default;

ARIS_REGISTRATION {
  aris::core::class_<SphereGeometry>("SphereGeometry")
      .inherit<GeometryOnPart>()
      .prop("radius", &SphereGeometry::setRadius, &SphereGeometry::radius);
}
}  // namespace sire::geometry