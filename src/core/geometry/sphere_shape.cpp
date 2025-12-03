#include "sire/core/geometry/sphere_shape.hpp"

#include <aris/core/reflection.hpp>

#include "sire/core/sire_assert.hpp"
#include "sire/core/string_utils.hpp"

namespace sire::geometry {
auto SphereShape::setRadius(double radius_in) -> void {
  if (radius_in < 0) {
    throw std::logic_error(sire::core::string_format(
        "Sphere radius should be >= 0 (was %d).", radius_in));
  }
  radius_ = radius_in;
}
auto SphereShape::getRadius() const -> double { return radius_; }
auto SphereShape::radius() -> double& { return radius_; }
auto SphereShape::radius() const -> double { return radius_; }

SphereShape::SphereShape(double radius_in)
    : ShapeBase(ShapeTag<SphereShape>()), radius_(radius_in) {
  if (radius_in < 0) {
    throw std::logic_error(sire::core::string_format(
        "Sphere radius should be >= 0 (was %d).", radius_in));
  }
  // setShapeType(ShapeType::GEOM_SPHERE);
}

SphereShape::~SphereShape() = default;
ARIS_REGISTRATION {
  aris::core::class_<SphereShape>("SphereShape")
      .prop("radius", &SphereShape::setRadius, &SphereShape::getRadius);
}
}  // namespace sire::geometry
