#include "sire/core/geometry/sphere_shape.hpp"

#include <aris/core/reflection.hpp>

#include "sire/core/sire_assert.hpp"

namespace sire::geometry {
auto SphereShape::setRadius(double radius_in) -> void {
  SIRE_DEMAND(radius_in > 0);
  radius_ = radius_in;
}
auto SphereShape::getRadius() const -> double { return radius_; }
auto SphereShape::radius() -> double& { return radius_; }
auto SphereShape::radius() const -> double { return radius_; }

SphereShape::SphereShape(double radius_in) : radius_(radius_in) {
  SIRE_DEMAND(radius_in > 0);
  setShapeType(ShapeType::GEOM_SPHERE);
}

SphereShape::~SphereShape() = default;
ARIS_REGISTRATION {
  aris::core::class_<SphereShape>("SphereShape")
      .prop("radius", &SphereShape::setRadius, &SphereShape::getRadius);
}
}  // namespace sire::geometry
