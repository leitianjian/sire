#include "sire/core/geometry/sphere_shape.hpp"

#include "sire/core/sire_assert.hpp"

namespace sire::geometry {
auto SphereShape::setRadius(double radius_in) -> void {
  SIRE_DEMAND(radius_in > 0);
  radius_ = radius_in;
}
auto SphereShape::radius() -> double { return radius_; }

SphereShape::SphereShape(double radius_in) : radius_(radius_in) {
  SIRE_DEMAND(radius_in > 0);
}

SphereShape::~SphereShape() = default;
}  // namespace sire::geometry
