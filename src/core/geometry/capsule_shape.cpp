#include "sire/core/geometry/capsule_shape.hpp"

#include <aris/core/expression_calculator.hpp>
#include <aris/core/reflection.hpp>

#include "sire/core/sire_assert.hpp"
#include "sire/core/string_utils.hpp"

namespace sire::geometry {
auto CapsuleShape::setRadius(double radius) -> void {
  SIRE_DEMAND(radius > 0);
  radius_ = radius;
}
auto CapsuleShape::setLength(double length) -> void {
  SIRE_DEMAND(length > 0);
  length_ = length;
}
CapsuleShape::CapsuleShape(double radius, double length)
    : ShapeBase(ShapeTag<CapsuleShape>()) {
  if (radius <= 0 || length <= 0) {
    throw std::logic_error(sire::core::string_format(
        "Capsule radius and length should all be > 0 (were %d, "
        "%d, respectively).",
        radius, length));
  }
  radius_ = radius;
  length_ = length;
  // setShapeType(ShapeType::GEOM_BOX);
}

CapsuleShape::~CapsuleShape() = default;
ARIS_DEFINE_BIG_FOUR_CPP(CapsuleShape)

ARIS_REGISTRATION {
  aris::core::class_<CapsuleShape>("CapsuleShape")
      .prop("radius", &CapsuleShape::setRadius, &CapsuleShape::radius)
      .prop("length", &CapsuleShape::setLength, &CapsuleShape::length);
}
}  // namespace sire::geometry
