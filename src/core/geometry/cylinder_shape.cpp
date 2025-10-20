#include "sire/core/geometry/cylinder_shape.hpp"

#include <aris/core/expression_calculator.hpp>
#include <aris/core/reflection.hpp>

#include "sire/core/sire_assert.hpp"
#include "sire/core/string_utils.hpp"

namespace sire::geometry {
auto CylinderShape::setRadius(double radius) -> void {
  SIRE_DEMAND(radius > 0);
  radius_ = radius;
}
auto CylinderShape::setLength(double length) -> void {
  SIRE_DEMAND(length > 0);
  length_ = length;
}
CylinderShape::CylinderShape(double radius, double length)
    : ShapeBase(ShapeTag<CylinderShape>()) {
  if (radius <= 0 || length <= 0) {
    throw std::logic_error(sire::core::string_format(
        "Cylinder radius and length should all be > 0 (were %d, "
        "%d, respectively).",
        radius, length));
  }
  radius_ = radius;
  length_ = length;
  // setShapeType(ShapeType::GEOM_BOX);
}

CylinderShape::~CylinderShape() = default;
ARIS_DEFINE_BIG_FOUR_CPP(CylinderShape)

ARIS_REGISTRATION {
  aris::core::class_<CylinderShape>("CylinderShape")
      .prop("radius", &CylinderShape::setRadius, &CylinderShape::radius)
      .prop("length", &CylinderShape::setLength, &CylinderShape::length);
}
}  // namespace sire::geometry
