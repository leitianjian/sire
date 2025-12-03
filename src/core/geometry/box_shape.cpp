#include "sire/core/geometry/box_shape.hpp"

#include <aris/core/expression_calculator.hpp>
#include <aris/core/reflection.hpp>

#include "sire/core/sire_assert.hpp"
#include "sire/core/string_utils.hpp"

namespace sire::geometry {
auto BoxShape::setSide(double* side_in) -> void {
  SIRE_ASSERT(side_in != nullptr);
  SIRE_DEMAND(side_in[0] > 0);
  SIRE_DEMAND(side_in[1] > 0);
  SIRE_DEMAND(side_in[2] > 0);
  side_[0] = side_in[0];
  side_[1] = side_in[1];
  side_[2] = side_in[2];
}
auto BoxShape::setSide(double length, double width, double height) -> void {
  SIRE_DEMAND(length > 0);
  SIRE_DEMAND(width > 0);
  SIRE_DEMAND(height > 0);
  side_[0] = length;
  side_[1] = width;
  side_[2] = height;
}

auto BoxShape::length() -> double& { return side_[0]; }

auto BoxShape::width() -> double& { return side_[1]; }

auto BoxShape::height() -> double& { return side_[2]; }

BoxShape BoxShape::MakeCube(double edge_size) {
  return BoxShape(edge_size, edge_size, edge_size);
}
BoxShape::BoxShape(double length, double width, double height)
    : ShapeBase(ShapeTag<BoxShape>()) {
  if (length <= 0 || width <= 0 || height <= 0) {
    throw std::logic_error(sire::core::string_format(
        "Box width, depth, and height should all be > 0 (were %d, "
        "%d, and %d, respectively).",
        length, width, height));
  }
  side_[0] = length;
  side_[1] = width;
  side_[2] = height;
  // setShapeType(ShapeType::GEOM_BOX);
}

BoxShape::BoxShape(const double* side_in)
    : BoxShape(side_in[0], side_in[1], side_in[2]) {}

BoxShape::~BoxShape() = default;
ARIS_DEFINE_BIG_FOUR_CPP(BoxShape)

ARIS_REGISTRATION {
  auto setSide = [](BoxShape* shape, aris::core::Matrix mat) -> void {
    shape->setSide(mat.data());
  };
  auto getSide = [](BoxShape* shape) -> aris::core::Matrix {
    return aris::core::Matrix(1, 3, shape->side());
  };
  aris::core::class_<BoxShape>("BoxShape").prop("side", &setSide, &getSide);
}
}  // namespace sire::geometry
