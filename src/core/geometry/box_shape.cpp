#include "sire/core/geometry/box_shape.hpp"

#include <aris/core/expression_calculator.hpp>
#include <aris/core/reflection.hpp>

#include "sire/core/sire_assert.hpp"

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
auto BoxShape::sidePtr() const -> const double* { return side_.data(); }

auto BoxShape::side() const -> const std::array<double, 3> { return side_; }

auto BoxShape::length() const -> double { return side_[0]; }

auto BoxShape::length() -> double& { return side_[0]; }

auto BoxShape::width() const -> double { return side_[1]; }

auto BoxShape::width() -> double& { return side_[1]; }

auto BoxShape::height() const -> double { return side_[2]; }

auto BoxShape::height() -> double& { return side_[2]; }

BoxShape::BoxShape(double length, double width, double height) {
  SIRE_DEMAND(length > 0);
  SIRE_DEMAND(width > 0);
  SIRE_DEMAND(height > 0);
  side_[0] = length;
  side_[1] = width;
  side_[2] = height;
  setShapeType(ShapeType::GEOM_BOX);
}

BoxShape::BoxShape(double* side_in) {
  SIRE_DEMAND(side_in != nullptr);
  SIRE_DEMAND(side_in[0] > 0);
  SIRE_DEMAND(side_in[1] > 0);
  SIRE_DEMAND(side_in[2] > 0);
  side_[0] = side_in[0];
  side_[1] = side_in[1];
  side_[2] = side_in[2];
  setShapeType(ShapeType::GEOM_BOX);
}

BoxShape::~BoxShape() = default;

ARIS_REGISTRATION {
  auto setSide = [](BoxShape* shape, aris::core::Matrix mat) -> void {
    shape->setSide(mat.data());
  };
  auto getSide = [](BoxShape* shape) -> aris::core::Matrix {
    return aris::core::Matrix(1, 3, shape->sidePtr());
  };
  aris::core::class_<BoxShape>("BoxShape")
      .inherit<ShapeBase>()
      .prop("side", &setSide, &getSide);
}
}  // namespace sire::geometry
