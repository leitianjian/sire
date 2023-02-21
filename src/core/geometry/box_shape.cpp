#include "sire/core/geometry/box_shape.hpp"

#include "sire/core/sire_assert.hpp"

namespace sire::geometry {
auto BoxShape::setHalfSide(double* half_side_in) -> void {
  SIRE_ASSERT(half_side_in != nullptr);
  SIRE_DEMAND(half_side_in[0] > 0);
  SIRE_DEMAND(half_side_in[1] > 0);
  SIRE_DEMAND(half_side_in[2] > 0);
  half_side_[0] = half_side_in[0];
  half_side_[1] = half_side_in[1];
  half_side_[2] = half_side_in[2];
}
auto BoxShape::setHalfSide(double x_in, double y_in, double z_in) -> void {
  SIRE_DEMAND(x_in > 0);
  SIRE_DEMAND(y_in > 0);
  SIRE_DEMAND(z_in > 0);
  half_side_[0] = x_in / 2.0;
  half_side_[1] = y_in / 2.0;
  half_side_[2] = z_in / 2.0;
}
auto BoxShape::halfSidePtr() -> const double* { return half_side_.data(); }
auto BoxShape::halfSide() -> std::array<double, 3> { return half_side_; }
auto BoxShape::side() -> std::array<double, 3> {
  std::array<double, 3> temp = half_side_;
  temp[0] *= 2;
  temp[1] *= 2;
  temp[2] *= 2;
  return temp;
}

BoxShape::BoxShape(double x, double y, double z) {
  SIRE_DEMAND(x > 0);
  SIRE_DEMAND(y > 0);
  SIRE_DEMAND(z > 0);
  half_side_[0] = x / 2.0;
  half_side_[1] = y / 2.0;
  half_side_[2] = z / 2.0;
}

BoxShape::BoxShape(double* side_in) {
  SIRE_DEMAND(side_in != nullptr);
  SIRE_DEMAND(side_in[0] > 0);
  SIRE_DEMAND(side_in[1] > 0);
  SIRE_DEMAND(side_in[2] > 0);
  half_side_[0] = side_in[0] / 2.0;
  half_side_[1] = side_in[1] / 2.0;
  half_side_[2] = side_in[2] / 2.0;
}

BoxShape::~BoxShape() = default;
}  // namespace sire::geometry
