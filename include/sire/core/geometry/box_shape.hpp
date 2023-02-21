#ifndef SIRE_BOX_SHAPE_HPP_
#define SIRE_BOX_SHAPE_HPP_

#include <array>

namespace sire::geometry {
class BoxShape {
 private:
  std::array<double, 3> half_side_{0};

 public:
  auto virtual setHalfSide(double* side_in) -> void;
  auto virtual setHalfSide(double x_in, double y_in, double z_in) -> void;
  auto virtual halfSidePtr() -> const double*;
  auto virtual halfSide() -> std::array<double, 3>;
  auto virtual side() -> std::array<double, 3>;

  explicit BoxShape(double x, double y, double z);
  explicit BoxShape(double* side_in);
  virtual ~BoxShape();
};
}  // namespace sire::geometry

#endif