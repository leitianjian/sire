#ifndef SIRE_BOX_SHAPE_HPP_
#define SIRE_BOX_SHAPE_HPP_

#include <array>

#include <sire_lib_export.h>

#include "sire/core/geometry/shape_base.hpp"

namespace sire::geometry {
class SIRE_API BoxShape : public ShapeBase {
 private:
  std::array<double, 3> side_{0.1, 0.1, 0.1};

 public:
  auto virtual setSide(double* side_in) -> void;
  auto virtual setSide(double length, double width, double height) -> void;
  auto virtual sidePtr() const -> const double*;
  auto virtual side() const -> const std::array<double, 3>;
  auto virtual length() -> double&;
  auto virtual length() const -> double;
  auto virtual width() -> double&;
  auto virtual width() const -> double;
  auto virtual height() -> double&;
  auto virtual height() const -> double;

  explicit BoxShape(double length = 0.1, double width = 0.1,
                    double height = 0.1);
  explicit BoxShape(double* side_in);
  virtual ~BoxShape();
};
}  // namespace sire::geometry

#endif