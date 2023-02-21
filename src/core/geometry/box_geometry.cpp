#include "sire/core/geometry/box_geometry.hpp"

#include <array>
#include <memory>
#include <string>
#include <string_view>

#include <aris/core/expression_calculator.hpp>
#include <aris/core/reflection.hpp>
#include <aris/dynamic/model.hpp>
#include <aris/server/control_server.hpp>

namespace sire::geometry {
auto BoxGeometry::setHalfSide(double* side_in) -> void override {
  BoxShape::setHalfSide(side_in);
}
auto BoxGeometry::setHalfSide(double x_in, double y_in, double z_in)
    -> void override {
  BoxShape::setHalfSide(x_in, y_in, z_in);
}
auto BoxGeometry::halfSidePtr() -> const double* override {
  return BoxShape::halfSidePtr();
}
auto BoxGeometry::halfSide() -> std::array<double, 3> override {
  return BoxShape::halfSide();
}
BoxGeometry::BoxGeometry(double x, double y, double z, const double* prt_pm)
    : GeometryOnPart(), BoxShape(x, y, z) {}
BoxGeometry::~BoxGeometry() = default;

ARIS_REGISTRATION {
  auto setHalfSize = [](BoxGeometry* box, aris::core::Matrix mat) -> void {
    box->setHalfSide(mat.data());
  };
  auto getHalfSize = [](BoxGeometry* box) -> aris::core::Matrix {
    return aris::core::Matrix(1, 3, box->halfSidePtr())
  };
  aris::core::class_<BoxGeometry>("BoxGeometry")
      .inherit<GeometryOnPart>()
      .prop("half_size", &setHalfSize, &getHalfSize);
}
}  // namespace sire::geometry