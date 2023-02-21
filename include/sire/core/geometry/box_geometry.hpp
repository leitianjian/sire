#ifndef SIRE_BOX_GEOMETRY_HPP_
#define SIRE_BOX_GEOMETRY_HPP_

#include <atomic>
#include <string>
#include <utility>

#include <sire_lib_export.h>

#include <aris/dynamic/model_basic.hpp>
#include <aris/dynamic/model_coordinate.hpp>

#include "sire/core/geometry/geometry_on_part.hpp"
#include "sire/core/geometry/box_shape.hpp"

namespace sire::geometry {
using namespace std;
class SIRE_API BoxGeometry : public GeometryOnPart, public BoxShape {
 public:
  auto setHalfSide(double* side_in) -> void override;
  auto setHalfSide(double x_in, double y_in, double z_in) -> void override;
  auto halfSidePtr() -> const double* override;
  auto halfSide() -> std::array<double, 3> override;
  explicit BoxGeometry(double x, double y, double z, const double* prt_pm = nullptr);
  virtual ~BoxGeometry();
  BoxGeometry(const BoxGeometry& other) = delete;
  BoxGeometry(BoxGeometry&& other) = delete;
  BoxGeometry& operator=(const BoxGeometry& other) = delete;
  BoxGeometry& operator=(BoxGeometry&& other) = delete;
};
}  // namespace sire::geometry
#endif