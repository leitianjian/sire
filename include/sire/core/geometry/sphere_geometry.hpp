#ifndef SIRE_SPHERE_GEOMETRY_HPP_
#define SIRE_SPHERE_GEOMETRY_HPP_

#include <atomic>
#include <string>
#include <utility>

#include <sire_lib_export.h>

#include <aris/dynamic/model_basic.hpp>
#include <aris/dynamic/model_coordinate.hpp>

#include "sire/core/geometry/geometry_on_part.hpp"
#include "sire/core/geometry/sphere_shape.hpp"

namespace sire::geometry {
using namespace std;
class SIRE_API SphereGeometry : public GeometryOnPart,
                                public SphereShape {
 public:
  auto radius() -> double override;
  auto setRadius(double radius_in) -> void override;
  explicit SphereGeometry(double radius = 0.1, const double* prt_pm = nullptr);
  virtual ~SphereGeometry();
  SphereGeometry(const SphereGeometry& other) = delete;
  SphereGeometry(SphereGeometry&& other) = delete;
  SphereGeometry& operator=(const SphereGeometry& other) = delete;
  SphereGeometry& operator=(SphereGeometry&& other) = delete;
};
}  // namespace sire::geometry
#endif