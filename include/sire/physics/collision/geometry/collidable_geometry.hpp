#ifndef SIRE_COLLIDABLE_GEOMETRY_HPP_
#define SIRE_COLLIDABLE_GEOMETRY_HPP_

#include <atomic>

#include <aris/core/object.hpp>
#include <aris/dynamic/model_basic.hpp>
#include <aris/dynamic/model_coordinate.hpp>

#include "sire/core/geometry/geometry_on_part.hpp"
#include "sire/physics/collision/geometry/collidable.hpp"

namespace sire::collision::geometry {
class CollidableGeometry : public sire::geometry::GeometryOnPart,
                           public Collidable {
 public:
  auto virtual updateLocation(const double* prt_pm) -> void override;
  auto virtual init() -> void override;
  explicit CollidableGeometry(const double* pm_in = nullptr);
  virtual ~CollidableGeometry();
};

}  // namespace sire::collision::geometry
#endif