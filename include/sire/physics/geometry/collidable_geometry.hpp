#ifndef SIRE_COLLIDABLE_GEOMETRY_HPP_
#define SIRE_COLLIDABLE_GEOMETRY_HPP_

#include <sire_lib_export.h>

#include <aris/core/object.hpp>
#include <aris/dynamic/model_basic.hpp>
#include <aris/dynamic/model_coordinate.hpp>

#include "sire/core/geometry/geometry_on_part.hpp"
#include "sire/physics/geometry/collidable.hpp"

namespace sire::physics::geometry {
class SIRE_API CollidableGeometry : public sire::geometry::GeometryOnPart,
                                    public Collidable {
 public:
  auto virtual updateLocation(const double* prt_pm) -> void override;
  auto virtual init() -> void override;
  explicit CollidableGeometry(const double* pm_in = nullptr, int part_id = 0,
                              bool is_dynamic = false);
  virtual ~CollidableGeometry();
  SIRE_DECLARE_MOVE_CTOR(CollidableGeometry);
};

}  // namespace sire::physics::geometry
#endif