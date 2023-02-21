#ifndef SIRE_BOX_COLLISION_GEOMETRY_HPP_
#define SIRE_BOX_COLLISION_GEOMETRY_HPP_

#include <atomic>
#include <string>
#include <utility>

#include <sire_lib_export.h>

#include <hpp/fcl/collision_object.h>

#include <aris/dynamic/model_basic.hpp>
#include <aris/dynamic/model_coordinate.hpp>

#include "sire/core/geometry/box_geometry.hpp"
#include "sire/core/geometry/box_shape.hpp"
#include "sire/physics/collision/geometry/collidable.hpp"
#include "sire/physics/collision/geometry/collidable_geometry.hpp"

namespace sire::collision {
namespace geometry {
/* unique geometry id for every added collision geometry */
using namespace std;
using namespace hpp;
using GeometryId = sire::geometry::GeometryId;
class SIRE_API BoxCollisionGeometry : public CollidableGeometry,
                                      public sire::geometry::BoxShape {
 public:
  auto init() -> void override;
  explicit BoxCollisionGeometry(double x = 0.1, double y = 0.1, double z = 0.1,
                                const double* prt_pm = nullptr);
  virtual ~BoxCollisionGeometry();
  BoxCollisionGeometry(const BoxCollisionGeometry& other) = delete;
  BoxCollisionGeometry(BoxCollisionGeometry&& other) = delete;
  BoxCollisionGeometry& operator=(const BoxCollisionGeometry& other) = delete;
  BoxCollisionGeometry& operator=(BoxCollisionGeometry&& other) = delete;
};
}  // namespace geometry
}  // namespace sire::collision
#endif