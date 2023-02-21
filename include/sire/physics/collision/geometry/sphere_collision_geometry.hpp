#ifndef SIRE_SPHERE_COLLISION_GEOMETRY_HPP_
#define SIRE_SPHERE_COLLISION_GEOMETRY_HPP_

#include <atomic>
#include <string>
#include <utility>

#include <sire_lib_export.h>

#include <hpp/fcl/collision_object.h>

#include <aris/dynamic/model_basic.hpp>
#include <aris/dynamic/model_coordinate.hpp>

#include "sire/core/geometry/sphere_geometry.hpp"
#include "sire/core/geometry/sphere_shape.hpp"
#include "sire/physics/collision/geometry/collidable.hpp"
#include "sire/physics/collision/geometry/collidable_geometry.hpp"

namespace sire::collision {
namespace geometry {
/* unique geometry id for every added collision geometry */
using namespace std;
using namespace hpp;
using GeometryId = sire::geometry::GeometryId;
class SIRE_API SphereCollisionGeometry : public CollidableGeometry,
                                         public sire::geometry::SphereShape {
 public:
  auto radius() -> double override;
  auto setRadius(double radius_in) -> void override;
  auto init() -> void override;
  explicit SphereCollisionGeometry(double radius = 0.1,
                                   const double* prt_pm = nullptr);
  virtual ~SphereCollisionGeometry();
  SphereCollisionGeometry(const SphereCollisionGeometry& other) = delete;
  SphereCollisionGeometry(SphereCollisionGeometry&& other) = delete;
  SphereCollisionGeometry& operator=(const SphereCollisionGeometry& other) =
      delete;
  SphereCollisionGeometry& operator=(SphereCollisionGeometry&& other) = delete;
};
}  // namespace geometry
}  // namespace sire::collision
#endif