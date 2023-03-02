#ifndef SIRE_SPHERE_COLLISION_GEOMETRY_HPP_
#define SIRE_SPHERE_COLLISION_GEOMETRY_HPP_

#include <atomic>
#include <string>
#include <utility>

#include <sire_lib_export.h>

#include <hpp/fcl/collision_object.h>

#include <aris/core/object.hpp>
#include <aris/dynamic/model_basic.hpp>
#include <aris/dynamic/model_coordinate.hpp>

#include "sire/core/geometry/sphere_geometry.hpp"
#include "sire/core/geometry/sphere_shape.hpp"
#include "sire/core/sire_decl_def_macro.hpp"
#include "sire/ext/json.hpp"
#include "sire/physics/geometry/collidable.hpp"
#include "sire/physics/geometry/collidable_geometry.hpp"

namespace sire::physics::geometry {
using json = nlohmann::json;
/* unique geometry id for every added collision geometry */
using namespace std;
using namespace hpp;
using GeometryId = sire::core::geometry::GeometryId;
class SIRE_API SphereCollisionGeometry : public sire::physics::geometry::CollidableGeometry,
                                         public sire::core::geometry::SphereShape {
 public:
  auto init() -> void override;
  explicit SphereCollisionGeometry(double radius = 0.1, const double* prt_pm = nullptr);
  virtual ~SphereCollisionGeometry();
  ARIS_DELETE_BIG_FOUR(SphereCollisionGeometry)
  SIRE_DECLARE_JSON_INTER_OVERRIDE_TWO
};
}  // namespace sire::physics::geometry
#endif