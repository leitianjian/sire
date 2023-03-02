#ifndef SIRE_BOX_COLLISION_GEOMETRY_HPP_
#define SIRE_BOX_COLLISION_GEOMETRY_HPP_

#include <atomic>
#include <string>
#include <utility>

#include <sire_lib_export.h>

#include <hpp/fcl/collision_object.h>

#include <aris/core/object.hpp>
#include <aris/dynamic/model_basic.hpp>
#include <aris/dynamic/model_coordinate.hpp>

#include "sire/core/geometry/box_geometry.hpp"
#include "sire/core/geometry/box_shape.hpp"
#include "sire/core/sire_decl_def_macro.hpp"
#include "sire/ext/json.hpp"
#include "sire/physics/geometry/collidable.hpp"
#include "sire/physics/geometry/collidable_geometry.hpp"

namespace sire::physics::geometry {
/* unique geometry id for every added collision geometry */
using json = nlohmann::json;
using namespace std;
using namespace hpp;
using GeometryId = sire::core::geometry::GeometryId;
class SIRE_API BoxCollisionGeometry : public sire::physics::geometry::CollidableGeometry,
                                      public sire::core::geometry::BoxShape {
 public:
  auto init() -> void override;
  explicit BoxCollisionGeometry(double x = 0.1, double y = 0.1, double z = 0.1,
                                const double* prt_pm = nullptr);
  virtual ~BoxCollisionGeometry();
  ARIS_DELETE_BIG_FOUR(BoxCollisionGeometry)
  // 类内部使用的to_json from_json的声明
  SIRE_DECLARE_JSON_INTER_OVERRIDE_TWO

  // nlohammn::json j = o;的时候会自动调用的to_json from_json的声明
  SIRE_DECLARE_JSON_FRIEND_TWO(BoxCollisionGeometry)
};
}  // namespace sire::physics::geometry
#endif