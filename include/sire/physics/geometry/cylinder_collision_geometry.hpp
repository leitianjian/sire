#ifndef SIRE_CYLINDER_COLLISION_GEOMETRY_HPP_
#define SIRE_CYLINDER_COLLISION_GEOMETRY_HPP_

#include <atomic>
#include <string>
#include <utility>

#include <sire_lib_export.h>

#include <coal/collision_object.h>

#include <aris/core/object.hpp>
#include <aris/dynamic/model_basic.hpp>
#include <aris/dynamic/model_coordinate.hpp>

#include "sire/core/geometry/cylinder_shape.hpp"
#include "sire/core/sire_decl_def_macro.hpp"
#include "sire/ext/json.hpp"
#include "sire/physics/geometry/collidable.hpp"
#include "sire/physics/geometry/collidable_geometry.hpp"

namespace sire::physics {
namespace geometry {
/* unique geometry id for every added collision geometry */
using json = nlohmann::json;
using namespace std;
using namespace coal;
using GeometryId = sire::geometry::GeometryId;
class CylinderCollisionGeometry : public CollidableGeometry {
 public:
  sire::geometry::CylinderShape cylinderShape;
  auto init() -> void override;
  explicit CylinderCollisionGeometry(double radius = 0.1, double length = 0.2,
                                     int part_id = 0, bool is_dynamic = false,
                                     const double* prt_pm = nullptr,
                                     const std::string& material = "m1",
                                     const std::string& propStr = "{}");
  virtual ~CylinderCollisionGeometry();
  SIRE_DECLARE_MOVE_CTOR(CylinderCollisionGeometry)
  // 类内部使用的to_json from_json的声明
  SIRE_DECLARE_JSON_INTER_OVERRIDE_TWO

  // nlohammn::json j = o;的时候会自动调用的to_json from_json的声明
  SIRE_DECLARE_JSON_FRIEND_TWO(CylinderCollisionGeometry)
};
}  // namespace geometry
}  // namespace sire::physics
#endif