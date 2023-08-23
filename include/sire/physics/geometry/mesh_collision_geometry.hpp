#ifndef SIRE_MESH_COLLISION_GEOMETRY_HPP_
#define SIRE_MESH_COLLISION_GEOMETRY_HPP_

#include <atomic>
#include <string>
#include <utility>

#include <sire_lib_export.h>

#include <hpp/fcl/collision_object.h>

#include <aris/dynamic/model_basic.hpp>
#include <aris/dynamic/model_coordinate.hpp>

#include "sire/core/geometry/geometry_on_part.hpp"
#include "sire/core/geometry/mesh_shape.hpp"
#include "sire/core/sire_decl_def_macro.hpp"
#include "sire/physics/geometry/collidable_geometry.hpp"

namespace sire::physics {
namespace geometry {
/* unique geometry id for every added collision geometry */
using namespace std;
using namespace hpp;
using GeometryId = sire::geometry::GeometryId;
class SIRE_API MeshCollisionGeometry : public CollidableGeometry,
                                       public sire::geometry::MeshShape {
 public:
  auto scale() const -> const double*;
  auto setScale(const double* scale) -> void;
  auto init() -> void override;
  explicit MeshCollisionGeometry(const string& resource_path = "",
                                 const double* prt_pm = nullptr);
  virtual ~MeshCollisionGeometry();
  SIRE_DECLARE_MOVE_CTOR(MeshCollisionGeometry);
  // 类内部使用的to_json from_json的声明
  SIRE_DECLARE_JSON_INTER_OVERRIDE_TWO

  // nlohammn::json j = o;的时候会自动调用的to_json from_json的声明
  SIRE_DECLARE_JSON_FRIEND_TWO(MeshCollisionGeometry)

 private:
  struct Imp;
  aris::core::ImpPtr<Imp> imp_;
};
}  // namespace geometry
}  // namespace sire::physics
#endif