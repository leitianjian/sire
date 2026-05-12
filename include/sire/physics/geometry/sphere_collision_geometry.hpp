#ifndef SIRE_SPHERE_COLLISION_GEOMETRY_HPP_
#define SIRE_SPHERE_COLLISION_GEOMETRY_HPP_

#include <atomic>
#include <string>
#include <utility>

#include <sire_lib_export.h>

#include <coal/collision_object.h>

#include <aris/core/object.hpp>
#include <aris/dynamic/model_basic.hpp>
#include <aris/dynamic/model_coordinate.hpp>

#include "sire/core/geometry/sphere_shape.hpp"
#include "sire/core/sire_decl_def_macro.hpp"
#include "sire/ext/json.hpp"
#include "sire/physics/geometry/collision_adapter.hpp"

namespace sire::physics::geometry {
class SphereCollisionGeometry
    : public CollisionAdapter<SphereCollisionGeometry,
                              sire::geometry::SphereShape> {
 public:
  auto init() -> void override;
  explicit SphereCollisionGeometry(double radius = 0.1, int part_id = 0,
                                   bool is_dynamic = false,
                                   const double* prt_pm = nullptr,
                                   const std::string& material = "m1",
                                   const std::string& propStr = "{}");
  virtual ~SphereCollisionGeometry();
  SIRE_DECLARE_MOVE_CTOR(SphereCollisionGeometry)

  auto to_json(nlohmann::json& j) const -> void override;
};
}  // namespace sire::physics::geometry
#endif
