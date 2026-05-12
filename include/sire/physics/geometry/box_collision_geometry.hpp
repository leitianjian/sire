#ifndef SIRE_BOX_COLLISION_GEOMETRY_HPP_
#define SIRE_BOX_COLLISION_GEOMETRY_HPP_

#include <atomic>
#include <string>
#include <utility>

#include <sire_lib_export.h>

#include <coal/collision_object.h>

#include <aris/core/object.hpp>
#include <aris/dynamic/model_basic.hpp>
#include <aris/dynamic/model_coordinate.hpp>

#include "sire/core/geometry/box_shape.hpp"
#include "sire/core/sire_decl_def_macro.hpp"
#include "sire/ext/json.hpp"
#include "sire/physics/geometry/collision_adapter.hpp"

namespace sire::physics::geometry {
class BoxCollisionGeometry
    : public CollisionAdapter<BoxCollisionGeometry, sire::geometry::BoxShape> {
 public:
  auto init() -> void override;
  explicit BoxCollisionGeometry(double x = 0.1, double y = 0.1, double z = 0.1,
                                int part_id = 0, bool is_dynamic = false,
                                const double* prt_pm = nullptr,
                                const std::string& material = "m1",
                                const std::string& propStr = "{}");
  virtual ~BoxCollisionGeometry();
  SIRE_DECLARE_MOVE_CTOR(BoxCollisionGeometry)

  auto to_json(nlohmann::json& j) const -> void override;
};
}  // namespace sire::physics::geometry
#endif
