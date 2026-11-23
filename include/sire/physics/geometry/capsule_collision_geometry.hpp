#ifndef SIRE_CAPSULE_COLLISION_GEOMETRY_HPP_
#define SIRE_CAPSULE_COLLISION_GEOMETRY_HPP_

#include "sire/core/geometry/capsule_shape.hpp"
#include "sire/core/sire_decl_def_macro.hpp"
#include "sire/physics/geometry/collision_adapter.hpp"

namespace sire::physics::geometry {
class CapsuleCollisionGeometry
    : public CollisionAdapter<CapsuleCollisionGeometry,
                              sire::geometry::CapsuleShape> {
 public:
  auto init() -> void override;
  explicit CapsuleCollisionGeometry(double radius = 0.1, double length = 0.1,
                                    int part_id = 0, bool is_dynamic = false,
                                    const double* prt_pm = nullptr,
                                    bool visible = true,
                                    const std::string& material = "m1",
                                    const std::string& propStr = "{}");
  virtual ~CapsuleCollisionGeometry();
  SIRE_DECLARE_MOVE_CTOR(CapsuleCollisionGeometry)

  auto to_json(nlohmann::json& j) const -> void override;
};
}  // namespace sire::physics::geometry
#endif
