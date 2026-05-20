#ifndef SIRE_CYLINDER_COLLISION_GEOMETRY_HPP_
#define SIRE_CYLINDER_COLLISION_GEOMETRY_HPP_

#include "sire/core/geometry/cylinder_shape.hpp"
#include "sire/core/sire_decl_def_macro.hpp"
#include "sire/physics/geometry/collision_adapter.hpp"

namespace sire::physics::geometry {
class CylinderCollisionGeometry
    : public CollisionAdapter<CylinderCollisionGeometry,
                              sire::geometry::CylinderShape> {
 public:
  auto init() -> void override;
  explicit CylinderCollisionGeometry(double radius = 0.1, double length = 0.1,
                                     int part_id = 0, bool is_dynamic = false,
                                     const double* prt_pm = nullptr,
                                     bool visible = true,
                                     const std::string& material = "m1",
                                     const std::string& propStr = "{}");
  virtual ~CylinderCollisionGeometry();
  SIRE_DECLARE_MOVE_CTOR(CylinderCollisionGeometry)

  auto to_json(nlohmann::json& j) const -> void override;
};
}  // namespace sire::physics::geometry
#endif
