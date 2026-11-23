#ifndef SIRE_MESH_COLLISION_GEOMETRY_HPP_
#define SIRE_MESH_COLLISION_GEOMETRY_HPP_

#include "sire/core/geometry/mesh_shape.hpp"
#include "sire/core/sire_decl_def_macro.hpp"
#include "sire/physics/geometry/collision_adapter.hpp"

namespace sire::physics::geometry {
class MeshCollisionGeometry
    : public CollisionAdapter<MeshCollisionGeometry,
                              sire::geometry::MeshShape> {
 public:
  auto init() -> void override;
  explicit MeshCollisionGeometry(std::string file_path = "plane.obj",
                                 const std::array<double, 3>& scale = {1.0},
                                 int part_id = 0, bool is_dynamic = false,
                                 const double* prt_pm = nullptr,
                                 bool visible = true,
                                 const std::string& material = "m1",
                                 const std::string& propStr = "{}");
  virtual ~MeshCollisionGeometry();
  SIRE_DECLARE_MOVE_CTOR(MeshCollisionGeometry)

  auto to_json(nlohmann::json& j) const -> void override;
};
}  // namespace sire::physics::geometry
#endif
