#ifndef SIRE_MESH_GEOMETRY_HPP_
#define SIRE_MESH_GEOMETRY_HPP_

#include "sire/core/geometry/geometry_adapter.hpp"
#include "sire/core/geometry/mesh_shape.hpp"
#include "sire/core/sire_decl_def_macro.hpp"

namespace sire::geometry {
class MeshGeometry : public GeometryAdapter<MeshGeometry, MeshShape> {
 public:
  explicit MeshGeometry(std::string file_path = "plane.obj",
                        const std::array<double, 3>& scale = {1.0},
                        int part_id = 0, bool is_dynamic = false,
                        const double* prt_pm = nullptr);
  virtual ~MeshGeometry();
  ARIS_DECLARE_BIG_FOUR(MeshGeometry)

  auto to_json(nlohmann::json& j) const -> void override;
};
}  // namespace sire::geometry
#endif
