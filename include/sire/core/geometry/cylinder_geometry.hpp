#ifndef SIRE_CYLINDER_GEOMETRY_HPP_
#define SIRE_CYLINDER_GEOMETRY_HPP_

#include "sire/core/geometry/cylinder_shape.hpp"
#include "sire/core/geometry/geometry_adapter.hpp"
#include "sire/core/sire_decl_def_macro.hpp"

namespace sire::geometry {
class CylinderGeometry : public GeometryAdapter<CylinderGeometry, CylinderShape> {
 public:
  explicit CylinderGeometry(double radius = 0.1, double length = 0.1,
                            int part_id = 0, bool is_dynamic = false,
                            const double* prt_pm = nullptr);
  virtual ~CylinderGeometry();
  ARIS_DECLARE_BIG_FOUR(CylinderGeometry)

  auto to_json(nlohmann::json& j) const -> void override;
};
}  // namespace sire::geometry
#endif
