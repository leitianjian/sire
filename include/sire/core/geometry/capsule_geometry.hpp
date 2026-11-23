#ifndef SIRE_CAPSULE_GEOMETRY_HPP_
#define SIRE_CAPSULE_GEOMETRY_HPP_

#include <atomic>
#include <string>
#include <utility>

#include <aris/core/object.hpp>
#include <aris/dynamic/model_basic.hpp>
#include <aris/dynamic/model_coordinate.hpp>

#include "sire/core/geometry/capsule_shape.hpp"
#include "sire/core/geometry/geometry_adapter.hpp"
#include "sire/core/sire_decl_def_macro.hpp"
#include "sire/ext/json.hpp"

namespace sire::geometry {
class CapsuleGeometry : public GeometryAdapter<CapsuleGeometry, CapsuleShape> {
 public:
  explicit CapsuleGeometry(double radius = 0.1, double length = 0.1,
                           int part_id = 0, bool is_dynamic = false,
                           const double* prt_pm = nullptr);
  virtual ~CapsuleGeometry();
  ARIS_DECLARE_BIG_FOUR(CapsuleGeometry)

  auto to_json(nlohmann::json& j) const -> void override;
};
}  // namespace sire::geometry
#endif
