#ifndef SIRE_SPHERE_GEOMETRY_HPP_
#define SIRE_SPHERE_GEOMETRY_HPP_

#include <atomic>
#include <string>
#include <utility>

#include <sire_lib_export.h>

#include <aris/core/object.hpp>
#include <aris/dynamic/model_basic.hpp>
#include <aris/dynamic/model_coordinate.hpp>

#include "sire/core/geometry/geometry_adapter.hpp"
#include "sire/core/geometry/sphere_shape.hpp"
#include "sire/core/sire_decl_def_macro.hpp"
#include "sire/ext/json.hpp"

namespace sire::geometry {
class SphereGeometry : public GeometryAdapter<SphereGeometry, SphereShape> {
 public:
  explicit SphereGeometry(double radius = 0.1, int part_id = 0,
                          bool is_dynamic = false,
                          const double* prt_pm = nullptr);
  virtual ~SphereGeometry();
  ARIS_DECLARE_BIG_FOUR(SphereGeometry)

  auto to_json(nlohmann::json& j) const -> void override;
};
}  // namespace sire::geometry
#endif
