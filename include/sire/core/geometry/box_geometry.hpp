#ifndef SIRE_BOX_GEOMETRY_HPP_
#define SIRE_BOX_GEOMETRY_HPP_

#include <atomic>
#include <string>
#include <utility>

#include <aris/core/object.hpp>
#include <aris/dynamic/model_basic.hpp>
#include <aris/dynamic/model_coordinate.hpp>

#include "sire/core/geometry/box_shape.hpp"
#include "sire/core/geometry/geometry_adapter.hpp"
#include "sire/core/sire_decl_def_macro.hpp"
#include "sire/ext/json.hpp"

namespace sire::geometry {
class BoxGeometry : public GeometryAdapter<BoxGeometry, BoxShape> {
 public:
  explicit BoxGeometry(double x = 0.1, double y = 0.1, double z = 0.1,
                       int part_id = 0, bool is_dynamic = false,
                       const double* prt_pm = nullptr);
  virtual ~BoxGeometry();
  ARIS_DECLARE_BIG_FOUR(BoxGeometry)

  auto to_json(nlohmann::json& j) const -> void override;
};
}  // namespace sire::geometry
#endif
