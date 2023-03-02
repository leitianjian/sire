#ifndef SIRE_SPHERE_GEOMETRY_HPP_
#define SIRE_SPHERE_GEOMETRY_HPP_

#include <atomic>
#include <string>
#include <utility>

#include <sire_lib_export.h>

#include <aris/core/object.hpp>
#include <aris/dynamic/model_basic.hpp>
#include <aris/dynamic/model_coordinate.hpp>

#include "sire/core/geometry/geometry_on_part.hpp"
#include "sire/core/geometry/sphere_shape.hpp"
#include "sire/core/sire_decl_def_macro.hpp"
#include "sire/ext/json.hpp"

namespace sire::core::geometry {
  using namespace std;
using json = nlohmann::json;
class SIRE_API SphereGeometry : public GeometryOnPart, public SphereShape {
 public:
  explicit SphereGeometry(double radius = 0.1, const double* prt_pm = nullptr);
  virtual ~SphereGeometry();
  ARIS_DECLARE_BIG_FOUR(SphereGeometry)

  // 类内部使用的to_json from_json的声明
  SIRE_DECLARE_JSON_INTER_OVERRIDE_TWO

  // nlohammn::json j = o;的时候会自动调用的to_json from_json的声明
  SIRE_DECLARE_JSON_FRIEND_TWO(SphereGeometry)
};
}  // namespace sire::geometry
#endif