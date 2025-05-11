#ifndef SIRE_CAPSULE_GEOMETRY_HPP_
#define SIRE_CAPSULE_GEOMETRY_HPP_

#include <atomic>
#include <string>
#include <utility>

#include <aris/core/object.hpp>
#include <aris/dynamic/model_basic.hpp>
#include <aris/dynamic/model_coordinate.hpp>

#include "sire/core/geometry/capsule_shape.hpp"
#include "sire/core/geometry/geometry_on_part.hpp"
#include "sire/core/sire_decl_def_macro.hpp"
#include "sire/ext/json.hpp"

namespace sire::geometry {
using namespace std;
using json = nlohmann::json;
class CapsuleGeometry : public GeometryOnPart {
 public:
  CapsuleShape capsuleShape;
  explicit CapsuleGeometry(double radius = 0.1, double length = 0.2,
                           const double* prt_pm = nullptr);
  virtual ~CapsuleGeometry();
  ARIS_DECLARE_BIG_FOUR(CapsuleGeometry)

  // 类内部使用的to_json from_json的声明
  SIRE_DECLARE_JSON_INTER_OVERRIDE_TWO

  // nlohammn::json j = o;的时候会自动调用的to_json from_json的声明
  SIRE_DECLARE_JSON_FRIEND_TWO(CapsuleGeometry)
};
}  // namespace sire::geometry
#endif