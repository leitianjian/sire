#ifndef SIRE_MESH_GEOMETRY_HPP_
#define SIRE_MESH_GEOMETRY_HPP_

#include <string>
#include <utility>

#include <sire_lib_export.h>

#include <aris/core/object.hpp>
#include <aris/dynamic/model_basic.hpp>
#include <aris/dynamic/model_coordinate.hpp>

#include "sire/core/geometry/geometry_on_part.hpp"
#include "sire/core/geometry/mesh_shape.hpp"
#include "sire/core/sire_decl_def_macro.hpp"
#include "sire/ext/json.hpp"

namespace sire::geometry {
using namespace std;
class SIRE_API MeshGeometry : public GeometryOnPart, public MeshShape {
 public:
  explicit MeshGeometry(string resource_path = "");
  virtual ~MeshGeometry();
  ARIS_DECLARE_BIG_FOUR(MeshGeometry)

  // 类内部使用的to_json from_json的声明
  SIRE_DECLARE_JSON_INTER_OVERRIDE_TWO

  // nlohammn::json j = o;的时候会自动调用的to_json from_json的声明
  SIRE_DECLARE_JSON_FRIEND_TWO(MeshGeometry)
};
}  // namespace sire::geometry
#endif