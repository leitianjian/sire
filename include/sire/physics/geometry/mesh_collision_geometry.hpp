#ifndef SIRE_MESH_COLLISION_GEOMETRY_HPP_
#define SIRE_MESH_COLLISION_GEOMETRY_HPP_

#include <atomic>
#include <string>
#include <utility>

#include <sire_lib_export.h>

#include <hpp/fcl/collision_object.h>

#include <aris/dynamic/model_basic.hpp>
#include <aris/dynamic/model_coordinate.hpp>

#include "sire/core/geometry/geometry_on_part.hpp"
#include "sire/physics/geometry/collidable_geometry.hpp"

namespace sire::physics::geometry {
/* unique geometry id for every added collision geometry */
using namespace std;
using namespace hpp;
using GeometryId = sire::core::geometry::GeometryId;
class SIRE_API MeshGeometry : public sire::physics::geometry::CollidableGeometry {
 public:
  auto resourcePath() const -> const string&;
  auto setResourcePath(string_view resource_path) -> void;
  auto scale() const -> const double*;
  auto setScale(const double* scale) -> void;
  auto init() -> void override;
  explicit MeshGeometry(const string& resource_path = "",
                        const double* prt_pm = nullptr);
  virtual ~MeshGeometry();
  MeshGeometry(const MeshGeometry& other) = delete;
  MeshGeometry(MeshGeometry&& other) = delete;
  MeshGeometry& operator=(const MeshGeometry& other) = delete;
  MeshGeometry& operator=(MeshGeometry&& other) = delete;

 private:
  struct Imp;
  aris::core::ImpPtr<Imp> imp_;
};
}  // namespace sire::physics::geometry
#endif