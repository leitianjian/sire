#include "sire/physics/geometry/collidable_geometry.hpp"

#include <array>
#include <memory>
#include <string>
#include <string_view>

#include <coal/BVH/BVH_model.h>
#include <coal/mesh_loader/assimp.h>
#include <coal/mesh_loader/loader.h>
#include <coal/shape/geometric_shapes.h>

#include <aris/core/reflection.hpp>
#include <aris/dynamic/model.hpp>
#include <aris/server/control_server.hpp>

#include "sire/core/constants.hpp"

namespace sire::physics::geometry {
// This prt_pm should be the part pose in world coordinate.
auto CollidableGeometry::updateLocation(const double* prt_pm) -> void {
  prt_pm = prt_pm ? prt_pm : sire::default_pm;
  double res[16]{0};
  aris::dynamic::s_pm_dot_pm(prt_pm, *pm(), res);
  getCollisionObject()->setTransform(
      Transform3s((Matrix3s() << res[0], res[1], res[2], res[4],
                        res[5], res[6], res[8], res[9], res[10])
                           .finished(),
                       (Vec3s() << res[3], res[7], res[11]).finished()));
  getCollisionObject()->computeAABB();
}
auto CollidableGeometry::init() -> void {}
CollidableGeometry::CollidableGeometry(const double* prt_pm, int part_id,
                                       bool is_dynamic)
    : sire::geometry::GeometryOnPart(prt_pm, part_id, is_dynamic) {}
CollidableGeometry::~CollidableGeometry() = default;
SIRE_DEFINE_MOVE_CTOR_CPP(CollidableGeometry);

ARIS_REGISTRATION {
  aris::core::class_<CollidableGeometry>("CollidableGeometry")
      .inherit<sire::geometry::GeometryOnPart>()
      .inherit<Collidable>();
}
}  // namespace sire::physics::geometry