#include "sire/physics/geometry/collidable_geometry.hpp"

#include <array>
#include <memory>
#include <string>
#include <string_view>

#include <hpp/fcl/bvh/BVH_model.h>
#include <hpp/fcl/mesh_loader/assimp.h>
#include <hpp/fcl/mesh_loader/loader.h>
#include <hpp/fcl/shape/geometric_shapes.h>

#include <aris/core/reflection.hpp>
#include <aris/dynamic/model.hpp>
#include <aris/server/control_server.hpp>

namespace sire::collision::geometry {
auto CollidableGeometry::updateLocation(const double* prt_pm) -> void {
  prt_pm = prt_pm ? prt_pm : sire::geometry::default_pm;
  double res[16];
  aris::dynamic::s_pm_dot_pm(prt_pm, *partPm(), res);
  getCollisionObject()->setTransform(
      fcl::Transform3f(fcl::Matrix3f{{res[0], res[1], res[2]},
                                     {res[4], res[5], res[6]},
                                     {res[8], res[9], res[10]}},
                       fcl::Vec3f{res[3], res[7], res[11]}));
  getCollisionObject()->computeAABB();
}
auto CollidableGeometry::init() -> void {}
CollidableGeometry::CollidableGeometry() : sire::geometry::GeometryOnPart() {
  aris::dynamic::s_vc(16, sire::geometry::default_pm,
                      const_cast<double*>(*partPm()));
}
CollidableGeometry::CollidableGeometry(const double* prt_pm)
    : sire::geometry::GeometryOnPart() {
  prt_pm = prt_pm ? prt_pm : sire::geometry::default_pm;
  aris::dynamic::s_vc(16, prt_pm, const_cast<double*>(*partPm()));
}
CollidableGeometry::~CollidableGeometry() = default;

ARIS_REGISTRATION {
  aris::core::class_<CollidableGeometry>("CollidableGeometry")
      .inherit<sire::geometry::GeometryOnPart>();
}
}  // namespace sire::collision::geometry