#include "sire/physics/collision/geometry/sphere_collision_geometry.hpp"

#include <array>
#include <memory>
#include <string>
#include <string_view>

#include <hpp/fcl/bvh/BVH_model.h>
#include <hpp/fcl/shape/geometric_shapes.h>

#include <aris/core/reflection.hpp>

namespace sire::collision::geometry {
auto SphereCollisionGeometry::radius() -> double {
  return SphereShape::radius();
}
auto SphereCollisionGeometry::setRadius(double radius_in) -> void {
  SphereShape::setRadius(radius_in);
}
auto SphereCollisionGeometry::init() -> void {
  fcl::Transform3f trans(
      fcl::Matrix3f{{partPm()[0][0], partPm()[0][1], partPm()[0][2]},
                    {partPm()[1][0], partPm()[1][1], partPm()[1][2]},
                    {partPm()[2][0], partPm()[2][1], partPm()[2][2]}},
      fcl::Vec3f{partPm()[0][3], partPm()[1][3], partPm()[2][3]});
  resetCollisionObject(
      new fcl::CollisionObject(make_shared<fcl::Sphere>(radius()), trans));
}
SphereCollisionGeometry::SphereCollisionGeometry(double radius,
                                                 const double* prt_pm)
    : CollidableGeometry(prt_pm), SphereShape(radius) {}
SphereCollisionGeometry::~SphereCollisionGeometry() = default;

ARIS_REGISTRATION {
  aris::core::class_<SphereCollisionGeometry>("SphereCollisionGeometry")
      .inherit<CollidableGeometry>()
      .prop("radius", &SphereCollisionGeometry::setRadius,
            &SphereCollisionGeometry::radius);
}
}  // namespace sire::collision::geometry