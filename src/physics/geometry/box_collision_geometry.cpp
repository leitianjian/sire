#include "sire/physics/geometry/box_collision_geometry.hpp"

#include <array>
#include <memory>
#include <string>
#include <string_view>

#include <hpp/fcl/bvh/BVH_model.h>
#include <hpp/fcl/shape/geometric_shapes.h>

#include <aris/core/reflection.hpp>

namespace sire::physics::geometry {
SIRE_DEFINE_TO_JSON_HEAD(BoxCollisionGeometry) {
  j = json{{"shape_type", shapeType()},
           {"length", length()},
           {"width", width()},
           {"height", height()}};
}

SIRE_DEFINE_FROM_JSON_HEAD(BoxCollisionGeometry) {
  j.at("shape_type").get_to(shapeType());
  j.at("length").get_to(length());
  j.at("width").get_to(width());
  j.at("height").get_to(height());
}
auto BoxCollisionGeometry::init() -> void {
  fcl::Transform3f trans(
      fcl::Matrix3f{{partPm()[0][0], partPm()[0][1], partPm()[0][2]},
                    {partPm()[1][0], partPm()[1][1], partPm()[1][2]},
                    {partPm()[2][0], partPm()[2][1], partPm()[2][2]}},
      fcl::Vec3f{partPm()[0][3], partPm()[1][3], partPm()[2][3]});
  std::array<double, 3> temp = side();
  resetCollisionObject(new fcl::CollisionObject(
      make_shared<fcl::Box>(temp[0], temp[1], temp[2]), trans));
}
BoxCollisionGeometry::BoxCollisionGeometry(double x, double y, double z,
                                           const double* prt_pm)
    : CollidableGeometry(prt_pm), BoxShape(x, y, z) {}
BoxCollisionGeometry::~BoxCollisionGeometry() = default;

SIRE_DEFINE_JSON_OUTER_TWO(BoxCollisionGeometry)

ARIS_REGISTRATION {
  aris::core::class_<BoxCollisionGeometry>("BoxCollisionGeometry")
      .inherit<CollidableGeometry>()
      .inherit<sire::core::geometry::BoxShape>();
}
}  // namespace sire::physics::geometry