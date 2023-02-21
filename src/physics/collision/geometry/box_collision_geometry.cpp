#include "sire/physics/collision/geometry/box_collision_geometry.hpp"

#include <array>
#include <memory>
#include <string>
#include <string_view>

#include <hpp/fcl/bvh/BVH_model.h>
#include <hpp/fcl/shape/geometric_shapes.h>

#include <aris/core/reflection.hpp>

namespace sire::collision::geometry {
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

ARIS_REGISTRATION {
  auto setHalfSize = [](BoxCollisionGeometry* box,
                        aris::core::Matrix mat) -> void {
    box->setHalfSide(mat.data());
  };
  auto getHalfSize = [](BoxCollisionGeometry* box) -> aris::core::Matrix {
    return aris::core::Matrix(1, 3, box->halfSidePtr());
  };
  aris::core::class_<BoxCollisionGeometry>("BoxCollisionGeometry")
      .inherit<CollidableGeometry>()
      .prop("half_side", &setHalfSize, &getHalfSize);
}
}  // namespace sire::collision::geometry