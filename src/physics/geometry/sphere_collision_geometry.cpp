#include "sire/physics/geometry/sphere_collision_geometry.hpp"

#include <array>
#include <memory>
#include <string>
#include <string_view>

#include <hpp/fcl/bvh/BVH_model.h>
#include <hpp/fcl/shape/geometric_shapes.h>

#include <aris/core/reflection.hpp>

namespace sire::collision::geometry {
SIRE_DEFINE_TO_JSON_HEAD(SphereCollisionGeometry) {
  GeometryOnPart::to_json(j);
  j["shape_type"] = shapeType();
  j["radius"] = radius();
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

// 借助类内部的from_json to_json定义，
// 使用宏定义完成用于json类型转换的from_json to_json的方法定义
SIRE_DEFINE_JSON_OUTER_TWO(SphereCollisionGeometry)

ARIS_REGISTRATION {
  aris::core::class_<SphereCollisionGeometry>("SphereCollisionGeometry")
      .inherit<CollidableGeometry>()
      .inherit<sire::geometry::SphereShape>();
}
}  // namespace sire::collision::geometry