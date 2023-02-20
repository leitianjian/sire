#include "sire/physics/collision/geometry/sphere_collision_geometry.hpp"

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
struct SphereGeometry::Imp {
  double radius_{0};
};
auto SphereGeometry::radius() -> double { return imp_->radius_; }
auto SphereGeometry::setRadius(double radius) -> void {
  if (radius < 0) {
    throw std::logic_error(string("Sphere radius should be >= 0 (was ") +
                           to_string(radius) + string(")."));
  }
  imp_->radius_ = radius;
}
auto SphereGeometry::init() -> void {
  if (imp_->radius_ < 0) {
    throw std::logic_error(string("Sphere radius should be >= 0 (was ") +
                           to_string(imp_->radius_) + string(")."));
  }
  if (imp_->radius_ != 0) {
    fcl::Transform3f trans(
        fcl::Matrix3f{{prtPm()[0][0], prtPm()[0][1], prtPm()[0][2]},
                      {prtPm()[1][0], prtPm()[1][1], prtPm()[1][2]},
                      {prtPm()[2][0], prtPm()[2][1], prtPm()[2][2]}},
        fcl::Vec3f{prtPm()[0][3], prtPm()[1][3], prtPm()[2][3]});
    resetCollisionObject(new fcl::CollisionObject(
        make_shared<fcl::Sphere>(imp_->radius_), trans));
  }
}
SphereGeometry::SphereGeometry(double radius, const double* prt_pm)
    : CollisionGeometry(prt_pm), imp_(new Imp) {
  if (radius < 0) {
    throw std::logic_error(string("Sphere radius should be >= 0 (was ") +
                           to_string(radius) + string(")."));
  } else {
    imp_->radius_ = radius;
  }
}
SphereGeometry::~SphereGeometry() = default;

ARIS_REGISTRATION {
  aris::core::class_<SphereGeometry>("SphereGeometry")
      .inherit<CollisionGeometry>()
      .prop("radius", &SphereGeometry::setRadius, &SphereGeometry::radius);
}
}  // namespace sire::collision::geometry