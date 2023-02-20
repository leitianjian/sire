#include "sire/physics/collision/geometry/mesh_collision_geometry.hpp"

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
struct MeshGeometry::Imp {
  aris::dynamic::double3 scale_{1, 1, 1};
  string resource_path_;
};
auto MeshGeometry::resourcePath() const -> const std::string& {
  return imp_->resource_path_;
}
auto MeshGeometry::setResourcePath(std::string_view resource_path) -> void {
  if (!resource_path.empty()) {
    imp_->resource_path_ = resource_path;
  }
}
auto MeshGeometry::scale() const -> const double* { return imp_->scale_; }
auto MeshGeometry::setScale(const double* scale) -> void {
  if (scale) std::copy_n(scale, 3, imp_->scale_);
}
auto MeshGeometry::init() -> void {
  shared_ptr<fcl::BVHModel<fcl::OBBRSS>> bvh_model =
      make_shared<fcl::BVHModel<fcl::OBBRSS>>();
  fcl::loadPolyhedronFromResource(imp_->resource_path_,
                                  fcl::Vec3f(imp_->scale_), bvh_model);
  fcl::Transform3f trans(
      fcl::Matrix3f{{prtPm()[0][0], prtPm()[0][1], prtPm()[0][2]},
                    {prtPm()[1][0], prtPm()[1][1], prtPm()[1][2]},
                    {prtPm()[2][0], prtPm()[2][1], prtPm()[2][2]}},
      fcl::Vec3f{prtPm()[0][3], prtPm()[1][3], prtPm()[2][3]});
  resetCollisionObject(new fcl::CollisionObject(bvh_model, trans));
}
MeshGeometry::MeshGeometry(const string& resource_path, const double* prt_pm)
    : CollisionGeometry(prt_pm), imp_(new Imp) {
  if (!resource_path.empty()) {
    imp_->resource_path_ = resource_path;
  }
}
MeshGeometry::~MeshGeometry() = default;

ARIS_REGISTRATION {
  auto getMeshGeometryScale = [](MeshGeometry* geo) {
    return aris::core::Matrix(1, 3, geo->scale());
  };
  auto setMeshGeometryScale = [](MeshGeometry* geo, aris::core::Matrix scale) {
    geo->setScale(scale.data());
  };
  aris::core::class_<MeshGeometry>("MeshGeometry")
      .inherit<CollisionGeometry>()
      .prop("resource_path", &MeshGeometry::setResourcePath,
            &MeshGeometry::resourcePath)
      .prop("scale", &setMeshGeometryScale, &getMeshGeometryScale);
}
}  // namespace sire::collision::geometry