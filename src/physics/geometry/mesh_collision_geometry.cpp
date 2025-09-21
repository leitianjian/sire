#include "sire/physics/geometry/mesh_collision_geometry.hpp"

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

#include "sire/core/geometry/shape_calculator.hpp"

namespace sire::physics::geometry {
SIRE_DEFINE_TO_JSON_HEAD(MeshCollisionGeometry) {
  GeometryOnPart::to_json(j);
  sire::geometry::ShapeToName cal;
  meshShape.Reify(&cal);
  j["shape_type"] = cal.string();
  j["resourcePath"] = meshShape.resourcePath();
  j["scale_x"] = *scale();
  j["scale_y"] = *(scale() + 1);
  j["scale_z"] = *(scale() + 2);
}
struct MeshCollisionGeometry::Imp {
  aris::dynamic::double3 scale_{1, 1, 1};
};
MeshCollisionGeometry::MeshCollisionGeometry(const string& resource_path,
                                             int part_id, bool is_dynamic,
                                             const double* prt_pm,
                                             const std::string& material,
                                             const std::string& propStr)
    : CollidableGeometry(prt_pm, part_id, is_dynamic, material, propStr),
      meshShape(resource_path),
      imp_(new Imp) {}
MeshCollisionGeometry::~MeshCollisionGeometry() = default;
SIRE_DEFINE_MOVE_CTOR_CPP(MeshCollisionGeometry);

auto MeshCollisionGeometry::scale() const -> const double* {
  return imp_->scale_;
}
auto MeshCollisionGeometry::setScale(const double* scale) -> void {
  if (scale) std::copy_n(scale, 3, imp_->scale_);
}
auto MeshCollisionGeometry::init() -> void {
  shared_ptr<BVHModel<OBBRSS>> bvh_model = make_shared<BVHModel<OBBRSS>>();
  loadPolyhedronFromResource(meshShape.getResourcePath(), Vec3s(imp_->scale_),
                             bvh_model);
  Transform3s trans(
      (Matrix3s() << partPm()[0][0], partPm()[0][1], partPm()[0][2],
       partPm()[1][0], partPm()[1][1], partPm()[1][2], partPm()[2][0],
       partPm()[2][1], partPm()[2][2])
          .finished(),
      (Vec3s() << partPm()[0][3], partPm()[1][3], partPm()[2][3]).finished());
  resetCollisionObject(new CollisionObject(bvh_model, trans));
}
ARIS_REGISTRATION {
  auto setResourcePath = [](MeshCollisionGeometry* geo,
                            const std::string& path) -> void {
    geo->meshShape.setResourcePath(path);
  };
  auto getResourcePath = [](MeshCollisionGeometry* geo) -> const std::string& {
    return geo->meshShape.resourcePath();
  };
  auto getMeshGeometryScale = [](MeshCollisionGeometry* geo) {
    return aris::core::Matrix(1, 3, geo->scale());
  };
  auto setMeshGeometryScale = [](MeshCollisionGeometry* geo,
                                 aris::core::Matrix scale) {
    geo->setScale(scale.data());
  };
  aris::core::class_<MeshCollisionGeometry>("MeshCollisionGeometry")
      .inherit<CollidableGeometry>()
      .prop("resource_path", &setResourcePath, &getResourcePath)
      .prop("scale", &setMeshGeometryScale, &getMeshGeometryScale);
}
}  // namespace sire::physics::geometry