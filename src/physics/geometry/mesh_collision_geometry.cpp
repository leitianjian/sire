#include "sire/physics/geometry/mesh_collision_geometry.hpp"

#include <coal/BVH/BVH_model.h>
#include <coal/mesh_loader/assimp.h>
#include <coal/mesh_loader/loader.h>
#include <coal/shape/geometric_shapes.h>

#include <aris/core/reflection.hpp>

namespace sire::physics::geometry {

auto MeshCollisionGeometry::to_json(nlohmann::json& j) const -> void {
  CollisionAdapter::to_json(j);
  j["scale_x"] = *typedShape.scale();
  j["scale_y"] = *(typedShape.scale() + 1);
  j["scale_z"] = *(typedShape.scale() + 2);
  j["resource_path"] = typedShape.resourcePath();
}
auto MeshCollisionGeometry::init() -> void {
  coal::MeshLoader loader;
  auto coal_scale = coal::Vec3s(*typedShape.scale(), *(typedShape.scale() + 1),
                                *(typedShape.scale() + 2));
  try {
    resetCollisionObject(new coal::CollisionObject(
        loader.load(typedShape.resourcePath(), coal_scale),
        getCoalTransform()));
  } catch (const std::exception& e) {
    throw std::runtime_error("Failed to load mesh: " +
                             typedShape.resourcePath());
  }
}

MeshCollisionGeometry::MeshCollisionGeometry(std::string file_path,
                                             const std::array<double, 3>& scale,
                                             int part_id, bool is_dynamic,
                                             const double* prt_pm,
                                             bool visible,
                                             const std::string& material,
                                             const std::string& propStr)
    : CollisionAdapter(part_id, is_dynamic, prt_pm, visible, material, propStr,
                       file_path, scale) {}

MeshCollisionGeometry::~MeshCollisionGeometry() = default;

SIRE_DEFINE_MOVE_CTOR_CPP(MeshCollisionGeometry)

ARIS_REGISTRATION {
  auto getScale = [](MeshCollisionGeometry* geo) {
    return aris::core::Matrix(1, 3, geo->typedShape.scale());
  };
  auto setScale = [](MeshCollisionGeometry* geo, aris::core::Matrix scale) {
    geo->typedShape.setScale(scale.data());
  };
  auto setResourcePath = [](MeshCollisionGeometry* geo,
                            const std::string& path) -> void {
    geo->typedShape.setResourcePath(path);
  };
  auto getResourcePath = [](MeshCollisionGeometry* geo) -> const std::string& {
    return geo->typedShape.resourcePath();
  };
  aris::core::class_<MeshCollisionGeometry>("MeshCollisionGeometry")
      .inherit<CollidableGeometry>()
      .prop("scale", &setScale, &getScale)
      .prop("resource_path", &setResourcePath, &getResourcePath);
}
}  // namespace sire::physics::geometry
