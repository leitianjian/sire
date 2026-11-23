#include "sire/core/geometry/mesh_geometry.hpp"

#include <aris/core/reflection.hpp>

namespace sire::geometry {
auto MeshGeometry::to_json(nlohmann::json& j) const -> void {
  GeometryAdapter::to_json(j);
  // j["scale"] = typedShape.getScale();
  j["resource_path"] = typedShape.resourcePath();
}

MeshGeometry::MeshGeometry(std::string file_path,
                           const std::array<double, 3>& scale, int part_id,
                           bool is_dynamic, const double* prt_pm)
    : GeometryAdapter(part_id, is_dynamic, prt_pm, file_path, scale) {}

MeshGeometry::~MeshGeometry() = default;

ARIS_DEFINE_BIG_FOUR_CPP(MeshGeometry)

ARIS_REGISTRATION {
  auto getScale = [](MeshGeometry* geo) {
    return aris::core::Matrix(1, 3, geo->typedShape.scale());
  };
  auto setScale = [](MeshGeometry* geo, aris::core::Matrix scale) {
    geo->typedShape.setScale(scale.data());
  };
  auto setResourcePath = [](MeshGeometry* geo,
                            const std::string& path) -> void {
    geo->typedShape.setResourcePath(path);
  };
  auto getResourcePath = [](MeshGeometry* geo) -> const std::string& {
    return geo->typedShape.resourcePath();
  };
  aris::core::class_<MeshGeometry>("MeshGeometry")
      .inherit<GeometryOnPart>()
      .prop("scale", &setScale, &getScale)
      .prop("resource_path", &setResourcePath, &getResourcePath);
}
}  // namespace sire::geometry
