#include "sire/core/geometry/mesh_shape.hpp"

#include <aris/core/reflection.hpp>

#include "sire/core/sire_assert.hpp"

namespace sire::geometry {
auto MeshShape::setResourcePath(const std::string& resource_path) -> void {
  resource_path_ = resource_path;
}
auto MeshShape::getResourcePath() const -> std::string {
  return resource_path_;
}
auto MeshShape::resourcePath() -> std::string& { return resource_path_; }
auto MeshShape::resourcePath() const -> std::string { return resource_path_; }

MeshShape::MeshShape(std::string resource_path)
    : resource_path_(resource_path) {
  setShapeType(ShapeType::GEOM_MESH);
}

MeshShape::~MeshShape() = default;
ARIS_REGISTRATION {
  aris::core::class_<MeshShape>("MeshShape")
      .inherit<ShapeBase>()
      .prop("resource_path", &MeshShape::setResourcePath,
            &MeshShape::getResourcePath);
}
}  // namespace sire::geometry
