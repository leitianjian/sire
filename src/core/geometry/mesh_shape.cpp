#include "sire/core/geometry/mesh_shape.hpp"

#include <cmath>

#include <aris/core/reflection.hpp>

#include "sire/core/sire_assert.hpp"

namespace sire::geometry {
auto MeshShape::setResourcePath(const std::string& resource_path) -> void {
  resource_path_ = resource_path;
}
auto MeshShape::setScale(double scale) -> void {
  if (std::abs(scale) < 1e-8) {
    throw std::logic_error("Mesh |scale| cannot be < 1e-8.");
  }
  scale_ = scale;
}

MeshShape::MeshShape(const std::string& resource_path, double scale)
    : ShapeBase(ShapeTag<MeshShape>()),
      resource_path_(resource_path),
      scale_(scale) {
  // setShapeType(ShapeType::GEOM_MESH);
}
ARIS_DEFINE_BIG_FOUR_CPP(MeshShape)

MeshShape::~MeshShape() = default;
ARIS_REGISTRATION {
  aris::core::class_<MeshShape>("MeshShape")
      .prop("resource_path", &MeshShape::setResourcePath,
            &MeshShape::getResourcePath)
      .prop("scale", &MeshShape::setScale, &MeshShape::getScale);
}
}  // namespace sire::geometry
