#include "sire/core/geometry/mesh_shape.hpp"

#include <cmath>

#include <aris/core/expression_calculator.hpp>
#include <aris/core/reflection.hpp>

#include "sire/core/sire_assert.hpp"

namespace sire::geometry {
auto MeshShape::setResourcePath(const std::string& resource_path) -> void {
  resource_path_ = resource_path;
}
auto MeshShape::setScale(double* scale) -> void {
  std::copy_n(scale, 3, scale_.begin());
}
MeshShape::MeshShape(const std::string& resource_path,
                     const std::array<double, 3>& scale)
    : ShapeBase(ShapeTag<MeshShape>()),
      resource_path_(resource_path),
      scale_(scale) {
  // setShapeType(ShapeType::GEOM_MESH);
}
ARIS_DEFINE_BIG_FOUR_CPP(MeshShape)

MeshShape::~MeshShape() = default;
ARIS_REGISTRATION {
  auto getScale = [](MeshShape* shape) {
    return aris::core::Matrix(1, 3, shape->scale());
  };
  auto setScale = [](MeshShape* shape, aris::core::Matrix scale) {
    shape->setScale(scale.data());
  };
  auto setResourcePath = [](MeshShape* shape, const std::string& path) -> void {
    shape->setResourcePath(path);
  };
  auto getResourcePath = [](MeshShape* shape) -> const std::string& {
    return shape->resourcePath();
  };
  aris::core::class_<MeshShape>("MeshShape")
      .prop("resource_path", &setResourcePath, &getResourcePath)
      .prop("scale", &setScale, &getScale);
}
}  // namespace sire::geometry
