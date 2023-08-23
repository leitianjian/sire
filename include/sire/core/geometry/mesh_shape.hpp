#ifndef SIRE_MESH_SHAPE_HPP_
#define SIRE_MESH_SHAPE_HPP_

#include <sire_lib_export.h>

#include "sire/core/geometry/shape_base.hpp"

namespace sire::geometry {
class SIRE_API MeshShape : public ShapeBase {
 private:
  std::string resource_path_;

 public:
  auto setResourcePath(const std::string& resource_path) -> void;
  auto getResourcePath() const -> std::string;
  auto resourcePath() const -> std::string;
  auto resourcePath() -> std::string&;

  explicit MeshShape(std::string resource_path = "");
  virtual ~MeshShape();
};
}  // namespace sire::geometry

#endif