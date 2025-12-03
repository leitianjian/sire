#ifndef SIRE_MESH_SHAPE_HPP_
#define SIRE_MESH_SHAPE_HPP_

#include "aris/core/object.hpp"

#include "sire/core/geometry/shape_base.hpp"

namespace sire::geometry {
class MeshShape final : public ShapeBase {
 private:
  std::string resource_path_;
  double scale_{};

 public:
  auto setResourcePath(const std::string& resource_path) -> void;
  auto getResourcePath() const -> const std::string& { return resourcePath(); };
  auto getScale() -> double { return scale_; };
  auto setScale(double scale) -> void;
  auto resourcePath() const -> const std::string& { return resource_path_; }
  auto resourcePath() -> std::string& {
    return const_cast<std::string&>(
        static_cast<const MeshShape*>(this)->resourcePath());
  };

  explicit MeshShape(const std::string& resource_path, double scale = 1.0);
  virtual ~MeshShape();
  ARIS_DECLARE_BIG_FOUR(MeshShape)
};
}  // namespace sire::geometry

#endif