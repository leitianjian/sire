#ifndef SIRE_MESH_SHAPE_HPP_
#define SIRE_MESH_SHAPE_HPP_

#include "sire/core/geometry/shape_base.hpp"

#include "aris/core/object.hpp"

namespace sire::geometry {
class MeshShape final : public ShapeBase {
 private:
  std::string resource_path_;
  std::array<double, 3> scale_{1, 1, 1};

 public:
  auto scale() -> double* {
    return const_cast<double*>(static_cast<const MeshShape*>(this)->scale());
  };
  auto scale() const -> const double* { return scale_.data(); };
  auto setScale(double* scale) -> void;
  auto setResourcePath(const std::string& resource_path) -> void;
  auto resourcePath() const -> const std::string& { return resource_path_; }
  auto resourcePath() -> std::string& {
    return const_cast<std::string&>(
        static_cast<const MeshShape*>(this)->resourcePath());
  };

  explicit MeshShape(const std::string& resource_path,
                     const std::array<double, 3>& scale = {1.0});
  virtual ~MeshShape();
  ARIS_DECLARE_BIG_FOUR(MeshShape)
};
}  // namespace sire::geometry

#endif