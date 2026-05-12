#ifndef SIRE_COLLISION_ADAPTER_HPP_
#define SIRE_COLLISION_ADAPTER_HPP_

#include <coal/BVH/BVH_model.h>
#include <coal/shape/geometric_shapes.h>

#include "sire/physics/geometry/collidable_geometry.hpp"
#include "sire/core/geometry/shape_calculator.hpp"
#include "sire/core/sire_decl_def_macro.hpp"

namespace sire::physics::geometry {

template <typename Derived, typename ShapeType>
class CollisionAdapter : public CollidableGeometry {
 public:
  ShapeType typedShape;

  template <typename... Args>
  explicit CollisionAdapter(int part_id = 0, bool is_dynamic = false,
                            const double* prt_pm = nullptr,
                            const std::string& material = "m1",
                            const std::string& propStr = "{}",
                            Args&&... args)
      : CollidableGeometry(prt_pm, part_id, is_dynamic, material, propStr),
        typedShape(std::forward<Args>(args)...) {}

  virtual ~CollisionAdapter() = default;

  CollisionAdapter(CollisionAdapter&& other) = default;
  CollisionAdapter& operator=(CollisionAdapter&& other) = default;

  auto to_json(nlohmann::json& j) const -> void override {
    sire::geometry::GeometryOnPart::to_json(j);
    sire::geometry::ShapeToName cal;
    typedShape.Reify(&cal);
    j["shape_type"] = cal.string();
  }

  friend auto to_json(nlohmann::json& j, const Derived& o) -> void {
    o.to_json(j);
  }

 protected:
  auto getCoalTransform() const -> coal::Transform3s {
    return coal::Transform3s(
        (coal::Matrix3s() << pm()[0][0], pm()[0][1], pm()[0][2],
         pm()[1][0], pm()[1][1], pm()[1][2], pm()[2][0],
         pm()[2][1], pm()[2][2])
            .finished(),
        (coal::Vec3s() << pm()[0][3], pm()[1][3], pm()[2][3])
            .finished());
  }
};

}  // namespace sire::physics::geometry
#endif
