#include "sire/physics/geometry/box_collision_geometry.hpp"

#include <array>
#include <memory>
#include <string>
#include <string_view>

#include <coal/BVH/BVH_model.h>
#include <coal/shape/geometric_shapes.h>

#include <aris/core/reflection.hpp>

namespace sire::physics::geometry {
auto BoxCollisionGeometry::to_json(nlohmann::json& j) const -> void {
  CollisionAdapter::to_json(j);
  j["length"] = typedShape.length();
  j["width"] = typedShape.width();
  j["height"] = typedShape.height();
}

auto BoxCollisionGeometry::init() -> void {
  resetCollisionObject(new coal::CollisionObject(
      std::make_shared<coal::Box>(typedShape.side()[0], typedShape.side()[1],
                                  typedShape.side()[2]),
      getCoalTransform()));
}

BoxCollisionGeometry::BoxCollisionGeometry(double x, double y, double z,
                                           int part_id, bool is_dynamic,
                                           const double* prt_pm,
                                           const std::string& material,
                                           const std::string& propStr)
    : CollisionAdapter(part_id, is_dynamic, prt_pm, material, propStr, x, y,
                       z) {}

BoxCollisionGeometry::~BoxCollisionGeometry() = default;

SIRE_DEFINE_MOVE_CTOR_CPP(BoxCollisionGeometry)

ARIS_REGISTRATION {
  auto setSide = [](BoxCollisionGeometry* box, aris::core::Matrix mat) -> void {
    box->typedShape.setSide(mat.data());
  };
  auto getSide = [](BoxCollisionGeometry* box) -> aris::core::Matrix {
    return aris::core::Matrix(1, 3, box->typedShape.side());
  };
  aris::core::class_<BoxCollisionGeometry>("BoxCollisionGeometry")
      .inherit<CollidableGeometry>()
      .prop("side", &setSide, &getSide);
}
}  // namespace sire::physics::geometry
