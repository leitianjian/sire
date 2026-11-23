#include "sire/core/geometry/box_geometry.hpp"

#include <array>
#include <memory>
#include <string>
#include <string_view>

#include <aris/core/expression_calculator.hpp>
#include <aris/core/reflection.hpp>
#include <aris/dynamic/model.hpp>
#include <aris/server/control_server.hpp>

namespace sire::geometry {
auto BoxGeometry::to_json(nlohmann::json& j) const -> void {
  GeometryAdapter::to_json(j);
  j["length"] = typedShape.length();
  j["width"] = typedShape.width();
  j["height"] = typedShape.height();
}

BoxGeometry::BoxGeometry(double x, double y, double z, int part_id,
                         bool is_dynamic, const double* prt_pm)
    : GeometryAdapter(part_id, is_dynamic, prt_pm, x, y, z) {}

BoxGeometry::~BoxGeometry() = default;

ARIS_DEFINE_BIG_FOUR_CPP(BoxGeometry)

ARIS_REGISTRATION {
  auto setSide = [](BoxGeometry* box, aris::core::Matrix mat) -> void {
    box->typedShape.setSide(mat.data());
  };
  auto getSide = [](BoxGeometry* box) -> aris::core::Matrix {
    return aris::core::Matrix(1, 3, box->typedShape.side());
  };
  aris::core::class_<BoxGeometry>("BoxGeometry")
      .inherit<GeometryOnPart>()
      .prop("side", &setSide, &getSide);
}
}  // namespace sire::geometry
