#include "sire/core/geometry/box_geometry.hpp"

#include <array>
#include <memory>
#include <string>
#include <string_view>

#include <aris/core/expression_calculator.hpp>
#include <aris/core/reflection.hpp>
#include <aris/dynamic/model.hpp>
#include <aris/server/control_server.hpp>

#include "sire/core/geometry/shape_calculator.hpp"

namespace sire::geometry {
SIRE_DEFINE_TO_JSON_HEAD(BoxGeometry) {
  GeometryOnPart::to_json(j);
  ShapeToName cal;
  boxShape.Reify(&cal);
  j["shape_type"] = cal.string();
  j["length"] = boxShape.length();
  j["width"] = boxShape.width();
  j["height"] = boxShape.height();
}

BoxGeometry::BoxGeometry(double x, double y, double z, int part_id,
                         bool is_dynamic, const double* prt_pm)
    : GeometryOnPart(prt_pm, part_id, is_dynamic), boxShape(x, y, z) {}

BoxGeometry::~BoxGeometry() = default;

ARIS_DEFINE_BIG_FOUR_CPP(BoxGeometry)

SIRE_DEFINE_JSON_OUTER_TWO(BoxGeometry)

ARIS_REGISTRATION {
  auto setSide = [](BoxGeometry* box, aris::core::Matrix mat) -> void {
    box->boxShape.setSide(mat.data());
  };
  auto getSide = [](BoxGeometry* box) -> aris::core::Matrix {
    return aris::core::Matrix(1, 3, box->boxShape.side());
  };
  aris::core::class_<BoxGeometry>("BoxGeometry")
      .inherit<GeometryOnPart>()
      .prop("side", &setSide, &getSide);
}
}  // namespace sire::geometry