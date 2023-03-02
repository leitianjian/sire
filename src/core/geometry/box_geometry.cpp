#include "sire/core/geometry/box_geometry.hpp"

#include <array>
#include <memory>
#include <string>
#include <string_view>

#include <aris/core/expression_calculator.hpp>
#include <aris/core/reflection.hpp>
#include <aris/dynamic/model.hpp>
#include <aris/server/control_server.hpp>

namespace sire::core::geometry {
SIRE_DEFINE_TO_JSON_HEAD(BoxGeometry) {
  j = json{{"shape_type", shapeType()},
           {"length", length()},
           {"width", width()},
           {"height", height()}};
}

SIRE_DEFINE_FROM_JSON_HEAD(BoxGeometry) {
  j.at("shape_type").get_to(shapeType());
  j.at("length").get_to(length());
  j.at("width").get_to(width());
  j.at("height").get_to(height());
}
BoxGeometry::BoxGeometry(double x, double y, double z, const double* prt_pm)
    : GeometryOnPart(prt_pm), BoxShape(x, y, z) {}

BoxGeometry::~BoxGeometry() = default;

ARIS_DEFINE_BIG_FOUR_CPP(BoxGeometry)

SIRE_DEFINE_JSON_OUTER_TWO(BoxGeometry)

ARIS_REGISTRATION {
  aris::core::class_<BoxGeometry>("BoxGeometry")
      .inherit<GeometryOnPart>()
      .inherit<BoxShape>();
}
}  // namespace sire::geometry