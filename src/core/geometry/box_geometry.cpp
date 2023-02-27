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
SIRE_DEFINE_TO_JSON_HEAD(BoxGeometry) {
  GeometryOnPart::to_json(j);
  j["shape_type"] = shapeType();
  j["length"] = length();
  j["width"] = width();
  j["height"] = height();
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