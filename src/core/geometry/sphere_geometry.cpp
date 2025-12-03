#include "sire/core/geometry/sphere_geometry.hpp"

#include <array>
#include <memory>
#include <string>
#include <string_view>

#include <aris/core/reflection.hpp>
#include <aris/dynamic/model.hpp>
#include <aris/server/control_server.hpp>

#include "sire/core/geometry/shape_calculator.hpp"

namespace sire::geometry {
SIRE_DEFINE_TO_JSON_HEAD(SphereGeometry) {
  GeometryOnPart::to_json(j);
  ShapeToName cal;
  sphereShape.Reify(&cal);
  j["shape_type"] = cal.string();
  j["radius"] = sphereShape.radius();
}

SphereGeometry::SphereGeometry(double radius, int part_id, bool is_dynamic,
                               const double* prt_pm)
    : GeometryOnPart(prt_pm, part_id, is_dynamic), sphereShape(radius) {}

SphereGeometry::~SphereGeometry() = default;

ARIS_DEFINE_BIG_FOUR_CPP(SphereGeometry)

// 借助类内部的from_json to_json定义，
// 使用宏定义完成用于json类型转换的from_json to_json的方法定义
SIRE_DEFINE_JSON_OUTER_TWO(SphereGeometry)

ARIS_REGISTRATION {
  auto setRadius = [](SphereGeometry* geo, double radius) -> void {
    geo->sphereShape.setRadius(radius);
  };
  auto getRadius = [](SphereGeometry* geo) -> double {
    return geo->sphereShape.getRadius();
  };
  aris::core::class_<SphereGeometry>("SphereGeometry")
      .inherit<GeometryOnPart>()
      .prop("radius", &setRadius, &getRadius);
}
}  // namespace sire::geometry