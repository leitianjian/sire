#include "sire/core/geometry/mesh_geometry.hpp"

#include <string>

#include <aris/core/reflection.hpp>
#include <aris/dynamic/model.hpp>
#include <aris/server/control_server.hpp>

#include "sire/core/geometry/shape_calculator.hpp"

namespace sire::geometry {
SIRE_DEFINE_TO_JSON_HEAD(MeshGeometry) {
  GeometryOnPart::to_json(j);
  ShapeToName cal;
  meshShape.Reify(&cal);
  j["shape_type"] = cal.string();
  j["resource_path"] = meshShape.resourcePath();
}

MeshGeometry::MeshGeometry(string resource_path, int part_id, bool is_dynamic,
                           const double* prt_pm)
    : GeometryOnPart(prt_pm, part_id, is_dynamic), meshShape(resource_path) {}

MeshGeometry::~MeshGeometry() = default;

ARIS_DEFINE_BIG_FOUR_CPP(MeshGeometry)

// 借助类内部的from_json to_json定义，
// 使用宏定义完成用于json类型转换的from_json to_json的方法定义
SIRE_DEFINE_JSON_OUTER_TWO(MeshGeometry)

ARIS_REGISTRATION {
  auto setResourcePath = [](MeshGeometry* geo,
                            const std::string& path) -> void {
    geo->meshShape.setResourcePath(path);
  };
  auto getResourcePath = [](MeshGeometry* geo) -> const std::string& {
    return geo->meshShape.resourcePath();
  };
  // auto setScale = [](MeshGeometry* geo, double scale) -> void {
  //   geo->meshShape.setScale(scale);
  // };
  // auto getScale = [](MeshGeometry* geo) -> double {
  //   return geo->meshShape.getScale();
  // };
  aris::core::class_<MeshGeometry>("MeshGeometry")
      .inherit<GeometryOnPart>()
      .prop("resource_path", &setResourcePath, &getResourcePath);
  // .prop("scale", &setScale, &getScale);
}
}  // namespace sire::geometry