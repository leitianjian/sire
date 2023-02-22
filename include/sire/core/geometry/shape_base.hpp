#ifndef SIRE_SHAPE_BASE_HPP_
#define SIRE_SHAPE_BASE_HPP_

#include <sire_lib_export.h>

#include "sire/ext/json.hpp"

namespace sire::geometry {
enum ShapeType {
  GEOM_GENERAL,
  GEOM_BOX,
  GEOM_SPHERE,
};

NLOHMANN_JSON_SERIALIZE_ENUM(ShapeType, {{GEOM_GENERAL, "general"},
                                         {GEOM_BOX, "box"},
                                         {GEOM_SPHERE, "sphere"}})

class SIRE_API ShapeBase {
 private:
  ShapeType type_;

 public:
  auto shapeType() -> ShapeType& { return type_; };
  auto shapeType() const -> const ShapeType { return type_; };
  auto setShapeType(ShapeType type_in) -> void { type_ = type_in; };
  explicit ShapeBase(ShapeType type = ShapeType::GEOM_GENERAL) : type_(type){};
};
}  // namespace sire::geometry

#endif