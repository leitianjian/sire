#ifndef SIRE_GEOMETRY_ON_PART_HPP_
#define SIRE_GEOMETRY_ON_PART_HPP_

#include <atomic>

#include <sire_lib_export.h>

#include <aris/core/object.hpp>
#include <aris/dynamic/model_basic.hpp>
#include <aris/dynamic/model_coordinate.hpp>

#include "sire/core/geometry/geometry_base.hpp"

namespace sire::geometry {
class SIRE_API GeometryOnPart : public GeometryBase {
 public:
  auto partPm() const -> const aris::dynamic::double4x4&;
  auto isDynamic() -> bool;
  auto setDynamic(bool is_dynamic) -> void;
  auto partId() -> int;
  auto part() -> aris::dynamic::Part*;
  auto setPart(aris::dynamic::Part* part, int part_id) -> void;
  explicit GeometryOnPart(const double* pm_in = default_pm);
  virtual ~GeometryOnPart();
  ARIS_DECLARE_BIG_FOUR(GeometryOnPart)
  SIRE_DECLARE_JSON_INTER_VIRTUAL_INTERFACE_TWO

 private:
  struct Imp;
  aris::core::ImpPtr<Imp> imp_;
};

}  // namespace sire::geometry
#endif