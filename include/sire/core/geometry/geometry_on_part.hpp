#ifndef SIRE_GEOMETRY_ON_PART_HPP_
#define SIRE_GEOMETRY_ON_PART_HPP_

#include <atomic>

#include <aris/core/object.hpp>
#include <aris/dynamic/model_basic.hpp>
#include <aris/dynamic/model_coordinate.hpp>

#include "sire/core/geometry/geometry_base.hpp"

namespace sire::geometry {
class GeometryOnPart : public GeometryBase {
 public:
  auto partPm() const -> const aris::dynamic::double4x4&;
  auto isDynamic() -> bool;
  auto setDynamic(bool is_dynamic) -> void;
  auto partId() -> int;
  auto part() -> aris::dynamic::Part*;
  auto setPart(aris::dynamic::Part* part, int part_id) -> void;
  explicit GeometryOnPart(const double* pm_in = nullptr);
  virtual ~GeometryOnPart();

 private:
  struct Imp;
  aris::core::ImpPtr<Imp> imp_;
};

}  // namespace sire::geometry
#endif