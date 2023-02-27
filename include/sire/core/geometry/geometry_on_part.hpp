#ifndef SIRE_GEOMETRY_ON_PART_HPP_
#define SIRE_GEOMETRY_ON_PART_HPP_

#include <atomic>

#include <sire_lib_export.h>

#include <aris/core/object.hpp>
#include <aris/dynamic/model_basic.hpp>
#include <aris/dynamic/model_coordinate.hpp>

#include "sire/core/geometry/geometry_base.hpp"
#include "sire/ext/json.hpp"

namespace sire::geometry {
// All geometry in aris should inherit GeomtryOnPart, because aris
// using Part as simulation model basic component.
//
// The Moving geometry can mount upon moving Part, which is non-zero
// index Part in the PartPool array. Static Geometry should
// mount upon ground Part, which is the first Part element in GeometryPool
// array by convention.
class SIRE_API GeometryOnPart : public GeometryBase {
 public:
  auto partPm() const -> const aris::dynamic::double4x4&;
  auto isDynamic() const -> bool;
  auto setDynamic(bool is_dynamic) -> void;
  auto relativeToPart() const -> bool;
  auto relativeToPart() -> bool&;
  auto partId() const -> int;
  auto partId() -> int&;
  auto setPartId(int part_id) -> void;
  explicit GeometryOnPart(const double* pm_in = default_pm);
  virtual ~GeometryOnPart();
  ARIS_DECLARE_BIG_FOUR(GeometryOnPart)
  SIRE_DECLARE_JSON_INTER_VIRTUAL_TWO

 private:
  struct Imp;
  aris::core::ImpPtr<Imp> imp_;
};

}  // namespace sire::geometry
#endif