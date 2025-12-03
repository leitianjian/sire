#ifndef SIRE_GEOMETRY_ON_PART_HPP_
#define SIRE_GEOMETRY_ON_PART_HPP_

#include <atomic>

#include <aris/core/object.hpp>
#include <aris/dynamic/model_basic.hpp>
#include <aris/dynamic/model_coordinate.hpp>

#include "sire/core/constants.hpp"
#include "sire/core/geometry/geometry_base.hpp"
#include "sire/ext/json.hpp"

namespace sire::geometry {
// All geometry in aris should inherit GeomtryOnPart, because aris
// using p  art as simulation model basic component.
//
// The moving geometry can mount upon moving part, which is non-zero
// index part in the PartPool array. Static geometry should
// mount upon ground part, which is the first part element in GeometryPool
// array by convention.
class GeometryOnPart : public GeometryBase {
 public:
  auto partPm() const -> const aris::dynamic::double4x4&;
  auto isDynamic() const -> bool;
  auto setDynamic(bool is_dynamic) -> void;
  auto relativeToPart() const -> bool;
  auto relativeToPart() -> bool&;
  auto partId() const -> sire::PartId;
  auto partId() -> sire::PartId&;
  auto setPartId(sire::PartId part_id) -> void;
  explicit GeometryOnPart(const double* pm_in = nullptr,
                          sire::PartId part_id = 0, bool is_dynamic = false);
  virtual ~GeometryOnPart();
  ARIS_DECLARE_BIG_FOUR(GeometryOnPart)
  SIRE_DECLARE_JSON_INTER_VIRTUAL_TWO

 private:
  struct Imp;
  aris::core::ImpPtr<Imp> imp_;
};

}  // namespace sire::geometry
#endif