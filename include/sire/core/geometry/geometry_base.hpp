#ifndef SIRE_GEOMETRY_BASE_HPP_
#define SIRE_GEOMETRY_BASE_HPP_

#include <atomic>

#include <sire_lib_export.h>

#include <aris/core/object.hpp>
#include <aris/dynamic/model_basic.hpp>
#include <aris/dynamic/model_coordinate.hpp>

#include "sire/core/sire_decl_def_macro.hpp"
#include "sire/ext/json.hpp"

namespace sire::core::geometry {
using GeometryId = int64_t;
static std::atomic<GeometryId> geometry_id_flag = 0;
static auto generate_new_id() -> GeometryId { return ++geometry_id_flag; }
static const double default_pm[16] = {1, 0, 0, 0, 0, 1, 0, 0,
                                      0, 0, 1, 0, 0, 0, 0, 1};

class SIRE_API GeometryBase : public aris::dynamic::Geometry {
 public:
  auto geometryId() -> GeometryId;
  auto setGeometryId(GeometryId id) -> void;
  auto pm() const -> const aris::dynamic::double4x4&;
  auto setPm(const double* pm_in) -> void;
  explicit GeometryBase(const double* pm_in = default_pm);
  virtual ~GeometryBase();
  ARIS_DECLARE_BIG_FOUR(GeometryBase)
  SIRE_DECLARE_JSON_INTER_VIRTUAL_INTERFACE_TWO
 private:
  struct Imp;
  aris::core::ImpPtr<Imp> imp_;
};

}  // namespace sire::geometry
#endif