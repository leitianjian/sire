#ifndef SIRE_GEOMETRY_BASE_HPP_
#define SIRE_GEOMETRY_BASE_HPP_

#include <atomic>

#include <aris/core/object.hpp>
#include <aris/dynamic/model_basic.hpp>
#include <aris/dynamic/model_coordinate.hpp>

namespace sire::geometry {
using GeometryId = int64_t;
static std::atomic<GeometryId> geometry_id_flag = 0;
static auto generate_new_id() -> GeometryId { return ++geometry_id_flag; }
static const double default_pm[16] = {1, 0, 0, 0, 0, 1, 0, 0,
                                      0, 0, 1, 0, 0, 0, 0, 1};

class GeometryBase : public aris::dynamic::Geometry {
 public:
  auto geometryId() -> GeometryId;
  auto setGeometryId(GeometryId id) -> void;
  auto pm() const -> const aris::dynamic::double4x4&;
  auto setPm(const double* pm_in) -> void;
  explicit GeometryBase(const double* pm_in = nullptr);
  virtual ~GeometryBase();

 private:
  struct Imp;
  aris::core::ImpPtr<Imp> imp_;
};

}  // namespace sire::geometry
#endif