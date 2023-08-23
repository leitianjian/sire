#include "sire/core/geometry/geometry_base.hpp"

#include <array>
#include <memory>
#include <string>
#include <string_view>

#include <aris/core/reflection.hpp>
#include <aris/dynamic/model.hpp>
#include <aris/server/control_server.hpp>

namespace sire::geometry {
GeometryId geometry_id_flag = 0;
struct GeometryBase::Imp {
  GeometryId id_{generate_new_id()};
  double pm_[4][4]{{0}};
};
auto GeometryBase::geometryId() const -> GeometryId { return imp_->id_; }
auto GeometryBase::setGeometryId(GeometryId id) -> void {
  if (id > geometry::geometry_id_flag) {
    geometry_id_flag = id;
  }
  imp_->id_ = id;
}
auto GeometryBase::pm() const -> const aris::dynamic::double4x4& {
  return imp_->pm_;
}
auto GeometryBase::setPm(const double* pm_in) -> void {
  pm_in = pm_in ? pm_in : default_pm;
  aris::dynamic::s_vc(16, pm_in, *imp_->pm_);
}
GeometryBase::GeometryBase(const double* pm_in)
    : aris::dynamic::Geometry(), imp_(new Imp) {
  pm_in = pm_in ? pm_in : default_pm;
  aris::dynamic::s_vc(16, pm_in, *imp_->pm_);
}
SIRE_DEFINE_TO_JSON_HEAD(GeometryBase) {
  j = nlohmann::json{
      {"geometry_id", geometryId()}};
}
ARIS_DEFINE_BIG_FOUR_CPP(GeometryBase)
GeometryBase::~GeometryBase() = default;

ARIS_REGISTRATION {
  auto getPm = [](GeometryBase* g) -> aris::core::Matrix {
    double pm[16];
    aris::dynamic::s_vc(16, *g->pm(), pm);
    return aris::core::Matrix(4, 4, pm);
  };
  auto setPm = [](GeometryBase* g, aris::core::Matrix pm) -> void {
    std::copy_n(pm.data(), 16, const_cast<double*>(*g->pm()));
  };
  aris::core::class_<GeometryBase>("GeometryBase")
      .inherit<aris::dynamic::Geometry>()
      .prop("id", &GeometryBase::setGeometryId, &GeometryBase::geometryId)
      .prop("pm", &setPm, &getPm);
}
}  // namespace sire::geometry