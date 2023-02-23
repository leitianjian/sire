#include "sire/core/geometry/geometry_on_part.hpp"

#include <array>
#include <memory>
#include <string>
#include <string_view>

#include <aris/core/reflection.hpp>
#include <aris/dynamic/model.hpp>
#include <aris/server/control_server.hpp>

namespace sire::geometry {
struct GeometryOnPart::Imp {
  double prt_pm_[4][4]{{0}};
  bool is_dynamic_{false};
  int part_id_{0};
  aris::dynamic::Part* part_ptr_{nullptr};
};
auto GeometryOnPart::partPm() const -> const aris::dynamic::double4x4& {
  return imp_->prt_pm_;
}
auto GeometryOnPart::isDynamic() -> bool { return imp_->is_dynamic_; }
auto GeometryOnPart::setDynamic(bool is_dynamic) -> void {
  imp_->is_dynamic_ = is_dynamic;
}
auto GeometryOnPart::partId() -> int { return imp_->part_id_; }
auto GeometryOnPart::setPartId(int part_id) -> void {
  imp_->part_id_ = part_id;
}
GeometryOnPart::GeometryOnPart(const double* pm_in)
    : GeometryBase(), imp_(new Imp) {
  pm_in = pm_in ? pm_in : default_pm;
  aris::dynamic::s_vc(16, pm_in, *imp_->prt_pm_);
}
ARIS_DEFINE_BIG_FOUR_CPP(GeometryOnPart)

GeometryOnPart::~GeometryOnPart() = default;

ARIS_REGISTRATION {
  auto getPartPm = [](GeometryOnPart* g) -> aris::core::Matrix {
    double pm[16];
    aris::dynamic::s_vc(16, *g->partPm(), pm);
    return aris::core::Matrix(4, 4, pm);
  };
  auto setPartPm = [](GeometryOnPart* g, aris::core::Matrix pm) -> void {
    std::copy_n(pm.data(), 16, const_cast<double*>(*g->partPm()));
  };
  auto setPartId = [](GeometryOnPart* geometry, int part_id) {
    geometry->setPartId(part_id);
  };
  auto getPartId = [](GeometryOnPart* geometry) { return geometry->partId(); };
  aris::core::class_<GeometryOnPart>("GeometryOnPart")
      .inherit<GeometryBase>()
      .prop("prt_pm", &setPartPm, &getPartPm)
      .prop("part_id", &setPartId, &getPartId)
      .prop("is_dynamic", &GeometryOnPart::setDynamic,
            &GeometryOnPart::isDynamic);
}
}  // namespace sire::geometry