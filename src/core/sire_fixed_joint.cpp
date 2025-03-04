#include "sire/core/sire_fixed_joint.hpp"

#include <aris/core/reflection.hpp>
#include <aris/dynamic/pose.hpp>
#include <aris/dynamic/screw.hpp>

namespace sire::core {
auto FixedJoint::locCmI() const noexcept -> const double* {
  // identity matrix 6 by 6
  static const double loc_cm_I[36]{1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                                   0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0,
                                   0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1};
  return loc_cm_I;
}
auto FixedJoint::cptCpFromPm(double* cp, const double* makI_pm,
                             const double* makJ_pm) const noexcept -> void {
  double pm_j2i[16];
  aris::dynamic::s_inv_pm_dot_pm(makI_pm, makJ_pm, pm_j2i);
  aris::dynamic::s_pm2ps(pm_j2i, cp);
}
auto FixedJoint::cptGlbDmFromPm(double* dm, const double* makI_pm,
                                const double* makJ_pm) const noexcept -> void {
  double pm[16];
  aris::dynamic::s_inv_pm(makI_pm, pm);
  aris::dynamic::s_tmf(pm, dm);
}
FixedJoint::FixedJoint(const std::string& name, aris::dynamic::Marker* makI,
                       aris::dynamic::Marker* makJ)
    : aris::dynamic::Joint(name, makI, makJ) {}
ARIS_REGISTRATION {
  aris::core::class_<FixedJoint>("FixedJoint").inherit<aris::dynamic::Joint>();
}
}  // namespace sire::core