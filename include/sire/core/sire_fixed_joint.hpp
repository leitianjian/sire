#ifndef SIRE_FIXED_JOINT_HPP_
#define SIRE_FIXED_JOINT_HPP_

#include <sire_lib_export.h>

#include <aris/dynamic/model.hpp>
#include <aris/dynamic/model_joint.hpp>

#include "sire/core/constants.hpp"

namespace sire::core {
class SIRE_API FixedJoint : public aris::dynamic::Joint {
 public:
  static auto Dim() -> sire::Size { return 6; }
  static auto add2ModelRelative(aris::dynamic::Model* m,
                                aris::dynamic::Part& first_part,
                                aris::dynamic::Part& second_part,
                                const double* position) -> void {
    double glb_pm[16]{1, 0, 0, position[0], 0, 1, 0, position[1],
                      0, 0, 1, position[2], 0, 0, 0, 1},
        loc_pm[16];
    auto name = "joint_" + std::to_string(m->jointPool().size());
    aris::dynamic::s_inv_pm_dot_pm(*first_part.pm(), glb_pm, loc_pm);
    auto& mak_i = first_part.addMarker(name + "_i", loc_pm);
    aris::dynamic::s_inv_pm_dot_pm(*second_part.pm(), glb_pm, loc_pm);
    auto& mak_j = second_part.addMarker(name + "_j", loc_pm);

    auto& ret = m->jointPool().add<FixedJoint>(name, &mak_i, &mak_j);
    ret.resetModel(m);
  }
  static auto add2ModelAbs(aris::dynamic::Model* m,
                           aris::dynamic::Part& first_part,
                           aris::dynamic::Part& second_part,
                           const double* first_position,
                           const double* second_position) -> void {
    double glb_pm[16]{1, 0, 0, first_position[0], 0, 1, 0, first_position[1],
                      0, 0, 1, first_position[2], 0, 0, 0, 1},
        loc_pm[16];
    auto name = "joint_" + std::to_string(m->jointPool().size());
    aris::dynamic::s_inv_pm_dot_pm(*first_part.pm(), glb_pm, loc_pm);
    auto& mak_i = first_part.addMarker(name + "_i", loc_pm);
    glb_pm[3] = second_position[0];
    glb_pm[7] = second_position[1];
    glb_pm[11] = second_position[2];
    aris::dynamic::s_inv_pm_dot_pm(*second_part.pm(), glb_pm, loc_pm);
    auto& mak_j = second_part.addMarker(name + "_j", loc_pm);

    auto& ret = m->jointPool().add<FixedJoint>(name, &mak_i, &mak_j);
    ret.resetModel(m);
  }
  auto virtual dim() const noexcept -> sire::Size override { return Dim(); }
  /**Return FixedJoint's constraint matrix in local frame, which
   * is a constant equal to
   * 1 0 0 0 0 0
   * 0 1 0 0 0 0
   * 0 0 1 0 0 0
   * 0 0 0 1 0 0
   * 0 0 0 0 1 0
   * 0 0 0 0 0 1 6 * 6 with all motion constrained
   */
  auto virtual locCmI() const noexcept -> const double* override;
  // 用于计算约束方向上的约束冲突的大小
  auto virtual cptCpFromPm(double* cp, const double* makI_pm,
                           const double* makJ_pm) const noexcept
      -> void override;
  // 用于计算世界坐标系下的矩阵 D 用来将对应 C 矩阵变成 [I 0]' 这种形式
  // 详情见 model_solver.cpp 中对矩阵 D 的注释
  auto virtual cptGlbDmFromPm(double* dm, const double* makI_pm,
                              const double* makJ_pm) const noexcept
      -> void override;

  virtual ~FixedJoint() = default;
  explicit FixedJoint(const std::string& name = "fixed_joint",
                      aris::dynamic::Marker* makI = nullptr,
                      aris::dynamic::Marker* makJ = nullptr);
  ARIS_DEFINE_BIG_FOUR(FixedJoint);
};
}  // namespace sire::core

#endif
