#ifndef SIRE_CONTACT_SOLVER_RESULT_HPP_
#define SIRE_CONTACT_SOLVER_RESULT_HPP_
#include <algorithm>
#include <vector>

#include <hpp/fcl/data_types.h>

#include <aris/core/basic_type.hpp>
#include <aris/core/object.hpp>

#include "sire/core/constants.hpp"
#include "sire/core/geometry/geometry_base.hpp"

namespace sire::physics::contact {
using namespace hpp;
struct ContactSolverResult {
  void resize(int num_velocities, int num_contacts) {
    vs_next.resize(num_velocities, 0);
    fn.resize(num_contacts, 0);
    ft.resize(2 * num_contacts, 0);
    vn.resize(num_contacts, 0);
    vt.resize(2 * num_contacts, 0);
  }
  // 下一时刻的速度旋量
  std::vector<double> vs_next;
  // 法向接触力
  std::vector<double> fn;
  // 切向接触力
  std::vector<double> ft;
  // 法向接触速度（可以用也可以不用）
  std::vector<double> vn;
  // 切向接触速度（可以用也可以不用）
  std::vector<double> vt;
};
}  // namespace sire::physics::contact
#endif