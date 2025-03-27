#ifndef SIRE_CONTACT_SOLVER_RESULT_HPP_
#define SIRE_CONTACT_SOLVER_RESULT_HPP_
#include <algorithm>
#include <map>
#include <vector>

#include <coal/data_types.h>

#include <aris/core/basic_type.hpp>
#include <aris/core/object.hpp>

#include "sire/core/constants.hpp"
#include "sire/core/geometry/geometry_base.hpp"
#include "sire/core/sorted_pair.hpp"

namespace sire::physics::contact {
using namespace coal;
struct ContactSolverResult {
  void resize(sire::Size num_velocities, sire::Size num_contacts) {
    vs_next.resize(num_velocities, 0);
    prtsA.resize(num_contacts, 0);
    prtsB.resize(num_contacts, 0);
    fn.resize(num_contacts, 0);
    ft.resize(2 * num_contacts, 0);
    vn.resize(num_contacts, 0);
    vt.resize(2 * num_contacts, 0);
    if (num_contacts != 0) isEmpty_ = false;
    dt = (isEmpty_) ? 0.0 : -1.0;
  }
  void reset() {
    prtsA.clear();
    prtsB.clear();
    vs_next.clear();
    fn.clear();
    ft.clear();
    vn.clear();
    vt.clear();
    isEmpty_ = true;
    dt = (isEmpty_) ? 0.0 : -1.0;
  }
  double dt{isEmpty_ ? 0.0 : -1.0};
  std::map<sire::core::SortedPair<sire::PartId>, sire::Size> contactPairIdxMap_;
  std::vector<sire::Size> prtsA;
  std::vector<sire::Size> prtsB;
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
  bool isEmpty_{true};
};
}  // namespace sire::physics::contact
#endif