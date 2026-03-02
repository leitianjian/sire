#ifndef SIRE_POINT_PAIR_CONTACT_INFO_HPP_
#define SIRE_POINT_PAIR_CONTACT_INFO_HPP_
#include <algorithm>

#include <coal/data_types.h>

#include <aris/core/basic_type.hpp>
#include <aris/core/object.hpp>

#include "sire/core/constants.hpp"
#include "sire/core/geometry/geometry_base.hpp"
#include "sire/physics/common/penetration_as_point_pair.hpp"

namespace sire::physics::common {
using namespace coal;
class PointPairContactInfo {
 public:
  PointPairContactInfo(sire::PartId partId_A, sire::PartId partId_B,
                       const double* pe_WC, double separation_speed,
                       double slip_speed,
                       const PenetrationAsPointPair& point_pair,
                       const double* fs_WC, const double* f_WC,
                       const double* fs_WC_vel = nullptr,
                       const double* f_WC_vel = nullptr)
      : partId_A_(partId_A),
        partId_B_(partId_B),
        separation_speed_(separation_speed),
        slip_speed_(slip_speed),
        point_pair_(point_pair) {
    std::copy(pe_WC, pe_WC + 6, pe_WC_);
    std::copy(fs_WC, fs_WC + 6, fs_WC_);
    std::copy(f_WC, f_WC + 3, f_WC_);
    if (fs_WC_vel == nullptr)
      std::fill(fs_WC_vel_, fs_WC_vel_ + 6, 0.0);
    else
      std::copy(fs_WC_vel, fs_WC_vel + 6, fs_WC_vel_);
    
    if (f_WC_vel == nullptr)
      std::fill(f_WC_vel_, f_WC_vel_ + 3, 0.0);
    else  
      std::copy(f_WC_vel, f_WC_vel + 3, f_WC_vel_);
  };
  ~PointPairContactInfo() {};
  SIRE_DEFINE_TO_JSON_HEAD(PointPairContactInfo) {
    j["partId_A"] = partId_A_;
    j["partId_B"] = partId_B_;
    j["contactWrench"] = std::vector<double>(fs_WC_, fs_WC_ + 6);
    j["contactWrenchVel"] = std::vector<double>(fs_WC_vel_, fs_WC_vel_ + 6);
    j["contactForce"] = std::vector<double>(f_WC_, f_WC_ + 3);
    j["contactForceVel"] = std::vector<double>(f_WC_vel_, f_WC_vel_ + 3);
    j["contact_point_pe"] = std::vector<double>(pe_WC_, pe_WC_ + 6);
    j["separation_speed"] = separation_speed_;
    point_pair_.to_json(j["point_pair"]);
    j["slip_speed"] = slip_speed_;
  }
  ARIS_DEFINE_BIG_FOUR(PointPairContactInfo);

  sire::PartId partId_A() const { return partId_A_; }

  sire::PartId partId_B() const { return partId_B_; }

  const double* contact_point_pe() const { return pe_WC_; };

  double separation_speed() const { return separation_speed_; };

  double slip_speed() const { return slip_speed_; };

  const PenetrationAsPointPair& point_pair() const { return point_pair_; }

  const double* contact_force() const { return fs_WC_; };

  const double* contact_force_vector() const { return f_WC_; };

  const double* contact_force_vel() const { return fs_WC_vel_; };

  const double* contact_force_vel_vector() const { return f_WC_vel_; };

 private:
  /** The id of the first geometry in the contact. */
  sire::PartId partId_A_;
  /** The id of the second geometry in the contact. */
  sire::PartId partId_B_;
  /** Contact point position euler angle 313 in world frame*/
  double pe_WC_[6];
  /** The penetration depth. Should be positive*/
  double separation_speed_;
  double slip_speed_;
  PenetrationAsPointPair point_pair_;
  /** Contact force screw from A to B in world frame. */
  double fs_WC_[6];
  /** Contact force vetor in world frame*/
  double f_WC_[3];
  // contact force for update velocity only
  double fs_WC_vel_[6];
  // contact force for update velocity only
  double f_WC_vel_[3];
};
}  // namespace sire::physics::common
#endif