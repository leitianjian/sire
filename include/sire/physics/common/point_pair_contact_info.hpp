#ifndef SIRE_POINT_PAIR_CONTACT_INFO_HPP_
#define SIRE_POINT_PAIR_CONTACT_INFO_HPP_
#include <algorithm>

#include <hpp/fcl/data_types.h>

#include <aris/core/basic_type.hpp>
#include <aris/core/object.hpp>

#include "sire/core/constants.hpp"
#include "sire/core/geometry/geometry_base.hpp"
#include "sire/physics/common/penetration_as_point_pair.hpp"

namespace sire::physics::common {
using namespace hpp;
class PointPairContactInfo {
 public:
  PointPairContactInfo(sire::PartId partId_A, sire::PartId partId_B,
                       const double* fs_WC, const double* pe_WC,
                       double separation_speed, double slip_speed,
                       const PenetrationAsPointPair& point_pair)
      : point_pair_(point_pair),
        partId_A_(partId_A),
        partId_B_(partId_B),
        fs_WC_(),
        pe_WC_(),
        separation_speed_(separation_speed),
        slip_speed_(slip_speed) {
    std::copy(fs_WC, fs_WC + 6, fs_WC_);
    std::copy(pe_WC, pe_WC + 6, pe_WC_);
  };
  ~PointPairContactInfo(){};
  ARIS_DEFINE_BIG_FOUR(PointPairContactInfo);

  sire::PartId partId_A() const { return partId_A_; }

  sire::PartId partId_B() const { return partId_B_; }

  const double* contact_force() const { return fs_WC_; };

  const double* contact_point_pe() const { return pe_WC_; };

  double slip_speed() const { return slip_speed_; };

  double separation_speed() const { return separation_speed_; };

  const PenetrationAsPointPair& point_pair() const { return point_pair_; }

 private:
  PenetrationAsPointPair point_pair_;

  /** The id of the first geometry in the contact. */
  sire::PartId partId_A_;
  /** The id of the second geometry in the contact. */
  sire::PartId partId_B_;
  /** Contact force screw from A to B in world frame. */
  double fs_WC_[6];
  /** Contact point position euler angle 313 in world frame*/
  double pe_WC_[6];
  /** The penetration depth. Should be positive*/
  double separation_speed_;
  double slip_speed_;
};

}  // namespace sire::physics::common
#endif