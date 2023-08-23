#ifndef SIRE_PENETRATION_AS_POINT_PAIR_HPP_
#define SIRE_PENETRATION_AS_POINT_PAIR_HPP_
#include <algorithm>

#include <hpp/fcl/data_types.h>

#include "sire/core/geometry/geometry_base.hpp"

namespace sire::physics::common {
using namespace hpp;
/** A characterization of the intersection of two penetrating geometries. The
 characterization consists of a pair of points and a normal. The points
 represent a point on each geometry that most deeply penetrates the other
 geometry (in the normal direction). For convenience, the penetration depth
 is provided and is equal to:

     depth = `(p_WCb - p_WCa) ⋅ nhat_AB_W`

 (`depth` is strictly positive when there is penetration and otherwise not
 defined.)*/
struct PenetrationAsPointPair {
  /** Swaps the interpretation of geometries A and B. */
  void SwapAAndB() {
    // Leave depth unchanged.
    std::swap(id_A, id_B);
    std::swap(p_WCa, p_WCb);
    nhat_AB_W = -nhat_AB_W;
  }

  /** The id of the first geometry in the contact. */
  sire::geometry::GeometryId id_A;
  /** The id of the second geometry in the contact. */
  sire::geometry::GeometryId id_B;
  /** The point on A that most deeply penetrates B, measured and expressed in
   the world frame. */
  fcl::Vec3f p_WCa;
  /** The point on B that most deeply penetrates A, measured and expressed in
   the world frame. */
  fcl::Vec3f p_WCb;
  /** Contact point position in world frame*/
  fcl::Vec3f p_WC;
  /** The unit-length normal which defines the penetration direction, pointing
   from geometry A into geometry B, measured and expressed in the world frame.
   It _approximates_ the normal to the plane on which the contact patch lies. */
  fcl::Vec3f nhat_AB_W;
  /** The penetration depth. Should be positive*/
  double depth{-1.0};
};

}  // namespace sire::physics::common
#endif