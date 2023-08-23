#ifndef SIRE_PENETRATION_AS_POINT_PAIR_HPP_
#define SIRE_PENETRATION_AS_POINT_PAIR_HPP_
#include <algorithm>

#include <hpp/fcl/data_types.h>

#include "sire/core/geometry/geometry_base.hpp"

namespace sire::physics::collision {
using namespace hpp;
/** The data for reporting the signed distance between two geometries, A and B.
 It provides the id's of the two geometries, the witness points Ca and Cb on
 the surfaces of A and B, the signed distance, and nhat_BA_W a direction of
 fastest increasing distance (always unit length and always point outward
 from B's surface).

 - When A and B are separated, distance > 0.
 - When A and B are touching or penetrating, distance <= 0.
 - By definition, nhat_AB_W must be in the opposite direction of nhat_BA_W.
 - (p_WCa - p_Wcb) = distance · nhat_BA_W.
 @warning For two geometries that are just touching (i.e., distance = 0), the
          underlying code can guarantee a correct value for nhat_BA_W only
          when one geometry is a sphere, and the other geometry is a sphere, a
          box, or a cylinder. Otherwise, the underlying code is not in place yet
          to guarantee a correct value for nhat_BA_W when surfaces are just
          touching, and the vector will be populated by NaN values.
 */
struct SignedDistancePair {
  // TODO(DamrongGuoy): When we have a full implementation of computing
  //  nhat_BA_W in ComputeSignedDistancePairwiseClosestPoints, check a
  //  condition like this (within epsilon):
  //      DRAKE_DEMAND(nhat_BA_W.norm() == T(1.));
  /** Constructor
   @param a             The id of the first geometry (A).
   @param b             The id of the second geometry (B).
   @param p_ACa_in      The witness point on geometry A's surface, in A's frame.
   @param p_BCb_in      The witness point on geometry B's surface, in B's frame.
   @param dist          The signed distance between p_A and p_B.
   @param nhat_BA_W_in  A direction of fastest increasing distance.
   @pre nhat_BA_W_in is unit-length. */
  SignedDistancePair(GeometryId a, GeometryId b, const Vector3<T>& p_ACa_in,
                     const Vector3<T>& p_BCb_in, const T& dist,
                     const Vector3<T>& nhat_BA_W_in)
      : id_A(a),
        id_B(b),
        p_ACa(p_ACa_in),
        p_BCb(p_BCb_in),
        distance(dist),
        nhat_BA_W(nhat_BA_W_in) {}
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
  /** The unit-length normal which defines the penetration direction, pointing
   from geometry A into geometry B, measured and expressed in the world frame.
   It _approximates_ the normal to the plane on which the contact patch lies. */
  fcl::Vec3f nhat_AB_W;
  /** The penetration depth. Should be positive*/
  double depth{-1.0};
};

}  // namespace sire::physics::collision
#endif