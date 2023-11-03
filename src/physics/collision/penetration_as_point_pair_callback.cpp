#include "sire/physics/collision/penetration_as_point_pair_callback.hpp"

#include <hpp/fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <hpp/fcl/distance.h>
#include <hpp/fcl/math/transform.h>

#include <aris/core/reflection.hpp>
#include <aris/server/control_server.hpp>

#include "sire/core/sire_assert.hpp"
#include "sire/physics/geometry/collidable_geometry.hpp"

namespace sire::physics::collision {
auto PenetrationAsPointPairCallback::calcDistance(
    const fcl::CollisionObject* a, const fcl::CollisionObject* b,
    const fcl::CollisionRequest& request,
    common::PenetrationAsPointPair* pair_data) -> void {
  SIRE_DEMAND(pair_data != nullptr);

  fcl::CollisionResult result;
  fcl::collide(a, b, request, result);

  if (!result.isCollision()) return;

  // Process the contact points
  // NOTE: This assumes that the request is configured to use a single contact.
  const fcl::Contact& contact = result.getContact(0);

  // Signed distance is negative when penetration depth is positive.
  const double depth = contact.penetration_depth;

  // TODO(leitianjian): Remove this test when FCL issue 375 is fixed.
  // FCL returns osculation as contact but doesn't guarantee a non-zero
  // normal. Sire isn't really in a position to define that normal from the
  // geometry or contact results so, if the geometry is sufficiently close
  // to osculation, we consider the geometries to be non-penetrating.
  if (depth <= std::numeric_limits<double>::epsilon()) return;
  pair_data->depth = depth;

  // By convention, Sire requires the contact normal to from A
  // to B. FCL uses the same convention.
  pair_data->nhat_AB_W = contact.normal;

  // FCL returns a single contact point centered between the two
  // penetrating surfaces. PenetrationAsPointPair expects
  // two, one on the surface of body A (Ac) and one on the surface of body
  // B (Bc). Choose points along the line defined by the contact point and
  // normal, equidistant to the contact point. Recall that depth
  // is non-negative, so depth * nhat_AB_W points from object A to object B.
  // Ac to Bc is negative depth * nhat_AB_W
  pair_data->p_WCa =
      contact.pos + 0.5 * pair_data->depth * pair_data->nhat_AB_W;
  pair_data->p_WCb =
      contact.pos - 0.5 * pair_data->depth * pair_data->nhat_AB_W;
  pair_data->p_WC = contact.pos;

  pair_data->id_A = filter_->queryGeometryIdByPtr(
      const_cast<fcl::CollisionObject*>(a)->collisionGeometry().get());
  pair_data->id_B = filter_->queryGeometryIdByPtr(
      const_cast<fcl::CollisionObject*>(b)->collisionGeometry().get());
}

auto PenetrationAsPointPairCallback::collide(
    fcl::CollisionObject* fcl_object_A_ptr,
    fcl::CollisionObject* fcl_object_B_ptr) -> bool {
  SIRE_DEMAND(point_pairs != nullptr);

  // keep the query order smaller geometry id as a.
  GeometryId id_A = filter_->queryGeometryIdByPtr(
      fcl_object_A_ptr->collisionGeometry().get());
  GeometryId id_B = filter_->queryGeometryIdByPtr(
      fcl_object_B_ptr->collisionGeometry().get());

  if (id_A > id_B) {
    std::swap(id_A, id_B);
    std::swap(fcl_object_A_ptr, fcl_object_B_ptr);
  }
  // NOTE: Here and below, false is returned regardless of whether collision
  // is detected or not because true tells the broadphase manager to terminate.
  // Since we want *all* collisions, we return false.
  if (!filter_->canCollideWith(fcl_object_A_ptr, fcl_object_B_ptr))
    return false;
  common::PenetrationAsPointPair penetration;
  calcDistance(fcl_object_A_ptr, fcl_object_B_ptr, request, &penetration);
  if (penetration.depth >= 0) {
    point_pairs->push_back(penetration);
    //std::cout << penetration.depth << std::endl;
  }
  return false;
}
PenetrationAsPointPairCallback::PenetrationAsPointPairCallback(
    CollisionFilter* filter_in,
    vector<common::PenetrationAsPointPair>* point_pairs_in)
    : fcl::CollisionCallBackBase(),
      filter_(filter_in),
      point_pairs(point_pairs_in) {
  request.num_max_contacts = 1;
  request.enable_contact = true;
  request.gjk_tolerance = 2e-12;
};
}  // namespace sire::physics::collision
