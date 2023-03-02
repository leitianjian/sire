#ifndef SIRE_CONTACT_ENGINE_HPP_
#define SIRE_CONTACT_ENGINE_HPP_

#include <map>
#include <string>

#include <hpp/fcl/broadphase/broadphase_callbacks.h>
#include <hpp/fcl/broadphase/broadphase_collision_manager.h>
#include <hpp/fcl/broadphase/default_broadphase_callbacks.h>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/collision_data.h>
#include <hpp/fcl/collision_object.h>

#include <aris/core/expression_calculator.hpp>

#include "sire/physics/collision/collided_objects_callback.hpp"
#include "sire/physics/collision/collision_filter.hpp"

namespace sire::physics::contact {
using namespace std;
using namespace hpp;
/* contact-based implementation
 */
class SIRE_API ContactEngine {
 public:
  auto resetCollisionFilter(collision::CollisionFilter* filter) -> void;
  auto collisionFilter() -> collision::CollisionFilter&;
  auto resetDynamicGeometryPool(
      aris::core::PointerArray<geometry::CollidableGeometry,
                               aris::dynamic::Geometry>* pool) -> void;
  auto dynamicGeometryPool()
      -> aris::core::PointerArray<geometry::CollidableGeometry,
                                  aris::dynamic::Geometry>&;
  auto resetAnchoredGeometryPool(
      aris::core::PointerArray<geometry::CollidableGeometry,
                               aris::dynamic::Geometry>* pool) -> void;
  auto anchoredGeometryPool()
      -> aris::core::PointerArray<geometry::CollidableGeometry,
                                  aris::dynamic::Geometry>&;
  auto addDynamicGeometry(geometry::CollidableGeometry& dynamic_geometry)
      -> bool;
  auto addAnchoredGeometry(geometry::CollidableGeometry& anchored_geometry)
      -> bool;
  auto removeGeometry() -> bool;
  auto clearDynamicGeometry() -> bool;
  auto clearAnchoredGeometry() -> bool;

  auto init() -> void;
  ContactEngine();
  virtual ~ContactEngine();
  ContactEngine(const ContactEngine& other) = delete;
  ContactEngine(ContactEngine&& other) = delete;
  ContactEngine& operator=(const ContactEngine& other) = delete;
  ContactEngine& operator=(ContactEngine&& other) = delete;

 private:
  struct Imp;
  unique_ptr<Imp> imp_;
};
}  // namespace sire::contact
#endif