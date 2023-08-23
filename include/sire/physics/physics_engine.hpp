#ifndef SIRE_PHYSICS_ENGINE_HPP_
#define SIRE_PHYSICS_ENGINE_HPP_

#include <sire_lib_export.h>

#include "sire/integrator/integrator_base.hpp"
#include "sire/physics/collision/collision_detection.hpp"
#include "sire/physics/contact/contact_solver.hpp"
#include "sire/physics/physics.hpp"

namespace sire {
namespace physics {
class PhysicsEngine {
 public:
  // Config get set method
  auto collisionDetectionFlag() const -> bool;
  auto setCollisionDetectionFlag(bool flag) -> void;
  auto contactSolverFlag() const -> bool;
  auto setContactSolverFlag(bool flag) -> void;
  // collision_detection //
  auto resetCollisionDetection(
      collision::CollisionDetection* collision_detection_in) -> void;
  ;
  auto collisionDetection() const -> const collision::CollisionDetection&;
  auto collisionDetection() -> collision::CollisionDetection& {
    return const_cast<collision::CollisionDetection&>(
        const_cast<const PhysicsEngine*>(this)->collisionDetection());
  }

  // contact_solver //
  auto resetContactSolver(contact::ContactSolver* contact_solver_in) -> void;
  auto contactSolver() const -> const contact::ContactSolver&;
  auto contactSolver() -> contact::ContactSolver& {
    return const_cast<contact::ContactSolver&>(
        const_cast<const PhysicsEngine*>(this)->contactSolver());
  }

  // collision filter
  auto resetCollisionFilter(collision::CollisionFilter* filter) -> void;
  auto collisionFilter() -> collision::CollisionFilter&;

  // geometry pool
  inline auto resetGeometryPool(
      aris::core::PointerArray<geometry::CollidableGeometry,
                               aris::dynamic::Geometry>* pool) -> void;
  inline auto geometryPool() noexcept
      -> aris::core::PointerArray<geometry::CollidableGeometry,
                                  aris::dynamic::Geometry>&;

  auto queryGeometryPoolById(const GeometryId& id) const
      -> geometry::CollidableGeometry*;
  auto queryGeometryPoolById(const GeometryId& id)
      -> geometry::CollidableGeometry* {
    return const_cast<const PhysicsEngine*>(this)->queryGeometryPoolById(id);
  };

  // objects map
  auto dynamicObjectsMap()
      -> std::unordered_map<GeometryId, geometry::CollidableGeometry*>&;
  auto anchoredObjectsMap()
      -> std::unordered_map<GeometryId, geometry::CollidableGeometry*>&;

  // continuous collision detection
  // will insert some time value which should be processed.
  auto continuousCollisionDetection() -> void{};

  // compute contact wrench of model
  auto cptModelContactWrench() -> void{};

  // compute point pair penetration and get result
  auto cptPointPairPenetration(
      std::vector<common::PenetrationAsPointPair>& pairs) -> void;

  // compute real contact time
  auto cptContactTime(const common::PenetrationAsPointPair& penetration)
      -> double;

  // engine state getter
  inline auto numGeometries() -> sire::Size { return geometryPool().size(); }
  inline auto numDynamicGeometries() -> sire::Size;

  // engine state control
  auto init() -> void;

  // this prt_pm represent the pose of geometry on part coordinate
  auto addSphereGeometry(double radius, int part_id = 0,
                         const double* prt_pm = nullptr,
                         bool is_dynamic = false) -> bool;
  auto addDynamicGeometry(geometry::CollidableGeometry& dynamic_geometry)
      -> bool;
  auto addAnchoredGeometry(geometry::CollidableGeometry& anchored_geometry)
      -> bool;

  auto removeGeometry() -> bool;
  auto clearDynamicGeometries() -> bool;
  auto clearAnchoredGeometries() -> bool;
  auto clearGeometries() -> bool;

  // Functions for the main usage
  auto updateGeometryLocationFromModel() -> void;
  auto hasCollision() -> bool;
  auto computePointPairPenetration()
      -> std::vector<common::PenetrationAsPointPair>;

  auto cptContactInfo(
      const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
      std::vector<common::PointPairContactInfo>& contact_info) -> bool;
  auto cptContactForceByPenaltyMethod(
      const std::vector<common::PointPairContactInfo>& contact_info) -> bool;

  auto handleContact() -> void;
  // 给每个杆件配备一个GeneralForce的Componenet，用来设置接触力
  auto initPartContactForce2Model() -> void;
  auto resetPartContactForce() -> void;
  auto setForcePoolSimulation() -> void;

  PhysicsEngine();
  virtual ~PhysicsEngine();
  PhysicsEngine(const PhysicsEngine&) = delete;
  PhysicsEngine& operator=(const PhysicsEngine&) = delete;

 private:
  struct Imp;
  aris::core::ImpPtr<Imp> imp_;
};
}  // namespace physics
}  // namespace sire
#endif