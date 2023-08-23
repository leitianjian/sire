#ifndef SIRE_CONTACT_SOLVER_HPP_
#define SIRE_CONTACT_SOLVER_HPP_

#include <map>
#include <string>

#include <hpp/fcl/broadphase/broadphase_callbacks.h>
#include <hpp/fcl/broadphase/broadphase_collision_manager.h>
#include <hpp/fcl/broadphase/default_broadphase_callbacks.h>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/collision_data.h>
#include <hpp/fcl/collision_object.h>

#include <aris/core/expression_calculator.hpp>
#include <aris/core/object.hpp>

#include "sire/physics/collision/collided_objects_callback.hpp"
#include "sire/physics/collision/collision_filter.hpp"
#include "sire/physics/common/penetration_as_point_pair.hpp"
#include "sire/physics/common/point_pair_contact_info.hpp"
#include "sire/physics/contact/contact_solver_result.hpp"

namespace sire::physics {
class PhysicsEngine;
namespace contact {
using namespace std;
using namespace hpp;
/* contact-based implementation
 */
class SIRE_API ContactSolver {
 public:
  ContactSolver();
  virtual ~ContactSolver();
  ARIS_DECLARE_BIG_FOUR(ContactSolver);
  auto init(physics::PhysicsEngine* engine_ptr) -> void;

  auto handleContact(const common::PenetrationAsPointPair& pair) -> void{};

  virtual auto cptContactSolverResult(
      const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
      const std::vector<std::array<double, 16>>& T_C_vec,
      ContactSolverResult& result) -> void;

 private:
  auto cptContactForce(double A, double B, double k, double D, double r,
                       double w, double t) -> double;
  auto cptPenaltyODE(double contact_time, double projected_start_diff_v,
                     double cr, double m, double k, double delta_t) -> double;
  struct Imp;
  aris::core::ImpPtr<Imp> imp_;
};
}  // namespace contact
}  // namespace sire::physics
#endif