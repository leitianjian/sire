#include "sire/physics/contact/stiffness_damping_contact_solver.hpp"

#include <fstream>
#include <mutex>
#include <string>
#include <thread>

#include <hpp/fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <hpp/fcl/distance.h>
#include <hpp/fcl/math/transform.h>
#include <hpp/fcl/mesh_loader/assimp.h>
#include <hpp/fcl/mesh_loader/loader.h>
#include <hpp/fcl/shape/geometric_shapes.h>

#include <aris/core/reflection.hpp>
#include <aris/server/control_server.hpp>

#include "sire/core/constants.hpp"
#include "sire/core/material_manager.hpp"
#include "sire/core/prop_map.hpp"
#include "sire/physics/collision/collision_detection.hpp"
#include "sire/physics/collision/collision_exists_callback.hpp"
#include "sire/physics/geometry/collidable_geometry.hpp"
#include "sire/physics/physics_engine.hpp"
#include "sire/transfer/part_pq_transfer.hpp"

namespace sire::physics::contact {
using PartPool =
    aris::core::PointerArray<aris::dynamic::Part, aris::dynamic::Element>;
struct StiffnessDampingContactSolver::Imp {
  double default_k_;
  double default_d_;

  auto combineContactParameter(double k1, double k2, double d1, double d2)
      -> std::pair<double, double> {
    auto safe_div = [](double number, double denominator) -> double {
      return denominator == 0.0 ? 0.0 : number / denominator;
    };
    return std::pair<double, double>(safe_div(k1 * k2, k1 + k2),
                                     safe_div(d1 * d2, d1 + d2));
  }

  explicit Imp() : default_k_(2.8e14), default_d_(2000) {}
};
StiffnessDampingContactSolver::StiffnessDampingContactSolver()
    : imp_(new Imp) {}
StiffnessDampingContactSolver::~StiffnessDampingContactSolver(){};
ARIS_DEFINE_BIG_FOUR_CPP(StiffnessDampingContactSolver);
auto StiffnessDampingContactSolver::setDefaultStiffness(double k) noexcept
    -> void {
  imp_->default_k_ = k;
}
auto StiffnessDampingContactSolver::defaultStiffness() noexcept -> double {
  return imp_->default_k_;
}
auto StiffnessDampingContactSolver::setDefaultDamping(double d) noexcept
    -> void {
  imp_->default_d_ = d;
}
auto StiffnessDampingContactSolver::defaultDamping() noexcept -> double {
  return imp_->default_d_;
}
auto StiffnessDampingContactSolver::cptContactSolverResult(
    const aris::dynamic::Model* current_state,
    const std::vector<common::PenetrationAsPointPair>& penetration_pairs,
    const std::vector<std::array<double, 16>>& T_C_vec,
    ContactSolverResult& result) -> void {
  for (int i = 0; i < penetration_pairs.size(); ++i) {
    const common::PenetrationAsPointPair& pair = penetration_pairs[i];
    if (pair.depth <= 0) {
      result.fn[i] = 0;
      continue;
    }
    core::PropMap& contact_prop_A =
        physicsEnginePtr()->queryGeometryPoolById(pair.id_A)->contactProp();
    core::PropMap& contact_prop_B =
        physicsEnginePtr()->queryGeometryPoolById(pair.id_B)->contactProp();
    auto [k, d] = imp_->combineContactParameter(
        contact_prop_A.getPropValueOrDefault("k", imp_->default_k_),
        contact_prop_B.getPropValueOrDefault("k", imp_->default_k_),
        contact_prop_A.getPropValueOrDefault("d", imp_->default_d_),
        contact_prop_B.getPropValueOrDefault("d", imp_->default_d_));
    double vn = physicsEnginePtr()->cptProximityVelocity(pair);
    result.fn[i] = k * pair.depth + d * vn;
  }
}
ARIS_REGISTRATION {
  typedef sire::physics::collision::CollisionFilter& (
      StiffnessDampingContactSolver::*CollisionFilterPoolFunc)();
  aris::core::class_<StiffnessDampingContactSolver>(
      "StiffnessDampingContactSolver")
      .inherit<ContactSolver>()
      .prop("default_k", &StiffnessDampingContactSolver::setDefaultStiffness,
            &StiffnessDampingContactSolver::defaultStiffness)
      .prop("default_d", &StiffnessDampingContactSolver::setDefaultDamping,
            &StiffnessDampingContactSolver::defaultDamping);
}
}  // namespace sire::physics::contact